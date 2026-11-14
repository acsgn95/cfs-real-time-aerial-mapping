/*
 * MAPPROD_APP worker task (mapprod_app.h): owns the map products
 * (OST_Map_Products*): fusion, orthomosaic, lifecycle, publication.
 */
#include "mapprod_app.h"
#include "mapprod_app_eventids.h"

#include "cfe_core_api_base_msgids.h"
#include "cfe_mission_eds_designparameters.h"
#include "georef_app_eds_designparameters.h"
#include "imaging_c_api.h"
#include "map_products_c_api.h"
#include "mapprod_app_eds_designparameters.h"
#include "ost_datastore.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define MAPPROD_APP_MAX_TILES EdsParam_MAPPROD_APP_MAX_TILES
#define MAPPROD_APP_MAX_SPANS 64 /* tiles of one keyframe, fusion or rendering */

typedef struct
{
    OST_Map_Products_t     *Products;
    OST_Imaging_Decoder_t  *Decoder;
    bool                    CreateReported;
    char                    ProjectedCrs[32 + 1];
    char                    VerticalCrs[32 + 1];
    char                    ProjDataDir[OS_MAX_LOCAL_PATH_LEN];
    char                    GridDir[OS_MAX_LOCAL_PATH_LEN];
    char                    OutputDir[OS_MAX_LOCAL_PATH_LEN];
    OST_Map_FullCamera_t    Camera;
    uint32                  SessionId;
    MAPPROD_APP_Item_t      Item;
    MAPPROD_APP_TileUpdatedTlm_t   UpdatedMsg;
    MAPPROD_APP_TilePublishedTlm_t PublishedMsg; /* finalise all */
    uint32                  UpdatedSequence;
    int64                   FirstPendingUtc; /* first update since the last publication; 0: none */
    uint32                  LastKeyframeId;
    OST_Map_TileSpan_t      Fused[MAPPROD_APP_MAX_SPANS];
    OST_Map_TileSpan_t      Rendered[MAPPROD_APP_MAX_SPANS];
    uint8                   Rgb[3u * MAPPROD_APP_MAX_ORTHO_PIXELS];
} MAPPROD_APP_Worker_t;

static MAPPROD_APP_Worker_t W;

static float MAPPROD_APP_Ms(int64 Ns)
{
    return (float)((double)Ns * 1e-6);
}

static float MAPPROD_APP_SecondsMs(double Seconds)
{
    return (float)(Seconds * 1e3);
}

/* /cf/... table paths to host paths. */
static void MAPPROD_APP_HostPath(const char *Virtual, size_t VirtualLen, char *Out, size_t OutLen)
{
    char Tmp[OS_MAX_PATH_LEN];

    snprintf(Tmp, sizeof(Tmp), "%.*s", (int)VirtualLen, Virtual);
    if (Tmp[0] == '\0' || OS_TranslatePath(Tmp, Out) != OS_SUCCESS)
    {
        snprintf(Out, OutLen, "%s", Tmp);
    }
}

static void MAPPROD_APP_UpdateStats(void)
{
    OST_Map_ProductsStats_t S;

    if (W.Products != NULL && OST_Map_ProductsGetStats(W.Products, &S) == OST_SUCCESS)
    {
        MAPPROD_APP_Data.Counters.ActiveTiles  = S.active_tiles;
        MAPPROD_APP_Data.Counters.ChangedTiles = S.changed_tiles;
        MAPPROD_APP_Data.Counters.MemoryMb     = (float)((double)S.memory_bytes / (1024.0 * 1024.0));
    }
}

/* -------------------------------------------------------------- products */

static void MAPPROD_APP_CreateProducts(void)
{
    const MAPPROD_APP_ConfigTable_t *T = &MAPPROD_APP_Data.Config;
    OST_Map_ProductsConfig_t         C;
    OST_Status_t                     Status;

    snprintf(W.ProjectedCrs, sizeof(W.ProjectedCrs), "%.*s", (int)sizeof(T->ProjectedCrs), T->ProjectedCrs);
    snprintf(W.VerticalCrs, sizeof(W.VerticalCrs), "%.*s", (int)sizeof(T->VerticalCrs), T->VerticalCrs);
    MAPPROD_APP_HostPath(T->ProjDataDir, sizeof(T->ProjDataDir), W.ProjDataDir, sizeof(W.ProjDataDir));
    MAPPROD_APP_HostPath(T->GridDir, sizeof(T->GridDir), W.GridDir, sizeof(W.GridDir));
    MAPPROD_APP_HostPath(T->OutputDir, sizeof(T->OutputDir), W.OutputDir, sizeof(W.OutputDir));

    OST_Map_ProductsDefaultConfig(&C);
    C.projected_crs       = W.ProjectedCrs;
    C.vertical_crs        = W.VerticalCrs;
    C.proj_data_dir       = W.ProjDataDir;
    C.grid_dir            = W.GridDir;
    C.output_dir          = W.OutputDir;
    C.cell_size_m         = T->CellSizeM;
    C.tile_cells          = (int32_t)T->TileCells;
    C.min_observations    = (int32_t)T->MinObservations;
    C.rejection_sigmas    = T->RejectionSigmas;
    C.min_sigma_m         = T->MinSigmaM;
    C.min_confidence      = T->MinConfidence;
    C.ortho_pixel_m       = T->OrthoPixelM;
    C.surface_smoothing_m = T->SurfaceSmoothingM;
    C.switch_margin_deg   = T->SwitchMarginDeg;
    C.render_margin_m     = T->RenderMarginM;
    C.max_active_tiles    = (int32_t)T->MaxActiveTiles;
    C.finalise_distance_m = T->FinaliseDistanceM;

    Status = OST_Map_ProductsCreate(&C, &W.Products);
    MAPPROD_APP_Data.Products = W.Products;
    if (Status == OST_SUCCESS && W.Decoder == NULL)
    {
        Status = OST_Imaging_DecoderCreate(&W.Decoder);
        if (Status != OST_SUCCESS)
        {
            OST_Map_ProductsDestroy(W.Products);
            W.Products                = NULL;
            MAPPROD_APP_Data.Products = NULL;
            W.Decoder                 = NULL;
        }
    }
    MAPPROD_APP_Data.Counters.Ready = Status == OST_SUCCESS;
    if (Status == OST_SUCCESS)
    {
        MAPPROD_APP_FaultResolve(OST_FaultId_LIBRARY_ERROR);
        CFE_EVS_SendEvent(MAPPROD_APP_READY_INF_EID, CFE_EVS_EventType_INFORMATION,
                          "map products ready: %s (heights %s), output %s", W.ProjectedCrs,
                          W.VerticalCrs[0] != '\0' ? W.VerticalCrs : "ellipsoidal", W.OutputDir);
        return;
    }
    W.Products = NULL;
    if (W.CreateReported)
    {
        return; /* tried again on every keyframe; reported once */
    }
    W.CreateReported = true;
    CFE_EVS_SendEvent(MAPPROD_APP_LIB_ERR_EID, CFE_EVS_EventType_ERROR,
                      "map products not created: %s (%s, PROJ data %s, grids %s, output %s)", OST_StatusName(Status),
                      W.ProjectedCrs, W.ProjDataDir, W.GridDir, W.OutputDir);
    MAPPROD_APP_FaultRaise(OST_FaultId_LIBRARY_ERROR, "map products or output directory not usable");
}

static void MAPPROD_APP_WriteFailed(const char *What, const char *Error)
{
    OS_MutSemTake(MAPPROD_APP_Data.CountMutex);
    ++MAPPROD_APP_Data.Counters.TileWriteFailures;
    OS_MutSemGive(MAPPROD_APP_Data.CountMutex);
    CFE_EVS_SendEvent(MAPPROD_APP_WRITE_ERR_EID, CFE_EVS_EventType_ERROR, "%s: %s", What, Error);
    MAPPROD_APP_FaultReport(OST_FaultId_TILE_WRITE, "tile not written");
}

void MAPPROD_APP_SendPublished(MAPPROD_APP_TilePublishedTlm_t *Msg, MAPPROD_APP_PublishReason_Enum_t Reason,
                               const OST_Map_WriteResult_t *R, uint32 LastKeyframeId, int64 FirstPendingUtc)
{
    MAPPROD_APP_Counters_t              *C   = &MAPPROD_APP_Data.Counters;
    MAPPROD_APP_TilePublished_Payload_t *P   = &Msg->Payload;
    const int64                          Now = OST_TimeUtcNs();
    const float                          Ms  = MAPPROD_APP_SecondsMs(R->write_s);

    CFE_MSG_Init(CFE_MSG_PTR(Msg->TelemetryHeader),
                 CFE_SB_ValueToMsgId(
                     CFE_PLATFORM_TLM_TOPICID_TO_MIDV(EdsParam_CFE_MISSION_MAPPROD_APP_TILE_PUBLISHED_TLM_TOPICID)),
                 sizeof(*Msg));
    OS_MutSemTake(MAPPROD_APP_Data.CountMutex);
    ++C->Publications;
    C->LastPublishMs = Ms;
    if (Ms > C->MaxPublishMs)
    {
        C->MaxPublishMs = Ms;
    }
    P->Header.Sequence = ++MAPPROD_APP_Data.PublishedSequence;
    OS_MutSemGive(MAPPROD_APP_Data.CountMutex);
    P->Header.SchemaVersion = 1;
    P->Time                 = Now;
    P->Reason               = Reason;
    P->TilesWritten         = R->tiles;
    P->ActiveTiles          = R->active_tiles;
    P->LastKeyframeId       = LastKeyframeId;
    P->WriteMs              = Ms;
    P->PendingS = FirstPendingUtc != 0 ? (float)((double)(Now - FirstPendingUtc) * 1e-9) : 0.0f;
    CFE_SB_TimeStampMsg(CFE_MSG_PTR(Msg->TelemetryHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(Msg->TelemetryHeader), true);
    CFE_EVS_SendEvent(MAPPROD_APP_PUBLISH_DBG_EID, CFE_EVS_EventType_DEBUG, "%lu tiles written in %.0f ms, delay %.1f s",
                      (unsigned long)R->tiles, P->WriteMs, P->PendingS);
}

/* Publication period: copies the changed tiles for the publisher. While
 * the publisher still writes the previous copy, the changes wait for the
 * next period. */
static void MAPPROD_APP_Snapshot(void)
{
    OST_Map_Snapshot_t *Snapshot = NULL;
    uint32              Tiles    = 0;
    OST_Status_t        Status;

    if (W.Products == NULL || MAPPROD_APP_Data.PublisherBusy)
    {
        return;
    }
    Status = OST_Map_ProductsSnapshot(W.Products, &Snapshot, &Tiles);
    if (Status != OST_SUCCESS)
    {
        MAPPROD_APP_WriteFailed("copying tiles for publication", OST_Map_ProductsLastError(W.Products));
        return;
    }
    MAPPROD_APP_UpdateStats();
    if (Snapshot == NULL)
    {
        return;
    }
    MAPPROD_APP_Data.Snapshot           = Snapshot;
    MAPPROD_APP_Data.SnapshotTiles      = Tiles;
    MAPPROD_APP_Data.SnapshotKeyframeId = W.LastKeyframeId;
    MAPPROD_APP_Data.SnapshotFirstUtc   = W.FirstPendingUtc;
    W.FirstPendingUtc                   = 0;
    MAPPROD_APP_Data.PublisherBusy      = true;
    OS_BinSemGive(MAPPROD_APP_Data.PublishSem);
}

/* Finalises every active tile (command, or the application stops); the
 * worker writes them itself. */
static void MAPPROD_APP_FinaliseAll(MAPPROD_APP_PublishReason_Enum_t Reason)
{
    OST_Map_WriteResult_t R;
    OST_Status_t          Status;

    if (W.Products == NULL)
    {
        return;
    }
    Status = OST_Map_ProductsFinaliseAll(W.Products, &R);
    if (Status != OST_SUCCESS)
    {
        MAPPROD_APP_WriteFailed("finalising all tiles", OST_Map_ProductsLastError(W.Products));
    }
    MAPPROD_APP_Data.Counters.TilesFinalised += R.tiles;
    CFE_EVS_SendEvent(MAPPROD_APP_FINALISE_INF_EID, CFE_EVS_EventType_INFORMATION,
                      "%lu tiles finalised and written in %.0f ms (%s)", (unsigned long)R.tiles,
                      MAPPROD_APP_SecondsMs(R.write_s),
                      Reason == MAPPROD_APP_PublishReason_FINALISE_ALL ? "command" : "application stopped");
    MAPPROD_APP_UpdateStats();
    if (R.tiles > 0)
    {
        MAPPROD_APP_SendPublished(&W.PublishedMsg, Reason, &R, W.LastKeyframeId, W.FirstPendingUtc);
        W.FirstPendingUtc = 0;
    }
}

/* ----------------------------------------------------------- calibration */

static void MAPPROD_APP_OnCalibration(const INGEST_APP_SessionCalibration_Payload_t *P)
{
    W.Camera.width  = (int32_t)P->Width;
    W.Camera.height = (int32_t)P->Height;
    W.Camera.fx     = P->Fx;
    W.Camera.fy     = P->Fy;
    W.Camera.cx     = P->Cx;
    W.Camera.cy     = P->Cy;
    W.Camera.k1     = P->K1;
    W.Camera.k2     = P->K2;
    W.Camera.k3     = P->K3;
    W.Camera.p1     = P->P1;
    W.Camera.p2     = P->P2;
    W.SessionId     = P->SessionId;
    MAPPROD_APP_Data.Counters.CalibrationKnown = P->Width > 0 && P->Height > 0 && P->Fx > 0.0 && P->Fy > 0.0;
    CFE_EVS_SendEvent(MAPPROD_APP_CALIBRATION_EID, CFE_EVS_EventType_INFORMATION,
                      "session %lu: camera %lux%lu, f %.1f px", (unsigned long)P->SessionId,
                      (unsigned long)P->Width, (unsigned long)P->Height, P->Fx);
}

/* -------------------------------------------------------------- keyframe */

/* Decodes the keyframe JPEG at the orthomosaic scale into W.Rgb. */
static OST_Status_t MAPPROD_APP_Decode(const OST_ObjectRef_t *Jpeg, OST_Map_RgbImage_t *Image, char *Text,
                                       size_t TextLen)
{
    const void            *Data = NULL;
    uint64                 Size = 0;
    int32_t                Width = 0, Height = 0;
    OST_Imaging_DctScale_t Scale;
    OST_Imaging_Image_t    Out;
    uint32                 MinWidth = MAPPROD_APP_Data.Config.OrthoWidth;
    OST_Status_t           Status;

    Status = OST_DS_Acquire(Jpeg, &Data, &Size);
    if (Status != OST_SUCCESS)
    {
        snprintf(Text, TextLen, "JPEG not readable: %s", OST_StatusName(Status));
        return Status;
    }
    Status = OST_Imaging_JpegSize(W.Decoder, (const uint8_t *)Data, (size_t)Size, &Width, &Height);
    if (Status == OST_SUCCESS && (Width != W.Camera.width || Height != W.Camera.height))
    {
        snprintf(Text, TextLen, "JPEG %ldx%ld, calibration %ldx%ld", (long)Width, (long)Height,
                 (long)W.Camera.width, (long)W.Camera.height);
        Status = OST_ERR_FORMAT;
    }
    if (Status == OST_SUCCESS)
    {
        if (MinWidth == 0)
        {
            MinWidth = (uint32)Width / 2u;
        }
        Status = OST_Imaging_ChooseDctScale(Width, Height, (int32_t)MinWidth, 0, &Scale);
        if (Status == OST_SUCCESS &&
            (uint64)Scale.width * (uint64)Scale.height > (uint64)MAPPROD_APP_MAX_ORTHO_PIXELS)
        {
            snprintf(Text, TextLen, "decoded size %ldx%ld over the platform limit", (long)Scale.width,
                     (long)Scale.height);
            Status = OST_ERR_INVALID_ARGUMENT;
        }
    }
    if (Status == OST_SUCCESS)
    {
        Out.data     = W.Rgb;
        Out.width    = Scale.width;
        Out.height   = Scale.height;
        Out.channels = 3;
        Out.stride   = (size_t)Scale.width * 3u;
        Status       = OST_Imaging_DecodeJpeg(W.Decoder, (const uint8_t *)Data, (size_t)Size, &Scale, &Out);
        if (Status != OST_SUCCESS)
        {
            snprintf(Text, TextLen, "%s (%s)", OST_StatusName(Status), OST_Imaging_DecoderLastError(W.Decoder));
        }
        Image->data   = W.Rgb;
        Image->width  = Scale.width;
        Image->height = Scale.height;
        Image->stride = Out.stride;
    }
    else if (Text[0] == '\0')
    {
        snprintf(Text, TextLen, "%s (%s)", OST_StatusName(Status), OST_Imaging_DecoderLastError(W.Decoder));
    }
    MAPPROD_APP_Release(Jpeg); /* the read count */
    return Status;
}

/* Adds the tile list of one step to the message list, without repeats. */
static void MAPPROD_APP_MergeTiles(MAPPROD_APP_TileUpdated_Payload_t *P, uint32 *Count, const OST_Map_TileSpan_t *Tiles,
                                   uint32 N)
{
    uint32 i, j;

    for (i = 0; i < N && i < MAPPROD_APP_MAX_SPANS; ++i)
    {
        for (j = 0; j < *Count && j < MAPPROD_APP_MAX_TILES; ++j)
        {
            if (P->Tiles[j].Column == Tiles[i].column && P->Tiles[j].Row == Tiles[i].row)
            {
                break;
            }
        }
        if (j < *Count && j < MAPPROD_APP_MAX_TILES)
        {
            continue;
        }
        if (*Count < MAPPROD_APP_MAX_TILES)
        {
            P->Tiles[*Count].Column = Tiles[i].column;
            P->Tiles[*Count].Row    = Tiles[i].row;
        }
        ++*Count;
    }
}

static void MAPPROD_APP_OnPoints(const MAPPROD_APP_Item_t *It)
{
    const GEOREF_APP_PointsReady_Payload_t *G     = &It->u.Points;
    const MAPPROD_APP_ConfigTable_t        *T     = &MAPPROD_APP_Data.Config;
    MAPPROD_APP_Counters_t                 *C     = &MAPPROD_APP_Data.Counters;
    MAPPROD_APP_TileUpdated_Payload_t      *P     = &W.UpdatedMsg.Payload;
    const int64                             Start = OST_TimeMonoNs();
    OST_Map_FusionResult_t                  F;
    OST_Map_RenderResult_t                  R;
    OST_Map_WriteResult_t                   L;
    OST_Map_PointSet_t                      Set;
    OST_Map_TileSpan_t                      Spans[EdsParam_GEOREF_APP_MAX_TILES];
    OST_Map_RgbImage_t                      Image;
    OST_Map_Pose_t                          Pose;
    const void                             *Points = NULL;
    uint64                                  Size   = 0;
    uint32                                  i, Tiles = 0;
    bool                                    Rendered = false;
    int64                                   T0;
    OST_Status_t                            Status;
    char                                    Text[160];
    float                                   DecodeMs = 0.0f;

    memset(&F, 0, sizeof(F));
    memset(&R, 0, sizeof(R));
    memset(&L, 0, sizeof(L));
    /* CFE_MSG_Init clears the whole message: first. */
    CFE_MSG_Init(CFE_MSG_PTR(W.UpdatedMsg.TelemetryHeader),
                 CFE_SB_ValueToMsgId(
                     CFE_PLATFORM_TLM_TOPICID_TO_MIDV(EdsParam_CFE_MISSION_MAPPROD_APP_TILE_UPDATED_TLM_TOPICID)),
                 sizeof(W.UpdatedMsg));
    Text[0] = '\0';
    P->WaitMs = MAPPROD_APP_Ms(Start - It->EntryMono);
    Pose.position_ecef[0]      = G->Keyframe.Pose.PositionEcef.X;
    Pose.position_ecef[1]      = G->Keyframe.Pose.PositionEcef.Y;
    Pose.position_ecef[2]      = G->Keyframe.Pose.PositionEcef.Z;
    Pose.q_ecef_from_camera[0] = G->Keyframe.Pose.AttitudeEcefFromCamera.W;
    Pose.q_ecef_from_camera[1] = G->Keyframe.Pose.AttitudeEcefFromCamera.X;
    Pose.q_ecef_from_camera[2] = G->Keyframe.Pose.AttitudeEcefFromCamera.Y;
    Pose.q_ecef_from_camera[3] = G->Keyframe.Pose.AttitudeEcefFromCamera.Z;
    if (W.Products == NULL && MAPPROD_APP_Data.ConfigValid)
    {
        MAPPROD_APP_CreateProducts();
    }
    if (W.Products == NULL)
    {
        MAPPROD_APP_Drop(G, "no map products");
        return;
    }

    /* The point set must be on this tile grid (ADR-007 section 1). */
    if (G->PointSet.ObjectId != 0 &&
        (G->TileCells != T->TileCells || fabsf(G->CellSizeM - T->CellSizeM) > 1e-6f ||
         strncmp(G->ProjectedCrs, T->ProjectedCrs, sizeof(T->ProjectedCrs)) != 0))
    {
        CFE_EVS_SendEvent(MAPPROD_APP_GRID_ERR_EID, CFE_EVS_EventType_ERROR,
                          "keyframe %lu: point set on %.32s, %lu x %.3f m; table %.32s, %lu x %.3f m",
                          (unsigned long)G->KeyframeId, G->ProjectedCrs, (unsigned long)G->TileCells, G->CellSizeM,
                          T->ProjectedCrs, (unsigned long)T->TileCells, T->CellSizeM);
        MAPPROD_APP_FaultRaise(OST_FaultId_TABLE_REJECTED, "tile grid differs from GEOREF_APP");
        MAPPROD_APP_Drop(G, "tile grid or CRS differs from the table");
        return;
    }

    /* DSM fusion. */
    if (G->PointSet.ObjectId != 0)
    {
        Status = OST_DS_Acquire(&G->PointSet, &Points, &Size);
        if (Status == OST_SUCCESS && Size < (uint64)G->PointCount * sizeof(OST_Map_Point_t))
        {
            MAPPROD_APP_Release(&G->PointSet);
            Status = OST_ERR_FORMAT;
        }
        if (Status != OST_SUCCESS)
        {
            snprintf(Text, sizeof(Text), "point set not readable: %s", OST_StatusName(Status));
            MAPPROD_APP_Drop(G, Text);
            return;
        }
        for (i = 0; i < G->TileCount && i < EdsParam_GEOREF_APP_MAX_TILES; ++i)
        {
            Spans[i].column = G->Tiles[i].Tile.Column;
            Spans[i].row    = G->Tiles[i].Tile.Row;
            Spans[i].first  = G->Tiles[i].FirstPoint;
            Spans[i].count  = G->Tiles[i].PointCount;
        }
        Set.points     = (const OST_Map_Point_t *)Points;
        Set.count      = G->PointCount;
        Set.tile_count = i;
        Set.tiles      = Spans;
        Set.origin[0]  = G->OriginEasting;
        Set.origin[1]  = G->OriginNorthing;
        Set.origin[2]  = G->OriginHeight;
        Status         = OST_Map_ProductsAddPoints(W.Products, &Set, W.Fused, MAPPROD_APP_MAX_SPANS, &F);
        MAPPROD_APP_Release(&G->PointSet); /* the read count */
        MAPPROD_APP_Release(&G->PointSet); /* the count of POINTS_READY */
        if (Status != OST_SUCCESS)
        {
            snprintf(Text, sizeof(Text), "fusion: %s (%s)", OST_StatusName(Status),
                     OST_Map_ProductsLastError(W.Products));
            MAPPROD_APP_Release(&G->Keyframe.Jpeg);
            OS_MutSemTake(MAPPROD_APP_Data.CountMutex);
            ++C->KeyframesFailed;
            OS_MutSemGive(MAPPROD_APP_Data.CountMutex);
            CFE_EVS_SendEvent(MAPPROD_APP_FAILED_EID, CFE_EVS_EventType_ERROR, "keyframe %lu (image %lu) dropped: %s",
                              (unsigned long)G->KeyframeId, (unsigned long)G->Keyframe.ImageSeq, Text);
            MAPPROD_APP_FaultReport(OST_FaultId_MAP_UPDATE_FAILED, "keyframe not in the map");
            return;
        }
        MAPPROD_APP_MergeTiles(P, &Tiles, W.Fused, F.tile_count);
    }

    /* Orthomosaic: the keyframe over its footprint, on the DSM it has just
     * updated. A keyframe without points renders nothing, as offline. */
    if (T->RenderOrtho && F.points > 0 && G->Keyframe.Jpeg.ObjectId != 0)
    {
        if (!C->CalibrationKnown)
        {
            ++C->OrthoSkipped;
            CFE_EVS_SendEvent(MAPPROD_APP_NO_ORTHO_EID, CFE_EVS_EventType_ERROR,
                              "keyframe %lu: no session calibration, not in the orthomosaic",
                              (unsigned long)G->KeyframeId);
        }
        else
        {
            T0     = OST_TimeMonoNs();
            Status = MAPPROD_APP_Decode(&G->Keyframe.Jpeg, &Image, Text, sizeof(Text));
            DecodeMs = MAPPROD_APP_Ms(OST_TimeMonoNs() - T0);
            if (Status != OST_SUCCESS)
            {
                ++C->OrthoSkipped;
                CFE_EVS_SendEvent(MAPPROD_APP_DECODE_ERR_EID, CFE_EVS_EventType_ERROR,
                                  "keyframe %lu: not in the orthomosaic: %s", (unsigned long)G->KeyframeId, Text);
                MAPPROD_APP_FaultReport(OST_FaultId_IMAGE_DECODE, "keyframe JPEG not decoded");
            }
            else
            {
                Status = OST_Map_ProductsRender(W.Products, &Image, &W.Camera, &Pose, F.height_median, F.extent,
                                                W.Rendered, MAPPROD_APP_MAX_SPANS, &R);
                if (Status != OST_SUCCESS)
                {
                    ++C->OrthoSkipped;
                    CFE_EVS_SendEvent(MAPPROD_APP_LIB_ERR_EID, CFE_EVS_EventType_ERROR,
                                      "keyframe %lu: rendering failed: %s (%s)", (unsigned long)G->KeyframeId,
                                      OST_StatusName(Status), OST_Map_ProductsLastError(W.Products));
                    MAPPROD_APP_FaultReport(OST_FaultId_LIBRARY_ERROR, "orthomosaic rendering failed");
                }
                else
                {
                    Rendered = true;
                    MAPPROD_APP_MergeTiles(P, &Tiles, W.Rendered, R.tile_count);
                }
            }
        }
    }
    MAPPROD_APP_Release(&G->Keyframe.Jpeg); /* the count of POINTS_READY: the end of the data path */

    /* Lifecycle: finalise tiles far from the vehicle (ADR-007 section 5);
     * the keyframe's camera centre stands for the vehicle. */
    Status = OST_Map_ProductsUpdate(W.Products, Pose.position_ecef, &L);
    if (Status != OST_SUCCESS)
    {
        MAPPROD_APP_WriteFailed("finalising tiles", OST_Map_ProductsLastError(W.Products));
    }
    C->TilesFinalised += L.tiles;
    MAPPROD_APP_UpdateStats();

    /* TILE_UPDATED: the map update of ADR-001 R2. */
    P->Header.SchemaVersion = 1;
    P->Header.Sequence      = ++W.UpdatedSequence;
    P->Trace.EntryMono      = It->EntryMono;
    P->KeyframeId           = G->KeyframeId;
    P->ImageSeq             = G->Keyframe.ImageSeq;
    P->CaptureUtc           = G->Keyframe.CaptureUtc;
    P->PoseSource           = G->PoseSource;
    P->Ortho                = Rendered ? 1 : 0;
    P->TileCount            = (uint8)(Tiles > 255u ? 255u : Tiles);
    P->PointsFused          = F.added;
    P->PointsRejected       = F.rejected;
    P->OrthoPixels          = R.pixels;
    P->ActiveTiles          = L.active_tiles;
    P->TilesFinalised       = L.tiles;
    P->FuseMs               = MAPPROD_APP_SecondsMs(F.fuse_s);
    P->DecodeMs             = DecodeMs;
    P->RenderMs             = MAPPROD_APP_SecondsMs(R.render_s);
    P->LifecycleMs          = MAPPROD_APP_SecondsMs(L.write_s);
    P->MemoryMb             = C->MemoryMb;
    P->Trace.ExitMono       = OST_TimeMonoNs();
    P->Trace.PublishUtc     = OST_TimeUtcNs();
    P->LatencyS             = (float)((double)(P->Trace.PublishUtc - G->Keyframe.CaptureUtc) * 1e-9);
    CFE_SB_TimeStampMsg(CFE_MSG_PTR(W.UpdatedMsg.TelemetryHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(W.UpdatedMsg.TelemetryHeader), true);

    if (Tiles > 0 && W.FirstPendingUtc == 0)
    {
        W.FirstPendingUtc = P->Trace.PublishUtc;
    }
    W.LastKeyframeId    = G->KeyframeId;
    ++C->KeyframesUpdated;
    C->LastKeyframeId   = G->KeyframeId;
    C->LastFuseMs       = P->FuseMs;
    C->LastDecodeMs     = P->DecodeMs;
    C->LastRenderMs     = P->RenderMs;
    C->LastLifecycleMs  = P->LifecycleMs;
    C->LastUpdateMs     = MAPPROD_APP_Ms(P->Trace.ExitMono - Start);
    C->LastLatencyS     = P->LatencyS;
    if (C->LastUpdateMs > C->MaxUpdateMs)
    {
        C->MaxUpdateMs = C->LastUpdateMs;
    }
    if (P->LatencyS > C->MaxLatencyS)
    {
        C->MaxLatencyS = P->LatencyS;
    }
    CFE_EVS_SendEvent(MAPPROD_APP_UPDATE_DBG_EID, CFE_EVS_EventType_DEBUG,
                      "keyframe %lu: %lu points (%lu rejected), %lu ortho pixels, %u tiles, fuse %.0f ms, decode "
                      "%.0f ms, render %.0f ms, lifecycle %.0f ms",
                      (unsigned long)G->KeyframeId, (unsigned long)F.added, (unsigned long)F.rejected,
                      (unsigned long)R.pixels, (unsigned)Tiles, P->FuseMs, P->DecodeMs, P->RenderMs, P->LifecycleMs);
}

/* ------------------------------------------------------------------ task */

void MAPPROD_APP_WorkerTask(void)
{
    memset(&W, 0, sizeof(W));
    if (MAPPROD_APP_Data.ConfigValid)
    {
        MAPPROD_APP_CreateProducts();
    }

    while (MAPPROD_APP_Data.RunStatus == CFE_ES_RunStatus_APP_RUN)
    {
        if (!MAPPROD_APP_Dequeue(&W.Item))
        {
            continue;
        }
        switch (W.Item.Type)
        {
            case MAPPROD_APP_ITEM_POINTS:
                MAPPROD_APP_OnPoints(&W.Item);
                break;
            case MAPPROD_APP_ITEM_CALIBRATION:
                MAPPROD_APP_OnCalibration(&W.Item.u.Calibration);
                break;
            case MAPPROD_APP_ITEM_PUBLISH:
                MAPPROD_APP_Snapshot();
                MAPPROD_APP_Data.PublishQueued = false;
                break;
            case MAPPROD_APP_ITEM_FINALISE_ALL:
                MAPPROD_APP_FinaliseAll(MAPPROD_APP_PublishReason_FINALISE_ALL);
                break;
            default:
                break;
        }
    }

    /* Point sets still queued carry their counts. */
    while (MAPPROD_APP_QueueDepth() > 0 && MAPPROD_APP_Dequeue(&W.Item))
    {
        if (W.Item.Type == MAPPROD_APP_ITEM_POINTS)
        {
            MAPPROD_APP_Drop(&W.Item.u.Points, "application stopped");
        }
    }
    /* The publisher finishes its copy first: the products stay until then. */
    MAPPROD_APP_Data.PublisherStop = true;
    OS_BinSemGive(MAPPROD_APP_Data.PublishSem);
    OS_BinSemTimedWait(MAPPROD_APP_Data.PublisherDone, MAPPROD_APP_STOP_WAIT_MS);
    if (MAPPROD_APP_Data.Config.FinaliseOnStop)
    {
        MAPPROD_APP_FinaliseAll(MAPPROD_APP_PublishReason_STOPPED);
    }
    MAPPROD_APP_Data.Products = NULL;
    OST_Map_ProductsDestroy(W.Products);
    W.Products = NULL;
    OST_Imaging_DecoderDestroy(W.Decoder);
    W.Decoder = NULL;
    OS_BinSemGive(MAPPROD_APP_Data.WorkerDone);
}

/* ------------------------------------------------------------- publisher */

void MAPPROD_APP_PublisherTask(void)
{
    static MAPPROD_APP_TilePublishedTlm_t Msg; /* publisher only */
    OST_Map_WriteResult_t                 R;
    OST_Status_t                          Status;

    for (;;)
    {
        /* The semaphore orders the worker's hand-over before these reads. */
        if (OS_BinSemTimedWait(MAPPROD_APP_Data.PublishSem, 500) != OS_SUCCESS)
        {
            continue;
        }
        if (MAPPROD_APP_Data.PublisherBusy)
        {
            Status = OST_Map_ProductsWriteSnapshot(MAPPROD_APP_Data.Products, MAPPROD_APP_Data.Snapshot, &R);
            MAPPROD_APP_Data.Snapshot = NULL; /* freed by the call */
            if (Status != OST_SUCCESS)
            {
                MAPPROD_APP_WriteFailed("publication", OST_Map_ProductsPublishError(MAPPROD_APP_Data.Products));
            }
            if (R.tiles > 0)
            {
                R.active_tiles = MAPPROD_APP_Data.Counters.ActiveTiles;
                MAPPROD_APP_SendPublished(&Msg, MAPPROD_APP_PublishReason_PERIODIC, &R,
                                          MAPPROD_APP_Data.SnapshotKeyframeId, MAPPROD_APP_Data.SnapshotFirstUtc);
            }
            MAPPROD_APP_Data.PublisherBusy = false;
        }
        if (MAPPROD_APP_Data.PublisherStop)
        {
            break;
        }
    }
    OS_BinSemGive(MAPPROD_APP_Data.PublisherDone);
}
