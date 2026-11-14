/*
 * GEOREF_APP worker task (georef_app.h): one keyframe at a time with the
 * map products library (OST_Map_Georef*), the publication of POINTS_READY
 * and the point cloud files.
 */
#include "georef_app.h"
#include "georef_app_eventids.h"

#include "cfe_core_api_base_msgids.h"
#include "cfe_mission_eds_designparameters.h"
#include "georef_app_eds_designparameters.h"
#include "map_products_c_api.h"
#include "ost_datastore.h"

#include <stdio.h>
#include <string.h>

#define GEOREF_APP_MAX_TILES EdsParam_GEOREF_APP_MAX_TILES

typedef struct
{
    OST_Map_Georef_t           *Georef;
    bool                        CreateReported;
    char                        ProjectedCrs[32 + 1];
    char                        VerticalCrs[32 + 1];
    char                        ProjDataDir[OS_MAX_LOCAL_PATH_LEN];
    char                        GridDir[OS_MAX_LOCAL_PATH_LEN];
    char                        OutputDir[OS_MAX_LOCAL_PATH_LEN];
    GEOREF_APP_Item_t           Item;
    GEOREF_APP_PointsReadyTlm_t OutMsg;
    uint32                      Sequence;
    OST_Map_TileSpan_t          Tiles[GEOREF_APP_MAX_TILES];
} GEOREF_APP_Worker_t;

static GEOREF_APP_Worker_t W;

static float GEOREF_APP_Ms(int64 Ns)
{
    return (float)((double)Ns * 1e-6);
}

/* /cf/... table paths to host paths. */
static void GEOREF_APP_HostPath(const char *Virtual, size_t VirtualLen, char *Out, size_t OutLen)
{
    char Tmp[OS_MAX_PATH_LEN];

    snprintf(Tmp, sizeof(Tmp), "%.*s", (int)VirtualLen, Virtual);
    if (Tmp[0] == '\0' || OS_TranslatePath(Tmp, Out) != OS_SUCCESS)
    {
        snprintf(Out, OutLen, "%s", Tmp);
    }
}

/* ------------------------------------------------------- product frame */

static void GEOREF_APP_CreateGeoref(void)
{
    const GEOREF_APP_ConfigTable_t *T = &GEOREF_APP_Data.Config;
    OST_Map_GeorefConfig_t          C;
    OST_Status_t                    Status;

    snprintf(W.ProjectedCrs, sizeof(W.ProjectedCrs), "%.*s", (int)sizeof(T->ProjectedCrs), T->ProjectedCrs);
    snprintf(W.VerticalCrs, sizeof(W.VerticalCrs), "%.*s", (int)sizeof(T->VerticalCrs), T->VerticalCrs);
    GEOREF_APP_HostPath(T->ProjDataDir, sizeof(T->ProjDataDir), W.ProjDataDir, sizeof(W.ProjDataDir));
    GEOREF_APP_HostPath(T->GridDir, sizeof(T->GridDir), W.GridDir, sizeof(W.GridDir));
    W.OutputDir[0] = '\0';
    if (T->WriteLaz)
    {
        GEOREF_APP_HostPath(T->OutputDir, sizeof(T->OutputDir), W.OutputDir, sizeof(W.OutputDir));
    }

    OST_Map_GeorefDefaultConfig(&C);
    C.projected_crs  = W.ProjectedCrs;
    C.vertical_crs   = W.VerticalCrs;
    C.proj_data_dir  = W.ProjDataDir;
    C.grid_dir       = W.GridDir;
    C.output_dir     = W.OutputDir;
    C.cell_size_m    = T->CellSizeM;
    C.tile_cells     = (int32_t)T->TileCells;
    C.min_confidence = T->MinConfidence;

    Status                           = OST_Map_GeorefCreate(&C, &W.Georef);
    GEOREF_APP_Data.Counters.Ready = Status == OST_SUCCESS;
    if (Status == OST_SUCCESS)
    {
        GEOREF_APP_FaultResolve(OST_FaultId_LIBRARY_ERROR);
        CFE_EVS_SendEvent(GEOREF_APP_READY_INF_EID, CFE_EVS_EventType_INFORMATION,
                          "product frame ready: %s, point cloud %s%s", W.ProjectedCrs,
                          W.OutputDir[0] != '\0' ? W.OutputDir : "off", W.OutputDir[0] != '\0' ? "/points" : "");
        return;
    }
    W.Georef = NULL;
    if (W.CreateReported)
    {
        return; /* tried again on every keyframe; reported once */
    }
    W.CreateReported = true;
    CFE_EVS_SendEvent(GEOREF_APP_LIB_ERR_EID, CFE_EVS_EventType_ERROR,
                      "OST_Map_GeorefCreate: %s (%s, PROJ data %s, grids %s, output %s)", OST_StatusName(Status),
                      W.ProjectedCrs, W.ProjDataDir, W.GridDir, W.OutputDir);
    GEOREF_APP_FaultRaise(OST_FaultId_LIBRARY_ERROR, "product frame or output directory not usable");
}

/* Point set within the quota, waiting up to QuotaWaitMs (ADR-015 section
 * 3: MAPPROD_APP releases point sets as it fuses them). */
static OST_Status_t GEOREF_APP_AllocatePointSet(uint64 Size, OST_ObjectRef_t *Ref, void **Data)
{
    const int64  Start  = OST_TimeMonoNs();
    bool         Waited = false;
    OST_Status_t Status;

    for (;;)
    {
        Status = OST_DS_Allocate(OST_DATASTORE_POINT_SET, Size, Ref, Data);
        if (Status != OST_ERR_QUOTA && Status != OST_ERR_NO_MEMORY)
        {
            return Status;
        }
        if (!Waited)
        {
            Waited = true;
            ++GEOREF_APP_Data.Counters.QuotaWaits;
        }
        if (OST_TimeMonoNs() - Start >= (int64)GEOREF_APP_Data.Config.QuotaWaitMs * 1000000 ||
            GEOREF_APP_Data.RunStatus != CFE_ES_RunStatus_APP_RUN)
        {
            return Status;
        }
        OS_TaskDelay(GEOREF_APP_QUOTA_RETRY_MS);
    }
}

/* ---------------------------------------------------------- point cloud */

static void GEOREF_APP_WriteLaz(const GEOREF_APP_PointsReady_Payload_t *P, const OST_Map_Point_t *Points)
{
    GEOREF_APP_Counters_t *C = &GEOREF_APP_Data.Counters;
    OST_Map_PointSet_t     Set;
    OST_Map_LazResult_t    R;
    OST_Status_t           Status;

    Set.points     = Points;
    Set.count      = P->PointCount;
    Set.tile_count = P->TileCount;
    Set.tiles      = W.Tiles;
    Set.origin[0]  = P->OriginEasting;
    Set.origin[1]  = P->OriginNorthing;
    Set.origin[2]  = P->OriginHeight;
    Status         = OST_Map_GeorefWriteLaz(W.Georef, P->KeyframeId, &Set, &R);
    if (Status != OST_SUCCESS)
    {
        ++C->LazFailures;
        CFE_EVS_SendEvent(GEOREF_APP_LAZ_ERR_EID, CFE_EVS_EventType_ERROR,
                          "keyframe %lu: point cloud not written: %s (%s)", (unsigned long)P->KeyframeId,
                          OST_StatusName(Status), OST_Map_GeorefLastError(W.Georef));
        GEOREF_APP_FaultReport(OST_FaultId_TILE_WRITE, "point cloud file not written");
        return;
    }
    C->LazFiles += R.files;
    C->LastLazMs = (float)(R.write_s * 1e3);
    if (C->LastLazMs > C->MaxLazMs)
    {
        C->MaxLazMs = C->LastLazMs;
    }
}

/* -------------------------------------------------------------- keyframe */

static void GEOREF_APP_OnDepthSet(const GEOREF_APP_Item_t *It)
{
    const DENSE_APP_DepthReady_Payload_t *D      = &It->Depth;
    GEOREF_APP_Counters_t                *C      = &GEOREF_APP_Data.Counters;
    GEOREF_APP_PointsReady_Payload_t     *P      = &W.OutMsg.Payload;
    const int64                           Start  = OST_TimeMonoNs();
    const uint64                          Pixels = (uint64)D->Intrinsics.Width * D->Intrinsics.Height;
    const bool                            Laz    = GEOREF_APP_Data.Config.WriteLaz != 0;
    const void                           *Depth  = NULL;
    const void                           *Image  = NULL;
    uint64                                DepthSize = 0, ImageSize = 0;
    OST_Map_DepthMap_t                    Map;
    OST_Map_Image_t                       Img;
    OST_Map_Pose_t                        Pose;
    OST_Map_GeorefResult_t                R;
    OST_ObjectRef_t                       PointSet = {0, 0};
    void                                 *Points   = NULL;
    uint32                                Count    = 0;
    const void                           *Held     = NULL; /* point set under a read count, for LAZ */
    uint64                                HeldSize = 0;
    uint32                                i;
    int64                                 T0;
    OST_Status_t                          Status;
    char                                  Text[160];
    float                                 Latency;

    C->LastWaitMs = GEOREF_APP_Ms(Start - It->EntryMono);
    if (W.Georef == NULL && GEOREF_APP_Data.ConfigValid)
    {
        GEOREF_APP_CreateGeoref();
    }
    if (W.Georef == NULL)
    {
        GEOREF_APP_Drop(D, "no product frame");
        return;
    }
    if (Pixels == 0 || Pixels > GEOREF_APP_MAX_PIXELS)
    {
        snprintf(Text, sizeof(Text), "depth map of %ux%u pixels", (unsigned)D->Intrinsics.Width,
                 (unsigned)D->Intrinsics.Height);
        GEOREF_APP_Drop(D, Text);
        return;
    }

    /* Read counts of the depth set and the working image, on top of the
     * counts DEPTH_READY gave this application. */
    Status = OST_DS_Acquire(&D->DepthSet, &Depth, &DepthSize);
    if (Status == OST_SUCCESS && DepthSize < 2u * Pixels * sizeof(float))
    {
        GEOREF_APP_Release(&D->DepthSet);
        Status = OST_ERR_FORMAT;
    }
    if (Status != OST_SUCCESS)
    {
        snprintf(Text, sizeof(Text), "depth set not readable: %s", OST_StatusName(Status));
        GEOREF_APP_Drop(D, Text);
        return;
    }
    memset(&Img, 0, sizeof(Img));
    if (D->Keyframe.WorkImage.ObjectId != 0 &&
        OST_DS_Acquire(&D->Keyframe.WorkImage, &Image, &ImageSize) == OST_SUCCESS)
    {
        if (ImageSize == Pixels || ImageSize == 3u * Pixels)
        {
            Img.data     = (const uint8_t *)Image;
            Img.channels = ImageSize == Pixels ? 1 : 3;
            Img.stride   = (size_t)D->Intrinsics.Width * (size_t)Img.channels;
        }
    }

    memset(&Map, 0, sizeof(Map));
    Map.camera.width  = D->Intrinsics.Width;
    Map.camera.height = D->Intrinsics.Height;
    Map.camera.fx     = D->Intrinsics.Fx;
    Map.camera.fy     = D->Intrinsics.Fy;
    Map.camera.cx     = D->Intrinsics.Cx;
    Map.camera.cy     = D->Intrinsics.Cy;
    Map.depth         = (const float *)Depth;
    Map.confidence    = Map.depth + Pixels;
    Pose.position_ecef[0]      = D->Keyframe.Pose.PositionEcef.X;
    Pose.position_ecef[1]      = D->Keyframe.Pose.PositionEcef.Y;
    Pose.position_ecef[2]      = D->Keyframe.Pose.PositionEcef.Z;
    Pose.q_ecef_from_camera[0] = D->Keyframe.Pose.AttitudeEcefFromCamera.W;
    Pose.q_ecef_from_camera[1] = D->Keyframe.Pose.AttitudeEcefFromCamera.X;
    Pose.q_ecef_from_camera[2] = D->Keyframe.Pose.AttitudeEcefFromCamera.Y;
    Pose.q_ecef_from_camera[3] = D->Keyframe.Pose.AttitudeEcefFromCamera.Z;

    /* Point set sized by the points the depth map gives. */
    T0     = OST_TimeMonoNs();
    Status = OST_Map_GeorefCount(W.Georef, &Map, &Count);
    if (Status == OST_SUCCESS && Count > 0)
    {
        Status = GEOREF_APP_AllocatePointSet((uint64)Count * sizeof(OST_Map_Point_t), &PointSet, &Points);
        if (Status != OST_SUCCESS)
        {
            CFE_EVS_SendEvent(GEOREF_APP_QUOTA_EID, CFE_EVS_EventType_ERROR, "no point set within %lu ms: %s",
                              (unsigned long)GEOREF_APP_Data.Config.QuotaWaitMs, OST_StatusName(Status));
            GEOREF_APP_FaultReport(OST_FaultId_DATASTORE_QUOTA, "point set quota exhausted");
        }
    }
    P->AllocateMs = GEOREF_APP_Ms(OST_TimeMonoNs() - T0);
    memset(&R, 0, sizeof(R));
    if (Status == OST_SUCCESS && Count > 0)
    {
        Status = OST_Map_GeorefConvert(W.Georef, &Map, Img.data != NULL ? &Img : NULL, &Pose,
                                       (OST_Map_Point_t *)Points, Count, W.Tiles, GEOREF_APP_MAX_TILES, &R);
        if (Status != OST_SUCCESS)
        {
            OST_DS_Abort(&PointSet);
        }
    }
    snprintf(Text, sizeof(Text), "%s (%s)", OST_StatusName(Status),
             Status != OST_SUCCESS ? OST_Map_GeorefLastError(W.Georef) : "");

    /* The depth set and the working image are done: read counts and the
     * counts of DEPTH_READY. */
    GEOREF_APP_Release(&D->DepthSet);
    GEOREF_APP_Release(&D->DepthSet);
    if (Image != NULL)
    {
        GEOREF_APP_Release(&D->Keyframe.WorkImage);
    }
    GEOREF_APP_Release(&D->Keyframe.WorkImage);
    if (Status != OST_SUCCESS)
    {
        GEOREF_APP_Release(&D->Keyframe.Jpeg);
        OS_MutSemTake(GEOREF_APP_Data.CountMutex);
        ++C->KeyframesFailed;
        OS_MutSemGive(GEOREF_APP_Data.CountMutex);
        CFE_EVS_SendEvent(GEOREF_APP_FAILED_EID, CFE_EVS_EventType_ERROR, "keyframe %lu (image %lu) dropped: %s",
                          (unsigned long)D->KeyframeId, (unsigned long)D->Keyframe.ImageSeq, Text);
        GEOREF_APP_FaultReport(OST_FaultId_POINTS_FAILED, "keyframe dropped by GEOREF_APP");
        return;
    }
    if (Count > 0)
    {
        OST_DS_Seal(&PointSet);
        if (Laz && OST_DS_Acquire(&PointSet, &Held, &HeldSize) != OST_SUCCESS)
        {
            Held = NULL; /* no point cloud for this keyframe */
        }
    }

    CFE_MSG_Init(CFE_MSG_PTR(W.OutMsg.TelemetryHeader),
                 CFE_SB_ValueToMsgId(
                     CFE_PLATFORM_TLM_TOPICID_TO_MIDV(EdsParam_CFE_MISSION_GEOREF_APP_POINTS_READY_TLM_TOPICID)),
                 sizeof(W.OutMsg));
    P->Header.SchemaVersion = 1;
    P->Header.Sequence      = ++W.Sequence;
    P->Trace.EntryMono      = It->EntryMono;
    P->KeyframeId           = D->KeyframeId;
    P->PoseSource           = D->PoseSource;
    P->TileCount            = (uint8)R.tile_count;
    P->Keyframe             = D->Keyframe;
    P->Keyframe.WorkImage.ObjectId   = 0; /* released above */
    P->Keyframe.WorkImage.Generation = 0;
    P->PointSet             = PointSet;
    P->OriginEasting        = R.origin[0];
    P->OriginNorthing       = R.origin[1];
    P->OriginHeight         = R.origin[2];
    P->PointCount           = R.points;
    P->TileCells            = GEOREF_APP_Data.Config.TileCells;
    P->CellSizeM            = GEOREF_APP_Data.Config.CellSizeM;
    P->HeightMinM           = (float)R.height_min;
    P->HeightMaxM           = (float)R.height_max;
    P->WaitMs               = C->LastWaitMs;
    P->ConvertMs            = (float)(R.convert_s * 1e3);
    memcpy(P->ProjectedCrs, GEOREF_APP_Data.Config.ProjectedCrs, sizeof(P->ProjectedCrs));
    memset(P->Tiles, 0, sizeof(P->Tiles));
    for (i = 0; i < R.tile_count && i < GEOREF_APP_MAX_TILES; ++i)
    {
        P->Tiles[i].Tile.Column = W.Tiles[i].column;
        P->Tiles[i].Tile.Row    = W.Tiles[i].row;
        P->Tiles[i].FirstPoint  = W.Tiles[i].first;
        P->Tiles[i].PointCount  = W.Tiles[i].count;
    }
    P->Trace.ExitMono   = OST_TimeMonoNs();
    P->Trace.PublishUtc = OST_TimeUtcNs();
    CFE_SB_TimeStampMsg(CFE_MSG_PTR(W.OutMsg.TelemetryHeader));
    if (CFE_SB_TransmitMsg(CFE_MSG_PTR(W.OutMsg.TelemetryHeader), true) != CFE_SUCCESS)
    {
        if (Held != NULL)
        {
            GEOREF_APP_Release(&PointSet); /* the read count */
        }
        GEOREF_APP_Release(&PointSet);
        GEOREF_APP_Release(&D->Keyframe.Jpeg);
        OS_MutSemTake(GEOREF_APP_Data.CountMutex);
        ++C->KeyframesFailed;
        OS_MutSemGive(GEOREF_APP_Data.CountMutex);
        CFE_EVS_SendEvent(GEOREF_APP_FAILED_EID, CFE_EVS_EventType_ERROR, "keyframe %lu: POINTS_READY not sent",
                          (unsigned long)D->KeyframeId);
        GEOREF_APP_FaultReport(OST_FaultId_POINTS_FAILED, "POINTS_READY not sent");
        return;
    }

    ++C->PointSetsPublished;
    Latency            = (float)((double)(P->Trace.PublishUtc - D->Keyframe.CaptureUtc) * 1e-9);
    C->LastKeyframeId  = D->KeyframeId;
    C->LastPointCount  = R.points;
    C->LastTileCount   = R.tile_count;
    C->LastAllocateMs  = P->AllocateMs;
    C->LastConvertMs   = P->ConvertMs;
    C->LastStageMs     = GEOREF_APP_Ms(P->Trace.ExitMono - It->EntryMono);
    C->LastLatencyS    = Latency;
    if (R.tile_count > C->MaxTileCount)
    {
        C->MaxTileCount = R.tile_count;
    }
    if (C->LastConvertMs > C->MaxConvertMs)
    {
        C->MaxConvertMs = C->LastConvertMs;
    }
    if (C->LastStageMs > C->MaxStageMs)
    {
        C->MaxStageMs = C->LastStageMs;
    }
    if (Latency > C->MaxLatencyS)
    {
        C->MaxLatencyS = Latency;
    }

    /* Point cloud files after publication: they are not on the path of the
     * map update (ADR-001 R2). */
    if (Held != NULL)
    {
        GEOREF_APP_WriteLaz(P, (const OST_Map_Point_t *)Held);
        GEOREF_APP_Release(&PointSet);
    }
    CFE_EVS_SendEvent(GEOREF_APP_POINTS_DBG_EID, CFE_EVS_EventType_DEBUG,
                      "keyframe %lu: %lu points in %u tiles, heights %.1f-%.1f m, convert %.0f ms, LAZ %.0f ms",
                      (unsigned long)D->KeyframeId, (unsigned long)R.points, (unsigned)R.tile_count,
                      P->HeightMinM, P->HeightMaxM, P->ConvertMs, C->LastLazMs);
}

/* ------------------------------------------------------------------ task */

void GEOREF_APP_WorkerTask(void)
{
    memset(&W, 0, sizeof(W));
    if (GEOREF_APP_Data.ConfigValid)
    {
        GEOREF_APP_CreateGeoref();
    }

    while (GEOREF_APP_Data.RunStatus == CFE_ES_RunStatus_APP_RUN)
    {
        if (GEOREF_APP_Dequeue(&W.Item))
        {
            GEOREF_APP_OnDepthSet(&W.Item);
        }
    }

    /* Depth sets still queued carry their counts. */
    while (GEOREF_APP_QueueDepth() > 0 && GEOREF_APP_Dequeue(&W.Item))
    {
        GEOREF_APP_Drop(&W.Item.Depth, "application stopped");
    }
    OST_Map_GeorefDestroy(W.Georef);
    W.Georef = NULL;
}
