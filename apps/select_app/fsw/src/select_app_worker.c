/*
 * SELECT_APP worker task (select_app.h): selection, decoding to the
 * working resolution, the image pool and the keyframe batches.
 *
 * Working images in the data store: rows packed, 8-bit, interleaved RGB
 * (WorkChannels 3) or grey (1), the size of KeyframeBatch.Intrinsics.
 */
#include "select_app.h"
#include "select_app_eventids.h"

#include "cfe_mission_eds_designparameters.h"
#include "imaging_c_api.h"
#include "ost_datastore.h"
#include "selection_c_api.h"

#include <stdio.h>
#include <string.h>

/* An image of the pool. */
typedef struct
{
    bool            InUse;
    uint64          Id; /* session << 32 | image sequence number */
    uint32          ImageSeq;
    int64           CaptureUtc;
    OST_ObjectRef_t Work; /* the pool's count */
    OST_Pose_t      Pose;
} SELECT_APP_PoolEntry_t;

typedef struct
{
    OST_Select_t                   *Select;
    OST_Imaging_WorkingConverter_t *Converter;
    OST_Geo_Camera_t                WorkCamera;
    uint32                          SessionId;
    bool                            Calibrated;
    uint32                          KeyframeId;

    char DemPath[OS_MAX_LOCAL_PATH_LEN];
    char ProjDataDir[OS_MAX_LOCAL_PATH_LEN];
    char GridDir[OS_MAX_LOCAL_PATH_LEN];
    char DemCrs[32 + 1];

    SELECT_APP_PoolEntry_t Pool[SELECT_APP_MAX_POOL];
    OST_KeyframeBatchTlm_t Batch;
    SELECT_APP_Item_t      Item;
} SELECT_APP_Worker_t;

static SELECT_APP_Worker_t W;

static uint32 SELECT_APP_Ms(int64 Ns)
{
    return (Ns <= 0) ? 0 : (uint32)(Ns / 1000000);
}

static void SELECT_APP_LibError(const char *Call, OST_Status_t Status, const char *Text)
{
    CFE_EVS_SendEvent(SELECT_APP_LIB_ERR_EID, CFE_EVS_EventType_ERROR, "%s: %s (%s)", Call, OST_StatusName(Status),
                      Text != NULL ? Text : "");
    SELECT_APP_FaultReport(OST_FaultId_LIBRARY_ERROR, Call);
}

/* /cf/... paths are translated to the host; other paths are used as they are. */
static void SELECT_APP_HostPath(const char *Virtual, size_t VirtualLen, char *Out, size_t OutLen)
{
    char Tmp[OS_MAX_PATH_LEN];

    snprintf(Tmp, sizeof(Tmp), "%.*s", (int)VirtualLen, Virtual);
    if (Tmp[0] == '\0' || OS_TranslatePath(Tmp, Out) != OS_SUCCESS)
    {
        snprintf(Out, OutLen, "%s", Tmp);
    }
}

/* ------------------------------------------------------------------ pool */

static SELECT_APP_PoolEntry_t *SELECT_APP_FindEntry(uint64 Id)
{
    uint32 i;
    for (i = 0; i < SELECT_APP_MAX_POOL; ++i)
    {
        if (W.Pool[i].InUse && W.Pool[i].Id == Id)
        {
            return &W.Pool[i];
        }
    }
    return NULL;
}

static SELECT_APP_PoolEntry_t *SELECT_APP_FreeEntry(void)
{
    uint32 i;
    for (i = 0; i < SELECT_APP_MAX_POOL; ++i)
    {
        if (!W.Pool[i].InUse)
        {
            return &W.Pool[i];
        }
    }
    return NULL;
}

static void SELECT_APP_DropEntry(uint64 Id)
{
    SELECT_APP_PoolEntry_t *E = SELECT_APP_FindEntry(Id);
    if (E != NULL)
    {
        SELECT_APP_Release(&E->Work);
        memset(E, 0, sizeof(*E));
    }
}

static void SELECT_APP_ClearPool(void)
{
    uint32 i;
    for (i = 0; i < SELECT_APP_MAX_POOL; ++i)
    {
        if (W.Pool[i].InUse)
        {
            SELECT_APP_Release(&W.Pool[i].Work);
        }
    }
    memset(W.Pool, 0, sizeof(W.Pool));
}

static void SELECT_APP_EvictPool(void)
{
    uint64       Ids[16];
    uint32       Count = 0;
    uint32       i;
    OST_Status_t Status;

    do
    {
        Status = OST_Select_Evict(W.Select, Ids, 16, &Count);
        if (Status != OST_SUCCESS)
        {
            SELECT_APP_LibError("OST_Select_Evict", Status, OST_Select_LastError(W.Select));
            return;
        }
        for (i = 0; i < Count; ++i)
        {
            SELECT_APP_DropEntry(Ids[i]);
            ++SELECT_APP_Data.Counters.PoolEvicted;
        }
    } while (Count == 16);
}

static void SELECT_APP_UpdateStats(void)
{
    OST_Select_Stats_t S;
    if (W.Select != NULL && OST_Select_GetStats(W.Select, &S) == OST_SUCCESS)
    {
        SELECT_APP_Data.Counters.PoolImages    = S.pool;
        SELECT_APP_Data.Counters.FlightLines   = S.flight_lines;
        SELECT_APP_Data.Counters.TerrainLoads  = S.terrain_loads;
        SELECT_APP_Data.Counters.TerrainLoaded = S.terrain_loaded != 0;
    }
}

/* ----------------------------------------------------------- calibration */

static void SELECT_APP_OnCalibration(const INGEST_APP_SessionCalibration_Payload_t *P)
{
    const SELECT_APP_ConfigTable_t *T = &SELECT_APP_Data.Config;
    OST_Geo_Camera_t                Cam;
    OST_Select_Config_t             C;
    OST_Select_Terrain_t            Terrain;
    OST_Status_t                    Status;

    /* A new session or a new calibration: the pool belongs to the old one. */
    SELECT_APP_ClearPool();
    W.SessionId                        = P->SessionId;
    W.Calibrated                       = false;
    SELECT_APP_Data.Counters.SessionId  = P->SessionId;
    SELECT_APP_Data.Counters.Calibrated = false;

    memset(&Cam, 0, sizeof(Cam));
    Cam.width  = (int32_t)P->Width;
    Cam.height = (int32_t)P->Height;
    Cam.fx     = P->Fx;
    Cam.fy     = P->Fy;
    Cam.cx     = P->Cx;
    Cam.cy     = P->Cy;
    Cam.k1     = P->K1;
    Cam.k2     = P->K2;
    Cam.k3     = P->K3;
    Cam.p1     = P->P1;
    Cam.p2     = P->P2;

    OST_Imaging_WorkingConverterDestroy(W.Converter);
    W.Converter = NULL;
    Status = OST_Imaging_WorkingConverterCreate(&W.Converter, &Cam, (int32_t)T->WorkWidth, T->WorkChannels);
    if (Status == OST_SUCCESS)
    {
        Status = OST_Imaging_WorkingConverterCamera(W.Converter, &W.WorkCamera);
    }
    if (Status != OST_SUCCESS)
    {
        CFE_EVS_SendEvent(SELECT_APP_CALIBRATION_ERR_EID, CFE_EVS_EventType_ERROR,
                          "session %lu: working image converter not created: %s", (unsigned long)P->SessionId,
                          OST_StatusName(Status));
        SELECT_APP_FaultReport(OST_FaultId_LIBRARY_ERROR, "working image converter");
        OST_Imaging_WorkingConverterDestroy(W.Converter);
        W.Converter = NULL;
        return;
    }

    if (W.Select != NULL)
    {
        Status = OST_Select_Reset(W.Select, &Cam);
        if (Status != OST_SUCCESS)
        {
            SELECT_APP_LibError("OST_Select_Reset", Status, OST_Select_LastError(W.Select));
            return;
        }
    }
    else
    {
        OST_Select_DefaultConfig(&C);
        C.nominal_height_agl_m              = T->NominalHeightAglM;
        C.footprint_iterations              = (int32_t)T->FootprintIterations;
        C.max_sources                       = (int32_t)T->MaxSources;
        C.max_tilt_deg                      = T->MaxTiltDeg;
        C.target_overlap                    = T->TargetOverlap;
        C.min_overlap                       = T->MinOverlap;
        C.min_baseline_ratio                = T->MinBaselineRatio;
        C.max_baseline_ratio                = T->MaxBaselineRatio;
        C.max_view_angle_deg                = T->MaxViewAngleDeg;
        C.preferred_triangulation_angle_deg = T->PreferredTriangulationAngleDeg;
        C.line_turn_deg                     = T->LineTurnDeg;
        C.pool_distance_m                   = T->PoolDistanceM;
        C.max_pool_images                   = T->MaxPoolImages;
        C.terrain_radius_m                  = T->TerrainRadiusM;
        Terrain.dem_path                    = W.DemPath;
        Terrain.dem_crs                     = W.DemCrs;
        Terrain.proj_data_dir               = W.ProjDataDir;
        Terrain.grid_dir                    = W.GridDir;
        Status                              = OST_Select_Create(&C, &Terrain, &Cam, &W.Select);
        if (Status == OST_ERR_IO)
        {
            CFE_EVS_SendEvent(SELECT_APP_TERRAIN_ERR_EID, CFE_EVS_EventType_ERROR,
                              "terrain prior %s (%s) or PROJ data %s not usable", W.DemPath, W.DemCrs, W.ProjDataDir);
            SELECT_APP_FaultRaise(OST_FaultId_TERRAIN_UNAVAILABLE, "elevation model or PROJ data not usable");
            W.Select = NULL;
            return;
        }
        if (Status != OST_SUCCESS)
        {
            SELECT_APP_LibError("OST_Select_Create", Status, "");
            W.Select = NULL;
            return;
        }
    }
    W.Calibrated                        = true;
    SELECT_APP_Data.Counters.Calibrated = true;
    CFE_EVS_SendEvent(SELECT_APP_SESSION_INF_EID, CFE_EVS_EventType_INFORMATION,
                      "session %lu: camera %ldx%ld, working images %ldx%ld, f %.1f px", (unsigned long)P->SessionId,
                      (long)Cam.width, (long)Cam.height, (long)W.WorkCamera.width, (long)W.WorkCamera.height,
                      W.WorkCamera.fx);
}

/* ----------------------------------------------------------------- batch */

static void SELECT_APP_FillView(OST_ViewRef_t *V, uint32 ImageSeq, int64 CaptureUtc, const OST_ObjectRef_t *Work,
                                const OST_ObjectRef_t *Jpeg, const OST_Pose_t *Pose)
{
    memset(V, 0, sizeof(*V));
    V->CaptureUtc = CaptureUtc;
    V->ImageSeq   = ImageSeq;
    V->WorkImage  = *Work;
    if (Jpeg != NULL)
    {
        V->Jpeg = *Jpeg;
    }
    V->Pose = *Pose;
}

static void SELECT_APP_ReleaseBatch(const OST_KeyframeBatch_Payload_t *P)
{
    uint32 i;
    SELECT_APP_Release(&P->Keyframe.WorkImage);
    SELECT_APP_Release(&P->Keyframe.Jpeg);
    for (i = 0; i < P->SourceCount; ++i)
    {
        SELECT_APP_Release(&P->Sources[i].WorkImage);
    }
}

/* ----------------------------------------------------------------- image */

/* Allocates the working image, evicting the oldest pool images while the
 * quota is exhausted. */
static OST_Status_t SELECT_APP_AllocateWork(uint64 Id, uint64 Size, OST_ObjectRef_t *Ref, void **Data)
{
    OST_Status_t Status = OST_ERR_QUOTA;
    uint32       Tries;
    uint64       Oldest = 0;
    int          Found  = 0;

    for (Tries = 0; Tries <= SELECT_APP_MAX_QUOTA_EVICTIONS; ++Tries)
    {
        Status = OST_DS_Allocate(OST_DATASTORE_WORK_IMAGE, Size, Ref, Data);
        if (Status != OST_ERR_QUOTA)
        {
            return Status;
        }
        if (OST_Select_EvictOldest(W.Select, &Oldest, &Found) != OST_SUCCESS || !Found)
        {
            return Status;
        }
        if (Oldest == Id)
        {
            return Status; /* only this image is left */
        }
        SELECT_APP_DropEntry(Oldest);
        ++SELECT_APP_Data.Counters.QuotaEvictions;
    }
    return Status;
}

static void SELECT_APP_OnImage(const ALIGN_APP_ImagePose_Payload_t *P, int64 EntryMono)
{
    SELECT_APP_Counters_t    *Cnt = &SELECT_APP_Data.Counters;
    OST_KeyframeBatch_Payload_t *B = &W.Batch.Payload;
    OST_Select_Decision_t     D;
    const uint64              Id = ((uint64)P->SessionId << 32) | P->ImageSeq;
    const double              Pos[3] = {P->Pose.PositionEcef.X, P->Pose.PositionEcef.Y, P->Pose.PositionEcef.Z};
    const double              Q[4]   = {P->Pose.AttitudeEcefFromCamera.W, P->Pose.AttitudeEcefFromCamera.X,
                                        P->Pose.AttitudeEcefFromCamera.Y, P->Pose.AttitudeEcefFromCamera.Z};
    OST_ObjectRef_t           Work;
    OST_Imaging_Image_t       Out;
    void                     *WorkData = NULL;
    const void               *Jpeg     = NULL;
    uint64                    JpegSize = 0;
    uint64                    WorkSize;
    OST_Status_t              Status;
    SELECT_APP_PoolEntry_t   *E;
    uint32                    i;
    int                       Found;
    const void               *Unused   = NULL;
    uint64                    UnusedSize = 0;
    int64                     T0;
    int64                     T1;
    uint32                    Ms;
    uint32                    Seq;
    char                      Text[64];

    if (P->SessionId != W.SessionId || !W.Calibrated)
    {
        ++Cnt->ImagesNoCalibration;
        SELECT_APP_Release(&P->Jpeg);
        return;
    }

    /* Selection. */
    T0     = OST_TimeMonoNs();
    Status = OST_Select_AddImage(W.Select, Id, Pos, Q, &D);
    Ms     = SELECT_APP_Ms(OST_TimeMonoNs() - T0);
    Cnt->LastSelectMs = Ms;
    if (Ms > Cnt->MaxSelectMs)
    {
        Cnt->MaxSelectMs = Ms;
    }
    if (Status != OST_SUCCESS)
    {
        SELECT_APP_LibError("OST_Select_AddImage", Status, OST_Select_LastError(W.Select));
        SELECT_APP_Release(&P->Jpeg);
        return;
    }
    if (D.terrain_available)
    {
        SELECT_APP_FaultResolve(OST_FaultId_TERRAIN_UNAVAILABLE);
    }
    else
    {
        ++Cnt->ImagesNoTerrain;
        CFE_EVS_SendEvent(SELECT_APP_TERRAIN_ERR_EID, CFE_EVS_EventType_ERROR, "image %lu: no terrain prior: %s",
                          (unsigned long)P->ImageSeq, OST_Select_LastError(W.Select));
        SELECT_APP_FaultRaise(OST_FaultId_TERRAIN_UNAVAILABLE, "no terrain at the image position");
    }
    if (!D.footprint)
    {
        ++Cnt->ImagesNoFootprint;
        SELECT_APP_Release(&P->Jpeg);
        SELECT_APP_EvictPool();
        SELECT_APP_UpdateStats();
        return;
    }

    /* A keyframe's sources get their batch counts before anything can
     * evict them. CFE_MSG_Init clears the whole message. */
    CFE_MSG_Init(CFE_MSG_PTR(W.Batch.TelemetryHeader),
                 SELECT_APP_Data.Config.RefinePose ? OST_BatchSelectedMsgId() : OST_BatchReadyMsgId(),
                 sizeof(W.Batch));
    if (D.keyframe)
    {
        for (i = 0; i < D.source_count && B->SourceCount < EdsParam_OST_MAX_SOURCE_VIEWS; ++i)
        {
            E = SELECT_APP_FindEntry(D.sources[i].id);
            if (E == NULL || OST_DS_Acquire(&E->Work, &Unused, &UnusedSize) != OST_SUCCESS)
            {
                continue;
            }
            SELECT_APP_FillView(&B->Sources[B->SourceCount], E->ImageSeq, E->CaptureUtc, &E->Work, NULL, &E->Pose);
            ++B->SourceCount;
        }
    }

    /* Working image. */
    T0       = OST_TimeMonoNs();
    WorkSize = (uint64)W.WorkCamera.width * (uint64)W.WorkCamera.height * SELECT_APP_Data.Config.WorkChannels;
    Status   = SELECT_APP_AllocateWork(Id, WorkSize, &Work, &WorkData);
    if (Status != OST_SUCCESS)
    {
        ++Cnt->QuotaFailures;
        CFE_EVS_SendEvent(SELECT_APP_QUOTA_ERR_EID, CFE_EVS_EventType_ERROR, "image %lu: working image not stored: %s",
                          (unsigned long)P->ImageSeq, OST_StatusName(Status));
        SELECT_APP_FaultReport(OST_FaultId_DATASTORE_QUOTA, "working image not stored");
        OST_Select_Remove(W.Select, Id, &Found);
        SELECT_APP_Release(&P->Jpeg);
        SELECT_APP_ReleaseBatch(B);
        return;
    }
    Out.data     = (uint8_t *)WorkData;
    Out.width    = W.WorkCamera.width;
    Out.height   = W.WorkCamera.height;
    Out.channels = SELECT_APP_Data.Config.WorkChannels;
    Out.stride   = (size_t)Out.width * (size_t)Out.channels;
    Status       = OST_DS_Acquire(&P->Jpeg, &Jpeg, &JpegSize);
    if (Status == OST_SUCCESS)
    {
        Status = OST_Imaging_ConvertJpeg(W.Converter, (const uint8_t *)Jpeg, (size_t)JpegSize, &Out);
        SELECT_APP_Release(&P->Jpeg); /* the count taken for reading */
    }
    if (Status == OST_SUCCESS)
    {
        Status = OST_DS_Seal(&Work);
    }
    if (Status != OST_SUCCESS)
    {
        ++Cnt->DecodeErrors;
        CFE_EVS_SendEvent(SELECT_APP_DECODE_ERR_EID, CFE_EVS_EventType_ERROR, "image %lu not decoded: %s (%s)",
                          (unsigned long)P->ImageSeq, OST_StatusName(Status),
                          OST_Imaging_WorkingConverterLastError(W.Converter));
        snprintf(Text, sizeof(Text), "image %lu", (unsigned long)P->ImageSeq);
        SELECT_APP_FaultReport(OST_FaultId_IMAGE_DECODE, Text);
        OST_DS_Abort(&Work);
        OST_Select_Remove(W.Select, Id, &Found);
        SELECT_APP_Release(&P->Jpeg);
        SELECT_APP_ReleaseBatch(B);
        return;
    }
    T1 = OST_TimeMonoNs();
    ++Cnt->ImagesDecoded;
    Cnt->LastDecodeMs = SELECT_APP_Ms(T1 - T0);
    if (Cnt->LastDecodeMs > Cnt->MaxDecodeMs)
    {
        Cnt->MaxDecodeMs = Cnt->LastDecodeMs;
    }

    /* Into the pool. MaxPoolImages < SELECT_APP_MAX_POOL, so a slot is free. */
    E = SELECT_APP_FreeEntry();
    if (E == NULL)
    {
        OST_Select_Remove(W.Select, Id, &Found);
        SELECT_APP_Release(&Work);
        SELECT_APP_Release(&P->Jpeg);
        SELECT_APP_ReleaseBatch(B);
        return;
    }
    E->InUse      = true;
    E->Id         = Id;
    E->ImageSeq   = P->ImageSeq;
    E->CaptureUtc = P->CaptureUtc;
    E->Work       = Work;
    E->Pose       = P->Pose;

    if (!D.keyframe)
    {
        SELECT_APP_Release(&P->Jpeg);
    }
    else if (OST_DS_Acquire(&Work, &Unused, &UnusedSize) != OST_SUCCESS)
    {
        SELECT_APP_Release(&P->Jpeg);
        SELECT_APP_ReleaseBatch(B);
    }
    else
    {
        /* The batch: its counts go to the consumer (ADR-013 section 3). */
        Seq = W.KeyframeId++;
        B->Header.SchemaVersion   = OST_KEYFRAME_BATCH_SCHEMA_VERSION;
        B->Header.Sequence        = Seq + 1;
        B->Trace.EntryMono        = EntryMono;
        B->KeyframeId             = Seq + 1;
        B->PoseSource             = OST_PoseSource_TELEMETRY;
        B->FootprintPlaneHeight   = D.plane_height_m;
        B->Intrinsics.Fx          = W.WorkCamera.fx;
        B->Intrinsics.Fy          = W.WorkCamera.fy;
        B->Intrinsics.Cx          = W.WorkCamera.cx;
        B->Intrinsics.Cy          = W.WorkCamera.cy;
        B->Intrinsics.Width       = (uint16)W.WorkCamera.width;
        B->Intrinsics.Height      = (uint16)W.WorkCamera.height;
        SELECT_APP_FillView(&B->Keyframe, P->ImageSeq, P->CaptureUtc, &Work, &P->Jpeg, &P->Pose);
        B->Trace.ExitMono   = OST_TimeMonoNs();
        B->Trace.PublishUtc = OST_TimeUtcNs();
        CFE_SB_TimeStampMsg(CFE_MSG_PTR(W.Batch.TelemetryHeader));
        if (CFE_SB_TransmitMsg(CFE_MSG_PTR(W.Batch.TelemetryHeader), true) != CFE_SUCCESS)
        {
            SELECT_APP_ReleaseBatch(B);
        }
        else
        {
            ++Cnt->BatchesPublished;
            Cnt->SourcesPublished += B->SourceCount;
            for (i = 0; i < D.source_count; ++i)
            {
                if (D.sources[i].other_line)
                {
                    ++Cnt->BatchesWithOtherLine;
                    break;
                }
            }
        }
        ++Cnt->Keyframes;
        Cnt->LastOverlap = (float)D.overlap_last;
        CFE_EVS_SendEvent(SELECT_APP_KEYFRAME_DBG_EID, CFE_EVS_EventType_DEBUG,
                          "keyframe %lu: image %lu, line %ld, overlap %.2f, %lu sources, plane %.1f m",
                          (unsigned long)(Seq + 1), (unsigned long)P->ImageSeq, (long)D.flight_line, D.overlap_last,
                          (unsigned long)B->SourceCount, D.plane_height_m);
    }

    SELECT_APP_EvictPool();
    SELECT_APP_UpdateStats();
}

/* ------------------------------------------------------------------ task */

void SELECT_APP_WorkerTask(void)
{
    const SELECT_APP_ConfigTable_t *T = &SELECT_APP_Data.Config;

    memset(&W, 0, sizeof(W));
    SELECT_APP_HostPath(T->DemPath, sizeof(T->DemPath), W.DemPath, sizeof(W.DemPath));
    SELECT_APP_HostPath(T->ProjDataDir, sizeof(T->ProjDataDir), W.ProjDataDir, sizeof(W.ProjDataDir));
    SELECT_APP_HostPath(T->GridDir, sizeof(T->GridDir), W.GridDir, sizeof(W.GridDir));
    snprintf(W.DemCrs, sizeof(W.DemCrs), "%.*s", (int)sizeof(T->DemCrs), T->DemCrs);

    while (SELECT_APP_Data.RunStatus == CFE_ES_RunStatus_APP_RUN)
    {
        if (!SELECT_APP_Dequeue(&W.Item, SELECT_APP_WORKER_TIMEOUT_MS))
        {
            continue;
        }
        if (W.Item.Type == SELECT_APP_ITEM_CALIBRATION)
        {
            SELECT_APP_OnCalibration(&W.Item.u.Calibration);
        }
        else if (W.Item.Type == SELECT_APP_ITEM_IMAGE)
        {
            SELECT_APP_OnImage(&W.Item.u.Image, W.Item.EntryMono);
        }
    }

    SELECT_APP_ClearPool();
    OST_Select_Destroy(W.Select);
    OST_Imaging_WorkingConverterDestroy(W.Converter);
    memset(&W, 0, sizeof(W));
}
