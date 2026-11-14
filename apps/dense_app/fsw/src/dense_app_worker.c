/*
 * DENSE_APP worker task (dense_app.h): one depth job at a time with the
 * dense library (OST_Dense_*), and the publication of DEPTH_READY.
 */
#include "dense_app.h"
#include "dense_app_eventids.h"

#include "cfe_core_api_base_msgids.h"
#include "cfe_mission_eds_designparameters.h"
#include "dense_c_api.h"
#include "ost_datastore.h"

#include <stdio.h>
#include <string.h>

#define DENSE_APP_VIEWS (1 + EdsParam_OST_MAX_SOURCE_VIEWS)

typedef struct
{
    OST_Dense_t           *Dense;
    bool                   CreateReported;
    char                   DemPath[OS_MAX_LOCAL_PATH_LEN];
    char                   ProjDataDir[OS_MAX_LOCAL_PATH_LEN];
    char                   GridDir[OS_MAX_LOCAL_PATH_LEN];
    char                   DemCrs[32 + 1];
    DENSE_APP_Item_t       Item;
    DENSE_APP_DepthReadyTlm_t OutMsg;
    uint32                 Sequence;
    OST_Dense_View_t       Views[DENSE_APP_VIEWS];
    uint8                  Grey[DENSE_APP_VIEWS][DENSE_APP_MAX_PIXELS];
} DENSE_APP_Worker_t;

static DENSE_APP_Worker_t W;

static float DENSE_APP_Ms(int64 Ns)
{
    return (float)((double)Ns * 1e-6);
}

/* /cf/... table paths to host paths. */
static void DENSE_APP_HostPath(const char *Virtual, size_t VirtualLen, char *Out, size_t OutLen)
{
    char Tmp[OS_MAX_PATH_LEN];

    snprintf(Tmp, sizeof(Tmp), "%.*s", (int)VirtualLen, Virtual);
    if (Tmp[0] == '\0' || OS_TranslatePath(Tmp, Out) != OS_SUCCESS)
    {
        snprintf(Out, OutLen, "%s", Tmp);
    }
}

/* -------------------------------------------------------------- estimator */

static void DENSE_APP_CreateEstimator(void)
{
    const DENSE_APP_ConfigTable_t *T = &DENSE_APP_Data.Config;
    OST_Dense_Config_t             C;
    OST_Dense_Terrain_t            Terrain;
    OST_Status_t                   Status;

    OST_Dense_DefaultConfig(&C);
    C.method                     = T->Method == DENSE_APP_Method_PATCHMATCH ? OST_DENSE_METHOD_PATCHMATCH
                                                                            : OST_DENSE_METHOD_SGBM;
    C.threads                    = (int32_t)T->Threads;
    C.sgbm_target_baseline_ratio = T->SgbmTargetBaselineRatio;
    C.sgbm_block_size            = (int32_t)T->SgbmBlockSize;
    C.sgbm_p1_factor             = (int32_t)T->SgbmP1Factor;
    C.sgbm_p2_factor             = (int32_t)T->SgbmP2Factor;
    C.sgbm_uniqueness_ratio      = (int32_t)T->SgbmUniquenessRatio;
    C.sgbm_disp12_max_diff       = (int32_t)T->SgbmDisp12MaxDiff;
    C.sgbm_speckle_window        = (int32_t)T->SgbmSpeckleWindow;
    C.sgbm_speckle_range         = (int32_t)T->SgbmSpeckleRange;
    C.sgbm_max_disparities       = (int32_t)T->SgbmMaxDisparities;
    C.pm_gpu_index               = (int32_t)T->PmGpuIndex;
    C.pm_iterations              = (int32_t)T->PmIterations;
    C.pm_window_radius           = (int32_t)T->PmWindowRadius;
    C.pm_window_step             = (int32_t)T->PmWindowStep;
    C.pm_samples                 = (int32_t)T->PmSamples;
    C.pm_filter_min_consistent   = (int32_t)T->PmFilterMinConsistent;
    C.pm_filter_min_ncc          = T->PmFilterMinNcc;
    C.pm_filter_min_angle_deg    = T->PmFilterMinAngleDeg;
    C.pm_min_free_gpu_mb         = T->PmMinFreeGpuMb;
    C.depth_margin_m             = T->DepthMarginM;
    C.terrain_gate_m             = T->TerrainGateM;
    C.gate_spacing_m             = T->GateSpacingM;
    C.gate_margin_m              = T->GateMarginM;
    C.terrain_radius_m           = T->TerrainRadiusM;

    DENSE_APP_HostPath(T->DemPath, sizeof(T->DemPath), W.DemPath, sizeof(W.DemPath));
    DENSE_APP_HostPath(T->ProjDataDir, sizeof(T->ProjDataDir), W.ProjDataDir, sizeof(W.ProjDataDir));
    DENSE_APP_HostPath(T->GridDir, sizeof(T->GridDir), W.GridDir, sizeof(W.GridDir));
    snprintf(W.DemCrs, sizeof(W.DemCrs), "%.*s", (int)sizeof(T->DemCrs), T->DemCrs);
    Terrain.dem_path      = W.DemPath;
    Terrain.dem_crs       = W.DemCrs;
    Terrain.proj_data_dir = W.ProjDataDir;
    Terrain.grid_dir      = W.GridDir;

    Status = OST_Dense_Create(&C, &Terrain, &W.Dense);
    DENSE_APP_Data.Counters.Ready = Status == OST_SUCCESS;
    if (Status == OST_SUCCESS)
    {
        DENSE_APP_FaultResolve(OST_FaultId_TERRAIN_UNAVAILABLE);
        CFE_EVS_SendEvent(DENSE_APP_READY_INF_EID, CFE_EVS_EventType_INFORMATION,
                          "estimator ready: %s, %lu OpenCV threads, terrain %s", C.method ? "PatchMatch" : "SGBM",
                          (unsigned long)T->Threads, W.DemPath);
        return;
    }
    W.Dense = NULL;
    if (W.CreateReported)
    {
        return; /* tried again on every batch; reported once */
    }
    W.CreateReported = true;
    if (Status == OST_ERR_IO)
    {
        CFE_EVS_SendEvent(DENSE_APP_TERRAIN_EID, CFE_EVS_EventType_ERROR,
                          "terrain prior %s (%s) or PROJ data %s not usable", W.DemPath, W.DemCrs, W.ProjDataDir);
        DENSE_APP_FaultRaise(OST_FaultId_TERRAIN_UNAVAILABLE, "elevation model or PROJ data not usable");
    }
    else if (Status == OST_ERR_UNSUPPORTED)
    {
        CFE_EVS_SendEvent(DENSE_APP_LIB_ERR_EID, CFE_EVS_EventType_ERROR,
                          "PatchMatch selected but not built (COLMAP)");
        DENSE_APP_FaultRaise(OST_FaultId_TABLE_REJECTED, "PatchMatch not available in this build");
    }
    else
    {
        CFE_EVS_SendEvent(DENSE_APP_LIB_ERR_EID, CFE_EVS_EventType_ERROR, "OST_Dense_Create: %s",
                          OST_StatusName(Status));
        DENSE_APP_FaultRaise(OST_FaultId_LIBRARY_ERROR, "OST_Dense_Create failed");
    }
}

/* ----------------------------------------------------------------- views */

/* Grey copy of a view's working image (read under the batch's count, +1
 * for reading). */
static OST_Status_t DENSE_APP_ReadView(const OST_ViewRef_t *V, uint32 Index, uint32 Width, uint64 Pixels)
{
    const void  *Data = NULL;
    uint64       Size = 0;
    const uint8 *Rgb;
    uint8       *Grey = W.Grey[Index];
    uint64       p;
    OST_Status_t Status;

    Status = OST_DS_Acquire(&V->WorkImage, &Data, &Size);
    if (Status != OST_SUCCESS)
    {
        return Status;
    }
    if (Size == Pixels)
    {
        memcpy(Grey, Data, (size_t)Pixels);
    }
    else if (Size == 3u * Pixels)
    {
        Rgb = (const uint8 *)Data;
        for (p = 0; p < Pixels; ++p)
        {
            Grey[p] = (uint8)((77u * Rgb[3 * p] + 150u * Rgb[3 * p + 1] + 29u * Rgb[3 * p + 2]) >> 8);
        }
    }
    else
    {
        Status = OST_ERR_FORMAT;
    }
    DENSE_APP_Release(&V->WorkImage);

    memset(&W.Views[Index], 0, sizeof(W.Views[Index]));
    W.Views[Index].grey                  = Grey;
    W.Views[Index].stride                = Width;
    W.Views[Index].position_ecef[0]      = V->Pose.PositionEcef.X;
    W.Views[Index].position_ecef[1]      = V->Pose.PositionEcef.Y;
    W.Views[Index].position_ecef[2]      = V->Pose.PositionEcef.Z;
    W.Views[Index].q_ecef_from_camera[0] = V->Pose.AttitudeEcefFromCamera.W;
    W.Views[Index].q_ecef_from_camera[1] = V->Pose.AttitudeEcefFromCamera.X;
    W.Views[Index].q_ecef_from_camera[2] = V->Pose.AttitudeEcefFromCamera.Y;
    W.Views[Index].q_ecef_from_camera[3] = V->Pose.AttitudeEcefFromCamera.Z;
    return Status;
}

/* --------------------------------------------------------------- outcome */

static void DENSE_APP_Fail(const DENSE_APP_Item_t *It, DENSE_APP_Outcome_Enum_t Outcome,
                           DENSE_APP_Reason_Enum_t Reason, const char *Text)
{
    CFE_EVS_SendEvent(Outcome == DENSE_APP_Outcome_SKIPPED ? DENSE_APP_SKIPPED_EID : DENSE_APP_FAILED_EID,
                      CFE_EVS_EventType_ERROR, "keyframe %lu (image %lu) %s: %s",
                      (unsigned long)It->Batch.KeyframeId, (unsigned long)It->Batch.Keyframe.ImageSeq,
                      Outcome == DENSE_APP_Outcome_SKIPPED ? "skipped" : "failed", Text);
    DENSE_APP_ReleaseBatch(&It->Batch);
    DENSE_APP_PublishStatus(&It->Batch, It->EntryMono, Outcome, Reason);
}

/* Depth set within the quota, waiting up to QuotaWaitMs. */
static OST_Status_t DENSE_APP_AllocateDepthSet(uint64 Size, OST_ObjectRef_t *Ref, void **Data)
{
    const int64  Start  = OST_TimeMonoNs();
    bool         Waited = false;
    OST_Status_t Status;

    for (;;)
    {
        Status = OST_DS_Allocate(OST_DATASTORE_DEPTH_SET, Size, Ref, Data);
        if (Status != OST_ERR_QUOTA && Status != OST_ERR_NO_MEMORY)
        {
            return Status;
        }
        if (!Waited)
        {
            Waited = true;
            ++DENSE_APP_Data.Counters.QuotaWaits;
        }
        if (OST_TimeMonoNs() - Start >= (int64)DENSE_APP_Data.Config.QuotaWaitMs * 1000000 ||
            DENSE_APP_Data.RunStatus != CFE_ES_RunStatus_APP_RUN)
        {
            return Status;
        }
        OS_TaskDelay(DENSE_APP_QUOTA_RETRY_MS);
    }
}

/* ----------------------------------------------------------------- batch */

static void DENSE_APP_OnBatch(const DENSE_APP_Item_t *It)
{
    const OST_KeyframeBatch_Payload_t *B     = &It->Batch;
    DENSE_APP_Counters_t              *C     = &DENSE_APP_Data.Counters;
    const int64                        Start = OST_TimeMonoNs();
    const uint64                       Pixels = (uint64)B->Intrinsics.Width * B->Intrinsics.Height;
    const bool                         Normals = DENSE_APP_Data.Config.StoreNormals != 0;
    DENSE_APP_DepthReady_Payload_t    *P     = &W.OutMsg.Payload;
    OST_Dense_Camera_t                 Cam;
    OST_Dense_Output_t                 Out;
    OST_Dense_Result_t                 R;
    OST_ObjectRef_t                    DepthSet = {0, 0};
    void                              *Data     = NULL;
    uint32                             i;
    uint32                             Sources = B->SourceCount;
    int64                              T0, ReadNs;
    OST_Status_t                       Status;
    char                               Text[128];
    float                              Latency;

    C->LastWaitMs = DENSE_APP_Ms(Start - It->EntryMono);
    if (W.Dense == NULL && DENSE_APP_Data.ConfigValid)
    {
        DENSE_APP_CreateEstimator();
    }
    if (W.Dense == NULL)
    {
        DENSE_APP_Fail(It, DENSE_APP_Outcome_FAILED, DENSE_APP_Reason_NOT_READY, "no estimator");
        return;
    }
    if (Sources == 0)
    {
        /* The first keyframe of a flight: nothing to match it with. */
        CFE_EVS_SendEvent(DENSE_APP_SKIPPED_EID, CFE_EVS_EventType_INFORMATION,
                          "keyframe %lu (image %lu) skipped: no source views", (unsigned long)B->KeyframeId,
                          (unsigned long)B->Keyframe.ImageSeq);
        DENSE_APP_ReleaseBatch(B);
        DENSE_APP_PublishStatus(B, It->EntryMono, DENSE_APP_Outcome_SKIPPED, DENSE_APP_Reason_NO_SOURCES);
        return;
    }
    if (Sources > EdsParam_OST_MAX_SOURCE_VIEWS || Pixels == 0 || Pixels > DENSE_APP_MAX_PIXELS)
    {
        snprintf(Text, sizeof(Text), "%lu sources, %ux%u working images", (unsigned long)Sources,
                 (unsigned)B->Intrinsics.Width, (unsigned)B->Intrinsics.Height);
        DENSE_APP_Fail(It, DENSE_APP_Outcome_FAILED, DENSE_APP_Reason_INVALID_BATCH, Text);
        return;
    }

    /* Working images to grey; the sources' working images are released
     * here, their counts with the batch below. */
    T0     = OST_TimeMonoNs();
    Status = DENSE_APP_ReadView(&B->Keyframe, 0, B->Intrinsics.Width, Pixels);
    for (i = 0; Status == OST_SUCCESS && i < Sources; ++i)
    {
        Status = DENSE_APP_ReadView(&B->Sources[i], i + 1, B->Intrinsics.Width, Pixels);
    }
    ReadNs = OST_TimeMonoNs() - T0;
    if (Status != OST_SUCCESS)
    {
        snprintf(Text, sizeof(Text), "working image not readable: %s", OST_StatusName(Status));
        DENSE_APP_Fail(It, DENSE_APP_Outcome_FAILED, DENSE_APP_Reason_INVALID_BATCH, Text);
        return;
    }

    /* Depth set: depth, confidence, optionally normals (dense_app.xml). */
    Status = DENSE_APP_AllocateDepthSet(Pixels * sizeof(float) * (Normals ? 5u : 2u), &DepthSet, &Data);
    if (Status != OST_SUCCESS)
    {
        CFE_EVS_SendEvent(DENSE_APP_QUOTA_EID, CFE_EVS_EventType_ERROR,
                          "no depth set within %lu ms: %s", (unsigned long)DENSE_APP_Data.Config.QuotaWaitMs,
                          OST_StatusName(Status));
        DENSE_APP_FaultReport(OST_FaultId_DATASTORE_QUOTA, "depth set quota exhausted");
        DENSE_APP_Fail(It, DENSE_APP_Outcome_SKIPPED, DENSE_APP_Reason_QUOTA, "no depth set");
        return;
    }
    Out.depth      = (float *)Data;
    Out.confidence = Out.depth + Pixels;
    Out.normals    = Normals ? Out.confidence + Pixels : NULL;

    Cam.width  = B->Intrinsics.Width;
    Cam.height = B->Intrinsics.Height;
    Cam.fx     = B->Intrinsics.Fx;
    Cam.fy     = B->Intrinsics.Fy;
    Cam.cx     = B->Intrinsics.Cx;
    Cam.cy     = B->Intrinsics.Cy;
    Status     = OST_Dense_Estimate(W.Dense, &Cam, &W.Views[0], &W.Views[1], Sources, B->FootprintPlaneHeight, &Out, &R);
    if (Status != OST_SUCCESS)
    {
        OST_DS_Abort(&DepthSet);
        snprintf(Text, sizeof(Text), "%s (%s)", OST_StatusName(Status), OST_Dense_LastError(W.Dense));
        if (Status == OST_ERR_GPU)
        {
            CFE_EVS_SendEvent(DENSE_APP_GPU_EID, CFE_EVS_EventType_ERROR, "PatchMatch not started: %s",
                              OST_Dense_LastError(W.Dense));
            DENSE_APP_FaultRaise(OST_FaultId_GPU_FAILURE, "GPU not available or not enough GPU memory");
            DENSE_APP_Fail(It, DENSE_APP_Outcome_FAILED, DENSE_APP_Reason_GPU, Text);
        }
        else
        {
            DENSE_APP_FaultReport(OST_FaultId_DEPTH_FAILED, "depth estimation failed");
            DENSE_APP_Fail(It, DENSE_APP_Outcome_FAILED,
                           Status == OST_ERR_INVALID_ARGUMENT ? DENSE_APP_Reason_INVALID_BATCH
                                                              : DENSE_APP_Reason_ESTIMATION,
                           Text);
        }
        return;
    }
    if (DENSE_APP_Data.Config.Method == DENSE_APP_Method_PATCHMATCH)
    {
        DENSE_APP_FaultResolve(OST_FaultId_GPU_FAILURE);
    }
    OST_DS_Seal(&DepthSet);

    /* No terrain under the keyframe: depth range from the plane, no gate. */
    C->LastTerrain  = R.terrain_available != 0;
    C->TerrainLoads = R.terrain_loads;
    if (!R.terrain_available)
    {
        CFE_EVS_SendEvent(DENSE_APP_TERRAIN_EID, CFE_EVS_EventType_ERROR,
                          "keyframe %lu: no terrain prior; depth range from the footprint plane, no gate (%s)",
                          (unsigned long)B->KeyframeId, OST_Dense_LastError(W.Dense));
        DENSE_APP_FaultRaise(OST_FaultId_TERRAIN_UNAVAILABLE, "no terrain prior under the keyframe");
    }
    else
    {
        DENSE_APP_FaultResolve(OST_FaultId_TERRAIN_UNAVAILABLE);
    }

    /* Sources are done; the keyframe's counts and the depth set go on. */
    for (i = 0; i < Sources; ++i)
    {
        DENSE_APP_Release(&B->Sources[i].WorkImage);
        DENSE_APP_Release(&B->Sources[i].Jpeg);
    }
    CFE_MSG_Init(CFE_MSG_PTR(W.OutMsg.TelemetryHeader),
                 CFE_SB_ValueToMsgId(CFE_PLATFORM_TLM_TOPICID_TO_MIDV(EdsParam_CFE_MISSION_DENSE_APP_DEPTH_READY_TLM_TOPICID)),
                 sizeof(W.OutMsg));
    P->Header.SchemaVersion = 1;
    P->Header.Sequence      = ++W.Sequence;
    P->Trace.EntryMono      = It->EntryMono;
    P->KeyframeId           = B->KeyframeId;
    P->PoseSource           = B->PoseSource;
    P->Method               = DENSE_APP_Data.Config.Method;
    P->HasNormals           = Normals ? 1 : 0;
    P->GateApplied          = R.gate_applied;
    P->Intrinsics           = B->Intrinsics;
    P->Keyframe             = B->Keyframe;
    P->DepthSet             = DepthSet;
    P->FrameOriginEcef.X    = R.origin_ecef[0];
    P->FrameOriginEcef.Y    = R.origin_ecef[1];
    P->FrameOriginEcef.Z    = R.origin_ecef[2];
    P->FrameLatitude        = R.latitude_rad;
    P->FrameLongitude       = R.longitude_rad;
    P->DepthMinM            = (float)R.depth_min_m;
    P->DepthMaxM            = (float)R.depth_max_m;
    P->ValidPixels          = R.valid_pixels;
    P->GatedPixels          = R.gated_pixels;
    P->PartnerImageSeq      = R.source_index >= 0 && (uint32)R.source_index < Sources
                                  ? B->Sources[R.source_index].ImageSeq
                                  : 0u;
    P->SourceCount          = (uint8)Sources;
    P->WaitMs               = C->LastWaitMs;
    P->ReadMs               = DENSE_APP_Ms(ReadNs);
    P->SetupMs              = (float)(R.setup_s * 1e3);
    P->EstimateMs           = (float)(R.estimate_s * 1e3);
    P->GateMs               = (float)(R.gate_s * 1e3);
    P->Trace.ExitMono       = OST_TimeMonoNs();
    P->Trace.PublishUtc     = OST_TimeUtcNs();
    CFE_SB_TimeStampMsg(CFE_MSG_PTR(W.OutMsg.TelemetryHeader));
    if (CFE_SB_TransmitMsg(CFE_MSG_PTR(W.OutMsg.TelemetryHeader), true) != CFE_SUCCESS)
    {
        DENSE_APP_Release(&B->Keyframe.WorkImage);
        DENSE_APP_Release(&B->Keyframe.Jpeg);
        DENSE_APP_Release(&DepthSet);
        CFE_EVS_SendEvent(DENSE_APP_FAILED_EID, CFE_EVS_EventType_ERROR, "keyframe %lu: DEPTH_READY not sent",
                          (unsigned long)B->KeyframeId);
        DENSE_APP_PublishStatus(B, It->EntryMono, DENSE_APP_Outcome_FAILED, DENSE_APP_Reason_NOT_SENT);
        return;
    }

    Latency              = (float)((double)(P->Trace.PublishUtc - B->Keyframe.CaptureUtc) * 1e-9);
    C->LastKeyframeId    = B->KeyframeId;
    C->LastValidFraction = (float)((double)R.valid_pixels / (double)Pixels);
    C->LastGatedFraction = (float)((double)R.gated_pixels / (double)Pixels);
    C->LastDepthMinM     = P->DepthMinM;
    C->LastDepthMaxM     = P->DepthMaxM;
    C->LastReadMs        = P->ReadMs;
    C->LastSetupMs       = P->SetupMs;
    C->LastEstimateMs    = P->EstimateMs;
    C->LastGateMs        = P->GateMs;
    C->LastJobMs         = DENSE_APP_Ms(P->Trace.ExitMono - Start);
    C->LastLatencyS      = Latency;
    if (C->LastEstimateMs > C->MaxEstimateMs)
    {
        C->MaxEstimateMs = C->LastEstimateMs;
    }
    if (C->LastJobMs > C->MaxJobMs)
    {
        C->MaxJobMs = C->LastJobMs;
    }
    if (Latency > C->MaxLatencyS)
    {
        C->MaxLatencyS = Latency;
    }
    CFE_EVS_SendEvent(DENSE_APP_BATCH_DBG_EID, CFE_EVS_EventType_DEBUG,
                      "keyframe %lu: valid %.2f, gated %.2f, depth %.0f-%.0f m, estimate %.0f ms, job %.0f ms",
                      (unsigned long)B->KeyframeId, C->LastValidFraction, C->LastGatedFraction, P->DepthMinM,
                      P->DepthMaxM, P->EstimateMs, C->LastJobMs);
    DENSE_APP_PublishStatus(B, It->EntryMono, DENSE_APP_Outcome_DONE, DENSE_APP_Reason_NONE);
}

/* ------------------------------------------------------------------ task */

void DENSE_APP_WorkerTask(void)
{
    memset(&W, 0, sizeof(W));
    if (DENSE_APP_Data.ConfigValid)
    {
        DENSE_APP_CreateEstimator();
    }

    while (DENSE_APP_Data.RunStatus == CFE_ES_RunStatus_APP_RUN)
    {
        if (DENSE_APP_Dequeue(&W.Item))
        {
            DENSE_APP_OnBatch(&W.Item);
        }
    }

    /* Batches still queued carry their counts. */
    while (DENSE_APP_QueueDepth() > 0 && DENSE_APP_Dequeue(&W.Item))
    {
        DENSE_APP_ReleaseBatch(&W.Item.Batch);
        DENSE_APP_PublishStatus(&W.Item.Batch, W.Item.EntryMono, DENSE_APP_Outcome_SKIPPED, DENSE_APP_Reason_STOPPED);
    }
    OST_Dense_Destroy(W.Dense);
    W.Dense = NULL;
}
