/*
 * REFINE_APP worker task (refine_app.h): features, the sliding window of
 * libs/refine (OST_Refine_*), and the publication of solved batches.
 */
#include "refine_app.h"
#include "refine_app_eventids.h"

#include "ost_datastore.h"
#include "refine_c_api.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define REFINE_APP_POLL_BATCH 4

/* A batch waiting for its solution. */
typedef struct
{
    bool                        InUse;
    int64                       EntryMono;
    OST_KeyframeBatch_Payload_t Batch;
} REFINE_APP_Held_t;

typedef struct
{
    OST_Refine_t       *Refine;
    OST_Refine_Camera_t Camera;
    REFINE_APP_Held_t   Held[REFINE_APP_MAX_HELD];
    REFINE_APP_Item_t   Item;
    OST_Refine_Result_t Results[REFINE_APP_POLL_BATCH];
    uint8               Grey[REFINE_APP_MAX_GREY_BYTES];
} REFINE_APP_Worker_t;

static REFINE_APP_Worker_t W;

static uint32 REFINE_APP_Ms(double Seconds)
{
    return Seconds <= 0.0 ? 0u : (uint32)(Seconds * 1000.0);
}

static void REFINE_APP_LibError(const char *Call, OST_Status_t Status)
{
    CFE_EVS_SendEvent(REFINE_APP_LIB_ERR_EID, CFE_EVS_EventType_ERROR, "%s: %s (%s)", Call, OST_StatusName(Status),
                      W.Refine != NULL ? OST_Refine_LastError(W.Refine) : "");
    REFINE_APP_FaultReport(OST_FaultId_LIBRARY_ERROR, Call);
}

static uint32 REFINE_APP_HeldCount(void)
{
    uint32 i, n = 0;
    for (i = 0; i < REFINE_APP_MAX_HELD; ++i)
    {
        n += W.Held[i].InUse ? 1u : 0u;
    }
    return n;
}

static void REFINE_APP_UpdateStats(void)
{
    OST_Refine_Stats_t S;

    REFINE_APP_Data.Counters.BatchesHeld = REFINE_APP_HeldCount();
    REFINE_APP_Data.Counters.Ready       = W.Refine != NULL;
    if (W.Refine != NULL && OST_Refine_GetStats(W.Refine, &S) == OST_SUCCESS)
    {
        REFINE_APP_Data.Counters.WindowImages   = S.images;
        REFINE_APP_Data.Counters.FeaturesTotalS = (float)S.features_s;
    }
}

/* -------------------------------------------------------------- publish */

static double REFINE_APP_AngleDeg(const OST_Quaternion_t *A, const double B[4])
{
    double Dot = fabs(A->W * B[0] + A->X * B[1] + A->Y * B[2] + A->Z * B[3]);
    if (Dot > 1.0)
    {
        Dot = 1.0;
    }
    return 2.0 * acos(Dot) * 180.0 / 3.14159265358979323846;
}

static void REFINE_APP_SetPose(OST_Pose_t *Pose, const OST_Refine_Pose_t *R)
{
    Pose->PositionEcef.X           = R->position_ecef[0];
    Pose->PositionEcef.Y           = R->position_ecef[1];
    Pose->PositionEcef.Z           = R->position_ecef[2];
    Pose->AttitudeEcefFromCamera.W = R->q_ecef_from_camera[0];
    Pose->AttitudeEcefFromCamera.X = R->q_ecef_from_camera[1];
    Pose->AttitudeEcefFromCamera.Y = R->q_ecef_from_camera[2];
    Pose->AttitudeEcefFromCamera.Z = R->q_ecef_from_camera[3];
}

/* Publishes a held batch with its solution (or as failed when R is NULL or
 * not refined) and frees its slot. */
static void REFINE_APP_Publish(REFINE_APP_Held_t *H, const OST_Refine_Result_t *R, const char *Reason)
{
    REFINE_APP_Counters_t       *C = &REFINE_APP_Data.Counters;
    OST_KeyframeBatch_Payload_t *B = &H->Batch;
    uint32                       i;

    if (R != NULL && R->refined && R->count == 1u + B->SourceCount)
    {
        C->LastCorrectionDeg = (float)REFINE_APP_AngleDeg(&B->Keyframe.Pose.AttitudeEcefFromCamera,
                                                          R->poses[0].q_ecef_from_camera);
        REFINE_APP_SetPose(&B->Keyframe.Pose, &R->poses[0]);
        for (i = 0; i < B->SourceCount; ++i)
        {
            REFINE_APP_SetPose(&B->Sources[i].Pose, &R->poses[i + 1]);
        }
        B->PoseSource = OST_PoseSource_REFINED;
        ++C->BatchesRefined;
    }
    else
    {
        if (R != NULL && R->reason[0] != '\0')
        {
            Reason = R->reason;
        }
        B->PoseSource = OST_PoseSource_REFINE_FAILED;
        CFE_EVS_SendEvent(REFINE_APP_FAILED_EID, CFE_EVS_EventType_ERROR,
                          "keyframe %lu (image %lu) not refined: %s; telemetry poses passed on",
                          (unsigned long)B->KeyframeId, (unsigned long)B->Keyframe.ImageSeq,
                          Reason != NULL ? Reason : "unknown");
        REFINE_APP_FaultReport(OST_FaultId_REFINE_REJECTED, Reason);
    }
    if (R != NULL)
    {
        if (R->covered)
        {
            ++C->BatchesCovered;
        }
        else
        {
            ++C->BatchesWaitLimit;
        }
        C->LastWaitedS      = (float)R->waited_s;
        C->LastRmsPx        = (float)R->rms_px;
        C->LastBoresightDeg = (float)R->boresight_deg;
        C->BoresightHeld    = R->boresight_held != 0;
        C->LastPoints       = R->points;
        C->LastIterations   = R->iterations;
        C->LastWindowImages = R->window_images;
        C->LastSolveMs      = REFINE_APP_Ms(R->matching_s + R->adjustment_s);
        if (C->LastSolveMs > C->MaxSolveMs)
        {
            C->MaxSolveMs = C->LastSolveMs;
        }
        CFE_EVS_SendEvent(REFINE_APP_BATCH_DBG_EID, CFE_EVS_EventType_DEBUG,
                          "keyframe %lu: %s, %s, waited %.0f s, rms %.2f px, %lu images, %lu points, %lu ms",
                          (unsigned long)B->KeyframeId, R->refined ? "refined" : "failed",
                          R->covered ? "covered" : "waiting limit", R->waited_s, R->rms_px,
                          (unsigned long)R->window_images, (unsigned long)R->points, (unsigned long)C->LastSolveMs);
    }
    REFINE_APP_PublishBatch(B, H->EntryMono);
    memset(H, 0, sizeof(*H));
}

static REFINE_APP_Held_t *REFINE_APP_FindHeld(uint64 BatchId)
{
    uint32 i;
    for (i = 0; i < REFINE_APP_MAX_HELD; ++i)
    {
        if (W.Held[i].InUse && W.Held[i].Batch.KeyframeId == BatchId)
        {
            return &W.Held[i];
        }
    }
    return NULL;
}

/* Publishes the batches that are solved at data time now (all with Flush). */
static void REFINE_APP_Poll(bool Flush)
{
    uint32             Count = 0;
    uint32             i;
    OST_Status_t       Status;
    REFINE_APP_Held_t *H;

    if (W.Refine == NULL)
    {
        return;
    }
    do
    {
        Status = OST_Refine_Poll(W.Refine, OST_TimeUtcNs(), Flush ? 1 : 0, W.Results, REFINE_APP_POLL_BATCH, &Count);
        if (Status != OST_SUCCESS)
        {
            REFINE_APP_LibError("OST_Refine_Poll", Status);
            return;
        }
        for (i = 0; i < Count; ++i)
        {
            H = REFINE_APP_FindHeld(W.Results[i].batch_id);
            if (H != NULL)
            {
                REFINE_APP_Publish(H, &W.Results[i], NULL);
            }
        }
    } while (Count == REFINE_APP_POLL_BATCH);
    REFINE_APP_UpdateStats();
}

/* Solves and publishes everything, then forgets the window (session end). */
static void REFINE_APP_FlushAll(bool Destroy)
{
    uint32 i;

    REFINE_APP_Poll(true);
    for (i = 0; i < REFINE_APP_MAX_HELD; ++i)
    {
        if (W.Held[i].InUse)
        {
            REFINE_APP_Publish(&W.Held[i], NULL, "not solved at the session end");
        }
    }
    if (Destroy)
    {
        OST_Refine_Destroy(W.Refine);
        W.Refine = NULL;
    }
    REFINE_APP_UpdateStats();
}

/* ----------------------------------------------------------------- batch */

static OST_Status_t REFINE_APP_CreateHandle(const OST_WorkIntrinsics_t *I)
{
    const REFINE_APP_ConfigTable_t *T = &REFINE_APP_Data.Config;
    OST_Refine_Config_t             C;
    OST_Status_t                    Status;

    OST_Refine_DefaultConfig(&C);
    C.precise_position           = T->PrecisePosition;
    C.estimate_boresight         = T->EstimateBoresight;
    C.position_sigma_m           = T->PositionSigmaM;
    C.precise_position_sigma_m   = T->PrecisePositionSigmaM;
    C.attitude_sigma_deg         = T->AttitudeSigmaDeg;
    C.boresight_sigma_deg        = T->BoresightSigmaDeg;
    C.boresight_settle_batches   = T->BoresightSettleBatches;
    C.held_boresight_sigma_deg   = T->HeldBoresightSigmaDeg;
    C.refined_attitude_sigma_deg = T->RefinedAttitudeSigmaDeg;
    C.max_features               = (int32_t)T->MaxFeatures;
    C.max_window_images          = T->MaxWindowImages;
    C.max_points                 = T->MaxPoints;
    C.max_iterations             = (int32_t)T->MaxIterations;
    C.max_wait_s                 = T->MaxWaitS;
    C.window_s                   = T->WindowS;
    C.min_image_observations     = T->MinImageObservations;
    C.max_reprojection_rms_px    = T->MaxReprojectionRmsPx;
    C.max_correction_sigmas      = T->MaxCorrectionSigmas;

    memset(&W.Camera, 0, sizeof(W.Camera));
    W.Camera.width  = I->Width;
    W.Camera.height = I->Height;
    W.Camera.fx     = I->Fx;
    W.Camera.fy     = I->Fy;
    W.Camera.cx     = I->Cx;
    W.Camera.cy     = I->Cy;
    Status          = OST_Refine_Create(&C, &W.Camera, &W.Refine);
    if (Status != OST_SUCCESS)
    {
        W.Refine = NULL;
        REFINE_APP_LibError("OST_Refine_Create", Status);
    }
    return Status;
}

static bool REFINE_APP_SameCamera(const OST_WorkIntrinsics_t *I)
{
    return W.Camera.width == I->Width && W.Camera.height == I->Height && W.Camera.fx == I->Fx &&
           W.Camera.fy == I->Fy && W.Camera.cx == I->Cx && W.Camera.cy == I->Cy;
}

static float REFINE_APP_Max3(float A, float B, float C)
{
    float M = A > B ? A : B;
    return M > C ? M : C;
}

/* Adds a view's working image to the window if it is not there yet. */
static OST_Status_t REFINE_APP_AddView(const OST_ViewRef_t *V)
{
    const uint64        W2 = (uint64)W.Camera.width;
    const uint64        H2 = (uint64)W.Camera.height;
    OST_Refine_Image_t  Img;
    const void         *Data = NULL;
    uint64              Size = 0;
    const uint8        *Rgb;
    uint64              p;
    int                 Has = 0;
    OST_Status_t        Status;

    Status = OST_Refine_HasImage(W.Refine, V->ImageSeq, &Has);
    if (Status != OST_SUCCESS || Has)
    {
        return Status;
    }
    /* Read under the batch's own count (+1 for reading, released below). */
    Status = OST_DS_Acquire(&V->WorkImage, &Data, &Size);
    if (Status != OST_SUCCESS)
    {
        return Status;
    }
    memset(&Img, 0, sizeof(Img));
    if (Size == W2 * H2)
    {
        Img.grey = (const uint8_t *)Data;
    }
    else if (Size == 3u * W2 * H2 && Size / 3u <= REFINE_APP_MAX_GREY_BYTES)
    {
        Rgb = (const uint8 *)Data;
        for (p = 0; p < W2 * H2; ++p)
        {
            W.Grey[p] = (uint8)((77u * Rgb[3 * p] + 150u * Rgb[3 * p + 1] + 29u * Rgb[3 * p + 2]) >> 8);
        }
        Img.grey = W.Grey;
    }
    else
    {
        REFINE_APP_Release(&V->WorkImage);
        return OST_ERR_INVALID_ARGUMENT;
    }
    Img.id             = V->ImageSeq;
    Img.capture_utc_ns = V->CaptureUtc;
    Img.width          = W.Camera.width;
    Img.height         = W.Camera.height;
    Img.stride         = (size_t)W.Camera.width;
    Img.pose.position_ecef[0]      = V->Pose.PositionEcef.X;
    Img.pose.position_ecef[1]      = V->Pose.PositionEcef.Y;
    Img.pose.position_ecef[2]      = V->Pose.PositionEcef.Z;
    Img.pose.q_ecef_from_camera[0] = V->Pose.AttitudeEcefFromCamera.W;
    Img.pose.q_ecef_from_camera[1] = V->Pose.AttitudeEcefFromCamera.X;
    Img.pose.q_ecef_from_camera[2] = V->Pose.AttitudeEcefFromCamera.Y;
    Img.pose.q_ecef_from_camera[3] = V->Pose.AttitudeEcefFromCamera.Z;
    Img.attitude_sigma_deg = REFINE_APP_Max3(V->Pose.Accuracy.AttitudeStdRoll, V->Pose.Accuracy.AttitudeStdPitch,
                                             V->Pose.Accuracy.AttitudeStdYaw);
    Img.position_sigma_m   = REFINE_APP_Max3(V->Pose.Accuracy.PositionStdNorth, V->Pose.Accuracy.PositionStdEast,
                                             V->Pose.Accuracy.PositionStdDown);
    Status = OST_Refine_AddImage(W.Refine, &Img);
    REFINE_APP_Release(&V->WorkImage);
    if (Status == OST_SUCCESS)
    {
        ++REFINE_APP_Data.Counters.ImagesAdded;
    }
    return Status;
}

static void REFINE_APP_OnBatch(const OST_KeyframeBatch_Payload_t *B, int64 EntryMono)
{
    REFINE_APP_Held_t *H = NULL;
    uint64             Ids[OST_REFINE_MAX_BATCH_IMAGES];
    uint32             Count = 0;
    uint32             i;
    OST_Status_t       Status;
    double             F0;
    OST_Refine_Stats_t S;
    char               Reason[96];

    /* Another camera starts a new window. */
    if (W.Refine != NULL && !REFINE_APP_SameCamera(&B->Intrinsics))
    {
        REFINE_APP_FlushAll(true);
    }
    if (W.Refine == NULL && REFINE_APP_CreateHandle(&B->Intrinsics) != OST_SUCCESS)
    {
        REFINE_APP_Held_t Tmp;
        memset(&Tmp, 0, sizeof(Tmp));
        Tmp.Batch     = *B;
        Tmp.EntryMono = EntryMono;
        REFINE_APP_Publish(&Tmp, NULL, "refinement not available");
        return;
    }

    for (i = 0; i < REFINE_APP_MAX_HELD && H == NULL; ++i)
    {
        if (!W.Held[i].InUse)
        {
            H = &W.Held[i];
        }
    }
    if (H == NULL)
    {
        CFE_EVS_SendEvent(REFINE_APP_HELD_FULL_EID, CFE_EVS_EventType_ERROR,
                          "%u batches waiting: all are solved now", (unsigned)REFINE_APP_MAX_HELD);
        REFINE_APP_FlushAll(false);
        H = &W.Held[0];
    }
    H->InUse     = true;
    H->EntryMono = EntryMono;
    H->Batch     = *B;

    /* Images, keyframe first. */
    F0     = OST_Refine_GetStats(W.Refine, &S) == OST_SUCCESS ? S.features_s : 0.0;
    Status = REFINE_APP_AddView(&B->Keyframe);
    Ids[Count++] = B->Keyframe.ImageSeq;
    for (i = 0; Status == OST_SUCCESS && i < B->SourceCount && Count < OST_REFINE_MAX_BATCH_IMAGES; ++i)
    {
        Status       = REFINE_APP_AddView(&B->Sources[i]);
        Ids[Count++] = B->Sources[i].ImageSeq;
    }
    if (OST_Refine_GetStats(W.Refine, &S) == OST_SUCCESS)
    {
        REFINE_APP_Data.Counters.LastFeaturesMs = REFINE_APP_Ms(S.features_s - F0);
    }
    if (Status == OST_SUCCESS)
    {
        Status = OST_Refine_AddBatch(W.Refine, B->KeyframeId, Ids, Count, B->FootprintPlaneHeight);
    }
    if (Status != OST_SUCCESS)
    {
        snprintf(Reason, sizeof(Reason), "batch not accepted: %s", OST_StatusName(Status));
        REFINE_APP_LibError("OST_Refine_AddBatch", Status);
        REFINE_APP_Publish(H, NULL, Reason);
        return;
    }
    REFINE_APP_Poll(false);
}

/* ------------------------------------------------------------------ task */

void REFINE_APP_WorkerTask(void)
{
    memset(&W, 0, sizeof(W));

    while (REFINE_APP_Data.RunStatus == CFE_ES_RunStatus_APP_RUN)
    {
        if (!REFINE_APP_Dequeue(&W.Item, REFINE_APP_WORKER_TIMEOUT_MS))
        {
            REFINE_APP_Poll(false); /* waiting limits */
            continue;
        }
        switch (W.Item.Type)
        {
            case REFINE_APP_ITEM_BATCH:
                REFINE_APP_OnBatch(&W.Item.Batch, W.Item.EntryMono);
                break;
            case REFINE_APP_ITEM_SESSION:
                if (W.Refine != NULL)
                {
                    CFE_EVS_SendEvent(REFINE_APP_SESSION_INF_EID, CFE_EVS_EventType_INFORMATION,
                                      "session %lu: the waiting batches are solved and the window is cleared",
                                      (unsigned long)W.Item.SessionId);
                }
                REFINE_APP_FlushAll(true);
                break;
            case REFINE_APP_ITEM_FLUSH:
                REFINE_APP_FlushAll(false);
                break;
            default:
                break;
        }
    }

    /* Waiting batches still carry their counts. */
    REFINE_APP_FlushAll(true);
}
