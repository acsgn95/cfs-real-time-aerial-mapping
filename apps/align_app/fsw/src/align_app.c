/*
 * ALIGN_APP main task (align_app.h).
 */
#include "align_app.h"
#include "align_app_eventids.h"
#include "align_app_version.h"

#include "cfe_config.h"
#include "cfe_core_api_base_msgids.h"
#include "cfe_mission_eds_designparameters.h"
#include "align_app_eds_dictionary.h"
#include "align_app_eds_dispatcher.h"
#include "ost_datastore.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define ALIGN_APP_SCHEMA_VERSION 1
#define ALIGN_APP_POLL_BATCH     16

ALIGN_APP_Data_t ALIGN_APP_Data;

static const OST_FaultId_Enum_t ALIGN_APP_FaultIds[ALIGN_APP_FAULT_COUNT] = {
    OST_FaultId_POSE_NO_TELEMETRY, /* 60 */
    OST_FaultId_POSE_ACCURACY_LOW, /* 61 */
    OST_FaultId_TABLE_REJECTED,    /* 62 */
    OST_FaultId_LIBRARY_ERROR,     /* 63 */
};

/* Per-item events repeat; the first 8 are sent (ADR-020 section 3), the
 * counts are in housekeeping. */
static CFE_EVS_BinFilter_t ALIGN_APP_EventFilters[] = {
    {ALIGN_APP_IMAGE_REJECTED_EID, CFE_EVS_FIRST_8_STOP},
    {ALIGN_APP_MSG_LEN_ERR_EID, CFE_EVS_FIRST_8_STOP},
    {ALIGN_APP_DATASTORE_ERR_EID, CFE_EVS_FIRST_8_STOP},
    {ALIGN_APP_LIB_ERR_EID, CFE_EVS_FIRST_8_STOP},
};

/* clang-format off */
static const EdsDispatchTable_EdsComponent_ALIGN_APP_Application_CFE_SB_Telecommand_t ALIGN_APP_TC_DISPATCH_TABLE = {
    .CMD = {
        .NoopCmd_indication               = ALIGN_APP_NoopCmd,
        .ResetCountersCmd_indication      = ALIGN_APP_ResetCountersCmd,
        .ClearLatchedFaultsCmd_indication = ALIGN_APP_ClearLatchedFaultsCmd,
    },
    .SEND_HK = { .indication = ALIGN_APP_SendHkCmd },
};
/* clang-format on */

static CFE_SB_MsgId_t ALIGN_APP_CmdMid(uint16 TopicId)
{
    return CFE_SB_ValueToMsgId(CFE_PLATFORM_CMD_TOPICID_TO_MIDV(TopicId));
}

static CFE_SB_MsgId_t ALIGN_APP_TlmMid(uint16 TopicId)
{
    return CFE_SB_ValueToMsgId(CFE_PLATFORM_TLM_TOPICID_TO_MIDV(TopicId));
}

static uint32 ALIGN_APP_Ms(int64 Ns)
{
    int64 Ms = Ns / 1000000;
    if (Ms < 0)
    {
        return 0;
    }
    return (Ms > 0xFFFFFFFE) ? 0xFFFFFFFEu : (uint32)Ms;
}

static void ALIGN_APP_LibraryError(const char *Call, OST_Status_t Status)
{
    CFE_EVS_SendEvent(ALIGN_APP_LIB_ERR_EID, CFE_EVS_EventType_ERROR, "%s: %s (%s)", Call, OST_StatusName(Status),
                      ALIGN_APP_Data.Align != NULL ? OST_Align_LastError(ALIGN_APP_Data.Align) : "");
    OST_FaultSet_Report(&ALIGN_APP_Data.Faults, OST_FaultId_LIBRARY_ERROR, Call);
}

/* ----------------------------------------------------------------- table */

static CFE_Status_t ALIGN_APP_ValidateConfig(void *TblData)
{
    const ALIGN_APP_ConfigTable_t *T = (const ALIGN_APP_ConfigTable_t *)TblData;

    if (T->MaxGapMs == 0 || T->BufferMs < T->MaxGapMs || T->MaxPendingImages == 0 ||
        T->MaxPendingImages > ALIGN_APP_MAX_IMAGES)
    {
        CFE_EVS_SendEvent(ALIGN_APP_TBL_ERR_EID, CFE_EVS_EventType_ERROR,
                          "ConfigTable: MaxGapMs %lu, BufferMs %lu, MaxPendingImages %lu (1-%u) invalid",
                          (unsigned long)T->MaxGapMs, (unsigned long)T->BufferMs, (unsigned long)T->MaxPendingImages,
                          (unsigned)ALIGN_APP_MAX_IMAGES);
        return CFE_STATUS_VALIDATION_FAILURE;
    }
    if (!isfinite(T->MaxPositionStdM) || T->MaxPositionStdM < 0.0f || !isfinite(T->MaxAttitudeStdDeg) ||
        T->MaxAttitudeStdDeg < 0.0f)
    {
        CFE_EVS_SendEvent(ALIGN_APP_TBL_ERR_EID, CFE_EVS_EventType_ERROR,
                          "ConfigTable: accuracy limits must be finite and >= 0");
        return CFE_STATUS_VALIDATION_FAILURE;
    }
    return CFE_SUCCESS;
}

/* Tables are loaded once at startup (ADR-016 section 5). */
static void ALIGN_APP_LoadConfig(void)
{
    CFE_Status_t       Status;
    void              *Addr = NULL;
    OST_Align_Config_t C;
    OST_Status_t       LibStatus;

    Status = CFE_TBL_Register(&ALIGN_APP_Data.TblHandle, "ConfigTable", sizeof(ALIGN_APP_ConfigTable_t),
                              CFE_TBL_OPT_DEFAULT, ALIGN_APP_ValidateConfig);
    if (Status == CFE_SUCCESS)
    {
        Status = CFE_TBL_Load(ALIGN_APP_Data.TblHandle, CFE_TBL_SRC_FILE, ALIGN_APP_TABLE_FILE);
    }
    if (Status == CFE_SUCCESS)
    {
        Status = CFE_TBL_GetAddress(&Addr, ALIGN_APP_Data.TblHandle);
        if (Status == CFE_TBL_INFO_UPDATED)
        {
            Status = CFE_SUCCESS;
        }
    }
    if (Status != CFE_SUCCESS || Addr == NULL)
    {
        CFE_EVS_SendEvent(ALIGN_APP_TBL_ERR_EID, CFE_EVS_EventType_ERROR,
                          "ConfigTable %s not loaded (0x%08lX); images are released without a pose",
                          ALIGN_APP_TABLE_FILE, (unsigned long)Status);
        OST_FaultSet_Raise(&ALIGN_APP_Data.Faults, OST_FaultId_TABLE_REJECTED, "ConfigTable not loaded");
        return;
    }
    memcpy(&ALIGN_APP_Data.Config, Addr, sizeof(ALIGN_APP_Data.Config));
    CFE_TBL_ReleaseAddress(ALIGN_APP_Data.TblHandle);
    ALIGN_APP_Data.ConfigValid = true;

    OST_Align_DefaultConfig(&C);
    C.max_gap_ns  = (int64)ALIGN_APP_Data.Config.MaxGapMs * 1000000;
    C.max_wait_ns = (int64)ALIGN_APP_Data.Config.MaxWaitMs * 1000000;
    C.buffer_ns   = (int64)ALIGN_APP_Data.Config.BufferMs * 1000000;
    C.max_pending = ALIGN_APP_Data.Config.MaxPendingImages;
    LibStatus     = OST_Align_Create(&C, &ALIGN_APP_Data.Align);
    if (LibStatus != OST_SUCCESS)
    {
        ALIGN_APP_Data.Align = NULL;
        ALIGN_APP_LibraryError("OST_Align_Create", LibStatus);
    }
}

/* ------------------------------------------------------------ publishing */

static ALIGN_APP_Image_t *ALIGN_APP_FindImage(uint64 Id)
{
    uint32 i;
    for (i = 0; i < ALIGN_APP_MAX_IMAGES; ++i)
    {
        if (ALIGN_APP_Data.Images[i].InUse && ALIGN_APP_Data.Images[i].Id == Id)
        {
            return &ALIGN_APP_Data.Images[i];
        }
    }
    return NULL;
}

static ALIGN_APP_Image_t *ALIGN_APP_FreeSlot(void)
{
    uint32 i;
    for (i = 0; i < ALIGN_APP_MAX_IMAGES; ++i)
    {
        if (!ALIGN_APP_Data.Images[i].InUse)
        {
            return &ALIGN_APP_Data.Images[i];
        }
    }
    return NULL;
}

static void ALIGN_APP_ReleaseJpeg(const OST_ObjectRef_t *Ref)
{
    OST_Status_t Status;

    if (Ref->ObjectId == 0)
    {
        return;
    }
    Status = OST_DS_Release(Ref);
    if (Status != OST_SUCCESS)
    {
        CFE_EVS_SendEvent(ALIGN_APP_DATASTORE_ERR_EID, CFE_EVS_EventType_ERROR,
                          "JPEG object %lu/%lu not released: %s", (unsigned long)Ref->ObjectId,
                          (unsigned long)Ref->Generation, OST_StatusName(Status));
    }
}

static void ALIGN_APP_CopyAccuracy(OST_PoseAccuracy_t *Dst, const OST_Align_Accuracy_t *Src)
{
    Dst->PositionStdNorth = Src->position_std_north_m;
    Dst->PositionStdEast  = Src->position_std_east_m;
    Dst->PositionStdDown  = Src->position_std_down_m;
    Dst->AttitudeStdRoll  = Src->attitude_std_roll_deg;
    Dst->AttitudeStdPitch = Src->attitude_std_pitch_deg;
    Dst->AttitudeStdYaw   = Src->attitude_std_yaw_deg;
    Dst->Fix              = Src->fix;
}

static bool ALIGN_APP_AccuracyLow(const OST_Align_Accuracy_t *A)
{
    const ALIGN_APP_ConfigTable_t *C   = &ALIGN_APP_Data.Config;
    float                          Pos = A->position_std_north_m;
    float                          Att = A->attitude_std_roll_deg;

    if (A->position_std_east_m > Pos)
    {
        Pos = A->position_std_east_m;
    }
    if (A->position_std_down_m > Pos)
    {
        Pos = A->position_std_down_m;
    }
    if (A->attitude_std_pitch_deg > Att)
    {
        Att = A->attitude_std_pitch_deg;
    }
    if (A->attitude_std_yaw_deg > Att)
    {
        Att = A->attitude_std_yaw_deg;
    }
    return (C->MaxPositionStdM > 0.0f && Pos > C->MaxPositionStdM) ||
           (C->MaxAttitudeStdDeg > 0.0f && Att > C->MaxAttitudeStdDeg);
}

/* Publishes the outcome of an image and frees its slot. The JPEG reference
 * count goes with a valid pose to the consumer (ADR-013 section 3);
 * otherwise it is released here. */
static void ALIGN_APP_Publish(ALIGN_APP_Image_t *Img, const OST_Align_ImagePose_t *Pose)
{
    ALIGN_APP_ImagePose_Payload_t *P     = &ALIGN_APP_Data.PoseTlm.Payload;
    ALIGN_APP_Counters_t          *C     = &ALIGN_APP_Data.Counters;
    const uint8                    Res   = Pose->result;
    const bool                     Valid = (Res == OST_ALIGN_RESULT_VALID);
    uint32                         Seq   = P->Header.Sequence;
    char                           Text[64];

    memset(P, 0, sizeof(*P));
    P->Header.SchemaVersion = ALIGN_APP_SCHEMA_VERSION;
    P->Header.Sequence      = Seq + 1;
    P->Trace.EntryMono      = Img->EntryMono;
    P->CaptureUtc           = Img->CaptureUtc;
    P->RecordedUtc          = Img->RecordedUtc;
    P->SessionId            = Img->SessionId;
    P->ImageSeq             = Img->ImageSeq;
    P->JpegSize             = Img->JpegSize;
    P->Result               = Res;
    P->PoseSource           = OST_PoseSource_TELEMETRY;
    P->SampleBeforeUtc      = Pose->before_ns;
    P->SampleAfterUtc       = Pose->after_ns;
    P->WaitedMs             = ALIGN_APP_Ms(Pose->waited_ns);
    ALIGN_APP_CopyAccuracy(&P->Pose.Accuracy, &Pose->accuracy);

    C->LastWaitedMs = P->WaitedMs;
    if (P->WaitedMs > C->MaxWaitedMs)
    {
        C->MaxWaitedMs = P->WaitedMs;
    }

    if (Valid)
    {
        P->Jpeg                          = Img->Jpeg;
        P->Pose.PositionEcef.X           = Pose->position_ecef[0];
        P->Pose.PositionEcef.Y           = Pose->position_ecef[1];
        P->Pose.PositionEcef.Z           = Pose->position_ecef[2];
        P->Pose.AttitudeEcefFromCamera.W = Pose->q_ecef_from_camera[0];
        P->Pose.AttitudeEcefFromCamera.X = Pose->q_ecef_from_camera[1];
        P->Pose.AttitudeEcefFromCamera.Y = Pose->q_ecef_from_camera[2];
        P->Pose.AttitudeEcefFromCamera.Z = Pose->q_ecef_from_camera[3];
        ++C->PosesValid;
        if (ALIGN_APP_AccuracyLow(&Pose->accuracy))
        {
            P->AccuracyLow = 1;
            ++C->AccuracyLow;
            OST_FaultSet_Raise(&ALIGN_APP_Data.Faults, OST_FaultId_POSE_ACCURACY_LOW, "pose accuracy beyond the limits");
        }
        else
        {
            OST_FaultSet_Resolve(&ALIGN_APP_Data.Faults, OST_FaultId_POSE_ACCURACY_LOW);
        }
    }
    else
    {
        ALIGN_APP_ReleaseJpeg(&Img->Jpeg);
        if (Res <= OST_ALIGN_RESULT_CLEARED)
        {
            ++C->Rejected[Res];
        }
        CFE_EVS_SendEvent(ALIGN_APP_IMAGE_REJECTED_EID, CFE_EVS_EventType_ERROR,
                          "image %lu of session %lu rejected: %s", (unsigned long)Img->ImageSeq,
                          (unsigned long)Img->SessionId, OST_Align_ResultName(Res));
        if (Res == OST_ALIGN_RESULT_NO_SAMPLE_BEFORE || Res == OST_ALIGN_RESULT_GAP_TOO_LARGE ||
            Res == OST_ALIGN_RESULT_WAIT_EXPIRED)
        {
            snprintf(Text, sizeof(Text), "image %lu: %s", (unsigned long)Img->ImageSeq, OST_Align_ResultName(Res));
            OST_FaultSet_Report(&ALIGN_APP_Data.Faults, OST_FaultId_POSE_NO_TELEMETRY, Text);
        }
    }

    P->Trace.ExitMono   = OST_TimeMonoNs();
    P->Trace.PublishUtc = OST_TimeUtcNs();
    CFE_SB_TimeStampMsg(CFE_MSG_PTR(ALIGN_APP_Data.PoseTlm.TelemetryHeader));
    if (CFE_SB_TransmitMsg(CFE_MSG_PTR(ALIGN_APP_Data.PoseTlm.TelemetryHeader), true) != CFE_SUCCESS && Valid)
    {
        ALIGN_APP_ReleaseJpeg(&Img->Jpeg); /* nobody owns the count */
    }
    memset(Img, 0, sizeof(*Img));
}

/* Publishes the images whose outcome is known at Now. */
static void ALIGN_APP_Drain(int64 NowMono)
{
    OST_Align_ImagePose_t Poses[ALIGN_APP_POLL_BATCH];
    uint32                Count = 0;
    uint32                i;
    OST_Status_t          Status;
    ALIGN_APP_Image_t    *Img;

    if (ALIGN_APP_Data.Align == NULL)
    {
        return;
    }
    do
    {
        Status = OST_Align_Poll(ALIGN_APP_Data.Align, NowMono, Poses, ALIGN_APP_POLL_BATCH, &Count);
        if (Status != OST_SUCCESS)
        {
            ALIGN_APP_LibraryError("OST_Align_Poll", Status);
            return;
        }
        for (i = 0; i < Count; ++i)
        {
            Img = ALIGN_APP_FindImage(Poses[i].id);
            if (Img != NULL)
            {
                ALIGN_APP_Publish(Img, &Poses[i]);
            }
        }
    } while (Count == ALIGN_APP_POLL_BATCH);
}

/* --------------------------------------------------------- input messages */

/* A message of another session starts a new one (align_app.h). */
static void ALIGN_APP_CheckSession(uint32 SessionId)
{
    OST_Status_t Status;

    if (SessionId == ALIGN_APP_Data.SessionId)
    {
        return;
    }
    Status = OST_Align_Clear(ALIGN_APP_Data.Align);
    if (Status != OST_SUCCESS)
    {
        ALIGN_APP_LibraryError("OST_Align_Clear", Status);
    }
    CFE_EVS_SendEvent(ALIGN_APP_SESSION_INF_EID, CFE_EVS_EventType_INFORMATION, "session %lu (previous %lu)",
                      (unsigned long)SessionId, (unsigned long)ALIGN_APP_Data.SessionId);
    ALIGN_APP_Data.SessionId  = SessionId;
    ALIGN_APP_Data.Calibrated = false;
}

static bool ALIGN_APP_CheckSize(const CFE_SB_Buffer_t *Buf, size_t Expected)
{
    CFE_MSG_Size_t Size = 0;
    CFE_SB_MsgId_t MsgId;

    CFE_MSG_GetSize(&Buf->Msg, &Size);
    if (Size == Expected)
    {
        return true;
    }
    CFE_MSG_GetMsgId(&Buf->Msg, &MsgId);
    ++ALIGN_APP_Data.Counters.MessagesRejected;
    CFE_EVS_SendEvent(ALIGN_APP_MSG_LEN_ERR_EID, CFE_EVS_EventType_ERROR,
                      "input MID 0x%x: length %lu, expected %lu", (unsigned)CFE_SB_MsgIdToValue(MsgId),
                      (unsigned long)Size, (unsigned long)Expected);
    return false;
}

static void ALIGN_APP_OnTelemetry(const INGEST_APP_TelemetrySample_Payload_t *P)
{
    OST_Align_Sample_t S;
    OST_Status_t       Status;
    int                Verdict = 0;

    ALIGN_APP_CheckSession(P->SessionId);
    memset(&S, 0, sizeof(S));
    S.time_ns            = P->MeasurementUtc;
    S.latitude_deg       = P->Latitude;
    S.longitude_deg      = P->Longitude;
    S.height_m           = P->Height;
    S.q_ned_from_body[0] = P->AttitudeNedFromBody.W;
    S.q_ned_from_body[1] = P->AttitudeNedFromBody.X;
    S.q_ned_from_body[2] = P->AttitudeNedFromBody.Y;
    S.q_ned_from_body[3] = P->AttitudeNedFromBody.Z;
    S.accuracy.position_std_north_m   = P->Accuracy.PositionStdNorth;
    S.accuracy.position_std_east_m    = P->Accuracy.PositionStdEast;
    S.accuracy.position_std_down_m    = P->Accuracy.PositionStdDown;
    S.accuracy.attitude_std_roll_deg  = P->Accuracy.AttitudeStdRoll;
    S.accuracy.attitude_std_pitch_deg = P->Accuracy.AttitudeStdPitch;
    S.accuracy.attitude_std_yaw_deg   = P->Accuracy.AttitudeStdYaw;
    S.accuracy.fix                    = P->Accuracy.Fix;

    /* The verdicts are counted by the library (housekeeping). */
    Status = OST_Align_AddSample(ALIGN_APP_Data.Align, &S, &Verdict);
    if (Status != OST_SUCCESS)
    {
        ALIGN_APP_LibraryError("OST_Align_AddSample", Status);
    }
}

static void ALIGN_APP_OnCalibration(const INGEST_APP_SessionCalibration_Payload_t *P)
{
    const double Q[4] = {P->BodyFromCamera.W, P->BodyFromCamera.X, P->BodyFromCamera.Y, P->BodyFromCamera.Z};
    const double T[3] = {P->CameraInBody.X, P->CameraInBody.Y, P->CameraInBody.Z};
    OST_Status_t Status;

    ALIGN_APP_CheckSession(P->SessionId);
    Status = OST_Align_SetMounting(ALIGN_APP_Data.Align, Q, T);
    if (Status != OST_SUCCESS)
    {
        CFE_EVS_SendEvent(ALIGN_APP_CALIBRATION_ERR_EID, CFE_EVS_EventType_ERROR,
                          "session %lu: camera mounting rejected: %s", (unsigned long)P->SessionId,
                          OST_Align_LastError(ALIGN_APP_Data.Align));
        return;
    }
    ALIGN_APP_Data.Calibrated = true;
    CFE_EVS_SendEvent(ALIGN_APP_CALIBRATION_INF_EID, CFE_EVS_EventType_INFORMATION,
                      "session %lu: camera mounting set, lever arm %.3f %.3f %.3f m", (unsigned long)P->SessionId,
                      T[0], T[1], T[2]);
}

static void ALIGN_APP_OnImage(const INGEST_APP_ImageArrived_Payload_t *P, int64 NowMono)
{
    ALIGN_APP_Image_t    *Img;
    OST_Align_ImagePose_t Rejected;
    OST_Status_t          Status;

    ++ALIGN_APP_Data.Counters.ImagesReceived;
    ALIGN_APP_CheckSession(P->SessionId);

    Img = ALIGN_APP_FreeSlot();
    if (Img == NULL)
    {
        ALIGN_APP_Image_t Tmp;
        memset(&Tmp, 0, sizeof(Tmp));
        memset(&Rejected, 0, sizeof(Rejected));
        Tmp.SessionId   = P->SessionId;
        Tmp.ImageSeq    = P->ImageSeq;
        Tmp.CaptureUtc  = P->CaptureUtc;
        Tmp.RecordedUtc = P->RecordedUtc;
        Tmp.EntryMono   = NowMono;
        Tmp.Jpeg        = P->Jpeg;
        Tmp.JpegSize    = P->JpegSize;
        Rejected.result = OST_ALIGN_RESULT_QUEUE_FULL;
        ALIGN_APP_Publish(&Tmp, &Rejected);
        return;
    }
    Img->InUse       = true;
    Img->Id          = ((uint64)P->SessionId << 32) | P->ImageSeq;
    Img->SessionId   = P->SessionId;
    Img->ImageSeq    = P->ImageSeq;
    Img->CaptureUtc  = P->CaptureUtc;
    Img->RecordedUtc = P->RecordedUtc;
    Img->EntryMono   = NowMono;
    Img->Jpeg        = P->Jpeg;
    Img->JpegSize    = P->JpegSize;

    Status = OST_Align_AddImage(ALIGN_APP_Data.Align, Img->Id, P->CaptureUtc, NowMono);
    if (Status != OST_SUCCESS)
    {
        ALIGN_APP_LibraryError("OST_Align_AddImage", Status);
        ALIGN_APP_ReleaseJpeg(&Img->Jpeg);
        memset(Img, 0, sizeof(*Img));
    }
}

/* Returns true if the buffer was an input message. */
static bool ALIGN_APP_ProcessInput(const CFE_SB_Buffer_t *Buf, CFE_SB_MsgId_t MsgId, int64 NowMono)
{
    if (CFE_SB_MsgId_Equal(MsgId, ALIGN_APP_Data.TelemetrySampleMid))
    {
        if (ALIGN_APP_Data.Align != NULL && ALIGN_APP_CheckSize(Buf, sizeof(INGEST_APP_TelemetrySampleTlm_t)))
        {
            ALIGN_APP_OnTelemetry(&((const INGEST_APP_TelemetrySampleTlm_t *)Buf)->Payload);
        }
        return true;
    }
    if (CFE_SB_MsgId_Equal(MsgId, ALIGN_APP_Data.ImageArrivedMid))
    {
        if (ALIGN_APP_CheckSize(Buf, sizeof(INGEST_APP_ImageArrivedTlm_t)))
        {
            const INGEST_APP_ImageArrived_Payload_t *P = &((const INGEST_APP_ImageArrivedTlm_t *)Buf)->Payload;
            if (ALIGN_APP_Data.Align != NULL)
            {
                ALIGN_APP_OnImage(P, NowMono);
            }
            else
            {
                /* No valid table (TABLE_REJECTED): the count is released. */
                ++ALIGN_APP_Data.Counters.ImagesReceived;
                ALIGN_APP_ReleaseJpeg(&P->Jpeg);
            }
        }
        return true;
    }
    if (CFE_SB_MsgId_Equal(MsgId, ALIGN_APP_Data.SessionCalibrationMid))
    {
        if (ALIGN_APP_Data.Align != NULL && ALIGN_APP_CheckSize(Buf, sizeof(INGEST_APP_SessionCalibrationTlm_t)))
        {
            ALIGN_APP_OnCalibration(&((const INGEST_APP_SessionCalibrationTlm_t *)Buf)->Payload);
        }
        return true;
    }
    return false;
}

/* ------------------------------------------------------------------ init */

static CFE_Status_t ALIGN_APP_Subscribe(CFE_SB_MsgId_t MsgId, uint16 Limit)
{
    return CFE_SB_SubscribeEx(MsgId, ALIGN_APP_Data.Pipe, CFE_SB_DEFAULT_QOS, Limit);
}

static CFE_Status_t ALIGN_APP_Init(void)
{
    CFE_Status_t Status;
    char         VersionString[ALIGN_APP_MAX_VERSION_STR_LEN];

    memset(&ALIGN_APP_Data, 0, sizeof(ALIGN_APP_Data));
    ALIGN_APP_Data.RunStatus = CFE_ES_RunStatus_APP_RUN;

    Status = CFE_EVS_Register(ALIGN_APP_EventFilters, sizeof(ALIGN_APP_EventFilters) / sizeof(ALIGN_APP_EventFilters[0]),
                              CFE_EVS_EventFilter_BINARY);
    if (Status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("ALIGN_APP: EVS register failed 0x%08lX\n", (unsigned long)Status);
        return Status;
    }
    Status = OST_FaultSet_Init(&ALIGN_APP_Data.Faults, OST_AppId_ALIGN_APP, ALIGN_APP_FaultIds, ALIGN_APP_FAULT_COUNT);
    if (Status != CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(ALIGN_APP_INIT_ERR_EID, CFE_EVS_EventType_ERROR, "fault set: 0x%08lX", (unsigned long)Status);
        return Status;
    }

    CFE_MSG_Init(CFE_MSG_PTR(ALIGN_APP_Data.HkTlm.TelemetryHeader),
                 ALIGN_APP_TlmMid(EdsParam_CFE_MISSION_ALIGN_APP_HK_TLM_TOPICID), sizeof(ALIGN_APP_Data.HkTlm));
    CFE_MSG_Init(CFE_MSG_PTR(ALIGN_APP_Data.PoseTlm.TelemetryHeader),
                 ALIGN_APP_TlmMid(EdsParam_CFE_MISSION_ALIGN_APP_IMAGE_POSE_TLM_TOPICID),
                 sizeof(ALIGN_APP_Data.PoseTlm));

    ALIGN_APP_Data.TelemetrySampleMid = ALIGN_APP_TlmMid(EdsParam_CFE_MISSION_INGEST_APP_TELEMETRY_SAMPLE_TLM_TOPICID);
    ALIGN_APP_Data.ImageArrivedMid    = ALIGN_APP_TlmMid(EdsParam_CFE_MISSION_INGEST_APP_IMAGE_ARRIVED_TLM_TOPICID);
    ALIGN_APP_Data.SessionCalibrationMid =
        ALIGN_APP_TlmMid(EdsParam_CFE_MISSION_INGEST_APP_SESSION_CALIBRATION_TLM_TOPICID);

    Status = CFE_SB_CreatePipe(&ALIGN_APP_Data.Pipe, ALIGN_APP_PIPE_DEPTH, "ALIGN_PIPE");
    if (Status == CFE_SUCCESS)
    {
        Status = ALIGN_APP_Subscribe(ALIGN_APP_CmdMid(EdsParam_CFE_MISSION_ALIGN_APP_CMD_TOPICID), 4);
    }
    if (Status == CFE_SUCCESS)
    {
        Status = ALIGN_APP_Subscribe(ALIGN_APP_CmdMid(EdsParam_CFE_MISSION_ALIGN_APP_SEND_HK_TOPICID), 2);
    }
    if (Status == CFE_SUCCESS)
    {
        Status = ALIGN_APP_Subscribe(ALIGN_APP_Data.TelemetrySampleMid, ALIGN_APP_TELEMETRY_LIMIT);
    }
    if (Status == CFE_SUCCESS)
    {
        Status = ALIGN_APP_Subscribe(ALIGN_APP_Data.ImageArrivedMid, ALIGN_APP_IMAGE_LIMIT);
    }
    if (Status == CFE_SUCCESS)
    {
        Status = ALIGN_APP_Subscribe(ALIGN_APP_Data.SessionCalibrationMid, 2);
    }
    if (Status != CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(ALIGN_APP_INIT_ERR_EID, CFE_EVS_EventType_ERROR, "pipe or subscription: 0x%08lX",
                          (unsigned long)Status);
        return Status;
    }

    ALIGN_APP_LoadConfig();

    CFE_Config_GetVersionString(VersionString, ALIGN_APP_MAX_VERSION_STR_LEN, "ALIGN_APP", ALIGN_APP_VERSION,
                                ALIGN_APP_BUILD_CODENAME, ALIGN_APP_LAST_OFFICIAL);
    CFE_EVS_SendEvent(ALIGN_APP_INIT_INF_EID, CFE_EVS_EventType_INFORMATION,
                      "ALIGN_APP initialised: gap %lu ms, wait %lu ms, buffer %lu ms. %s",
                      (unsigned long)ALIGN_APP_Data.Config.MaxGapMs, (unsigned long)ALIGN_APP_Data.Config.MaxWaitMs,
                      (unsigned long)ALIGN_APP_Data.Config.BufferMs, VersionString);
    return CFE_SUCCESS;
}

/* -------------------------------------------------------------- commands */

static void ALIGN_APP_ProcessCommand(const CFE_SB_Buffer_t *Buf, CFE_SB_MsgId_t MsgId)
{
    CFE_Status_t      Status;
    CFE_MSG_Size_t    Size = 0;
    CFE_MSG_FcnCode_t Fc   = 0;

    Status = EdsDispatch_EdsComponent_ALIGN_APP_Application_Telecommand(Buf, &ALIGN_APP_TC_DISPATCH_TABLE);
    if (Status == CFE_SUCCESS)
    {
        return;
    }
    CFE_MSG_GetSize(&Buf->Msg, &Size);
    CFE_MSG_GetFcnCode(&Buf->Msg, &Fc);
    ++ALIGN_APP_Data.CommandErrorCounter;
    if (Status == CFE_STATUS_UNKNOWN_MSG_ID)
    {
        CFE_EVS_SendEvent(ALIGN_APP_MID_ERR_EID, CFE_EVS_EventType_ERROR, "invalid message ID 0x%x",
                          (unsigned)CFE_SB_MsgIdToValue(MsgId));
    }
    else if (Status == CFE_STATUS_WRONG_MSG_LENGTH)
    {
        CFE_EVS_SendEvent(ALIGN_APP_CMD_LEN_ERR_EID, CFE_EVS_EventType_ERROR,
                          "invalid length: MID 0x%x, CC %u, length %u", (unsigned)CFE_SB_MsgIdToValue(MsgId),
                          (unsigned)Fc, (unsigned)Size);
    }
    else
    {
        CFE_EVS_SendEvent(ALIGN_APP_CC_ERR_EID, CFE_EVS_EventType_ERROR, "invalid command code %u", (unsigned)Fc);
    }
}

CFE_Status_t ALIGN_APP_NoopCmd(const ALIGN_APP_NoopCmd_t *Msg)
{
    (void)Msg;
    ++ALIGN_APP_Data.CommandCounter;
    CFE_EVS_SendEvent(ALIGN_APP_NOOP_INF_EID, CFE_EVS_EventType_INFORMATION, "no-op, version %s", ALIGN_APP_VERSION);
    return CFE_SUCCESS;
}

CFE_Status_t ALIGN_APP_ResetCountersCmd(const ALIGN_APP_ResetCountersCmd_t *Msg)
{
    (void)Msg;
    ALIGN_APP_Data.CommandCounter      = 0;
    ALIGN_APP_Data.CommandErrorCounter = 0;
    memset(&ALIGN_APP_Data.Counters, 0, sizeof(ALIGN_APP_Data.Counters));
    CFE_EVS_SendEvent(ALIGN_APP_RESET_INF_EID, CFE_EVS_EventType_INFORMATION,
                      "counters reset (sample counts are kept by the library)");
    return CFE_SUCCESS;
}

CFE_Status_t ALIGN_APP_ClearLatchedFaultsCmd(const ALIGN_APP_ClearLatchedFaultsCmd_t *Msg)
{
    uint32 Cleared;

    (void)Msg;
    Cleared = OST_FaultSet_ClearLatched(&ALIGN_APP_Data.Faults);
    ++ALIGN_APP_Data.CommandCounter;
    CFE_EVS_SendEvent(ALIGN_APP_CLEAR_INF_EID, CFE_EVS_EventType_INFORMATION, "%lu latched faults cleared",
                      (unsigned long)Cleared);
    return CFE_SUCCESS;
}

CFE_Status_t ALIGN_APP_SendHkCmd(const ALIGN_APP_SendHkCmd_t *Msg)
{
    ALIGN_APP_HkTlm_Payload_t  *P = &ALIGN_APP_Data.HkTlm.Payload;
    const ALIGN_APP_Counters_t *C = &ALIGN_APP_Data.Counters;
    OST_Align_Stats_t           S;

    (void)Msg;
    memset(&S, 0, sizeof(S));
    if (ALIGN_APP_Data.Align != NULL)
    {
        OST_Align_GetStats(ALIGN_APP_Data.Align, &S);
    }
    P->Common.CommandCounter      = ALIGN_APP_Data.CommandCounter;
    P->Common.CommandErrorCounter = ALIGN_APP_Data.CommandErrorCounter;
    OST_FaultSet_Summarise(&ALIGN_APP_Data.Faults, &P->Common);
    P->SessionId              = ALIGN_APP_Data.SessionId;
    P->Calibrated             = ALIGN_APP_Data.Calibrated ? 1 : 0;
    P->SamplesAccepted        = (uint32)S.samples_accepted;
    P->SamplesOutOfOrder      = (uint32)S.samples_out_of_order;
    P->SamplesDuplicate       = (uint32)S.samples_duplicate;
    P->SamplesInvalid         = (uint32)S.samples_invalid;
    P->ImagesReceived         = C->ImagesReceived;
    P->PosesValid             = C->PosesValid;
    P->RejectedNoSampleBefore = C->Rejected[OST_ALIGN_RESULT_NO_SAMPLE_BEFORE];
    P->RejectedGap            = C->Rejected[OST_ALIGN_RESULT_GAP_TOO_LARGE];
    P->RejectedWait           = C->Rejected[OST_ALIGN_RESULT_WAIT_EXPIRED];
    P->RejectedNoCalibration  = C->Rejected[OST_ALIGN_RESULT_NO_CALIBRATION];
    P->RejectedQueueFull      = C->Rejected[OST_ALIGN_RESULT_QUEUE_FULL];
    P->RejectedCleared        = C->Rejected[OST_ALIGN_RESULT_CLEARED];
    P->AccuracyLow            = C->AccuracyLow;
    P->MessagesRejected       = C->MessagesRejected;
    P->BufferedSamples        = S.buffered_samples;
    P->BufferSpanMs           = ALIGN_APP_Ms(S.buffer_span_ns);
    P->PendingImages          = S.pending_images;
    P->LastWaitedMs           = C->LastWaitedMs;
    P->MaxWaitedMs            = C->MaxWaitedMs;

    CFE_SB_TimeStampMsg(CFE_MSG_PTR(ALIGN_APP_Data.HkTlm.TelemetryHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(ALIGN_APP_Data.HkTlm.TelemetryHeader), true);
    return CFE_SUCCESS;
}

/* ------------------------------------------------------------------ main */

void ALIGN_APP_Main(void)
{
    CFE_Status_t     Status;
    CFE_SB_Buffer_t *Buf = NULL;
    CFE_SB_MsgId_t   MsgId;

    Status = ALIGN_APP_Init();
    if (Status != CFE_SUCCESS)
    {
        ALIGN_APP_Data.RunStatus = CFE_ES_RunStatus_APP_ERROR;
    }

    while (CFE_ES_RunLoop(&ALIGN_APP_Data.RunStatus))
    {
        Status = CFE_SB_ReceiveBuffer(&Buf, ALIGN_APP_Data.Pipe, ALIGN_APP_PIPE_TIMEOUT_MS);
        if (Status == CFE_SUCCESS)
        {
            MsgId = CFE_SB_INVALID_MSG_ID;
            CFE_MSG_GetMsgId(&Buf->Msg, &MsgId);
            if (!ALIGN_APP_ProcessInput(Buf, MsgId, OST_TimeMonoNs()))
            {
                ALIGN_APP_ProcessCommand(Buf, MsgId);
            }
        }
        else if (Status != CFE_SB_TIME_OUT && Status != CFE_SB_NO_MESSAGE)
        {
            CFE_EVS_SendEvent(ALIGN_APP_PIPE_ERR_EID, CFE_EVS_EventType_ERROR, "pipe read error 0x%08lX",
                              (unsigned long)Status);
            ALIGN_APP_Data.RunStatus = CFE_ES_RunStatus_APP_ERROR;
        }
        ALIGN_APP_Drain(OST_TimeMonoNs());
    }

    OST_Align_Destroy(ALIGN_APP_Data.Align);
    ALIGN_APP_Data.Align = NULL;
    CFE_ES_ExitApp(ALIGN_APP_Data.RunStatus);
}
