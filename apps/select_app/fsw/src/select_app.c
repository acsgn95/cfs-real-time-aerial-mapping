/*
 * SELECT_APP main task: initialisation, commands, housekeeping and the
 * hand-over of the input messages to the worker (select_app.h).
 */
#include "select_app.h"
#include "select_app_eventids.h"
#include "select_app_version.h"

#include "cfe_config.h"
#include "cfe_core_api_base_msgids.h"
#include "cfe_mission_eds_designparameters.h"
#include "ost_datastore.h"
#include "select_app_eds_dictionary.h"
#include "select_app_eds_dispatcher.h"

#include <math.h>
#include <string.h>

SELECT_APP_Data_t SELECT_APP_Data;

static const OST_FaultId_Enum_t SELECT_APP_FaultIds[SELECT_APP_FAULT_COUNT] = {
    OST_FaultId_TERRAIN_UNAVAILABLE, /* 60 */
    OST_FaultId_IMAGE_DECODE,        /* 61 */
    OST_FaultId_DATASTORE_QUOTA,     /* 62 */
    OST_FaultId_TABLE_REJECTED,      /* 63 */
    OST_FaultId_LIBRARY_ERROR,       /* 64 */
};

static CFE_EVS_BinFilter_t SELECT_APP_EventFilters[] = {
    {SELECT_APP_TERRAIN_ERR_EID, CFE_EVS_FIRST_8_STOP},  {SELECT_APP_DECODE_ERR_EID, CFE_EVS_FIRST_8_STOP},
    {SELECT_APP_QUOTA_ERR_EID, CFE_EVS_FIRST_8_STOP},    {SELECT_APP_QUEUE_FULL_EID, CFE_EVS_FIRST_8_STOP},
    {SELECT_APP_MSG_LEN_ERR_EID, CFE_EVS_FIRST_8_STOP},  {SELECT_APP_LIB_ERR_EID, CFE_EVS_FIRST_8_STOP},
    {SELECT_APP_DATASTORE_ERR_EID, CFE_EVS_FIRST_8_STOP},
};

/* clang-format off */
static const EdsDispatchTable_EdsComponent_SELECT_APP_Application_CFE_SB_Telecommand_t SELECT_APP_TC_DISPATCH_TABLE = {
    .CMD = {
        .NoopCmd_indication               = SELECT_APP_NoopCmd,
        .ResetCountersCmd_indication      = SELECT_APP_ResetCountersCmd,
        .ClearLatchedFaultsCmd_indication = SELECT_APP_ClearLatchedFaultsCmd,
    },
    .SEND_HK = { .indication = SELECT_APP_SendHkCmd },
};
/* clang-format on */

static CFE_SB_MsgId_t SELECT_APP_CmdMid(uint16 TopicId)
{
    return CFE_SB_ValueToMsgId(CFE_PLATFORM_CMD_TOPICID_TO_MIDV(TopicId));
}

static CFE_SB_MsgId_t SELECT_APP_TlmMid(uint16 TopicId)
{
    return CFE_SB_ValueToMsgId(CFE_PLATFORM_TLM_TOPICID_TO_MIDV(TopicId));
}

/* ---------------------------------------------------------------- faults */

void SELECT_APP_FaultReport(OST_FaultId_Enum_t Id, const char *Text)
{
    OS_MutSemTake(SELECT_APP_Data.FaultMutex);
    OST_FaultSet_Report(&SELECT_APP_Data.Faults, Id, Text);
    OS_MutSemGive(SELECT_APP_Data.FaultMutex);
}

void SELECT_APP_FaultRaise(OST_FaultId_Enum_t Id, const char *Text)
{
    OS_MutSemTake(SELECT_APP_Data.FaultMutex);
    OST_FaultSet_Raise(&SELECT_APP_Data.Faults, Id, Text);
    OS_MutSemGive(SELECT_APP_Data.FaultMutex);
}

void SELECT_APP_FaultResolve(OST_FaultId_Enum_t Id)
{
    OS_MutSemTake(SELECT_APP_Data.FaultMutex);
    OST_FaultSet_Resolve(&SELECT_APP_Data.Faults, Id);
    OS_MutSemGive(SELECT_APP_Data.FaultMutex);
}

void SELECT_APP_Release(const OST_ObjectRef_t *Ref)
{
    OST_Status_t Status;

    if (Ref->ObjectId == 0)
    {
        return;
    }
    Status = OST_DS_Release(Ref);
    if (Status != OST_SUCCESS)
    {
        CFE_EVS_SendEvent(SELECT_APP_DATASTORE_ERR_EID, CFE_EVS_EventType_ERROR, "object %lu/%lu not released: %s",
                          (unsigned long)Ref->ObjectId, (unsigned long)Ref->Generation, OST_StatusName(Status));
    }
}

/* ----------------------------------------------------------------- table */

static CFE_Status_t SELECT_APP_ValidateConfig(void *TblData)
{
    const SELECT_APP_ConfigTable_t *T = (const SELECT_APP_ConfigTable_t *)TblData;
    const float                     F[] = {T->NominalHeightAglM, T->MaxTiltDeg,         T->TargetOverlap,
                                           T->MinOverlap,        T->MinBaselineRatio,   T->MaxBaselineRatio,
                                           T->MaxViewAngleDeg,   T->PreferredTriangulationAngleDeg,
                                           T->LineTurnDeg,       T->PoolDistanceM,      T->TerrainRadiusM};
    uint32                          i;

    for (i = 0; i < sizeof(F) / sizeof(F[0]); ++i)
    {
        if (!isfinite(F[i]) || F[i] < 0.0f)
        {
            CFE_EVS_SendEvent(SELECT_APP_TBL_ERR_EID, CFE_EVS_EventType_ERROR,
                              "ConfigTable: value %lu not finite or negative", (unsigned long)i);
            return CFE_STATUS_VALIDATION_FAILURE;
        }
    }
    if (T->RefinePose > 1 || (T->WorkChannels != 1 && T->WorkChannels != 3) || T->WorkWidth < 64 ||
        T->WorkWidth > 8192 || T->MaxSources == 0 || T->MaxSources > EdsParam_OST_MAX_SOURCE_VIEWS ||
        T->MaxPoolImages == 0 || T->MaxPoolImages >= SELECT_APP_MAX_POOL || T->FootprintIterations > 20 ||
        T->TargetOverlap <= 0.0f || T->TargetOverlap >= 1.0f || T->MaxBaselineRatio <= T->MinBaselineRatio ||
        T->NominalHeightAglM <= 0.0f || T->PoolDistanceM <= 0.0f || T->TerrainRadiusM <= 0.0f ||
        T->DemPath[0] == '\0' || T->DemCrs[0] == '\0' || T->ProjDataDir[0] == '\0')
    {
        CFE_EVS_SendEvent(SELECT_APP_TBL_ERR_EID, CFE_EVS_EventType_ERROR,
                          "ConfigTable: limits invalid (sources 1-%u, pool below %u, channels 1 or 3, paths set)",
                          (unsigned)EdsParam_OST_MAX_SOURCE_VIEWS, (unsigned)SELECT_APP_MAX_POOL);
        return CFE_STATUS_VALIDATION_FAILURE;
    }
    return CFE_SUCCESS;
}

static void SELECT_APP_LoadConfig(void)
{
    CFE_Status_t Status;
    void        *Addr = NULL;

    Status = CFE_TBL_Register(&SELECT_APP_Data.TblHandle, "ConfigTable", sizeof(SELECT_APP_ConfigTable_t),
                              CFE_TBL_OPT_DEFAULT, SELECT_APP_ValidateConfig);
    if (Status == CFE_SUCCESS)
    {
        Status = CFE_TBL_Load(SELECT_APP_Data.TblHandle, CFE_TBL_SRC_FILE, SELECT_APP_TABLE_FILE);
    }
    if (Status == CFE_SUCCESS)
    {
        Status = CFE_TBL_GetAddress(&Addr, SELECT_APP_Data.TblHandle);
        if (Status == CFE_TBL_INFO_UPDATED)
        {
            Status = CFE_SUCCESS;
        }
    }
    if (Status == CFE_SUCCESS && Addr != NULL)
    {
        memcpy(&SELECT_APP_Data.Config, Addr, sizeof(SELECT_APP_Data.Config));
        CFE_TBL_ReleaseAddress(SELECT_APP_Data.TblHandle);
        SELECT_APP_Data.ConfigValid = true;
        return;
    }
    CFE_EVS_SendEvent(SELECT_APP_TBL_ERR_EID, CFE_EVS_EventType_ERROR,
                      "ConfigTable %s not loaded (0x%08lX); images are released without selection",
                      SELECT_APP_TABLE_FILE, (unsigned long)Status);
    SELECT_APP_FaultRaise(OST_FaultId_TABLE_REJECTED, "ConfigTable not loaded");
}

/* --------------------------------------------------------- input messages */

static bool SELECT_APP_CheckSize(const CFE_SB_Buffer_t *Buf, CFE_SB_MsgId_t MsgId, size_t Expected)
{
    CFE_MSG_Size_t Size = 0;

    CFE_MSG_GetSize(&Buf->Msg, &Size);
    if (Size == Expected)
    {
        return true;
    }
    ++SELECT_APP_Data.Counters.MessagesRejected;
    CFE_EVS_SendEvent(SELECT_APP_MSG_LEN_ERR_EID, CFE_EVS_EventType_ERROR, "input MID 0x%x: length %lu, expected %lu",
                      (unsigned)CFE_SB_MsgIdToValue(MsgId), (unsigned long)Size, (unsigned long)Expected);
    return false;
}

static bool SELECT_APP_Enqueue(const SELECT_APP_Item_t *Item)
{
    bool Put = false;

    OS_MutSemTake(SELECT_APP_Data.RingMutex);
    if (SELECT_APP_Data.QueuedItems - SELECT_APP_Data.TakenItems < SELECT_APP_QUEUE_DEPTH)
    {
        SELECT_APP_Data.Ring[SELECT_APP_Data.QueuedItems % SELECT_APP_QUEUE_DEPTH] = *Item;
        ++SELECT_APP_Data.QueuedItems;
        Put = true;
    }
    OS_MutSemGive(SELECT_APP_Data.RingMutex);
    if (Put)
    {
        OS_CountSemGive(SELECT_APP_Data.ItemSem);
    }
    return Put;
}

bool SELECT_APP_Dequeue(SELECT_APP_Item_t *Item, uint32 TimeoutMs)
{
    if (OS_CountSemTimedWait(SELECT_APP_Data.ItemSem, TimeoutMs) != OS_SUCCESS)
    {
        return false;
    }
    OS_MutSemTake(SELECT_APP_Data.RingMutex);
    *Item = SELECT_APP_Data.Ring[SELECT_APP_Data.TakenItems % SELECT_APP_QUEUE_DEPTH];
    ++SELECT_APP_Data.TakenItems;
    OS_MutSemGive(SELECT_APP_Data.RingMutex);
    return true;
}

static void SELECT_APP_OnImagePose(const ALIGN_APP_ImagePose_Payload_t *P, int64 NowMono)
{
    static SELECT_APP_Item_t Item; /* main task only */

    ++SELECT_APP_Data.Counters.ImagesReceived;
    if (P->Result != ALIGN_APP_AlignResult_VALID || !SELECT_APP_Data.ConfigValid)
    {
        ++SELECT_APP_Data.Counters.ImagesInvalidPose;
        SELECT_APP_Release(&P->Jpeg); /* empty for invalid poses */
        return;
    }
    memset(&Item, 0, sizeof(Item));
    Item.Type      = SELECT_APP_ITEM_IMAGE;
    Item.EntryMono = NowMono;
    Item.u.Image   = *P;
    if (!SELECT_APP_Enqueue(&Item))
    {
        ++SELECT_APP_Data.Counters.QueueOverflows;
        CFE_EVS_SendEvent(SELECT_APP_QUEUE_FULL_EID, CFE_EVS_EventType_ERROR,
                          "image %lu dropped: worker queue full", (unsigned long)P->ImageSeq);
        SELECT_APP_Release(&P->Jpeg);
    }
}

static void SELECT_APP_OnCalibration(const INGEST_APP_SessionCalibration_Payload_t *P, int64 NowMono)
{
    static SELECT_APP_Item_t Item;

    if (!SELECT_APP_Data.ConfigValid)
    {
        return;
    }
    memset(&Item, 0, sizeof(Item));
    Item.Type          = SELECT_APP_ITEM_CALIBRATION;
    Item.EntryMono     = NowMono;
    Item.u.Calibration = *P;
    if (!SELECT_APP_Enqueue(&Item))
    {
        ++SELECT_APP_Data.Counters.QueueOverflows;
        CFE_EVS_SendEvent(SELECT_APP_QUEUE_FULL_EID, CFE_EVS_EventType_ERROR,
                          "calibration of session %lu dropped: worker queue full", (unsigned long)P->SessionId);
    }
}

static bool SELECT_APP_ProcessInput(const CFE_SB_Buffer_t *Buf, CFE_SB_MsgId_t MsgId)
{
    if (CFE_SB_MsgId_Equal(MsgId, SELECT_APP_Data.ImagePoseMid))
    {
        if (SELECT_APP_CheckSize(Buf, MsgId, sizeof(ALIGN_APP_ImagePoseTlm_t)))
        {
            SELECT_APP_OnImagePose(&((const ALIGN_APP_ImagePoseTlm_t *)Buf)->Payload, OST_TimeMonoNs());
        }
        return true;
    }
    if (CFE_SB_MsgId_Equal(MsgId, SELECT_APP_Data.SessionCalibrationMid))
    {
        if (SELECT_APP_CheckSize(Buf, MsgId, sizeof(INGEST_APP_SessionCalibrationTlm_t)))
        {
            SELECT_APP_OnCalibration(&((const INGEST_APP_SessionCalibrationTlm_t *)Buf)->Payload, OST_TimeMonoNs());
        }
        return true;
    }
    return false;
}

/* ------------------------------------------------------------------ init */

static CFE_Status_t SELECT_APP_Init(void)
{
    CFE_Status_t    Status;
    CFE_ES_TaskId_t TaskId;
    char            VersionString[SELECT_APP_MAX_VERSION_STR_LEN];

    memset(&SELECT_APP_Data, 0, sizeof(SELECT_APP_Data));
    SELECT_APP_Data.RunStatus = CFE_ES_RunStatus_APP_RUN;

    Status = CFE_EVS_Register(SELECT_APP_EventFilters, sizeof(SELECT_APP_EventFilters) / sizeof(SELECT_APP_EventFilters[0]),
                              CFE_EVS_EventFilter_BINARY);
    if (Status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("SELECT_APP: EVS register failed 0x%08lX\n", (unsigned long)Status);
        return Status;
    }
    if (OS_MutSemCreate(&SELECT_APP_Data.FaultMutex, "SELECT_FLT", 0) != OS_SUCCESS ||
        OS_MutSemCreate(&SELECT_APP_Data.RingMutex, "SELECT_RING", 0) != OS_SUCCESS ||
        OS_CountSemCreate(&SELECT_APP_Data.ItemSem, "SELECT_ITEMS", 0, 0) != OS_SUCCESS)
    {
        CFE_EVS_SendEvent(SELECT_APP_INIT_ERR_EID, CFE_EVS_EventType_ERROR, "fault mutex or worker queue not created");
        return CFE_STATUS_EXTERNAL_RESOURCE_FAIL;
    }
    Status = OST_FaultSet_Init(&SELECT_APP_Data.Faults, OST_AppId_SELECT_APP, SELECT_APP_FaultIds,
                               SELECT_APP_FAULT_COUNT);
    if (Status != CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(SELECT_APP_INIT_ERR_EID, CFE_EVS_EventType_ERROR, "fault set: 0x%08lX", (unsigned long)Status);
        return Status;
    }

    CFE_MSG_Init(CFE_MSG_PTR(SELECT_APP_Data.HkTlm.TelemetryHeader),
                 SELECT_APP_TlmMid(EdsParam_CFE_MISSION_SELECT_APP_HK_TLM_TOPICID), sizeof(SELECT_APP_Data.HkTlm));
    SELECT_APP_Data.ImagePoseMid = SELECT_APP_TlmMid(EdsParam_CFE_MISSION_ALIGN_APP_IMAGE_POSE_TLM_TOPICID);
    SELECT_APP_Data.SessionCalibrationMid =
        SELECT_APP_TlmMid(EdsParam_CFE_MISSION_INGEST_APP_SESSION_CALIBRATION_TLM_TOPICID);

    Status = CFE_SB_CreatePipe(&SELECT_APP_Data.Pipe, SELECT_APP_PIPE_DEPTH, "SELECT_PIPE");
    if (Status == CFE_SUCCESS)
    {
        Status = CFE_SB_SubscribeEx(SELECT_APP_CmdMid(EdsParam_CFE_MISSION_SELECT_APP_CMD_TOPICID), SELECT_APP_Data.Pipe,
                                    CFE_SB_DEFAULT_QOS, 4);
    }
    if (Status == CFE_SUCCESS)
    {
        Status = CFE_SB_SubscribeEx(SELECT_APP_CmdMid(EdsParam_CFE_MISSION_SELECT_APP_SEND_HK_TOPICID),
                                    SELECT_APP_Data.Pipe, CFE_SB_DEFAULT_QOS, 2);
    }
    if (Status == CFE_SUCCESS)
    {
        /* An ImagePose lost to a full pipe would also lose its JPEG count. */
        Status = CFE_SB_SubscribeEx(SELECT_APP_Data.ImagePoseMid, SELECT_APP_Data.Pipe, CFE_SB_DEFAULT_QOS, 24);
    }
    if (Status == CFE_SUCCESS)
    {
        Status = CFE_SB_SubscribeEx(SELECT_APP_Data.SessionCalibrationMid, SELECT_APP_Data.Pipe, CFE_SB_DEFAULT_QOS, 2);
    }
    if (Status != CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(SELECT_APP_INIT_ERR_EID, CFE_EVS_EventType_ERROR, "pipe or subscription: 0x%08lX",
                          (unsigned long)Status);
        return Status;
    }

    SELECT_APP_LoadConfig();

    Status = CFE_ES_CreateChildTask(&TaskId, "SELECT_WORK", SELECT_APP_WorkerTask, NULL, SELECT_APP_WORKER_STACK_SIZE,
                                    SELECT_APP_WORKER_PRIORITY, 0);
    if (Status != CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(SELECT_APP_INIT_ERR_EID, CFE_EVS_EventType_ERROR, "worker task: 0x%08lX",
                          (unsigned long)Status);
        return Status;
    }

    CFE_Config_GetVersionString(VersionString, SELECT_APP_MAX_VERSION_STR_LEN, "SELECT_APP", SELECT_APP_VERSION,
                                SELECT_APP_BUILD_CODENAME, SELECT_APP_LAST_OFFICIAL);
    CFE_EVS_SendEvent(SELECT_APP_INIT_INF_EID, CFE_EVS_EventType_INFORMATION,
                      "SELECT_APP initialised: %u px %s, K %lu, batches to %s. %s",
                      (unsigned)SELECT_APP_Data.Config.WorkWidth, SELECT_APP_Data.Config.WorkChannels == 3 ? "RGB" : "grey",
                      (unsigned long)SELECT_APP_Data.Config.MaxSources,
                      SELECT_APP_Data.Config.RefinePose ? "REFINE_APP" : "DENSE_APP", VersionString);
    return CFE_SUCCESS;
}

/* -------------------------------------------------------------- commands */

static void SELECT_APP_ProcessCommand(const CFE_SB_Buffer_t *Buf, CFE_SB_MsgId_t MsgId)
{
    CFE_Status_t      Status;
    CFE_MSG_Size_t    Size = 0;
    CFE_MSG_FcnCode_t Fc   = 0;

    Status = EdsDispatch_EdsComponent_SELECT_APP_Application_Telecommand(Buf, &SELECT_APP_TC_DISPATCH_TABLE);
    if (Status == CFE_SUCCESS)
    {
        return;
    }
    CFE_MSG_GetSize(&Buf->Msg, &Size);
    CFE_MSG_GetFcnCode(&Buf->Msg, &Fc);
    ++SELECT_APP_Data.CommandErrorCounter;
    if (Status == CFE_STATUS_UNKNOWN_MSG_ID)
    {
        CFE_EVS_SendEvent(SELECT_APP_MID_ERR_EID, CFE_EVS_EventType_ERROR, "invalid message ID 0x%x",
                          (unsigned)CFE_SB_MsgIdToValue(MsgId));
    }
    else if (Status == CFE_STATUS_WRONG_MSG_LENGTH)
    {
        CFE_EVS_SendEvent(SELECT_APP_CMD_LEN_ERR_EID, CFE_EVS_EventType_ERROR,
                          "invalid length: MID 0x%x, CC %u, length %u", (unsigned)CFE_SB_MsgIdToValue(MsgId),
                          (unsigned)Fc, (unsigned)Size);
    }
    else
    {
        CFE_EVS_SendEvent(SELECT_APP_CC_ERR_EID, CFE_EVS_EventType_ERROR, "invalid command code %u", (unsigned)Fc);
    }
}

CFE_Status_t SELECT_APP_NoopCmd(const SELECT_APP_NoopCmd_t *Msg)
{
    (void)Msg;
    ++SELECT_APP_Data.CommandCounter;
    CFE_EVS_SendEvent(SELECT_APP_NOOP_INF_EID, CFE_EVS_EventType_INFORMATION, "no-op, version %s", SELECT_APP_VERSION);
    return CFE_SUCCESS;
}

CFE_Status_t SELECT_APP_ResetCountersCmd(const SELECT_APP_ResetCountersCmd_t *Msg)
{
    SELECT_APP_Counters_t *C = &SELECT_APP_Data.Counters;

    (void)Msg;
    SELECT_APP_Data.CommandCounter      = 0;
    SELECT_APP_Data.CommandErrorCounter = 0;
    /* Event counters only; the pool and session state are kept. A worker
     * update racing with this reset may survive it. */
    C->ImagesReceived = C->ImagesInvalidPose = C->QueueOverflows = C->MessagesRejected = 0;
    C->ImagesNoCalibration = C->ImagesNoFootprint = C->ImagesNoTerrain = C->ImagesDecoded = 0;
    C->DecodeErrors = C->QuotaFailures = C->Keyframes = C->BatchesPublished = 0;
    C->SourcesPublished = C->BatchesWithOtherLine = C->PoolEvicted = C->QuotaEvictions = 0;
    C->MaxDecodeMs = C->MaxSelectMs = 0;
    CFE_EVS_SendEvent(SELECT_APP_RESET_INF_EID, CFE_EVS_EventType_INFORMATION, "counters reset");
    return CFE_SUCCESS;
}

CFE_Status_t SELECT_APP_ClearLatchedFaultsCmd(const SELECT_APP_ClearLatchedFaultsCmd_t *Msg)
{
    uint32 Cleared;

    (void)Msg;
    OS_MutSemTake(SELECT_APP_Data.FaultMutex);
    Cleared = OST_FaultSet_ClearLatched(&SELECT_APP_Data.Faults);
    OS_MutSemGive(SELECT_APP_Data.FaultMutex);
    ++SELECT_APP_Data.CommandCounter;
    CFE_EVS_SendEvent(SELECT_APP_CLEAR_INF_EID, CFE_EVS_EventType_INFORMATION, "%lu latched faults cleared",
                      (unsigned long)Cleared);
    return CFE_SUCCESS;
}

CFE_Status_t SELECT_APP_SendHkCmd(const SELECT_APP_SendHkCmd_t *Msg)
{
    SELECT_APP_HkTlm_Payload_t  *P = &SELECT_APP_Data.HkTlm.Payload;
    const SELECT_APP_Counters_t *C = &SELECT_APP_Data.Counters;

    (void)Msg;
    P->Common.CommandCounter      = SELECT_APP_Data.CommandCounter;
    P->Common.CommandErrorCounter = SELECT_APP_Data.CommandErrorCounter;
    OS_MutSemTake(SELECT_APP_Data.FaultMutex);
    OST_FaultSet_Summarise(&SELECT_APP_Data.Faults, &P->Common);
    OS_MutSemGive(SELECT_APP_Data.FaultMutex);
    P->SessionId            = C->SessionId;
    P->Calibrated           = C->Calibrated ? 1 : 0;
    P->TerrainLoaded        = C->TerrainLoaded ? 1 : 0;
    P->RefinePose           = SELECT_APP_Data.Config.RefinePose;
    P->ImagesReceived       = C->ImagesReceived;
    P->ImagesInvalidPose    = C->ImagesInvalidPose;
    P->ImagesNoCalibration  = C->ImagesNoCalibration;
    P->ImagesNoFootprint    = C->ImagesNoFootprint;
    P->ImagesNoTerrain      = C->ImagesNoTerrain;
    P->ImagesDecoded        = C->ImagesDecoded;
    P->DecodeErrors         = C->DecodeErrors;
    P->QuotaFailures        = C->QuotaFailures;
    P->QueueOverflows       = C->QueueOverflows;
    P->Keyframes            = C->Keyframes;
    P->BatchesPublished     = C->BatchesPublished;
    P->SourcesPublished     = C->SourcesPublished;
    P->BatchesWithOtherLine = C->BatchesWithOtherLine;
    P->PoolImages           = C->PoolImages;
    P->PoolEvicted          = C->PoolEvicted;
    P->QuotaEvictions       = C->QuotaEvictions;
    P->FlightLines          = C->FlightLines;
    P->TerrainLoads         = C->TerrainLoads;
    P->QueueDepth           = SELECT_APP_Data.QueuedItems - SELECT_APP_Data.TakenItems;
    P->LastDecodeMs         = C->LastDecodeMs;
    P->MaxDecodeMs          = C->MaxDecodeMs;
    P->LastSelectMs         = C->LastSelectMs;
    P->MaxSelectMs          = C->MaxSelectMs;
    P->LastOverlap          = C->LastOverlap;

    CFE_SB_TimeStampMsg(CFE_MSG_PTR(SELECT_APP_Data.HkTlm.TelemetryHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(SELECT_APP_Data.HkTlm.TelemetryHeader), true);
    return CFE_SUCCESS;
}

/* ------------------------------------------------------------------ main */

void SELECT_APP_Main(void)
{
    CFE_Status_t     Status;
    CFE_SB_Buffer_t *Buf = NULL;
    CFE_SB_MsgId_t   MsgId;

    Status = SELECT_APP_Init();
    if (Status != CFE_SUCCESS)
    {
        SELECT_APP_Data.RunStatus = CFE_ES_RunStatus_APP_ERROR;
    }

    while (CFE_ES_RunLoop(&SELECT_APP_Data.RunStatus))
    {
        Status = CFE_SB_ReceiveBuffer(&Buf, SELECT_APP_Data.Pipe, SELECT_APP_PIPE_TIMEOUT_MS);
        if (Status == CFE_SUCCESS)
        {
            MsgId = CFE_SB_INVALID_MSG_ID;
            CFE_MSG_GetMsgId(&Buf->Msg, &MsgId);
            if (!SELECT_APP_ProcessInput(Buf, MsgId))
            {
                SELECT_APP_ProcessCommand(Buf, MsgId);
            }
        }
        else if (Status != CFE_SB_TIME_OUT && Status != CFE_SB_NO_MESSAGE)
        {
            CFE_EVS_SendEvent(SELECT_APP_PIPE_ERR_EID, CFE_EVS_EventType_ERROR, "pipe read error 0x%08lX",
                              (unsigned long)Status);
            SELECT_APP_Data.RunStatus = CFE_ES_RunStatus_APP_ERROR;
        }
    }

    /* The worker stops when it sees the run status (select_app_worker.c). */
    CFE_ES_ExitApp(SELECT_APP_Data.RunStatus);
}
