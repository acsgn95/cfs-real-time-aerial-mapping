/*
 * REFINE_APP main task: initialisation, commands, housekeeping, the worker
 * queue and batch publication (refine_app.h).
 */
#include "refine_app.h"
#include "refine_app_eventids.h"
#include "refine_app_version.h"

#include "cfe_config.h"
#include "cfe_core_api_base_msgids.h"
#include "cfe_mission_eds_designparameters.h"
#include "ost_datastore.h"
#include "refine_app_eds_dictionary.h"
#include "refine_app_eds_dispatcher.h"

#include <math.h>
#include <string.h>

REFINE_APP_Data_t REFINE_APP_Data;

static const OST_FaultId_Enum_t REFINE_APP_FaultIds[REFINE_APP_FAULT_COUNT] = {
    OST_FaultId_REFINE_REJECTED, /* 60 */
    OST_FaultId_REFINE_LATENCY,  /* 61 */
    OST_FaultId_TABLE_REJECTED,  /* 62 */
    OST_FaultId_LIBRARY_ERROR,   /* 63 */
};

static CFE_EVS_BinFilter_t REFINE_APP_EventFilters[] = {
    {REFINE_APP_FAILED_EID, CFE_EVS_FIRST_8_STOP},       {REFINE_APP_LATENCY_EID, CFE_EVS_FIRST_8_STOP},
    {REFINE_APP_QUEUE_FULL_EID, CFE_EVS_FIRST_8_STOP},   {REFINE_APP_MSG_LEN_ERR_EID, CFE_EVS_FIRST_8_STOP},
    {REFINE_APP_LIB_ERR_EID, CFE_EVS_FIRST_8_STOP},      {REFINE_APP_DATASTORE_ERR_EID, CFE_EVS_FIRST_8_STOP},
    {REFINE_APP_HELD_FULL_EID, CFE_EVS_FIRST_8_STOP},
};

/* clang-format off */
static const EdsDispatchTable_EdsComponent_REFINE_APP_Application_CFE_SB_Telecommand_t REFINE_APP_TC_DISPATCH_TABLE = {
    .CMD = {
        .NoopCmd_indication               = REFINE_APP_NoopCmd,
        .ResetCountersCmd_indication      = REFINE_APP_ResetCountersCmd,
        .ClearLatchedFaultsCmd_indication = REFINE_APP_ClearLatchedFaultsCmd,
        .FlushCmd_indication              = REFINE_APP_FlushCmd,
    },
    .SEND_HK = { .indication = REFINE_APP_SendHkCmd },
};
/* clang-format on */

static OST_KeyframeBatchTlm_t REFINE_APP_OutMsg; /* under PublishMutex */

static CFE_SB_MsgId_t REFINE_APP_CmdMid(uint16 TopicId)
{
    return CFE_SB_ValueToMsgId(CFE_PLATFORM_CMD_TOPICID_TO_MIDV(TopicId));
}

static CFE_SB_MsgId_t REFINE_APP_TlmMid(uint16 TopicId)
{
    return CFE_SB_ValueToMsgId(CFE_PLATFORM_TLM_TOPICID_TO_MIDV(TopicId));
}

/* -------------------------------------------------------- shared helpers */

void REFINE_APP_FaultReport(OST_FaultId_Enum_t Id, const char *Text)
{
    OS_MutSemTake(REFINE_APP_Data.FaultMutex);
    OST_FaultSet_Report(&REFINE_APP_Data.Faults, Id, Text);
    OS_MutSemGive(REFINE_APP_Data.FaultMutex);
}

void REFINE_APP_FaultRaise(OST_FaultId_Enum_t Id, const char *Text)
{
    OS_MutSemTake(REFINE_APP_Data.FaultMutex);
    OST_FaultSet_Raise(&REFINE_APP_Data.Faults, Id, Text);
    OS_MutSemGive(REFINE_APP_Data.FaultMutex);
}

void REFINE_APP_Release(const OST_ObjectRef_t *Ref)
{
    OST_Status_t Status;

    if (Ref->ObjectId == 0)
    {
        return;
    }
    Status = OST_DS_Release(Ref);
    if (Status != OST_SUCCESS)
    {
        CFE_EVS_SendEvent(REFINE_APP_DATASTORE_ERR_EID, CFE_EVS_EventType_ERROR, "object %lu/%lu not released: %s",
                          (unsigned long)Ref->ObjectId, (unsigned long)Ref->Generation, OST_StatusName(Status));
    }
}

static void REFINE_APP_ReleaseBatch(const OST_KeyframeBatch_Payload_t *B)
{
    uint32 i;

    REFINE_APP_Release(&B->Keyframe.WorkImage);
    REFINE_APP_Release(&B->Keyframe.Jpeg);
    for (i = 0; i < B->SourceCount && i < EdsParam_OST_MAX_SOURCE_VIEWS; ++i)
    {
        REFINE_APP_Release(&B->Sources[i].WorkImage);
        REFINE_APP_Release(&B->Sources[i].Jpeg);
    }
}

bool REFINE_APP_PublishBatch(const OST_KeyframeBatch_Payload_t *Batch, int64 EntryMono)
{
    static uint32          Sequence = 0;
    REFINE_APP_Counters_t *C        = &REFINE_APP_Data.Counters;
    OST_KeyframeBatch_Payload_t *P  = &REFINE_APP_OutMsg.Payload;
    float                  Latency;
    bool                   Sent;

    OS_MutSemTake(REFINE_APP_Data.PublishMutex);
    CFE_MSG_Init(CFE_MSG_PTR(REFINE_APP_OutMsg.TelemetryHeader), OST_BatchReadyMsgId(), sizeof(REFINE_APP_OutMsg));
    *P                   = *Batch;
    P->Header.Sequence   = ++Sequence;
    P->Trace.EntryMono   = EntryMono;
    P->Trace.ExitMono    = OST_TimeMonoNs();
    P->Trace.PublishUtc  = OST_TimeUtcNs();
    Latency              = (float)((double)(P->Trace.PublishUtc - P->Keyframe.CaptureUtc) * 1e-9);
    CFE_SB_TimeStampMsg(CFE_MSG_PTR(REFINE_APP_OutMsg.TelemetryHeader));
    Sent = CFE_SB_TransmitMsg(CFE_MSG_PTR(REFINE_APP_OutMsg.TelemetryHeader), true) == CFE_SUCCESS;
    if (!Sent)
    {
        REFINE_APP_ReleaseBatch(Batch); /* nobody owns the counts */
    }
    else
    {
        ++C->BatchesPublished;
        if (Batch->PoseSource == OST_PoseSource_REFINE_FAILED)
        {
            ++C->BatchesFailed;
        }
    }
    C->LastLatencyS = Latency;
    if (Latency > C->MaxLatencyObservedS)
    {
        C->MaxLatencyObservedS = Latency;
    }
    OS_MutSemGive(REFINE_APP_Data.PublishMutex);

    if (REFINE_APP_Data.ConfigValid && Latency > REFINE_APP_Data.Config.MaxLatencyS)
    {
        ++C->LatencyExceeded;
        CFE_EVS_SendEvent(REFINE_APP_LATENCY_EID, CFE_EVS_EventType_ERROR,
                          "keyframe %lu published %.1f s after its capture (limit %.0f s)",
                          (unsigned long)Batch->KeyframeId, Latency, REFINE_APP_Data.Config.MaxLatencyS);
        REFINE_APP_FaultReport(OST_FaultId_REFINE_LATENCY, "refinement over its latency limit");
    }
    return Sent;
}

/* ------------------------------------------------------------- the queue */

static bool REFINE_APP_Enqueue(const REFINE_APP_Item_t *Item)
{
    bool Put = false;

    OS_MutSemTake(REFINE_APP_Data.RingMutex);
    if (REFINE_APP_Data.QueuedItems - REFINE_APP_Data.TakenItems < REFINE_APP_QUEUE_DEPTH)
    {
        REFINE_APP_Data.Ring[REFINE_APP_Data.QueuedItems % REFINE_APP_QUEUE_DEPTH] = *Item;
        ++REFINE_APP_Data.QueuedItems;
        Put = true;
    }
    OS_MutSemGive(REFINE_APP_Data.RingMutex);
    if (Put)
    {
        OS_CountSemGive(REFINE_APP_Data.ItemSem);
    }
    return Put;
}

bool REFINE_APP_Dequeue(REFINE_APP_Item_t *Item, uint32 TimeoutMs)
{
    if (OS_CountSemTimedWait(REFINE_APP_Data.ItemSem, TimeoutMs) != OS_SUCCESS)
    {
        return false;
    }
    OS_MutSemTake(REFINE_APP_Data.RingMutex);
    *Item = REFINE_APP_Data.Ring[REFINE_APP_Data.TakenItems % REFINE_APP_QUEUE_DEPTH];
    ++REFINE_APP_Data.TakenItems;
    OS_MutSemGive(REFINE_APP_Data.RingMutex);
    return true;
}

/* ----------------------------------------------------------------- table */

static CFE_Status_t REFINE_APP_ValidateConfig(void *TblData)
{
    const REFINE_APP_ConfigTable_t *T   = (const REFINE_APP_ConfigTable_t *)TblData;
    const float                     F[] = {T->MaxLatencyS,           T->MaxWaitS,          T->WindowS,
                                           T->PositionSigmaM,        T->PrecisePositionSigmaM, T->AttitudeSigmaDeg,
                                           T->BoresightSigmaDeg,     T->HeldBoresightSigmaDeg, T->RefinedAttitudeSigmaDeg,
                                           T->MaxReprojectionRmsPx,  T->MaxCorrectionSigmas};
    uint32                          i;

    for (i = 0; i < sizeof(F) / sizeof(F[0]); ++i)
    {
        if (!isfinite(F[i]) || F[i] < 0.0f)
        {
            CFE_EVS_SendEvent(REFINE_APP_TBL_ERR_EID, CFE_EVS_EventType_ERROR,
                              "ConfigTable: value %lu not finite or negative", (unsigned long)i);
            return CFE_STATUS_VALIDATION_FAILURE;
        }
    }
    if (T->PrecisePosition > 1 || T->EstimateBoresight > 1 || T->WindowS <= T->MaxWaitS ||
        T->MaxWindowImages < 2 || T->MaxWindowImages > EdsParam_OST_MAX_SOURCE_VIEWS + 64 || T->MaxPoints == 0 ||
        T->MaxIterations == 0 || T->MaxFeatures == 0)
    {
        CFE_EVS_SendEvent(REFINE_APP_TBL_ERR_EID, CFE_EVS_EventType_ERROR,
                          "ConfigTable: limits invalid (WindowS > MaxWaitS, window >= 2 images, counts > 0)");
        return CFE_STATUS_VALIDATION_FAILURE;
    }
    return CFE_SUCCESS;
}

static void REFINE_APP_LoadConfig(void)
{
    CFE_Status_t Status;
    void        *Addr = NULL;

    Status = CFE_TBL_Register(&REFINE_APP_Data.TblHandle, "ConfigTable", sizeof(REFINE_APP_ConfigTable_t),
                              CFE_TBL_OPT_DEFAULT, REFINE_APP_ValidateConfig);
    if (Status == CFE_SUCCESS)
    {
        Status = CFE_TBL_Load(REFINE_APP_Data.TblHandle, CFE_TBL_SRC_FILE, REFINE_APP_TABLE_FILE);
    }
    if (Status == CFE_SUCCESS)
    {
        Status = CFE_TBL_GetAddress(&Addr, REFINE_APP_Data.TblHandle);
        if (Status == CFE_TBL_INFO_UPDATED)
        {
            Status = CFE_SUCCESS;
        }
    }
    if (Status == CFE_SUCCESS && Addr != NULL)
    {
        memcpy(&REFINE_APP_Data.Config, Addr, sizeof(REFINE_APP_Data.Config));
        CFE_TBL_ReleaseAddress(REFINE_APP_Data.TblHandle);
        REFINE_APP_Data.ConfigValid = true;
        return;
    }
    CFE_EVS_SendEvent(REFINE_APP_TBL_ERR_EID, CFE_EVS_EventType_ERROR,
                      "ConfigTable %s not loaded (0x%08lX); batches are passed on unrefined", REFINE_APP_TABLE_FILE,
                      (unsigned long)Status);
    REFINE_APP_FaultRaise(OST_FaultId_TABLE_REJECTED, "ConfigTable not loaded");
}

/* --------------------------------------------------------- input messages */

static bool REFINE_APP_CheckSize(const CFE_SB_Buffer_t *Buf, CFE_SB_MsgId_t MsgId, size_t Expected)
{
    CFE_MSG_Size_t Size = 0;

    CFE_MSG_GetSize(&Buf->Msg, &Size);
    if (Size == Expected)
    {
        return true;
    }
    ++REFINE_APP_Data.Counters.MessagesRejected;
    CFE_EVS_SendEvent(REFINE_APP_MSG_LEN_ERR_EID, CFE_EVS_EventType_ERROR, "input MID 0x%x: length %lu, expected %lu",
                      (unsigned)CFE_SB_MsgIdToValue(MsgId), (unsigned long)Size, (unsigned long)Expected);
    return false;
}

/* A batch that cannot be refined is passed on with its telemetry poses. */
static void REFINE_APP_PassOn(const OST_KeyframeBatch_Payload_t *Batch, int64 EntryMono, const char *Reason)
{
    static OST_KeyframeBatch_Payload_t Copy; /* main task only */

    Copy            = *Batch;
    Copy.PoseSource = OST_PoseSource_REFINE_FAILED;
    CFE_EVS_SendEvent(REFINE_APP_QUEUE_FULL_EID, CFE_EVS_EventType_ERROR, "keyframe %lu passed on unrefined: %s",
                      (unsigned long)Batch->KeyframeId, Reason);
    REFINE_APP_FaultReport(OST_FaultId_REFINE_REJECTED, Reason);
    REFINE_APP_PublishBatch(&Copy, EntryMono);
}

static bool REFINE_APP_ProcessInput(const CFE_SB_Buffer_t *Buf, CFE_SB_MsgId_t MsgId)
{
    static REFINE_APP_Item_t Item; /* main task only */
    const int64              Now = OST_TimeMonoNs();

    if (CFE_SB_MsgId_Equal(MsgId, REFINE_APP_Data.BatchSelectedMid))
    {
        const OST_KeyframeBatch_Payload_t *B = &((const OST_KeyframeBatchTlm_t *)Buf)->Payload;
        if (!REFINE_APP_CheckSize(Buf, MsgId, sizeof(OST_KeyframeBatchTlm_t)))
        {
            return true; /* its references cannot be trusted */
        }
        ++REFINE_APP_Data.Counters.BatchesReceived;
        if (!REFINE_APP_Data.ConfigValid)
        {
            REFINE_APP_PassOn(B, Now, "no configuration table");
            return true;
        }
        memset(&Item, 0, sizeof(Item));
        Item.Type      = REFINE_APP_ITEM_BATCH;
        Item.EntryMono = Now;
        Item.Batch     = *B;
        if (!REFINE_APP_Enqueue(&Item))
        {
            ++REFINE_APP_Data.Counters.QueueOverflows;
            REFINE_APP_PassOn(B, Now, "worker queue full");
        }
        return true;
    }
    if (CFE_SB_MsgId_Equal(MsgId, REFINE_APP_Data.SessionCalibrationMid))
    {
        if (REFINE_APP_CheckSize(Buf, MsgId, sizeof(INGEST_APP_SessionCalibrationTlm_t)))
        {
            memset(&Item, 0, sizeof(Item));
            Item.Type      = REFINE_APP_ITEM_SESSION;
            Item.EntryMono = Now;
            Item.SessionId = ((const INGEST_APP_SessionCalibrationTlm_t *)Buf)->Payload.SessionId;
            if (!REFINE_APP_Enqueue(&Item))
            {
                ++REFINE_APP_Data.Counters.QueueOverflows;
            }
        }
        return true;
    }
    return false;
}

/* ------------------------------------------------------------------ init */

static CFE_Status_t REFINE_APP_Init(void)
{
    CFE_Status_t    Status;
    CFE_ES_TaskId_t TaskId;
    char            VersionString[REFINE_APP_MAX_VERSION_STR_LEN];

    memset(&REFINE_APP_Data, 0, sizeof(REFINE_APP_Data));
    REFINE_APP_Data.RunStatus = CFE_ES_RunStatus_APP_RUN;

    Status = CFE_EVS_Register(REFINE_APP_EventFilters, sizeof(REFINE_APP_EventFilters) / sizeof(REFINE_APP_EventFilters[0]),
                              CFE_EVS_EventFilter_BINARY);
    if (Status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("REFINE_APP: EVS register failed 0x%08lX\n", (unsigned long)Status);
        return Status;
    }
    if (OS_MutSemCreate(&REFINE_APP_Data.FaultMutex, "REFINE_FLT", 0) != OS_SUCCESS ||
        OS_MutSemCreate(&REFINE_APP_Data.PublishMutex, "REFINE_PUB", 0) != OS_SUCCESS ||
        OS_MutSemCreate(&REFINE_APP_Data.RingMutex, "REFINE_RING", 0) != OS_SUCCESS ||
        OS_CountSemCreate(&REFINE_APP_Data.ItemSem, "REFINE_ITEMS", 0, 0) != OS_SUCCESS)
    {
        CFE_EVS_SendEvent(REFINE_APP_INIT_ERR_EID, CFE_EVS_EventType_ERROR, "mutexes or worker queue not created");
        return CFE_STATUS_EXTERNAL_RESOURCE_FAIL;
    }
    Status = OST_FaultSet_Init(&REFINE_APP_Data.Faults, OST_AppId_REFINE_APP, REFINE_APP_FaultIds,
                               REFINE_APP_FAULT_COUNT);
    if (Status != CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(REFINE_APP_INIT_ERR_EID, CFE_EVS_EventType_ERROR, "fault set: 0x%08lX", (unsigned long)Status);
        return Status;
    }

    CFE_MSG_Init(CFE_MSG_PTR(REFINE_APP_Data.HkTlm.TelemetryHeader),
                 REFINE_APP_TlmMid(EdsParam_CFE_MISSION_REFINE_APP_HK_TLM_TOPICID), sizeof(REFINE_APP_Data.HkTlm));
    REFINE_APP_Data.BatchSelectedMid = OST_BatchSelectedMsgId();
    REFINE_APP_Data.SessionCalibrationMid =
        REFINE_APP_TlmMid(EdsParam_CFE_MISSION_INGEST_APP_SESSION_CALIBRATION_TLM_TOPICID);

    Status = CFE_SB_CreatePipe(&REFINE_APP_Data.Pipe, REFINE_APP_PIPE_DEPTH, "REFINE_PIPE");
    if (Status == CFE_SUCCESS)
    {
        Status = CFE_SB_SubscribeEx(REFINE_APP_CmdMid(EdsParam_CFE_MISSION_REFINE_APP_CMD_TOPICID), REFINE_APP_Data.Pipe,
                                    CFE_SB_DEFAULT_QOS, 4);
    }
    if (Status == CFE_SUCCESS)
    {
        Status = CFE_SB_SubscribeEx(REFINE_APP_CmdMid(EdsParam_CFE_MISSION_REFINE_APP_SEND_HK_TOPICID),
                                    REFINE_APP_Data.Pipe, CFE_SB_DEFAULT_QOS, 2);
    }
    if (Status == CFE_SUCCESS)
    {
        /* A batch lost to a full pipe would also lose its counts. */
        Status = CFE_SB_SubscribeEx(REFINE_APP_Data.BatchSelectedMid, REFINE_APP_Data.Pipe, CFE_SB_DEFAULT_QOS, 24);
    }
    if (Status == CFE_SUCCESS)
    {
        Status = CFE_SB_SubscribeEx(REFINE_APP_Data.SessionCalibrationMid, REFINE_APP_Data.Pipe, CFE_SB_DEFAULT_QOS, 2);
    }
    if (Status != CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(REFINE_APP_INIT_ERR_EID, CFE_EVS_EventType_ERROR, "pipe or subscription: 0x%08lX",
                          (unsigned long)Status);
        return Status;
    }

    REFINE_APP_LoadConfig();

    Status = CFE_ES_CreateChildTask(&TaskId, "REFINE_WORK", REFINE_APP_WorkerTask, NULL, REFINE_APP_WORKER_STACK_SIZE,
                                    REFINE_APP_WORKER_PRIORITY, 0);
    if (Status != CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(REFINE_APP_INIT_ERR_EID, CFE_EVS_EventType_ERROR, "worker task: 0x%08lX",
                          (unsigned long)Status);
        return Status;
    }

    CFE_Config_GetVersionString(VersionString, REFINE_APP_MAX_VERSION_STR_LEN, "REFINE_APP", REFINE_APP_VERSION,
                                REFINE_APP_BUILD_CODENAME, REFINE_APP_LAST_OFFICIAL);
    CFE_EVS_SendEvent(REFINE_APP_INIT_INF_EID, CFE_EVS_EventType_INFORMATION,
                      "REFINE_APP initialised: precise position %u, window %lu images, wait %.0f s, latency limit %.0f s. %s",
                      (unsigned)REFINE_APP_Data.Config.PrecisePosition,
                      (unsigned long)REFINE_APP_Data.Config.MaxWindowImages, REFINE_APP_Data.Config.MaxWaitS,
                      REFINE_APP_Data.Config.MaxLatencyS, VersionString);
    return CFE_SUCCESS;
}

/* -------------------------------------------------------------- commands */

static void REFINE_APP_ProcessCommand(const CFE_SB_Buffer_t *Buf, CFE_SB_MsgId_t MsgId)
{
    CFE_Status_t      Status;
    CFE_MSG_Size_t    Size = 0;
    CFE_MSG_FcnCode_t Fc   = 0;

    Status = EdsDispatch_EdsComponent_REFINE_APP_Application_Telecommand(Buf, &REFINE_APP_TC_DISPATCH_TABLE);
    if (Status == CFE_SUCCESS)
    {
        return;
    }
    CFE_MSG_GetSize(&Buf->Msg, &Size);
    CFE_MSG_GetFcnCode(&Buf->Msg, &Fc);
    ++REFINE_APP_Data.CommandErrorCounter;
    if (Status == CFE_STATUS_UNKNOWN_MSG_ID)
    {
        CFE_EVS_SendEvent(REFINE_APP_MID_ERR_EID, CFE_EVS_EventType_ERROR, "invalid message ID 0x%x",
                          (unsigned)CFE_SB_MsgIdToValue(MsgId));
    }
    else if (Status == CFE_STATUS_WRONG_MSG_LENGTH)
    {
        CFE_EVS_SendEvent(REFINE_APP_CMD_LEN_ERR_EID, CFE_EVS_EventType_ERROR,
                          "invalid length: MID 0x%x, CC %u, length %u", (unsigned)CFE_SB_MsgIdToValue(MsgId),
                          (unsigned)Fc, (unsigned)Size);
    }
    else
    {
        CFE_EVS_SendEvent(REFINE_APP_CC_ERR_EID, CFE_EVS_EventType_ERROR, "invalid command code %u", (unsigned)Fc);
    }
}

CFE_Status_t REFINE_APP_NoopCmd(const REFINE_APP_NoopCmd_t *Msg)
{
    (void)Msg;
    ++REFINE_APP_Data.CommandCounter;
    CFE_EVS_SendEvent(REFINE_APP_NOOP_INF_EID, CFE_EVS_EventType_INFORMATION, "no-op, version %s", REFINE_APP_VERSION);
    return CFE_SUCCESS;
}

CFE_Status_t REFINE_APP_ResetCountersCmd(const REFINE_APP_ResetCountersCmd_t *Msg)
{
    REFINE_APP_Counters_t *C = &REFINE_APP_Data.Counters;

    (void)Msg;
    REFINE_APP_Data.CommandCounter      = 0;
    REFINE_APP_Data.CommandErrorCounter = 0;
    C->BatchesReceived = C->QueueOverflows = C->MessagesRejected = 0;
    C->BatchesPublished = C->BatchesRefined = C->BatchesFailed = C->BatchesCovered = C->BatchesWaitLimit = 0;
    C->LatencyExceeded = C->ImagesAdded = C->MaxSolveMs = 0;
    C->MaxLatencyObservedS = 0.0f;
    CFE_EVS_SendEvent(REFINE_APP_RESET_INF_EID, CFE_EVS_EventType_INFORMATION, "counters reset");
    return CFE_SUCCESS;
}

CFE_Status_t REFINE_APP_ClearLatchedFaultsCmd(const REFINE_APP_ClearLatchedFaultsCmd_t *Msg)
{
    uint32 Cleared;

    (void)Msg;
    OS_MutSemTake(REFINE_APP_Data.FaultMutex);
    Cleared = OST_FaultSet_ClearLatched(&REFINE_APP_Data.Faults);
    OS_MutSemGive(REFINE_APP_Data.FaultMutex);
    ++REFINE_APP_Data.CommandCounter;
    CFE_EVS_SendEvent(REFINE_APP_CLEAR_INF_EID, CFE_EVS_EventType_INFORMATION, "%lu latched faults cleared",
                      (unsigned long)Cleared);
    return CFE_SUCCESS;
}

CFE_Status_t REFINE_APP_FlushCmd(const REFINE_APP_FlushCmd_t *Msg)
{
    static REFINE_APP_Item_t Item;

    (void)Msg;
    memset(&Item, 0, sizeof(Item));
    Item.Type      = REFINE_APP_ITEM_FLUSH;
    Item.EntryMono = OST_TimeMonoNs();
    if (!REFINE_APP_Enqueue(&Item))
    {
        ++REFINE_APP_Data.CommandErrorCounter;
        CFE_EVS_SendEvent(REFINE_APP_FLUSH_INF_EID, CFE_EVS_EventType_ERROR, "flush refused: worker queue full");
        return CFE_SUCCESS;
    }
    ++REFINE_APP_Data.CommandCounter;
    CFE_EVS_SendEvent(REFINE_APP_FLUSH_INF_EID, CFE_EVS_EventType_INFORMATION, "flush: waiting batches are solved now");
    return CFE_SUCCESS;
}

CFE_Status_t REFINE_APP_SendHkCmd(const REFINE_APP_SendHkCmd_t *Msg)
{
    REFINE_APP_HkTlm_Payload_t  *P = &REFINE_APP_Data.HkTlm.Payload;
    const REFINE_APP_Counters_t *C = &REFINE_APP_Data.Counters;

    (void)Msg;
    P->Common.CommandCounter      = REFINE_APP_Data.CommandCounter;
    P->Common.CommandErrorCounter = REFINE_APP_Data.CommandErrorCounter;
    OS_MutSemTake(REFINE_APP_Data.FaultMutex);
    OST_FaultSet_Summarise(&REFINE_APP_Data.Faults, &P->Common);
    OS_MutSemGive(REFINE_APP_Data.FaultMutex);
    P->PrecisePosition     = REFINE_APP_Data.Config.PrecisePosition;
    P->BoresightHeld       = C->BoresightHeld ? 1 : 0;
    P->Ready               = C->Ready ? 1 : 0;
    P->BatchesReceived     = C->BatchesReceived;
    P->BatchesPublished    = C->BatchesPublished;
    P->BatchesRefined      = C->BatchesRefined;
    P->BatchesFailed       = C->BatchesFailed;
    P->BatchesCovered      = C->BatchesCovered;
    P->BatchesWaitLimit    = C->BatchesWaitLimit;
    P->BatchesHeld         = C->BatchesHeld;
    P->LatencyExceeded     = C->LatencyExceeded;
    P->QueueOverflows      = C->QueueOverflows;
    P->MessagesRejected    = C->MessagesRejected;
    P->ImagesAdded         = C->ImagesAdded;
    P->WindowImages        = C->WindowImages;
    P->QueueDepth          = REFINE_APP_Data.QueuedItems - REFINE_APP_Data.TakenItems;
    P->LastPoints          = C->LastPoints;
    P->LastIterations      = C->LastIterations;
    P->LastWindowImages    = C->LastWindowImages;
    P->LastWaitedS         = C->LastWaitedS;
    P->LastLatencyS        = C->LastLatencyS;
    P->MaxLatencyObservedS = C->MaxLatencyObservedS;
    P->LastRmsPx           = C->LastRmsPx;
    P->LastBoresightDeg    = C->LastBoresightDeg;
    P->LastCorrectionDeg   = C->LastCorrectionDeg;
    P->LastSolveMs         = C->LastSolveMs;
    P->MaxSolveMs          = C->MaxSolveMs;
    P->LastFeaturesMs      = C->LastFeaturesMs;
    P->FeaturesTotalS      = C->FeaturesTotalS;

    CFE_SB_TimeStampMsg(CFE_MSG_PTR(REFINE_APP_Data.HkTlm.TelemetryHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(REFINE_APP_Data.HkTlm.TelemetryHeader), true);
    return CFE_SUCCESS;
}

/* ------------------------------------------------------------------ main */

void REFINE_APP_Main(void)
{
    CFE_Status_t     Status;
    CFE_SB_Buffer_t *Buf = NULL;
    CFE_SB_MsgId_t   MsgId;

    Status = REFINE_APP_Init();
    if (Status != CFE_SUCCESS)
    {
        REFINE_APP_Data.RunStatus = CFE_ES_RunStatus_APP_ERROR;
    }

    while (CFE_ES_RunLoop(&REFINE_APP_Data.RunStatus))
    {
        Status = CFE_SB_ReceiveBuffer(&Buf, REFINE_APP_Data.Pipe, REFINE_APP_PIPE_TIMEOUT_MS);
        if (Status == CFE_SUCCESS)
        {
            MsgId = CFE_SB_INVALID_MSG_ID;
            CFE_MSG_GetMsgId(&Buf->Msg, &MsgId);
            if (!REFINE_APP_ProcessInput(Buf, MsgId))
            {
                REFINE_APP_ProcessCommand(Buf, MsgId);
            }
        }
        else if (Status != CFE_SB_TIME_OUT && Status != CFE_SB_NO_MESSAGE)
        {
            CFE_EVS_SendEvent(REFINE_APP_PIPE_ERR_EID, CFE_EVS_EventType_ERROR, "pipe read error 0x%08lX",
                              (unsigned long)Status);
            REFINE_APP_Data.RunStatus = CFE_ES_RunStatus_APP_ERROR;
        }
    }
    CFE_ES_ExitApp(REFINE_APP_Data.RunStatus);
}
