/*
 * DENSE_APP main task: initialisation, commands, housekeeping, the worker
 * queue with its drop rule, and DENSE_STATUS (dense_app.h).
 */
#include "dense_app.h"
#include "dense_app_eventids.h"
#include "dense_app_version.h"

#include "cfe_config.h"
#include "cfe_core_api_base_msgids.h"
#include "cfe_mission_eds_designparameters.h"
#include "dense_app_eds_dictionary.h"
#include "dense_app_eds_dispatcher.h"
#include "ost_datastore.h"

#include <math.h>
#include <string.h>

DENSE_APP_Data_t DENSE_APP_Data;

static const OST_FaultId_Enum_t DENSE_APP_FaultIds[DENSE_APP_FAULT_COUNT] = {
    OST_FaultId_KEYFRAMES_SKIPPED,   /* 60 */
    OST_FaultId_DEPTH_FAILED,        /* 61 */
    OST_FaultId_GPU_FAILURE,         /* 62 */
    OST_FaultId_DATASTORE_QUOTA,     /* 63 */
    OST_FaultId_TERRAIN_UNAVAILABLE, /* 64 */
    OST_FaultId_TABLE_REJECTED,      /* 65 */
    OST_FaultId_LIBRARY_ERROR,       /* 66 */
};

static CFE_EVS_BinFilter_t DENSE_APP_EventFilters[] = {
    {DENSE_APP_SKIPPED_EID, CFE_EVS_FIRST_8_STOP},       {DENSE_APP_FAILED_EID, CFE_EVS_FIRST_8_STOP},
    {DENSE_APP_QUOTA_EID, CFE_EVS_FIRST_8_STOP},         {DENSE_APP_MSG_LEN_ERR_EID, CFE_EVS_FIRST_8_STOP},
    {DENSE_APP_LIB_ERR_EID, CFE_EVS_FIRST_8_STOP},       {DENSE_APP_DATASTORE_ERR_EID, CFE_EVS_FIRST_8_STOP},
    {DENSE_APP_TERRAIN_EID, CFE_EVS_FIRST_8_STOP},       {DENSE_APP_GPU_EID, CFE_EVS_FIRST_8_STOP},
};

/* clang-format off */
static const EdsDispatchTable_EdsComponent_DENSE_APP_Application_CFE_SB_Telecommand_t DENSE_APP_TC_DISPATCH_TABLE = {
    .CMD = {
        .NoopCmd_indication               = DENSE_APP_NoopCmd,
        .ResetCountersCmd_indication      = DENSE_APP_ResetCountersCmd,
        .ClearLatchedFaultsCmd_indication = DENSE_APP_ClearLatchedFaultsCmd,
    },
    .SEND_HK = { .indication = DENSE_APP_SendHkCmd },
};
/* clang-format on */

static DENSE_APP_DenseStatusTlm_t DENSE_APP_StatusMsg; /* under StatusMutex */

static CFE_SB_MsgId_t DENSE_APP_CmdMid(uint16 TopicId)
{
    return CFE_SB_ValueToMsgId(CFE_PLATFORM_CMD_TOPICID_TO_MIDV(TopicId));
}

static CFE_SB_MsgId_t DENSE_APP_TlmMid(uint16 TopicId)
{
    return CFE_SB_ValueToMsgId(CFE_PLATFORM_TLM_TOPICID_TO_MIDV(TopicId));
}

/* -------------------------------------------------------- shared helpers */

void DENSE_APP_FaultReport(OST_FaultId_Enum_t Id, const char *Text)
{
    OS_MutSemTake(DENSE_APP_Data.FaultMutex);
    OST_FaultSet_Report(&DENSE_APP_Data.Faults, Id, Text);
    OS_MutSemGive(DENSE_APP_Data.FaultMutex);
}

void DENSE_APP_FaultRaise(OST_FaultId_Enum_t Id, const char *Text)
{
    OS_MutSemTake(DENSE_APP_Data.FaultMutex);
    OST_FaultSet_Raise(&DENSE_APP_Data.Faults, Id, Text);
    OS_MutSemGive(DENSE_APP_Data.FaultMutex);
}

void DENSE_APP_FaultResolve(OST_FaultId_Enum_t Id)
{
    OS_MutSemTake(DENSE_APP_Data.FaultMutex);
    OST_FaultSet_Resolve(&DENSE_APP_Data.Faults, Id);
    OS_MutSemGive(DENSE_APP_Data.FaultMutex);
}

void DENSE_APP_Release(const OST_ObjectRef_t *Ref)
{
    OST_Status_t Status;

    if (Ref->ObjectId == 0)
    {
        return;
    }
    Status = OST_DS_Release(Ref);
    if (Status != OST_SUCCESS)
    {
        CFE_EVS_SendEvent(DENSE_APP_DATASTORE_ERR_EID, CFE_EVS_EventType_ERROR, "object %lu/%lu not released: %s",
                          (unsigned long)Ref->ObjectId, (unsigned long)Ref->Generation, OST_StatusName(Status));
    }
}

void DENSE_APP_ReleaseBatch(const OST_KeyframeBatch_Payload_t *B)
{
    uint32 i;

    DENSE_APP_Release(&B->Keyframe.WorkImage);
    DENSE_APP_Release(&B->Keyframe.Jpeg);
    for (i = 0; i < B->SourceCount && i < EdsParam_OST_MAX_SOURCE_VIEWS; ++i)
    {
        DENSE_APP_Release(&B->Sources[i].WorkImage);
        DENSE_APP_Release(&B->Sources[i].Jpeg);
    }
}

void DENSE_APP_PublishStatus(const OST_KeyframeBatch_Payload_t *Batch, int64 EntryMono,
                             DENSE_APP_Outcome_Enum_t Outcome, DENSE_APP_Reason_Enum_t Reason)
{
    static uint32                Sequence = 0;
    DENSE_APP_Counters_t        *C        = &DENSE_APP_Data.Counters;
    DENSE_APP_DenseStatus_Payload_t *P    = &DENSE_APP_StatusMsg.Payload;
    uint32                       Depth    = DENSE_APP_QueueDepth();

    OS_MutSemTake(DENSE_APP_Data.StatusMutex);
    if (Outcome == DENSE_APP_Outcome_DONE)
    {
        ++C->BatchesDone;
    }
    else if (Outcome == DENSE_APP_Outcome_SKIPPED)
    {
        ++C->BatchesSkipped;
    }
    else
    {
        ++C->BatchesFailed;
    }
    CFE_MSG_Init(CFE_MSG_PTR(DENSE_APP_StatusMsg.TelemetryHeader),
                 DENSE_APP_TlmMid(EdsParam_CFE_MISSION_DENSE_APP_DENSE_STATUS_TLM_TOPICID), sizeof(DENSE_APP_StatusMsg));
    P->Header.SchemaVersion = 1;
    P->Header.Sequence      = ++Sequence;
    P->Time                 = OST_TimeUtcNs();
    P->KeyframeId           = Batch->KeyframeId;
    P->ImageSeq             = Batch->Keyframe.ImageSeq;
    P->Outcome              = Outcome;
    P->Reason               = Reason;
    P->QueueDepth           = (uint8)(Depth > 255u ? 255u : Depth);
    P->BatchesDone          = C->BatchesDone;
    P->BatchesSkipped       = C->BatchesSkipped;
    P->BatchesFailed        = C->BatchesFailed;
    P->TotalMs              = (float)((double)(OST_TimeMonoNs() - EntryMono) * 1e-6);
    CFE_SB_TimeStampMsg(CFE_MSG_PTR(DENSE_APP_StatusMsg.TelemetryHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(DENSE_APP_StatusMsg.TelemetryHeader), true);
    OS_MutSemGive(DENSE_APP_Data.StatusMutex);
}

/* ------------------------------------------------------------- the queue */

uint32 DENSE_APP_QueueDepth(void)
{
    uint32 Depth;

    OS_MutSemTake(DENSE_APP_Data.RingMutex);
    Depth = DENSE_APP_Data.QueuedItems - DENSE_APP_Data.TakenItems;
    OS_MutSemGive(DENSE_APP_Data.RingMutex);
    return Depth;
}

/* Puts a batch into the ring. When MaxWaitingBatches are waiting, the
 * oldest leaves the ring into *Replaced (newest wins) and the function
 * returns true. */
static bool DENSE_APP_Enqueue(const DENSE_APP_Item_t *Item, DENSE_APP_Item_t *Replaced)
{
    uint32 Limit = DENSE_APP_Data.Config.MaxWaitingBatches;
    bool   Dropped = false;

    if (Limit < 1u || Limit > DENSE_APP_QUEUE_DEPTH)
    {
        Limit = 1u;
    }
    OS_MutSemTake(DENSE_APP_Data.RingMutex);
    if (DENSE_APP_Data.QueuedItems - DENSE_APP_Data.TakenItems >= Limit)
    {
        *Replaced = DENSE_APP_Data.Ring[DENSE_APP_Data.TakenItems % DENSE_APP_QUEUE_DEPTH];
        ++DENSE_APP_Data.TakenItems;
        Dropped = true;
    }
    DENSE_APP_Data.Ring[DENSE_APP_Data.QueuedItems % DENSE_APP_QUEUE_DEPTH] = *Item;
    ++DENSE_APP_Data.QueuedItems;
    OS_MutSemGive(DENSE_APP_Data.RingMutex);
    if (!Dropped)
    {
        OS_CountSemGive(DENSE_APP_Data.ItemSem); /* one count per batch in the ring */
    }
    return Dropped;
}

bool DENSE_APP_Dequeue(DENSE_APP_Item_t *Item)
{
    if (OS_CountSemTimedWait(DENSE_APP_Data.ItemSem, 500) != OS_SUCCESS)
    {
        return false;
    }
    OS_MutSemTake(DENSE_APP_Data.RingMutex);
    *Item = DENSE_APP_Data.Ring[DENSE_APP_Data.TakenItems % DENSE_APP_QUEUE_DEPTH];
    ++DENSE_APP_Data.TakenItems;
    OS_MutSemGive(DENSE_APP_Data.RingMutex);
    return true;
}

/* ----------------------------------------------------------------- table */

static CFE_Status_t DENSE_APP_ValidateConfig(void *TblData)
{
    const DENSE_APP_ConfigTable_t *T   = (const DENSE_APP_ConfigTable_t *)TblData;
    const float                    F[] = {T->SgbmTargetBaselineRatio, T->PmFilterMinNcc, T->PmFilterMinAngleDeg,
                                          T->PmMinFreeGpuMb,          T->DepthMarginM,   T->TerrainGateM,
                                          T->GateSpacingM,            T->GateMarginM,    T->TerrainRadiusM};
    uint32                         i;

    for (i = 0; i < sizeof(F) / sizeof(F[0]); ++i)
    {
        if (!isfinite(F[i]) || F[i] < 0.0f)
        {
            CFE_EVS_SendEvent(DENSE_APP_TBL_ERR_EID, CFE_EVS_EventType_ERROR,
                              "ConfigTable: value %lu not finite or negative", (unsigned long)i);
            return CFE_STATUS_VALIDATION_FAILURE;
        }
    }
    if (T->Method > DENSE_APP_Method_PATCHMATCH || T->StoreNormals > 1 || T->MaxWaitingBatches < 1 ||
        T->MaxWaitingBatches > DENSE_APP_QUEUE_DEPTH || T->Threads > 64 || T->SgbmBlockSize < 1 ||
        T->SgbmBlockSize % 2 == 0 || T->SgbmP1Factor < 1 || T->SgbmP2Factor <= T->SgbmP1Factor ||
        T->SgbmMaxDisparities < 16 || T->SgbmTargetBaselineRatio <= 0.0f || T->GateSpacingM <= 0.0f ||
        T->TerrainRadiusM <= 0.0f || T->PmIterations < 1 || T->PmWindowRadius < 1 || T->PmWindowStep < 1 ||
        T->PmWindowStep > 2 || T->PmSamples < 1)
    {
        CFE_EVS_SendEvent(DENSE_APP_TBL_ERR_EID, CFE_EVS_EventType_ERROR,
                          "ConfigTable: limits invalid (method, waiting batches 1-%u, SGBM block odd, P2 > P1, "
                          "PatchMatch counts > 0)",
                          (unsigned)DENSE_APP_QUEUE_DEPTH);
        return CFE_STATUS_VALIDATION_FAILURE;
    }
    return CFE_SUCCESS;
}

static void DENSE_APP_LoadConfig(void)
{
    CFE_Status_t Status;
    void        *Addr = NULL;

    Status = CFE_TBL_Register(&DENSE_APP_Data.TblHandle, "ConfigTable", sizeof(DENSE_APP_ConfigTable_t),
                              CFE_TBL_OPT_DEFAULT, DENSE_APP_ValidateConfig);
    if (Status == CFE_SUCCESS)
    {
        Status = CFE_TBL_Load(DENSE_APP_Data.TblHandle, CFE_TBL_SRC_FILE, DENSE_APP_TABLE_FILE);
    }
    if (Status == CFE_SUCCESS)
    {
        Status = CFE_TBL_GetAddress(&Addr, DENSE_APP_Data.TblHandle);
        if (Status == CFE_TBL_INFO_UPDATED)
        {
            Status = CFE_SUCCESS;
        }
    }
    if (Status == CFE_SUCCESS && Addr != NULL)
    {
        memcpy(&DENSE_APP_Data.Config, Addr, sizeof(DENSE_APP_Data.Config));
        CFE_TBL_ReleaseAddress(DENSE_APP_Data.TblHandle);
        DENSE_APP_Data.ConfigValid = true;
        return;
    }
    CFE_EVS_SendEvent(DENSE_APP_TBL_ERR_EID, CFE_EVS_EventType_ERROR,
                      "ConfigTable %s not loaded (0x%08lX); batches are released unprocessed", DENSE_APP_TABLE_FILE,
                      (unsigned long)Status);
    DENSE_APP_FaultRaise(OST_FaultId_TABLE_REJECTED, "ConfigTable not loaded");
}

/* --------------------------------------------------------- input messages */

static bool DENSE_APP_ProcessInput(const CFE_SB_Buffer_t *Buf, CFE_SB_MsgId_t MsgId)
{
    static DENSE_APP_Item_t Item;     /* main task only */
    static DENSE_APP_Item_t Replaced; /* main task only */
    CFE_MSG_Size_t          Size = 0;
    const OST_KeyframeBatch_Payload_t *B;

    if (!CFE_SB_MsgId_Equal(MsgId, DENSE_APP_Data.BatchReadyMid))
    {
        return false;
    }
    CFE_MSG_GetSize(&Buf->Msg, &Size);
    if (Size != sizeof(OST_KeyframeBatchTlm_t))
    {
        /* Its references cannot be trusted. */
        ++DENSE_APP_Data.Counters.MessagesRejected;
        CFE_EVS_SendEvent(DENSE_APP_MSG_LEN_ERR_EID, CFE_EVS_EventType_ERROR, "batch: length %lu, expected %lu",
                          (unsigned long)Size, (unsigned long)sizeof(OST_KeyframeBatchTlm_t));
        return true;
    }
    B = &((const OST_KeyframeBatchTlm_t *)Buf)->Payload;
    ++DENSE_APP_Data.Counters.BatchesReceived;
    memset(&Item, 0, sizeof(Item));
    Item.EntryMono = OST_TimeMonoNs();
    Item.Batch     = *B;
    if (!DENSE_APP_Data.ConfigValid)
    {
        DENSE_APP_ReleaseBatch(B);
        DENSE_APP_PublishStatus(B, Item.EntryMono, DENSE_APP_Outcome_FAILED, DENSE_APP_Reason_NOT_READY);
        return true;
    }
    if (DENSE_APP_Enqueue(&Item, &Replaced))
    {
        CFE_EVS_SendEvent(DENSE_APP_SKIPPED_EID, CFE_EVS_EventType_ERROR,
                          "keyframe %lu (image %lu) skipped: replaced by keyframe %lu (newest wins)",
                          (unsigned long)Replaced.Batch.KeyframeId, (unsigned long)Replaced.Batch.Keyframe.ImageSeq,
                          (unsigned long)B->KeyframeId);
        DENSE_APP_FaultReport(OST_FaultId_KEYFRAMES_SKIPPED, "keyframe skipped: dense backlog");
        DENSE_APP_ReleaseBatch(&Replaced.Batch);
        DENSE_APP_PublishStatus(&Replaced.Batch, Replaced.EntryMono, DENSE_APP_Outcome_SKIPPED,
                                DENSE_APP_Reason_NEWER_BATCH);
    }
    return true;
}

/* ------------------------------------------------------------------ init */

static CFE_Status_t DENSE_APP_Init(void)
{
    CFE_Status_t    Status;
    CFE_ES_TaskId_t TaskId;
    char            VersionString[DENSE_APP_MAX_VERSION_STR_LEN];

    memset(&DENSE_APP_Data, 0, sizeof(DENSE_APP_Data));
    DENSE_APP_Data.RunStatus = CFE_ES_RunStatus_APP_RUN;

    Status = CFE_EVS_Register(DENSE_APP_EventFilters, sizeof(DENSE_APP_EventFilters) / sizeof(DENSE_APP_EventFilters[0]),
                              CFE_EVS_EventFilter_BINARY);
    if (Status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("DENSE_APP: EVS register failed 0x%08lX\n", (unsigned long)Status);
        return Status;
    }
    if (OS_MutSemCreate(&DENSE_APP_Data.FaultMutex, "DENSE_FLT", 0) != OS_SUCCESS ||
        OS_MutSemCreate(&DENSE_APP_Data.StatusMutex, "DENSE_STAT", 0) != OS_SUCCESS ||
        OS_MutSemCreate(&DENSE_APP_Data.RingMutex, "DENSE_RING", 0) != OS_SUCCESS ||
        OS_CountSemCreate(&DENSE_APP_Data.ItemSem, "DENSE_ITEMS", 0, 0) != OS_SUCCESS)
    {
        CFE_EVS_SendEvent(DENSE_APP_INIT_ERR_EID, CFE_EVS_EventType_ERROR, "mutexes or worker queue not created");
        return CFE_STATUS_EXTERNAL_RESOURCE_FAIL;
    }
    Status = OST_FaultSet_Init(&DENSE_APP_Data.Faults, OST_AppId_DENSE_APP, DENSE_APP_FaultIds, DENSE_APP_FAULT_COUNT);
    if (Status != CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(DENSE_APP_INIT_ERR_EID, CFE_EVS_EventType_ERROR, "fault set: 0x%08lX", (unsigned long)Status);
        return Status;
    }

    CFE_MSG_Init(CFE_MSG_PTR(DENSE_APP_Data.HkTlm.TelemetryHeader),
                 DENSE_APP_TlmMid(EdsParam_CFE_MISSION_DENSE_APP_HK_TLM_TOPICID), sizeof(DENSE_APP_Data.HkTlm));
    DENSE_APP_Data.BatchReadyMid = OST_BatchReadyMsgId();

    Status = CFE_SB_CreatePipe(&DENSE_APP_Data.Pipe, DENSE_APP_PIPE_DEPTH, "DENSE_PIPE");
    if (Status == CFE_SUCCESS)
    {
        Status = CFE_SB_SubscribeEx(DENSE_APP_CmdMid(EdsParam_CFE_MISSION_DENSE_APP_CMD_TOPICID), DENSE_APP_Data.Pipe,
                                    CFE_SB_DEFAULT_QOS, 4);
    }
    if (Status == CFE_SUCCESS)
    {
        Status = CFE_SB_SubscribeEx(DENSE_APP_CmdMid(EdsParam_CFE_MISSION_DENSE_APP_SEND_HK_TOPICID),
                                    DENSE_APP_Data.Pipe, CFE_SB_DEFAULT_QOS, 2);
    }
    if (Status == CFE_SUCCESS)
    {
        /* A batch lost to a full pipe would also lose its counts; the main
         * task takes batches off the pipe at once. */
        Status = CFE_SB_SubscribeEx(DENSE_APP_Data.BatchReadyMid, DENSE_APP_Data.Pipe, CFE_SB_DEFAULT_QOS, 24);
    }
    if (Status != CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(DENSE_APP_INIT_ERR_EID, CFE_EVS_EventType_ERROR, "pipe or subscription: 0x%08lX",
                          (unsigned long)Status);
        return Status;
    }

    DENSE_APP_LoadConfig();

    Status = CFE_ES_CreateChildTask(&TaskId, "DENSE_WORK", DENSE_APP_WorkerTask, NULL, DENSE_APP_WORKER_STACK_SIZE,
                                    DENSE_APP_WORKER_PRIORITY, 0);
    if (Status != CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(DENSE_APP_INIT_ERR_EID, CFE_EVS_EventType_ERROR, "worker task: 0x%08lX",
                          (unsigned long)Status);
        return Status;
    }

    CFE_Config_GetVersionString(VersionString, DENSE_APP_MAX_VERSION_STR_LEN, "DENSE_APP", DENSE_APP_VERSION,
                                DENSE_APP_BUILD_CODENAME, DENSE_APP_LAST_OFFICIAL);
    CFE_EVS_SendEvent(DENSE_APP_INIT_INF_EID, CFE_EVS_EventType_INFORMATION,
                      "DENSE_APP initialised: method %s, %u waiting batch(es), terrain gate %.0f m. %s",
                      DENSE_APP_Data.Config.Method == DENSE_APP_Method_PATCHMATCH ? "PatchMatch" : "SGBM",
                      (unsigned)DENSE_APP_Data.Config.MaxWaitingBatches, DENSE_APP_Data.Config.TerrainGateM,
                      VersionString);
    return CFE_SUCCESS;
}

/* -------------------------------------------------------------- commands */

static void DENSE_APP_ProcessCommand(const CFE_SB_Buffer_t *Buf, CFE_SB_MsgId_t MsgId)
{
    CFE_Status_t      Status;
    CFE_MSG_Size_t    Size = 0;
    CFE_MSG_FcnCode_t Fc   = 0;

    Status = EdsDispatch_EdsComponent_DENSE_APP_Application_Telecommand(Buf, &DENSE_APP_TC_DISPATCH_TABLE);
    if (Status == CFE_SUCCESS)
    {
        return;
    }
    CFE_MSG_GetSize(&Buf->Msg, &Size);
    CFE_MSG_GetFcnCode(&Buf->Msg, &Fc);
    ++DENSE_APP_Data.CommandErrorCounter;
    if (Status == CFE_STATUS_UNKNOWN_MSG_ID)
    {
        CFE_EVS_SendEvent(DENSE_APP_MID_ERR_EID, CFE_EVS_EventType_ERROR, "invalid message ID 0x%x",
                          (unsigned)CFE_SB_MsgIdToValue(MsgId));
    }
    else if (Status == CFE_STATUS_WRONG_MSG_LENGTH)
    {
        CFE_EVS_SendEvent(DENSE_APP_CMD_LEN_ERR_EID, CFE_EVS_EventType_ERROR,
                          "invalid length: MID 0x%x, CC %u, length %u", (unsigned)CFE_SB_MsgIdToValue(MsgId),
                          (unsigned)Fc, (unsigned)Size);
    }
    else
    {
        CFE_EVS_SendEvent(DENSE_APP_CC_ERR_EID, CFE_EVS_EventType_ERROR, "invalid command code %u", (unsigned)Fc);
    }
}

CFE_Status_t DENSE_APP_NoopCmd(const DENSE_APP_NoopCmd_t *Msg)
{
    (void)Msg;
    ++DENSE_APP_Data.CommandCounter;
    CFE_EVS_SendEvent(DENSE_APP_NOOP_INF_EID, CFE_EVS_EventType_INFORMATION, "no-op, version %s", DENSE_APP_VERSION);
    return CFE_SUCCESS;
}

CFE_Status_t DENSE_APP_ResetCountersCmd(const DENSE_APP_ResetCountersCmd_t *Msg)
{
    DENSE_APP_Counters_t *C = &DENSE_APP_Data.Counters;

    (void)Msg;
    DENSE_APP_Data.CommandCounter      = 0;
    DENSE_APP_Data.CommandErrorCounter = 0;
    C->BatchesReceived = C->MessagesRejected = C->QuotaWaits = 0;
    OS_MutSemTake(DENSE_APP_Data.StatusMutex);
    C->BatchesDone = C->BatchesSkipped = C->BatchesFailed = 0;
    OS_MutSemGive(DENSE_APP_Data.StatusMutex);
    C->MaxEstimateMs = C->MaxJobMs = C->MaxLatencyS = 0.0f;
    CFE_EVS_SendEvent(DENSE_APP_RESET_INF_EID, CFE_EVS_EventType_INFORMATION, "counters reset");
    return CFE_SUCCESS;
}

CFE_Status_t DENSE_APP_ClearLatchedFaultsCmd(const DENSE_APP_ClearLatchedFaultsCmd_t *Msg)
{
    uint32 Cleared;

    (void)Msg;
    OS_MutSemTake(DENSE_APP_Data.FaultMutex);
    Cleared = OST_FaultSet_ClearLatched(&DENSE_APP_Data.Faults);
    OS_MutSemGive(DENSE_APP_Data.FaultMutex);
    ++DENSE_APP_Data.CommandCounter;
    CFE_EVS_SendEvent(DENSE_APP_CLEAR_INF_EID, CFE_EVS_EventType_INFORMATION, "%lu latched faults cleared",
                      (unsigned long)Cleared);
    return CFE_SUCCESS;
}

CFE_Status_t DENSE_APP_SendHkCmd(const DENSE_APP_SendHkCmd_t *Msg)
{
    DENSE_APP_HkTlm_Payload_t  *P = &DENSE_APP_Data.HkTlm.Payload;
    const DENSE_APP_Counters_t *C = &DENSE_APP_Data.Counters;

    (void)Msg;
    P->Common.CommandCounter      = DENSE_APP_Data.CommandCounter;
    P->Common.CommandErrorCounter = DENSE_APP_Data.CommandErrorCounter;
    OS_MutSemTake(DENSE_APP_Data.FaultMutex);
    OST_FaultSet_Summarise(&DENSE_APP_Data.Faults, &P->Common);
    OS_MutSemGive(DENSE_APP_Data.FaultMutex);
    P->Method            = DENSE_APP_Data.Config.Method;
    P->Ready             = C->Ready ? 1 : 0;
    P->LastTerrain       = C->LastTerrain ? 1 : 0;
    P->BatchesReceived   = C->BatchesReceived;
    OS_MutSemTake(DENSE_APP_Data.StatusMutex);
    P->BatchesDone       = C->BatchesDone;
    P->BatchesSkipped    = C->BatchesSkipped;
    P->BatchesFailed     = C->BatchesFailed;
    OS_MutSemGive(DENSE_APP_Data.StatusMutex);
    P->QuotaWaits        = C->QuotaWaits;
    P->MessagesRejected  = C->MessagesRejected;
    P->QueueDepth        = DENSE_APP_QueueDepth();
    P->TerrainLoads      = C->TerrainLoads;
    P->LastKeyframeId    = C->LastKeyframeId;
    P->LastValidFraction = C->LastValidFraction;
    P->LastGatedFraction = C->LastGatedFraction;
    P->LastDepthMinM     = C->LastDepthMinM;
    P->LastDepthMaxM     = C->LastDepthMaxM;
    P->LastWaitMs        = C->LastWaitMs;
    P->LastReadMs        = C->LastReadMs;
    P->LastSetupMs       = C->LastSetupMs;
    P->LastEstimateMs    = C->LastEstimateMs;
    P->MaxEstimateMs     = C->MaxEstimateMs;
    P->LastGateMs        = C->LastGateMs;
    P->LastJobMs         = C->LastJobMs;
    P->MaxJobMs          = C->MaxJobMs;
    P->LastLatencyS      = C->LastLatencyS;
    P->MaxLatencyS       = C->MaxLatencyS;

    CFE_SB_TimeStampMsg(CFE_MSG_PTR(DENSE_APP_Data.HkTlm.TelemetryHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(DENSE_APP_Data.HkTlm.TelemetryHeader), true);
    return CFE_SUCCESS;
}

/* ------------------------------------------------------------------ main */

void DENSE_APP_Main(void)
{
    CFE_Status_t     Status;
    CFE_SB_Buffer_t *Buf = NULL;
    CFE_SB_MsgId_t   MsgId;

    Status = DENSE_APP_Init();
    if (Status != CFE_SUCCESS)
    {
        DENSE_APP_Data.RunStatus = CFE_ES_RunStatus_APP_ERROR;
    }

    while (CFE_ES_RunLoop(&DENSE_APP_Data.RunStatus))
    {
        Status = CFE_SB_ReceiveBuffer(&Buf, DENSE_APP_Data.Pipe, DENSE_APP_PIPE_TIMEOUT_MS);
        if (Status == CFE_SUCCESS)
        {
            MsgId = CFE_SB_INVALID_MSG_ID;
            CFE_MSG_GetMsgId(&Buf->Msg, &MsgId);
            if (!DENSE_APP_ProcessInput(Buf, MsgId))
            {
                DENSE_APP_ProcessCommand(Buf, MsgId);
            }
        }
        else if (Status != CFE_SB_TIME_OUT && Status != CFE_SB_NO_MESSAGE)
        {
            CFE_EVS_SendEvent(DENSE_APP_PIPE_ERR_EID, CFE_EVS_EventType_ERROR, "pipe read error 0x%08lX",
                              (unsigned long)Status);
            DENSE_APP_Data.RunStatus = CFE_ES_RunStatus_APP_ERROR;
        }
    }
    CFE_ES_ExitApp(DENSE_APP_Data.RunStatus);
}
