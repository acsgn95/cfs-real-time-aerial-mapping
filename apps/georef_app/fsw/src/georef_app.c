/*
 * GEOREF_APP main task: initialisation, commands, housekeeping and the
 * worker queue (georef_app.h).
 */
#include "georef_app.h"
#include "georef_app_eventids.h"
#include "georef_app_version.h"

#include "cfe_config.h"
#include "cfe_core_api_base_msgids.h"
#include "cfe_mission_eds_designparameters.h"
#include "georef_app_eds_dictionary.h"
#include "georef_app_eds_dispatcher.h"
#include "ost_datastore.h"

#include <math.h>
#include <string.h>

GEOREF_APP_Data_t GEOREF_APP_Data;

static const OST_FaultId_Enum_t GEOREF_APP_FaultIds[GEOREF_APP_FAULT_COUNT] = {
    OST_FaultId_POINTS_FAILED,   /* 60 */
    OST_FaultId_DATASTORE_QUOTA, /* 61 */
    OST_FaultId_TILE_WRITE,      /* 62 */
    OST_FaultId_TABLE_REJECTED,  /* 63 */
    OST_FaultId_LIBRARY_ERROR,   /* 64 */
};

static CFE_EVS_BinFilter_t GEOREF_APP_EventFilters[] = {
    {GEOREF_APP_FAILED_EID, CFE_EVS_FIRST_8_STOP},  {GEOREF_APP_QUOTA_EID, CFE_EVS_FIRST_8_STOP},
    {GEOREF_APP_MSG_LEN_ERR_EID, CFE_EVS_FIRST_8_STOP}, {GEOREF_APP_LIB_ERR_EID, CFE_EVS_FIRST_8_STOP},
    {GEOREF_APP_DATASTORE_ERR_EID, CFE_EVS_FIRST_8_STOP}, {GEOREF_APP_LAZ_ERR_EID, CFE_EVS_FIRST_8_STOP},
};

/* clang-format off */
static const EdsDispatchTable_EdsComponent_GEOREF_APP_Application_CFE_SB_Telecommand_t GEOREF_APP_TC_DISPATCH_TABLE = {
    .CMD = {
        .NoopCmd_indication               = GEOREF_APP_NoopCmd,
        .ResetCountersCmd_indication      = GEOREF_APP_ResetCountersCmd,
        .ClearLatchedFaultsCmd_indication = GEOREF_APP_ClearLatchedFaultsCmd,
    },
    .SEND_HK = { .indication = GEOREF_APP_SendHkCmd },
};
/* clang-format on */

static CFE_SB_MsgId_t GEOREF_APP_CmdMid(uint16 TopicId)
{
    return CFE_SB_ValueToMsgId(CFE_PLATFORM_CMD_TOPICID_TO_MIDV(TopicId));
}

static CFE_SB_MsgId_t GEOREF_APP_TlmMid(uint16 TopicId)
{
    return CFE_SB_ValueToMsgId(CFE_PLATFORM_TLM_TOPICID_TO_MIDV(TopicId));
}

/* -------------------------------------------------------- shared helpers */

void GEOREF_APP_FaultReport(OST_FaultId_Enum_t Id, const char *Text)
{
    OS_MutSemTake(GEOREF_APP_Data.FaultMutex);
    OST_FaultSet_Report(&GEOREF_APP_Data.Faults, Id, Text);
    OS_MutSemGive(GEOREF_APP_Data.FaultMutex);
}

void GEOREF_APP_FaultRaise(OST_FaultId_Enum_t Id, const char *Text)
{
    OS_MutSemTake(GEOREF_APP_Data.FaultMutex);
    OST_FaultSet_Raise(&GEOREF_APP_Data.Faults, Id, Text);
    OS_MutSemGive(GEOREF_APP_Data.FaultMutex);
}

void GEOREF_APP_FaultResolve(OST_FaultId_Enum_t Id)
{
    OS_MutSemTake(GEOREF_APP_Data.FaultMutex);
    OST_FaultSet_Resolve(&GEOREF_APP_Data.Faults, Id);
    OS_MutSemGive(GEOREF_APP_Data.FaultMutex);
}

void GEOREF_APP_Release(const OST_ObjectRef_t *Ref)
{
    OST_Status_t Status;

    if (Ref->ObjectId == 0)
    {
        return;
    }
    Status = OST_DS_Release(Ref);
    if (Status != OST_SUCCESS)
    {
        CFE_EVS_SendEvent(GEOREF_APP_DATASTORE_ERR_EID, CFE_EVS_EventType_ERROR, "object %lu/%lu not released: %s",
                          (unsigned long)Ref->ObjectId, (unsigned long)Ref->Generation, OST_StatusName(Status));
    }
}

void GEOREF_APP_Drop(const DENSE_APP_DepthReady_Payload_t *D, const char *Text)
{
    GEOREF_APP_Release(&D->DepthSet);
    GEOREF_APP_Release(&D->Keyframe.WorkImage);
    GEOREF_APP_Release(&D->Keyframe.Jpeg);
    OS_MutSemTake(GEOREF_APP_Data.CountMutex);
    ++GEOREF_APP_Data.Counters.KeyframesFailed;
    OS_MutSemGive(GEOREF_APP_Data.CountMutex);
    CFE_EVS_SendEvent(GEOREF_APP_FAILED_EID, CFE_EVS_EventType_ERROR, "keyframe %lu (image %lu) dropped: %s",
                      (unsigned long)D->KeyframeId, (unsigned long)D->Keyframe.ImageSeq, Text);
    GEOREF_APP_FaultReport(OST_FaultId_POINTS_FAILED, "keyframe dropped by GEOREF_APP");
}

/* ------------------------------------------------------------- the queue */

uint32 GEOREF_APP_QueueDepth(void)
{
    uint32 Depth;

    OS_MutSemTake(GEOREF_APP_Data.RingMutex);
    Depth = GEOREF_APP_Data.QueuedItems - GEOREF_APP_Data.TakenItems;
    OS_MutSemGive(GEOREF_APP_Data.RingMutex);
    return Depth;
}

/* Puts a depth set into the ring; false when the ring is full. */
static bool GEOREF_APP_Enqueue(const GEOREF_APP_Item_t *Item)
{
    bool Full;

    OS_MutSemTake(GEOREF_APP_Data.RingMutex);
    Full = GEOREF_APP_Data.QueuedItems - GEOREF_APP_Data.TakenItems >= GEOREF_APP_QUEUE_DEPTH;
    if (!Full)
    {
        GEOREF_APP_Data.Ring[GEOREF_APP_Data.QueuedItems % GEOREF_APP_QUEUE_DEPTH] = *Item;
        ++GEOREF_APP_Data.QueuedItems;
    }
    OS_MutSemGive(GEOREF_APP_Data.RingMutex);
    if (!Full)
    {
        OS_CountSemGive(GEOREF_APP_Data.ItemSem);
    }
    return !Full;
}

bool GEOREF_APP_Dequeue(GEOREF_APP_Item_t *Item)
{
    if (OS_CountSemTimedWait(GEOREF_APP_Data.ItemSem, 500) != OS_SUCCESS)
    {
        return false;
    }
    OS_MutSemTake(GEOREF_APP_Data.RingMutex);
    *Item = GEOREF_APP_Data.Ring[GEOREF_APP_Data.TakenItems % GEOREF_APP_QUEUE_DEPTH];
    ++GEOREF_APP_Data.TakenItems;
    OS_MutSemGive(GEOREF_APP_Data.RingMutex);
    return true;
}

/* ----------------------------------------------------------------- table */

static bool GEOREF_APP_HasText(const char *Text, size_t Len)
{
    return Len > 0 && Text[0] != '\0' && memchr(Text, '\0', Len) != NULL;
}

static CFE_Status_t GEOREF_APP_ValidateConfig(void *TblData)
{
    const GEOREF_APP_ConfigTable_t *T = (const GEOREF_APP_ConfigTable_t *)TblData;

    if (!isfinite(T->CellSizeM) || T->CellSizeM <= 0.0f || T->TileCells < 16 || T->TileCells > 8192 ||
        !isfinite(T->MinConfidence) || T->MinConfidence < 0.0f || T->MinConfidence > 1.0f || T->WriteLaz > 1 ||
        !GEOREF_APP_HasText(T->ProjectedCrs, sizeof(T->ProjectedCrs)) ||
        !GEOREF_APP_HasText(T->ProjDataDir, sizeof(T->ProjDataDir)) ||
        memchr(T->VerticalCrs, '\0', sizeof(T->VerticalCrs)) == NULL ||
        memchr(T->GridDir, '\0', sizeof(T->GridDir)) == NULL ||
        (T->WriteLaz && !GEOREF_APP_HasText(T->OutputDir, sizeof(T->OutputDir))))
    {
        CFE_EVS_SendEvent(GEOREF_APP_TBL_ERR_EID, CFE_EVS_EventType_ERROR,
                          "ConfigTable: limits invalid (cell size > 0, 16-8192 cells per tile, confidence 0-1, "
                          "CRS and PROJ directory set, output directory with WriteLaz)");
        return CFE_STATUS_VALIDATION_FAILURE;
    }
    return CFE_SUCCESS;
}

static void GEOREF_APP_LoadConfig(void)
{
    CFE_Status_t Status;
    void        *Addr = NULL;

    Status = CFE_TBL_Register(&GEOREF_APP_Data.TblHandle, "ConfigTable", sizeof(GEOREF_APP_ConfigTable_t),
                              CFE_TBL_OPT_DEFAULT, GEOREF_APP_ValidateConfig);
    if (Status == CFE_SUCCESS)
    {
        Status = CFE_TBL_Load(GEOREF_APP_Data.TblHandle, CFE_TBL_SRC_FILE, GEOREF_APP_TABLE_FILE);
    }
    if (Status == CFE_SUCCESS)
    {
        Status = CFE_TBL_GetAddress(&Addr, GEOREF_APP_Data.TblHandle);
        if (Status == CFE_TBL_INFO_UPDATED)
        {
            Status = CFE_SUCCESS;
        }
    }
    if (Status == CFE_SUCCESS && Addr != NULL)
    {
        memcpy(&GEOREF_APP_Data.Config, Addr, sizeof(GEOREF_APP_Data.Config));
        CFE_TBL_ReleaseAddress(GEOREF_APP_Data.TblHandle);
        GEOREF_APP_Data.ConfigValid = true;
        return;
    }
    CFE_EVS_SendEvent(GEOREF_APP_TBL_ERR_EID, CFE_EVS_EventType_ERROR,
                      "ConfigTable %s not loaded (0x%08lX); depth sets are dropped", GEOREF_APP_TABLE_FILE,
                      (unsigned long)Status);
    GEOREF_APP_FaultRaise(OST_FaultId_TABLE_REJECTED, "ConfigTable not loaded");
}

/* --------------------------------------------------------- input messages */

static bool GEOREF_APP_ProcessInput(const CFE_SB_Buffer_t *Buf, CFE_SB_MsgId_t MsgId)
{
    static GEOREF_APP_Item_t              Item; /* main task only */
    CFE_MSG_Size_t                        Size = 0;
    const DENSE_APP_DepthReady_Payload_t *D;

    if (!CFE_SB_MsgId_Equal(MsgId, GEOREF_APP_Data.DepthReadyMid))
    {
        return false;
    }
    CFE_MSG_GetSize(&Buf->Msg, &Size);
    if (Size != sizeof(DENSE_APP_DepthReadyTlm_t))
    {
        /* Its references cannot be trusted. */
        ++GEOREF_APP_Data.Counters.MessagesRejected;
        CFE_EVS_SendEvent(GEOREF_APP_MSG_LEN_ERR_EID, CFE_EVS_EventType_ERROR, "depth set: length %lu, expected %lu",
                          (unsigned long)Size, (unsigned long)sizeof(DENSE_APP_DepthReadyTlm_t));
        return true;
    }
    D = &((const DENSE_APP_DepthReadyTlm_t *)Buf)->Payload;
    ++GEOREF_APP_Data.Counters.DepthSetsReceived;
    if (!GEOREF_APP_Data.ConfigValid)
    {
        GEOREF_APP_Drop(D, "no configuration table");
        return true;
    }
    memset(&Item, 0, sizeof(Item));
    Item.EntryMono = OST_TimeMonoNs();
    Item.Depth     = *D;
    if (!GEOREF_APP_Enqueue(&Item))
    {
        GEOREF_APP_Drop(D, "worker queue full");
    }
    return true;
}

/* ------------------------------------------------------------------ init */

static CFE_Status_t GEOREF_APP_Init(void)
{
    CFE_Status_t    Status;
    CFE_ES_TaskId_t TaskId;
    char            VersionString[GEOREF_APP_MAX_VERSION_STR_LEN];

    memset(&GEOREF_APP_Data, 0, sizeof(GEOREF_APP_Data));
    GEOREF_APP_Data.RunStatus = CFE_ES_RunStatus_APP_RUN;

    Status = CFE_EVS_Register(GEOREF_APP_EventFilters,
                              sizeof(GEOREF_APP_EventFilters) / sizeof(GEOREF_APP_EventFilters[0]),
                              CFE_EVS_EventFilter_BINARY);
    if (Status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("GEOREF_APP: EVS register failed 0x%08lX\n", (unsigned long)Status);
        return Status;
    }
    if (OS_MutSemCreate(&GEOREF_APP_Data.FaultMutex, "GEOREF_FLT", 0) != OS_SUCCESS ||
        OS_MutSemCreate(&GEOREF_APP_Data.CountMutex, "GEOREF_CNT", 0) != OS_SUCCESS ||
        OS_MutSemCreate(&GEOREF_APP_Data.RingMutex, "GEOREF_RING", 0) != OS_SUCCESS ||
        OS_CountSemCreate(&GEOREF_APP_Data.ItemSem, "GEOREF_ITEMS", 0, 0) != OS_SUCCESS)
    {
        CFE_EVS_SendEvent(GEOREF_APP_INIT_ERR_EID, CFE_EVS_EventType_ERROR, "mutexes or worker queue not created");
        return CFE_STATUS_EXTERNAL_RESOURCE_FAIL;
    }
    Status = OST_FaultSet_Init(&GEOREF_APP_Data.Faults, OST_AppId_GEOREF_APP, GEOREF_APP_FaultIds,
                               GEOREF_APP_FAULT_COUNT);
    if (Status != CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(GEOREF_APP_INIT_ERR_EID, CFE_EVS_EventType_ERROR, "fault set: 0x%08lX",
                          (unsigned long)Status);
        return Status;
    }

    CFE_MSG_Init(CFE_MSG_PTR(GEOREF_APP_Data.HkTlm.TelemetryHeader),
                 GEOREF_APP_TlmMid(EdsParam_CFE_MISSION_GEOREF_APP_HK_TLM_TOPICID), sizeof(GEOREF_APP_Data.HkTlm));
    GEOREF_APP_Data.DepthReadyMid = GEOREF_APP_TlmMid(EdsParam_CFE_MISSION_DENSE_APP_DEPTH_READY_TLM_TOPICID);

    Status = CFE_SB_CreatePipe(&GEOREF_APP_Data.Pipe, GEOREF_APP_PIPE_DEPTH, "GEOREF_PIPE");
    if (Status == CFE_SUCCESS)
    {
        Status = CFE_SB_SubscribeEx(GEOREF_APP_CmdMid(EdsParam_CFE_MISSION_GEOREF_APP_CMD_TOPICID),
                                    GEOREF_APP_Data.Pipe, CFE_SB_DEFAULT_QOS, 4);
    }
    if (Status == CFE_SUCCESS)
    {
        Status = CFE_SB_SubscribeEx(GEOREF_APP_CmdMid(EdsParam_CFE_MISSION_GEOREF_APP_SEND_HK_TOPICID),
                                    GEOREF_APP_Data.Pipe, CFE_SB_DEFAULT_QOS, 2);
    }
    if (Status == CFE_SUCCESS)
    {
        /* A depth set lost to a full pipe would also lose its counts; the
         * main task takes them off the pipe at once. */
        Status = CFE_SB_SubscribeEx(GEOREF_APP_Data.DepthReadyMid, GEOREF_APP_Data.Pipe, CFE_SB_DEFAULT_QOS, 24);
    }
    if (Status != CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(GEOREF_APP_INIT_ERR_EID, CFE_EVS_EventType_ERROR, "pipe or subscription: 0x%08lX",
                          (unsigned long)Status);
        return Status;
    }

    GEOREF_APP_LoadConfig();

    Status = CFE_ES_CreateChildTask(&TaskId, "GEOREF_WORK", GEOREF_APP_WorkerTask, NULL,
                                    GEOREF_APP_WORKER_STACK_SIZE, GEOREF_APP_WORKER_PRIORITY, 0);
    if (Status != CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(GEOREF_APP_INIT_ERR_EID, CFE_EVS_EventType_ERROR, "worker task: 0x%08lX",
                          (unsigned long)Status);
        return Status;
    }

    CFE_Config_GetVersionString(VersionString, GEOREF_APP_MAX_VERSION_STR_LEN, "GEOREF_APP", GEOREF_APP_VERSION,
                                GEOREF_APP_BUILD_CODENAME, GEOREF_APP_LAST_OFFICIAL);
    CFE_EVS_SendEvent(GEOREF_APP_INIT_INF_EID, CFE_EVS_EventType_INFORMATION,
                      "GEOREF_APP initialised: P %.32s, tiles of %lu x %.2f m, point cloud %s. %s",
                      GEOREF_APP_Data.Config.ProjectedCrs, (unsigned long)GEOREF_APP_Data.Config.TileCells,
                      GEOREF_APP_Data.Config.CellSizeM, GEOREF_APP_Data.Config.WriteLaz ? "on" : "off",
                      VersionString);
    return CFE_SUCCESS;
}

/* -------------------------------------------------------------- commands */

static void GEOREF_APP_ProcessCommand(const CFE_SB_Buffer_t *Buf, CFE_SB_MsgId_t MsgId)
{
    CFE_Status_t      Status;
    CFE_MSG_Size_t    Size = 0;
    CFE_MSG_FcnCode_t Fc   = 0;

    Status = EdsDispatch_EdsComponent_GEOREF_APP_Application_Telecommand(Buf, &GEOREF_APP_TC_DISPATCH_TABLE);
    if (Status == CFE_SUCCESS)
    {
        return;
    }
    CFE_MSG_GetSize(&Buf->Msg, &Size);
    CFE_MSG_GetFcnCode(&Buf->Msg, &Fc);
    ++GEOREF_APP_Data.CommandErrorCounter;
    if (Status == CFE_STATUS_UNKNOWN_MSG_ID)
    {
        CFE_EVS_SendEvent(GEOREF_APP_MID_ERR_EID, CFE_EVS_EventType_ERROR, "invalid message ID 0x%x",
                          (unsigned)CFE_SB_MsgIdToValue(MsgId));
    }
    else if (Status == CFE_STATUS_WRONG_MSG_LENGTH)
    {
        CFE_EVS_SendEvent(GEOREF_APP_CMD_LEN_ERR_EID, CFE_EVS_EventType_ERROR,
                          "invalid length: MID 0x%x, CC %u, length %u", (unsigned)CFE_SB_MsgIdToValue(MsgId),
                          (unsigned)Fc, (unsigned)Size);
    }
    else
    {
        CFE_EVS_SendEvent(GEOREF_APP_CC_ERR_EID, CFE_EVS_EventType_ERROR, "invalid command code %u", (unsigned)Fc);
    }
}

CFE_Status_t GEOREF_APP_NoopCmd(const GEOREF_APP_NoopCmd_t *Msg)
{
    (void)Msg;
    ++GEOREF_APP_Data.CommandCounter;
    CFE_EVS_SendEvent(GEOREF_APP_NOOP_INF_EID, CFE_EVS_EventType_INFORMATION, "no-op, version %s",
                      GEOREF_APP_VERSION);
    return CFE_SUCCESS;
}

CFE_Status_t GEOREF_APP_ResetCountersCmd(const GEOREF_APP_ResetCountersCmd_t *Msg)
{
    GEOREF_APP_Counters_t *C = &GEOREF_APP_Data.Counters;

    (void)Msg;
    GEOREF_APP_Data.CommandCounter      = 0;
    GEOREF_APP_Data.CommandErrorCounter = 0;
    C->DepthSetsReceived = C->MessagesRejected = C->PointSetsPublished = C->QuotaWaits = 0;
    C->LazFiles = C->LazFailures = C->MaxTileCount = 0;
    OS_MutSemTake(GEOREF_APP_Data.CountMutex);
    C->KeyframesFailed = 0;
    OS_MutSemGive(GEOREF_APP_Data.CountMutex);
    C->MaxConvertMs = C->MaxStageMs = C->MaxLazMs = C->MaxLatencyS = 0.0f;
    CFE_EVS_SendEvent(GEOREF_APP_RESET_INF_EID, CFE_EVS_EventType_INFORMATION, "counters reset");
    return CFE_SUCCESS;
}

CFE_Status_t GEOREF_APP_ClearLatchedFaultsCmd(const GEOREF_APP_ClearLatchedFaultsCmd_t *Msg)
{
    uint32 Cleared;

    (void)Msg;
    OS_MutSemTake(GEOREF_APP_Data.FaultMutex);
    Cleared = OST_FaultSet_ClearLatched(&GEOREF_APP_Data.Faults);
    OS_MutSemGive(GEOREF_APP_Data.FaultMutex);
    ++GEOREF_APP_Data.CommandCounter;
    CFE_EVS_SendEvent(GEOREF_APP_CLEAR_INF_EID, CFE_EVS_EventType_INFORMATION, "%lu latched faults cleared",
                      (unsigned long)Cleared);
    return CFE_SUCCESS;
}

CFE_Status_t GEOREF_APP_SendHkCmd(const GEOREF_APP_SendHkCmd_t *Msg)
{
    GEOREF_APP_HkTlm_Payload_t  *P = &GEOREF_APP_Data.HkTlm.Payload;
    const GEOREF_APP_Counters_t *C = &GEOREF_APP_Data.Counters;

    (void)Msg;
    P->Common.CommandCounter      = GEOREF_APP_Data.CommandCounter;
    P->Common.CommandErrorCounter = GEOREF_APP_Data.CommandErrorCounter;
    OS_MutSemTake(GEOREF_APP_Data.FaultMutex);
    OST_FaultSet_Summarise(&GEOREF_APP_Data.Faults, &P->Common);
    OS_MutSemGive(GEOREF_APP_Data.FaultMutex);
    P->Ready              = C->Ready ? 1 : 0;
    P->WriteLaz           = GEOREF_APP_Data.Config.WriteLaz;
    P->DepthSetsReceived  = C->DepthSetsReceived;
    P->PointSetsPublished = C->PointSetsPublished;
    OS_MutSemTake(GEOREF_APP_Data.CountMutex);
    P->KeyframesFailed = C->KeyframesFailed;
    OS_MutSemGive(GEOREF_APP_Data.CountMutex);
    P->QuotaWaits       = C->QuotaWaits;
    P->MessagesRejected = C->MessagesRejected;
    P->QueueDepth       = GEOREF_APP_QueueDepth();
    P->LazFiles         = C->LazFiles;
    P->LazFailures      = C->LazFailures;
    P->LastKeyframeId   = C->LastKeyframeId;
    P->LastPointCount   = C->LastPointCount;
    P->LastTileCount    = C->LastTileCount;
    P->MaxTileCount     = C->MaxTileCount;
    P->LastWaitMs       = C->LastWaitMs;
    P->LastAllocateMs   = C->LastAllocateMs;
    P->LastConvertMs    = C->LastConvertMs;
    P->MaxConvertMs     = C->MaxConvertMs;
    P->LastStageMs      = C->LastStageMs;
    P->MaxStageMs       = C->MaxStageMs;
    P->LastLazMs        = C->LastLazMs;
    P->MaxLazMs         = C->MaxLazMs;
    P->LastLatencyS     = C->LastLatencyS;
    P->MaxLatencyS      = C->MaxLatencyS;

    CFE_SB_TimeStampMsg(CFE_MSG_PTR(GEOREF_APP_Data.HkTlm.TelemetryHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(GEOREF_APP_Data.HkTlm.TelemetryHeader), true);
    return CFE_SUCCESS;
}

/* ------------------------------------------------------------------ main */

void GEOREF_APP_Main(void)
{
    CFE_Status_t     Status;
    CFE_SB_Buffer_t *Buf = NULL;
    CFE_SB_MsgId_t   MsgId;

    Status = GEOREF_APP_Init();
    if (Status != CFE_SUCCESS)
    {
        GEOREF_APP_Data.RunStatus = CFE_ES_RunStatus_APP_ERROR;
    }

    while (CFE_ES_RunLoop(&GEOREF_APP_Data.RunStatus))
    {
        Status = CFE_SB_ReceiveBuffer(&Buf, GEOREF_APP_Data.Pipe, GEOREF_APP_PIPE_TIMEOUT_MS);
        if (Status == CFE_SUCCESS)
        {
            MsgId = CFE_SB_INVALID_MSG_ID;
            CFE_MSG_GetMsgId(&Buf->Msg, &MsgId);
            if (!GEOREF_APP_ProcessInput(Buf, MsgId))
            {
                GEOREF_APP_ProcessCommand(Buf, MsgId);
            }
        }
        else if (Status != CFE_SB_TIME_OUT && Status != CFE_SB_NO_MESSAGE)
        {
            CFE_EVS_SendEvent(GEOREF_APP_PIPE_ERR_EID, CFE_EVS_EventType_ERROR, "pipe read error 0x%08lX",
                              (unsigned long)Status);
            GEOREF_APP_Data.RunStatus = CFE_ES_RunStatus_APP_ERROR;
        }
    }
    CFE_ES_ExitApp(GEOREF_APP_Data.RunStatus);
}
