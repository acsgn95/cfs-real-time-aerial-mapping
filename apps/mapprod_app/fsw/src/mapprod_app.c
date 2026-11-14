/*
 * MAPPROD_APP main task: initialisation, commands, housekeeping, the
 * publication period and the worker queue (mapprod_app.h).
 */
#include "mapprod_app.h"
#include "mapprod_app_eventids.h"
#include "mapprod_app_version.h"

#include "cfe_config.h"
#include "cfe_core_api_base_msgids.h"
#include "cfe_mission_eds_designparameters.h"
#include "mapprod_app_eds_dictionary.h"
#include "mapprod_app_eds_dispatcher.h"
#include "ost_datastore.h"

#include <math.h>
#include <string.h>

MAPPROD_APP_Data_t MAPPROD_APP_Data;

static const OST_FaultId_Enum_t MAPPROD_APP_FaultIds[MAPPROD_APP_FAULT_COUNT] = {
    OST_FaultId_MAP_UPDATE_FAILED, /* 60 */
    OST_FaultId_IMAGE_DECODE,      /* 61 */
    OST_FaultId_TILE_WRITE,        /* 62 */
    OST_FaultId_TABLE_REJECTED,    /* 63 */
    OST_FaultId_LIBRARY_ERROR,     /* 64 */
};

static CFE_EVS_BinFilter_t MAPPROD_APP_EventFilters[] = {
    {MAPPROD_APP_FAILED_EID, CFE_EVS_FIRST_8_STOP},      {MAPPROD_APP_MSG_LEN_ERR_EID, CFE_EVS_FIRST_8_STOP},
    {MAPPROD_APP_LIB_ERR_EID, CFE_EVS_FIRST_8_STOP},     {MAPPROD_APP_DATASTORE_ERR_EID, CFE_EVS_FIRST_8_STOP},
    {MAPPROD_APP_DECODE_ERR_EID, CFE_EVS_FIRST_8_STOP},  {MAPPROD_APP_WRITE_ERR_EID, CFE_EVS_FIRST_8_STOP},
    {MAPPROD_APP_NO_ORTHO_EID, CFE_EVS_FIRST_8_STOP},    {MAPPROD_APP_GRID_ERR_EID, CFE_EVS_FIRST_8_STOP},
};

/* clang-format off */
static const EdsDispatchTable_EdsComponent_MAPPROD_APP_Application_CFE_SB_Telecommand_t MAPPROD_APP_TC_DISPATCH_TABLE = {
    .CMD = {
        .NoopCmd_indication               = MAPPROD_APP_NoopCmd,
        .ResetCountersCmd_indication      = MAPPROD_APP_ResetCountersCmd,
        .ClearLatchedFaultsCmd_indication = MAPPROD_APP_ClearLatchedFaultsCmd,
        .FinaliseAllCmd_indication        = MAPPROD_APP_FinaliseAllCmd,
    },
    .SEND_HK = { .indication = MAPPROD_APP_SendHkCmd },
    .WAKEUP  = { .indication = MAPPROD_APP_WakeupCmd },
};
/* clang-format on */

static CFE_SB_MsgId_t MAPPROD_APP_CmdMid(uint16 TopicId)
{
    return CFE_SB_ValueToMsgId(CFE_PLATFORM_CMD_TOPICID_TO_MIDV(TopicId));
}

static CFE_SB_MsgId_t MAPPROD_APP_TlmMid(uint16 TopicId)
{
    return CFE_SB_ValueToMsgId(CFE_PLATFORM_TLM_TOPICID_TO_MIDV(TopicId));
}

/* -------------------------------------------------------- shared helpers */

void MAPPROD_APP_FaultReport(OST_FaultId_Enum_t Id, const char *Text)
{
    OS_MutSemTake(MAPPROD_APP_Data.FaultMutex);
    OST_FaultSet_Report(&MAPPROD_APP_Data.Faults, Id, Text);
    OS_MutSemGive(MAPPROD_APP_Data.FaultMutex);
}

void MAPPROD_APP_FaultRaise(OST_FaultId_Enum_t Id, const char *Text)
{
    OS_MutSemTake(MAPPROD_APP_Data.FaultMutex);
    OST_FaultSet_Raise(&MAPPROD_APP_Data.Faults, Id, Text);
    OS_MutSemGive(MAPPROD_APP_Data.FaultMutex);
}

void MAPPROD_APP_FaultResolve(OST_FaultId_Enum_t Id)
{
    OS_MutSemTake(MAPPROD_APP_Data.FaultMutex);
    OST_FaultSet_Resolve(&MAPPROD_APP_Data.Faults, Id);
    OS_MutSemGive(MAPPROD_APP_Data.FaultMutex);
}

void MAPPROD_APP_Release(const OST_ObjectRef_t *Ref)
{
    OST_Status_t Status;

    if (Ref->ObjectId == 0)
    {
        return;
    }
    Status = OST_DS_Release(Ref);
    if (Status != OST_SUCCESS)
    {
        CFE_EVS_SendEvent(MAPPROD_APP_DATASTORE_ERR_EID, CFE_EVS_EventType_ERROR, "object %lu/%lu not released: %s",
                          (unsigned long)Ref->ObjectId, (unsigned long)Ref->Generation, OST_StatusName(Status));
    }
}

void MAPPROD_APP_Drop(const GEOREF_APP_PointsReady_Payload_t *P, const char *Text)
{
    MAPPROD_APP_Release(&P->PointSet);
    MAPPROD_APP_Release(&P->Keyframe.WorkImage); /* 0 from GEOREF_APP */
    MAPPROD_APP_Release(&P->Keyframe.Jpeg);
    OS_MutSemTake(MAPPROD_APP_Data.CountMutex);
    ++MAPPROD_APP_Data.Counters.KeyframesFailed;
    OS_MutSemGive(MAPPROD_APP_Data.CountMutex);
    CFE_EVS_SendEvent(MAPPROD_APP_FAILED_EID, CFE_EVS_EventType_ERROR, "keyframe %lu (image %lu) dropped: %s",
                      (unsigned long)P->KeyframeId, (unsigned long)P->Keyframe.ImageSeq, Text);
    MAPPROD_APP_FaultReport(OST_FaultId_MAP_UPDATE_FAILED, "keyframe not in the map");
}

/* ------------------------------------------------------------- the queue */

uint32 MAPPROD_APP_QueueDepth(void)
{
    uint32 Depth;

    OS_MutSemTake(MAPPROD_APP_Data.RingMutex);
    Depth = MAPPROD_APP_Data.QueuedItems - MAPPROD_APP_Data.TakenItems;
    OS_MutSemGive(MAPPROD_APP_Data.RingMutex);
    return Depth;
}

/* Puts an item into the ring; false when the ring is full. */
static bool MAPPROD_APP_Enqueue(const MAPPROD_APP_Item_t *Item)
{
    bool Full;

    OS_MutSemTake(MAPPROD_APP_Data.RingMutex);
    Full = MAPPROD_APP_Data.QueuedItems - MAPPROD_APP_Data.TakenItems >= MAPPROD_APP_QUEUE_DEPTH;
    if (!Full)
    {
        MAPPROD_APP_Data.Ring[MAPPROD_APP_Data.QueuedItems % MAPPROD_APP_QUEUE_DEPTH] = *Item;
        ++MAPPROD_APP_Data.QueuedItems;
    }
    OS_MutSemGive(MAPPROD_APP_Data.RingMutex);
    if (!Full)
    {
        OS_CountSemGive(MAPPROD_APP_Data.ItemSem);
    }
    return !Full;
}

bool MAPPROD_APP_Dequeue(MAPPROD_APP_Item_t *Item)
{
    if (OS_CountSemTimedWait(MAPPROD_APP_Data.ItemSem, 500) != OS_SUCCESS)
    {
        return false;
    }
    OS_MutSemTake(MAPPROD_APP_Data.RingMutex);
    *Item = MAPPROD_APP_Data.Ring[MAPPROD_APP_Data.TakenItems % MAPPROD_APP_QUEUE_DEPTH];
    ++MAPPROD_APP_Data.TakenItems;
    OS_MutSemGive(MAPPROD_APP_Data.RingMutex);
    return true;
}

/* ----------------------------------------------------------------- table */

static bool MAPPROD_APP_HasText(const char *Text, size_t Len)
{
    return Len > 0 && Text[0] != '\0' && memchr(Text, '\0', Len) != NULL;
}

static CFE_Status_t MAPPROD_APP_ValidateConfig(void *TblData)
{
    const MAPPROD_APP_ConfigTable_t *T   = (const MAPPROD_APP_ConfigTable_t *)TblData;
    const float                      F[] = {T->CellSizeM,       T->RejectionSigmas, T->MinSigmaM,
                                            T->MinConfidence,   T->OrthoPixelM,     T->SurfaceSmoothingM,
                                            T->SwitchMarginDeg, T->RenderMarginM,   T->FinaliseDistanceM};
    uint32                           i;

    for (i = 0; i < sizeof(F) / sizeof(F[0]); ++i)
    {
        if (!isfinite(F[i]) || F[i] < 0.0f)
        {
            CFE_EVS_SendEvent(MAPPROD_APP_TBL_ERR_EID, CFE_EVS_EventType_ERROR,
                              "ConfigTable: value %lu not finite or negative", (unsigned long)i);
            return CFE_STATUS_VALIDATION_FAILURE;
        }
    }
    if (T->CellSizeM <= 0.0f || T->TileCells < 16 || T->TileCells > 8192 || T->MinObservations < 1 ||
        T->RejectionSigmas <= 0.0f || T->MinConfidence > 1.0f || T->OrthoPixelM <= 0.0f ||
        T->FinaliseDistanceM <= 0.0f || T->MaxActiveTiles < 1 || T->RenderOrtho > 1 || T->FinaliseOnStop > 1 ||
        !MAPPROD_APP_HasText(T->ProjectedCrs, sizeof(T->ProjectedCrs)) ||
        !MAPPROD_APP_HasText(T->ProjDataDir, sizeof(T->ProjDataDir)) ||
        !MAPPROD_APP_HasText(T->OutputDir, sizeof(T->OutputDir)) ||
        memchr(T->VerticalCrs, '\0', sizeof(T->VerticalCrs)) == NULL ||
        memchr(T->GridDir, '\0', sizeof(T->GridDir)) == NULL)
    {
        CFE_EVS_SendEvent(MAPPROD_APP_TBL_ERR_EID, CFE_EVS_EventType_ERROR,
                          "ConfigTable: limits invalid (cell size and pixel size > 0, 16-8192 cells per tile, "
                          "observations and active tiles >= 1, confidence 0-1, CRS and directories set)");
        return CFE_STATUS_VALIDATION_FAILURE;
    }
    return CFE_SUCCESS;
}

static void MAPPROD_APP_LoadConfig(void)
{
    CFE_Status_t Status;
    void        *Addr = NULL;

    Status = CFE_TBL_Register(&MAPPROD_APP_Data.TblHandle, "ConfigTable", sizeof(MAPPROD_APP_ConfigTable_t),
                              CFE_TBL_OPT_DEFAULT, MAPPROD_APP_ValidateConfig);
    if (Status == CFE_SUCCESS)
    {
        Status = CFE_TBL_Load(MAPPROD_APP_Data.TblHandle, CFE_TBL_SRC_FILE, MAPPROD_APP_TABLE_FILE);
    }
    if (Status == CFE_SUCCESS)
    {
        Status = CFE_TBL_GetAddress(&Addr, MAPPROD_APP_Data.TblHandle);
        if (Status == CFE_TBL_INFO_UPDATED)
        {
            Status = CFE_SUCCESS;
        }
    }
    if (Status == CFE_SUCCESS && Addr != NULL)
    {
        memcpy(&MAPPROD_APP_Data.Config, Addr, sizeof(MAPPROD_APP_Data.Config));
        CFE_TBL_ReleaseAddress(MAPPROD_APP_Data.TblHandle);
        MAPPROD_APP_Data.ConfigValid = true;
        return;
    }
    CFE_EVS_SendEvent(MAPPROD_APP_TBL_ERR_EID, CFE_EVS_EventType_ERROR,
                      "ConfigTable %s not loaded (0x%08lX); point sets are dropped", MAPPROD_APP_TABLE_FILE,
                      (unsigned long)Status);
    MAPPROD_APP_FaultRaise(OST_FaultId_TABLE_REJECTED, "ConfigTable not loaded");
}

/* --------------------------------------------------------- input messages */

static bool MAPPROD_APP_ProcessInput(const CFE_SB_Buffer_t *Buf, CFE_SB_MsgId_t MsgId)
{
    static MAPPROD_APP_Item_t Item; /* main task only */
    CFE_MSG_Size_t            Size = 0;
    bool                      Points;

    Points = CFE_SB_MsgId_Equal(MsgId, MAPPROD_APP_Data.PointsReadyMid);
    if (!Points && !CFE_SB_MsgId_Equal(MsgId, MAPPROD_APP_Data.CalibrationMid))
    {
        return false;
    }
    CFE_MSG_GetSize(&Buf->Msg, &Size);
    if (Size != (Points ? sizeof(GEOREF_APP_PointsReadyTlm_t) : sizeof(INGEST_APP_SessionCalibrationTlm_t)))
    {
        /* Its references cannot be trusted. */
        ++MAPPROD_APP_Data.Counters.MessagesRejected;
        CFE_EVS_SendEvent(MAPPROD_APP_MSG_LEN_ERR_EID, CFE_EVS_EventType_ERROR, "%s: length %lu",
                          Points ? "point set" : "calibration", (unsigned long)Size);
        return true;
    }
    memset(&Item, 0, sizeof(Item));
    Item.EntryMono = OST_TimeMonoNs();
    if (Points)
    {
        const GEOREF_APP_PointsReady_Payload_t *P = &((const GEOREF_APP_PointsReadyTlm_t *)Buf)->Payload;

        ++MAPPROD_APP_Data.Counters.PointSetsReceived;
        if (!MAPPROD_APP_Data.ConfigValid)
        {
            MAPPROD_APP_Drop(P, "no configuration table");
            return true;
        }
        Item.Type     = MAPPROD_APP_ITEM_POINTS;
        Item.u.Points = *P;
        if (!MAPPROD_APP_Enqueue(&Item))
        {
            MAPPROD_APP_Drop(P, "worker queue full");
        }
        return true;
    }
    Item.Type          = MAPPROD_APP_ITEM_CALIBRATION;
    Item.u.Calibration = ((const INGEST_APP_SessionCalibrationTlm_t *)Buf)->Payload;
    if (!MAPPROD_APP_Enqueue(&Item))
    {
        CFE_EVS_SendEvent(MAPPROD_APP_FAILED_EID, CFE_EVS_EventType_ERROR,
                          "calibration of session %lu dropped: worker queue full",
                          (unsigned long)Item.u.Calibration.SessionId);
    }
    return true;
}

/* ------------------------------------------------------------------ init */

static CFE_Status_t MAPPROD_APP_Init(void)
{
    CFE_Status_t    Status;
    CFE_ES_TaskId_t TaskId;
    char            VersionString[MAPPROD_APP_MAX_VERSION_STR_LEN];

    memset(&MAPPROD_APP_Data, 0, sizeof(MAPPROD_APP_Data));
    MAPPROD_APP_Data.RunStatus = CFE_ES_RunStatus_APP_RUN;

    Status = CFE_EVS_Register(MAPPROD_APP_EventFilters,
                              sizeof(MAPPROD_APP_EventFilters) / sizeof(MAPPROD_APP_EventFilters[0]),
                              CFE_EVS_EventFilter_BINARY);
    if (Status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("MAPPROD_APP: EVS register failed 0x%08lX\n", (unsigned long)Status);
        return Status;
    }
    if (OS_MutSemCreate(&MAPPROD_APP_Data.FaultMutex, "MAPPROD_FLT", 0) != OS_SUCCESS ||
        OS_MutSemCreate(&MAPPROD_APP_Data.CountMutex, "MAPPROD_CNT", 0) != OS_SUCCESS ||
        OS_MutSemCreate(&MAPPROD_APP_Data.RingMutex, "MAPPROD_RING", 0) != OS_SUCCESS ||
        OS_CountSemCreate(&MAPPROD_APP_Data.ItemSem, "MAPPROD_ITEMS", 0, 0) != OS_SUCCESS ||
        OS_BinSemCreate(&MAPPROD_APP_Data.WorkerDone, "MAPPROD_DONE", 0, 0) != OS_SUCCESS ||
        OS_BinSemCreate(&MAPPROD_APP_Data.PublishSem, "MAPPROD_PUBS", 0, 0) != OS_SUCCESS ||
        OS_BinSemCreate(&MAPPROD_APP_Data.PublisherDone, "MAPPROD_PUBD", 0, 0) != OS_SUCCESS)
    {
        CFE_EVS_SendEvent(MAPPROD_APP_INIT_ERR_EID, CFE_EVS_EventType_ERROR, "mutexes or worker queue not created");
        return CFE_STATUS_EXTERNAL_RESOURCE_FAIL;
    }
    Status = OST_FaultSet_Init(&MAPPROD_APP_Data.Faults, OST_AppId_MAPPROD_APP, MAPPROD_APP_FaultIds,
                               MAPPROD_APP_FAULT_COUNT);
    if (Status != CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(MAPPROD_APP_INIT_ERR_EID, CFE_EVS_EventType_ERROR, "fault set: 0x%08lX",
                          (unsigned long)Status);
        return Status;
    }

    CFE_MSG_Init(CFE_MSG_PTR(MAPPROD_APP_Data.HkTlm.TelemetryHeader),
                 MAPPROD_APP_TlmMid(EdsParam_CFE_MISSION_MAPPROD_APP_HK_TLM_TOPICID), sizeof(MAPPROD_APP_Data.HkTlm));
    MAPPROD_APP_Data.PointsReadyMid = MAPPROD_APP_TlmMid(EdsParam_CFE_MISSION_GEOREF_APP_POINTS_READY_TLM_TOPICID);
    MAPPROD_APP_Data.CalibrationMid =
        MAPPROD_APP_TlmMid(EdsParam_CFE_MISSION_INGEST_APP_SESSION_CALIBRATION_TLM_TOPICID);

    Status = CFE_SB_CreatePipe(&MAPPROD_APP_Data.Pipe, MAPPROD_APP_PIPE_DEPTH, "MAPPROD_PIPE");
    if (Status == CFE_SUCCESS)
    {
        Status = CFE_SB_SubscribeEx(MAPPROD_APP_CmdMid(EdsParam_CFE_MISSION_MAPPROD_APP_CMD_TOPICID),
                                    MAPPROD_APP_Data.Pipe, CFE_SB_DEFAULT_QOS, 4);
    }
    if (Status == CFE_SUCCESS)
    {
        Status = CFE_SB_SubscribeEx(MAPPROD_APP_CmdMid(EdsParam_CFE_MISSION_MAPPROD_APP_SEND_HK_TOPICID),
                                    MAPPROD_APP_Data.Pipe, CFE_SB_DEFAULT_QOS, 2);
    }
    if (Status == CFE_SUCCESS)
    {
        Status = CFE_SB_SubscribeEx(MAPPROD_APP_CmdMid(EdsParam_CFE_MISSION_MAPPROD_APP_WAKEUP_TOPICID),
                                    MAPPROD_APP_Data.Pipe, CFE_SB_DEFAULT_QOS, 2);
    }
    if (Status == CFE_SUCCESS)
    {
        Status = CFE_SB_SubscribeEx(MAPPROD_APP_Data.CalibrationMid, MAPPROD_APP_Data.Pipe, CFE_SB_DEFAULT_QOS, 2);
    }
    if (Status == CFE_SUCCESS)
    {
        /* A point set lost to a full pipe would also lose its counts; the
         * main task takes them off the pipe at once. */
        Status = CFE_SB_SubscribeEx(MAPPROD_APP_Data.PointsReadyMid, MAPPROD_APP_Data.Pipe, CFE_SB_DEFAULT_QOS, 20);
    }
    if (Status != CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(MAPPROD_APP_INIT_ERR_EID, CFE_EVS_EventType_ERROR, "pipe or subscription: 0x%08lX",
                          (unsigned long)Status);
        return Status;
    }

    MAPPROD_APP_LoadConfig();

    Status = CFE_ES_CreateChildTask(&TaskId, "MAPPROD_WORK", MAPPROD_APP_WorkerTask, NULL,
                                    MAPPROD_APP_WORKER_STACK_SIZE, MAPPROD_APP_WORKER_PRIORITY, 0);
    if (Status != CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(MAPPROD_APP_INIT_ERR_EID, CFE_EVS_EventType_ERROR, "worker task: 0x%08lX",
                          (unsigned long)Status);
        return Status;
    }
    MAPPROD_APP_Data.WorkerStarted = true;
    Status = CFE_ES_CreateChildTask(&TaskId, "MAPPROD_PUB", MAPPROD_APP_PublisherTask, NULL,
                                    MAPPROD_APP_PUBLISHER_STACK_SIZE, MAPPROD_APP_PUBLISHER_PRIORITY, 0);
    if (Status != CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(MAPPROD_APP_INIT_ERR_EID, CFE_EVS_EventType_ERROR, "publisher task: 0x%08lX",
                          (unsigned long)Status);
        return Status;
    }

    CFE_Config_GetVersionString(VersionString, MAPPROD_APP_MAX_VERSION_STR_LEN, "MAPPROD_APP", MAPPROD_APP_VERSION,
                                MAPPROD_APP_BUILD_CODENAME, MAPPROD_APP_LAST_OFFICIAL);
    CFE_EVS_SendEvent(MAPPROD_APP_INIT_INF_EID, CFE_EVS_EventType_INFORMATION,
                      "MAPPROD_APP initialised: P %.32s, DSM %.2f m in tiles of %lu, orthomosaic %.3f m %s, "
                      "publication every %lu s. %s",
                      MAPPROD_APP_Data.Config.ProjectedCrs, MAPPROD_APP_Data.Config.CellSizeM,
                      (unsigned long)MAPPROD_APP_Data.Config.TileCells, MAPPROD_APP_Data.Config.OrthoPixelM,
                      MAPPROD_APP_Data.Config.RenderOrtho ? "on" : "off",
                      (unsigned long)MAPPROD_APP_Data.Config.PublishPeriodS, VersionString);
    return CFE_SUCCESS;
}

/* -------------------------------------------------------------- commands */

static void MAPPROD_APP_ProcessCommand(const CFE_SB_Buffer_t *Buf, CFE_SB_MsgId_t MsgId)
{
    CFE_Status_t      Status;
    CFE_MSG_Size_t    Size = 0;
    CFE_MSG_FcnCode_t Fc   = 0;

    Status = EdsDispatch_EdsComponent_MAPPROD_APP_Application_Telecommand(Buf, &MAPPROD_APP_TC_DISPATCH_TABLE);
    if (Status == CFE_SUCCESS)
    {
        return;
    }
    CFE_MSG_GetSize(&Buf->Msg, &Size);
    CFE_MSG_GetFcnCode(&Buf->Msg, &Fc);
    ++MAPPROD_APP_Data.CommandErrorCounter;
    if (Status == CFE_STATUS_UNKNOWN_MSG_ID)
    {
        CFE_EVS_SendEvent(MAPPROD_APP_MID_ERR_EID, CFE_EVS_EventType_ERROR, "invalid message ID 0x%x",
                          (unsigned)CFE_SB_MsgIdToValue(MsgId));
    }
    else if (Status == CFE_STATUS_WRONG_MSG_LENGTH)
    {
        CFE_EVS_SendEvent(MAPPROD_APP_CMD_LEN_ERR_EID, CFE_EVS_EventType_ERROR,
                          "invalid length: MID 0x%x, CC %u, length %u", (unsigned)CFE_SB_MsgIdToValue(MsgId),
                          (unsigned)Fc, (unsigned)Size);
    }
    else
    {
        CFE_EVS_SendEvent(MAPPROD_APP_CC_ERR_EID, CFE_EVS_EventType_ERROR, "invalid command code %u", (unsigned)Fc);
    }
}

CFE_Status_t MAPPROD_APP_NoopCmd(const MAPPROD_APP_NoopCmd_t *Msg)
{
    (void)Msg;
    ++MAPPROD_APP_Data.CommandCounter;
    CFE_EVS_SendEvent(MAPPROD_APP_NOOP_INF_EID, CFE_EVS_EventType_INFORMATION, "no-op, version %s",
                      MAPPROD_APP_VERSION);
    return CFE_SUCCESS;
}

CFE_Status_t MAPPROD_APP_ResetCountersCmd(const MAPPROD_APP_ResetCountersCmd_t *Msg)
{
    MAPPROD_APP_Counters_t *C = &MAPPROD_APP_Data.Counters;

    (void)Msg;
    MAPPROD_APP_Data.CommandCounter      = 0;
    MAPPROD_APP_Data.CommandErrorCounter = 0;
    C->PointSetsReceived = C->MessagesRejected = C->KeyframesUpdated = C->OrthoSkipped = 0;
    C->TilesFinalised = 0;
    OS_MutSemTake(MAPPROD_APP_Data.CountMutex);
    C->KeyframesFailed = C->Publications = C->TileWriteFailures = 0;
    C->MaxPublishMs    = 0.0f;
    OS_MutSemGive(MAPPROD_APP_Data.CountMutex);
    C->MaxUpdateMs = C->MaxLatencyS = 0.0f;
    CFE_EVS_SendEvent(MAPPROD_APP_RESET_INF_EID, CFE_EVS_EventType_INFORMATION, "counters reset");
    return CFE_SUCCESS;
}

CFE_Status_t MAPPROD_APP_ClearLatchedFaultsCmd(const MAPPROD_APP_ClearLatchedFaultsCmd_t *Msg)
{
    uint32 Cleared;

    (void)Msg;
    OS_MutSemTake(MAPPROD_APP_Data.FaultMutex);
    Cleared = OST_FaultSet_ClearLatched(&MAPPROD_APP_Data.Faults);
    OS_MutSemGive(MAPPROD_APP_Data.FaultMutex);
    ++MAPPROD_APP_Data.CommandCounter;
    CFE_EVS_SendEvent(MAPPROD_APP_CLEAR_INF_EID, CFE_EVS_EventType_INFORMATION, "%lu latched faults cleared",
                      (unsigned long)Cleared);
    return CFE_SUCCESS;
}

CFE_Status_t MAPPROD_APP_FinaliseAllCmd(const MAPPROD_APP_FinaliseAllCmd_t *Msg)
{
    static MAPPROD_APP_Item_t Item; /* main task only */

    (void)Msg;
    memset(&Item, 0, sizeof(Item));
    Item.Type      = MAPPROD_APP_ITEM_FINALISE_ALL;
    Item.EntryMono = OST_TimeMonoNs();
    if (!MAPPROD_APP_Enqueue(&Item))
    {
        ++MAPPROD_APP_Data.CommandErrorCounter;
        CFE_EVS_SendEvent(MAPPROD_APP_CC_ERR_EID, CFE_EVS_EventType_ERROR, "finalise all: worker queue full");
        return CFE_SUCCESS;
    }
    ++MAPPROD_APP_Data.CommandCounter;
    return CFE_SUCCESS;
}

CFE_Status_t MAPPROD_APP_WakeupCmd(const MAPPROD_APP_WakeupCmd_t *Msg)
{
    static MAPPROD_APP_Item_t Item; /* main task only */
    const uint32              Period = MAPPROD_APP_Data.Config.PublishPeriodS;

    (void)Msg;
    if (!MAPPROD_APP_Data.ConfigValid || Period == 0)
    {
        return CFE_SUCCESS;
    }
    if (++MAPPROD_APP_Data.SecondsSincePublish < Period || MAPPROD_APP_Data.PublishQueued)
    {
        return CFE_SUCCESS;
    }
    memset(&Item, 0, sizeof(Item));
    Item.Type      = MAPPROD_APP_ITEM_PUBLISH;
    Item.EntryMono = OST_TimeMonoNs();
    MAPPROD_APP_Data.PublishQueued = true;
    if (MAPPROD_APP_Enqueue(&Item))
    {
        MAPPROD_APP_Data.SecondsSincePublish = 0;
    }
    else
    {
        MAPPROD_APP_Data.PublishQueued = false; /* tried again next second */
    }
    return CFE_SUCCESS;
}

CFE_Status_t MAPPROD_APP_SendHkCmd(const MAPPROD_APP_SendHkCmd_t *Msg)
{
    MAPPROD_APP_HkTlm_Payload_t  *P = &MAPPROD_APP_Data.HkTlm.Payload;
    const MAPPROD_APP_Counters_t *C = &MAPPROD_APP_Data.Counters;

    (void)Msg;
    P->Common.CommandCounter      = MAPPROD_APP_Data.CommandCounter;
    P->Common.CommandErrorCounter = MAPPROD_APP_Data.CommandErrorCounter;
    OS_MutSemTake(MAPPROD_APP_Data.FaultMutex);
    OST_FaultSet_Summarise(&MAPPROD_APP_Data.Faults, &P->Common);
    OS_MutSemGive(MAPPROD_APP_Data.FaultMutex);
    P->Ready             = C->Ready ? 1 : 0;
    P->CalibrationKnown  = C->CalibrationKnown ? 1 : 0;
    P->RenderOrtho       = MAPPROD_APP_Data.Config.RenderOrtho;
    P->PointSetsReceived = C->PointSetsReceived;
    P->KeyframesUpdated  = C->KeyframesUpdated;
    OS_MutSemTake(MAPPROD_APP_Data.CountMutex);
    P->KeyframesFailed   = C->KeyframesFailed;
    P->Publications      = C->Publications;
    P->TileWriteFailures = C->TileWriteFailures;
    P->LastPublishMs     = C->LastPublishMs;
    P->MaxPublishMs      = C->MaxPublishMs;
    OS_MutSemGive(MAPPROD_APP_Data.CountMutex);
    P->OrthoSkipped      = C->OrthoSkipped;
    P->MessagesRejected  = C->MessagesRejected;
    P->QueueDepth        = MAPPROD_APP_QueueDepth();
    P->ActiveTiles       = C->ActiveTiles;
    P->ChangedTiles      = C->ChangedTiles;
    P->TilesFinalised    = C->TilesFinalised;
    P->LastKeyframeId    = C->LastKeyframeId;
    P->MemoryMb          = C->MemoryMb;
    P->LastFuseMs        = C->LastFuseMs;
    P->LastDecodeMs      = C->LastDecodeMs;
    P->LastRenderMs      = C->LastRenderMs;
    P->LastLifecycleMs   = C->LastLifecycleMs;
    P->LastUpdateMs      = C->LastUpdateMs;
    P->MaxUpdateMs       = C->MaxUpdateMs;
    P->LastLatencyS      = C->LastLatencyS;
    P->MaxLatencyS       = C->MaxLatencyS;

    CFE_SB_TimeStampMsg(CFE_MSG_PTR(MAPPROD_APP_Data.HkTlm.TelemetryHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(MAPPROD_APP_Data.HkTlm.TelemetryHeader), true);
    return CFE_SUCCESS;
}

/* ------------------------------------------------------------------ main */

void MAPPROD_APP_Main(void)
{
    CFE_Status_t     Status;
    CFE_SB_Buffer_t *Buf = NULL;
    CFE_SB_MsgId_t   MsgId;

    Status = MAPPROD_APP_Init();
    if (Status != CFE_SUCCESS)
    {
        MAPPROD_APP_Data.RunStatus = CFE_ES_RunStatus_APP_ERROR;
    }

    while (CFE_ES_RunLoop(&MAPPROD_APP_Data.RunStatus))
    {
        Status = CFE_SB_ReceiveBuffer(&Buf, MAPPROD_APP_Data.Pipe, MAPPROD_APP_PIPE_TIMEOUT_MS);
        if (Status == CFE_SUCCESS)
        {
            MsgId = CFE_SB_INVALID_MSG_ID;
            CFE_MSG_GetMsgId(&Buf->Msg, &MsgId);
            if (!MAPPROD_APP_ProcessInput(Buf, MsgId))
            {
                MAPPROD_APP_ProcessCommand(Buf, MsgId);
            }
        }
        else if (Status != CFE_SB_TIME_OUT && Status != CFE_SB_NO_MESSAGE)
        {
            CFE_EVS_SendEvent(MAPPROD_APP_PIPE_ERR_EID, CFE_EVS_EventType_ERROR, "pipe read error 0x%08lX",
                              (unsigned long)Status);
            MAPPROD_APP_Data.RunStatus = CFE_ES_RunStatus_APP_ERROR;
        }
    }
    /* Child tasks end with the application: let the worker release the
     * queued point sets and write the active tiles first. */
    if (MAPPROD_APP_Data.WorkerStarted)
    {
        OS_BinSemTimedWait(MAPPROD_APP_Data.WorkerDone, MAPPROD_APP_STOP_WAIT_MS);
    }
    CFE_ES_ExitApp(MAPPROD_APP_Data.RunStatus);
}
