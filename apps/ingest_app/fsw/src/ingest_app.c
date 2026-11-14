/*
 * INGEST_APP main task: initialisation, commands, housekeeping and the
 * staleness faults (ADR-014, ADR-015, ADR-020).
 */
#include "ingest_app.h"
#include "ingest_app_eventids.h"
#include "ingest_app_version.h"

#include "cfe_config.h"
#include "cfe_core_api_base_msgids.h"
#include "cfe_mission_eds_designparameters.h"
#include "ingest_app_eds_dictionary.h"
#include "ingest_app_eds_dispatcher.h"

#include <string.h>

INGEST_APP_Data_t INGEST_APP_Data;

static const OST_FaultId_Enum_t INGEST_APP_FaultIds[INGEST_APP_FAULT_COUNT] = {
    OST_FaultId_TELEMETRY_STALE,    /* 60 */
    OST_FaultId_IMAGE_STREAM_STALE, /* 61 */
    OST_FaultId_LINK_FORMAT,        /* 62 */
    OST_FaultId_DATASTORE_QUOTA,    /* 63 */
    OST_FaultId_TABLE_REJECTED,     /* 64 */
};

/* Per-item events repeat; the first 8 are sent (ADR-020 section 3), the
 * counts are in housekeeping. A reset of the EVS filter counters re-enables
 * them. */
static CFE_EVS_BinFilter_t INGEST_APP_EventFilters[] = {
    {INGEST_APP_IMAGE_DROPPED_EID, CFE_EVS_FIRST_8_STOP},
    {INGEST_APP_FRAME_REJECTED_EID, CFE_EVS_FIRST_8_STOP},
    {INGEST_APP_TLM_REJECTED_EID, CFE_EVS_FIRST_8_STOP},
    {INGEST_APP_SOCKET_ERR_EID, CFE_EVS_FIRST_8_STOP},
};

/* clang-format off */
static const EdsDispatchTable_EdsComponent_INGEST_APP_Application_CFE_SB_Telecommand_t INGEST_APP_TC_DISPATCH_TABLE = {
    .CMD = {
        .NoopCmd_indication               = INGEST_APP_NoopCmd,
        .ResetCountersCmd_indication      = INGEST_APP_ResetCountersCmd,
        .ClearLatchedFaultsCmd_indication = INGEST_APP_ClearLatchedFaultsCmd,
        .StartSessionCmd_indication       = INGEST_APP_StartSessionCmd,
        .StopSessionCmd_indication        = INGEST_APP_StopSessionCmd,
    },
    .SEND_HK = { .indication = INGEST_APP_SendHkCmd },
    .WAKEUP  = { .indication = INGEST_APP_WakeupCmd },
};
/* clang-format on */

static CFE_SB_MsgId_t INGEST_APP_CmdMid(uint16 TopicId)
{
    return CFE_SB_ValueToMsgId(CFE_PLATFORM_CMD_TOPICID_TO_MIDV(TopicId));
}

/* ---------------------------------------------------------------- faults */

void INGEST_APP_FaultReport(OST_FaultId_Enum_t Id, const char *Text)
{
    OS_MutSemTake(INGEST_APP_Data.FaultMutex);
    OST_FaultSet_Report(&INGEST_APP_Data.Faults, Id, Text);
    OS_MutSemGive(INGEST_APP_Data.FaultMutex);
}

void INGEST_APP_FaultRaise(OST_FaultId_Enum_t Id, const char *Text)
{
    OS_MutSemTake(INGEST_APP_Data.FaultMutex);
    OST_FaultSet_Raise(&INGEST_APP_Data.Faults, Id, Text);
    OS_MutSemGive(INGEST_APP_Data.FaultMutex);
}

void INGEST_APP_FaultResolve(OST_FaultId_Enum_t Id)
{
    OS_MutSemTake(INGEST_APP_Data.FaultMutex);
    OST_FaultSet_Resolve(&INGEST_APP_Data.Faults, Id);
    OS_MutSemGive(INGEST_APP_Data.FaultMutex);
}

/* ----------------------------------------------------------------- table */

static CFE_Status_t INGEST_APP_ValidateConfig(void *TblData)
{
    const INGEST_APP_ConfigTable_t *T = (const INGEST_APP_ConfigTable_t *)TblData;

    if (T->UdpPort == 0 || T->TcpPort == 0 || T->UdpPort == T->TcpPort)
    {
        CFE_EVS_SendEvent(INGEST_APP_TBL_ERR_EID, CFE_EVS_EventType_ERROR,
                          "ConfigTable: ports invalid (UDP %u, TCP %u)", (unsigned)T->UdpPort, (unsigned)T->TcpPort);
        return CFE_STATUS_VALIDATION_FAILURE;
    }
    if (T->TelemetryStaleMs == 0 || T->ImageStaleMs == 0 || T->AutoStart > 1)
    {
        CFE_EVS_SendEvent(INGEST_APP_TBL_ERR_EID, CFE_EVS_EventType_ERROR,
                          "ConfigTable: stale limits must be > 0 and AutoStart 0 or 1");
        return CFE_STATUS_VALIDATION_FAILURE;
    }
    return CFE_SUCCESS;
}

/* Tables are loaded once at startup (ADR-016 section 5); the values are
 * copied and the table is not consulted again. */
static void INGEST_APP_LoadConfig(void)
{
    CFE_Status_t Status;
    void        *Addr = NULL;

    Status = CFE_TBL_Register(&INGEST_APP_Data.TblHandle, "ConfigTable", sizeof(INGEST_APP_ConfigTable_t),
                              CFE_TBL_OPT_DEFAULT, INGEST_APP_ValidateConfig);
    if (Status == CFE_SUCCESS)
    {
        Status = CFE_TBL_Load(INGEST_APP_Data.TblHandle, CFE_TBL_SRC_FILE, INGEST_APP_TABLE_FILE);
    }
    if (Status == CFE_SUCCESS)
    {
        Status = CFE_TBL_GetAddress(&Addr, INGEST_APP_Data.TblHandle);
        if (Status == CFE_TBL_INFO_UPDATED)
        {
            Status = CFE_SUCCESS;
        }
    }
    if (Status == CFE_SUCCESS && Addr != NULL)
    {
        memcpy(&INGEST_APP_Data.Config, Addr, sizeof(INGEST_APP_Data.Config));
        CFE_TBL_ReleaseAddress(INGEST_APP_Data.TblHandle);
        INGEST_APP_Data.ConfigValid = true;
        return;
    }
    CFE_EVS_SendEvent(INGEST_APP_TBL_ERR_EID, CFE_EVS_EventType_ERROR,
                      "ConfigTable %s not loaded (0x%08lX); sessions cannot start", INGEST_APP_TABLE_FILE,
                      (unsigned long)Status);
    INGEST_APP_FaultRaise(OST_FaultId_TABLE_REJECTED, "ConfigTable not loaded");
}

/* ------------------------------------------------------------------ init */

static CFE_Status_t INGEST_APP_Init(void)
{
    CFE_Status_t   Status;
    CFE_ES_TaskId_t TaskId;
    char           VersionString[INGEST_APP_MAX_VERSION_STR_LEN];

    memset(&INGEST_APP_Data, 0, sizeof(INGEST_APP_Data));
    INGEST_APP_Data.RunStatus = CFE_ES_RunStatus_APP_RUN;
    INGEST_APP_Data.State     = INGEST_APP_SessionState_STOPPED;

    Status = CFE_EVS_Register(INGEST_APP_EventFilters, sizeof(INGEST_APP_EventFilters) / sizeof(INGEST_APP_EventFilters[0]),
                              CFE_EVS_EventFilter_BINARY);
    if (Status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("INGEST_APP: EVS register failed 0x%08lX\n", (unsigned long)Status);
        return Status;
    }
    if (OS_MutSemCreate(&INGEST_APP_Data.FaultMutex, "INGEST_FLT", 0) != OS_SUCCESS)
    {
        CFE_EVS_SendEvent(INGEST_APP_INIT_ERR_EID, CFE_EVS_EventType_ERROR, "fault mutex not created");
        return CFE_STATUS_EXTERNAL_RESOURCE_FAIL;
    }
    Status = OST_FaultSet_Init(&INGEST_APP_Data.Faults, OST_AppId_INGEST_APP, INGEST_APP_FaultIds,
                               INGEST_APP_FAULT_COUNT);
    if (Status != CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(INGEST_APP_INIT_ERR_EID, CFE_EVS_EventType_ERROR, "fault set: 0x%08lX",
                          (unsigned long)Status);
        return Status;
    }

    CFE_MSG_Init(CFE_MSG_PTR(INGEST_APP_Data.HkTlm.TelemetryHeader),
                 CFE_SB_ValueToMsgId(CFE_PLATFORM_TLM_TOPICID_TO_MIDV(EdsParam_CFE_MISSION_INGEST_APP_HK_TLM_TOPICID)),
                 sizeof(INGEST_APP_Data.HkTlm));

    Status = CFE_SB_CreatePipe(&INGEST_APP_Data.CommandPipe, INGEST_APP_PIPE_DEPTH, "INGEST_CMD_PIPE");
    if (Status == CFE_SUCCESS)
    {
        Status = CFE_SB_Subscribe(INGEST_APP_CmdMid(EdsParam_CFE_MISSION_INGEST_APP_CMD_TOPICID),
                                  INGEST_APP_Data.CommandPipe);
    }
    if (Status == CFE_SUCCESS)
    {
        Status = CFE_SB_Subscribe(INGEST_APP_CmdMid(EdsParam_CFE_MISSION_INGEST_APP_SEND_HK_TOPICID),
                                  INGEST_APP_Data.CommandPipe);
    }
    if (Status == CFE_SUCCESS)
    {
        Status = CFE_SB_Subscribe(INGEST_APP_CmdMid(EdsParam_CFE_MISSION_INGEST_APP_WAKEUP_TOPICID),
                                  INGEST_APP_Data.CommandPipe);
    }
    if (Status != CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(INGEST_APP_INIT_ERR_EID, CFE_EVS_EventType_ERROR, "pipe or subscription: 0x%08lX",
                          (unsigned long)Status);
        return Status;
    }

    INGEST_APP_LoadConfig();

    Status = CFE_ES_CreateChildTask(&TaskId, "INGEST_UDP", INGEST_APP_UdpTask, NULL, INGEST_APP_NET_TASK_STACK_SIZE,
                                    INGEST_APP_NET_TASK_PRIORITY, 0);
    if (Status == CFE_SUCCESS)
    {
        Status = CFE_ES_CreateChildTask(&TaskId, "INGEST_TCP", INGEST_APP_TcpTask, NULL,
                                        INGEST_APP_NET_TASK_STACK_SIZE, INGEST_APP_NET_TASK_PRIORITY, 0);
    }
    if (Status != CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(INGEST_APP_INIT_ERR_EID, CFE_EVS_EventType_ERROR, "receiver tasks: 0x%08lX",
                          (unsigned long)Status);
        return Status;
    }

    if (INGEST_APP_Data.ConfigValid && INGEST_APP_Data.Config.AutoStart)
    {
        INGEST_APP_Data.SocketsWanted = true;
    }

    CFE_Config_GetVersionString(VersionString, INGEST_APP_MAX_VERSION_STR_LEN, "INGEST_APP", INGEST_APP_VERSION,
                                INGEST_APP_BUILD_CODENAME, INGEST_APP_LAST_OFFICIAL);
    CFE_EVS_SendEvent(INGEST_APP_INIT_INF_EID, CFE_EVS_EventType_INFORMATION,
                      "INGEST_APP initialised: UDP %u, TCP %u, %s. %s", (unsigned)INGEST_APP_Data.Config.UdpPort,
                      (unsigned)INGEST_APP_Data.Config.TcpPort,
                      INGEST_APP_Data.SocketsWanted ? "listening" : "stopped", VersionString);
    return CFE_SUCCESS;
}

/* -------------------------------------------------------------- commands */

static void INGEST_APP_ProcessPacket(const CFE_SB_Buffer_t *Buf)
{
    CFE_Status_t      Status;
    CFE_SB_MsgId_t    MsgId = CFE_SB_INVALID_MSG_ID;
    CFE_MSG_Size_t    Size  = 0;
    CFE_MSG_FcnCode_t Fc    = 0;

    Status = EdsDispatch_EdsComponent_INGEST_APP_Application_Telecommand(Buf, &INGEST_APP_TC_DISPATCH_TABLE);
    if (Status == CFE_SUCCESS)
    {
        return;
    }
    CFE_MSG_GetMsgId(&Buf->Msg, &MsgId);
    CFE_MSG_GetSize(&Buf->Msg, &Size);
    CFE_MSG_GetFcnCode(&Buf->Msg, &Fc);
    ++INGEST_APP_Data.CommandErrorCounter;
    if (Status == CFE_STATUS_UNKNOWN_MSG_ID)
    {
        CFE_EVS_SendEvent(INGEST_APP_MID_ERR_EID, CFE_EVS_EventType_ERROR, "invalid message ID 0x%x",
                          (unsigned)CFE_SB_MsgIdToValue(MsgId));
    }
    else if (Status == CFE_STATUS_WRONG_MSG_LENGTH)
    {
        CFE_EVS_SendEvent(INGEST_APP_CMD_LEN_ERR_EID, CFE_EVS_EventType_ERROR,
                          "invalid length: MID 0x%x, CC %u, length %u", (unsigned)CFE_SB_MsgIdToValue(MsgId),
                          (unsigned)Fc, (unsigned)Size);
    }
    else
    {
        CFE_EVS_SendEvent(INGEST_APP_CC_ERR_EID, CFE_EVS_EventType_ERROR, "invalid command code %u", (unsigned)Fc);
    }
}

CFE_Status_t INGEST_APP_NoopCmd(const INGEST_APP_NoopCmd_t *Msg)
{
    (void)Msg;
    ++INGEST_APP_Data.CommandCounter;
    CFE_EVS_SendEvent(INGEST_APP_NOOP_INF_EID, CFE_EVS_EventType_INFORMATION, "no-op, version %s",
                      INGEST_APP_VERSION);
    return CFE_SUCCESS;
}

CFE_Status_t INGEST_APP_ResetCountersCmd(const INGEST_APP_ResetCountersCmd_t *Msg)
{
    INGEST_APP_Counters_t *C = &INGEST_APP_Data.Counters;

    (void)Msg;
    INGEST_APP_Data.CommandCounter      = 0;
    INGEST_APP_Data.CommandErrorCounter = 0;
    C->TelemetryReceived  = 0;
    C->TelemetryRejected  = 0;
    C->TelemetryLost      = 0;
    C->HeartbeatsReceived = 0;
    C->ImagesReceived     = 0;
    C->ImagesDropped      = 0;
    C->FramesRejected     = 0;
    C->Connections        = 0;
    C->ImageBytes         = 0;
    CFE_EVS_SendEvent(INGEST_APP_RESET_INF_EID, CFE_EVS_EventType_INFORMATION, "counters reset");
    return CFE_SUCCESS;
}

CFE_Status_t INGEST_APP_ClearLatchedFaultsCmd(const INGEST_APP_ClearLatchedFaultsCmd_t *Msg)
{
    uint32 Cleared;

    (void)Msg;
    OS_MutSemTake(INGEST_APP_Data.FaultMutex);
    Cleared = OST_FaultSet_ClearLatched(&INGEST_APP_Data.Faults);
    OS_MutSemGive(INGEST_APP_Data.FaultMutex);
    ++INGEST_APP_Data.CommandCounter;
    CFE_EVS_SendEvent(INGEST_APP_CLEAR_INF_EID, CFE_EVS_EventType_INFORMATION, "%lu latched faults cleared",
                      (unsigned long)Cleared);
    return CFE_SUCCESS;
}

CFE_Status_t INGEST_APP_StartSessionCmd(const INGEST_APP_StartSessionCmd_t *Msg)
{
    (void)Msg;
    if (!INGEST_APP_Data.ConfigValid)
    {
        ++INGEST_APP_Data.CommandErrorCounter;
        CFE_EVS_SendEvent(INGEST_APP_CMD_REFUSED_EID, CFE_EVS_EventType_ERROR,
                          "start session refused: no valid configuration table");
        return CFE_SUCCESS;
    }
    ++INGEST_APP_Data.CommandCounter;
    INGEST_APP_Data.SocketsWanted = true;
    CFE_EVS_SendEvent(INGEST_APP_START_INF_EID, CFE_EVS_EventType_INFORMATION, "start session: opening sockets");
    return CFE_SUCCESS;
}

CFE_Status_t INGEST_APP_StopSessionCmd(const INGEST_APP_StopSessionCmd_t *Msg)
{
    (void)Msg;
    ++INGEST_APP_Data.CommandCounter;
    INGEST_APP_Data.SocketsWanted = false;
    CFE_EVS_SendEvent(INGEST_APP_STOP_INF_EID, CFE_EVS_EventType_INFORMATION, "stop session: closing sockets");
    return CFE_SUCCESS;
}

/* ---------------------------------------------------- housekeeping, wake-up */

static uint32 INGEST_APP_AgeMs(int64 LastMono, int64 NowMono)
{
    int64 Age;

    if (LastMono == 0)
    {
        return 0xFFFFFFFFu;
    }
    Age = (NowMono - LastMono) / 1000000;
    return (Age > 0xFFFFFFFE) ? 0xFFFFFFFEu : (uint32)Age;
}

CFE_Status_t INGEST_APP_SendHkCmd(const INGEST_APP_SendHkCmd_t *Msg)
{
    INGEST_APP_HkTlm_Payload_t  *P = &INGEST_APP_Data.HkTlm.Payload;
    const INGEST_APP_Counters_t *C = &INGEST_APP_Data.Counters;
    const int64                  Now = OST_TimeMonoNs();

    (void)Msg;
    P->Common.CommandCounter      = INGEST_APP_Data.CommandCounter;
    P->Common.CommandErrorCounter = INGEST_APP_Data.CommandErrorCounter;
    OS_MutSemTake(INGEST_APP_Data.FaultMutex);
    OST_FaultSet_Summarise(&INGEST_APP_Data.Faults, &P->Common);
    OS_MutSemGive(INGEST_APP_Data.FaultMutex);
    P->State               = INGEST_APP_Data.State;
    P->SessionId           = INGEST_APP_Data.SessionId;
    P->TelemetryReceived   = C->TelemetryReceived;
    P->TelemetryRejected   = C->TelemetryRejected;
    P->TelemetryLost       = C->TelemetryLost;
    P->HeartbeatsReceived  = C->HeartbeatsReceived;
    P->InjectorSamplesSent = C->InjectorSamplesSent;
    P->InjectorImagesSent  = C->InjectorImagesSent;
    P->ImagesReceived      = C->ImagesReceived;
    P->ImagesDropped       = C->ImagesDropped;
    P->FramesRejected      = C->FramesRejected;
    P->Connections         = C->Connections;
    P->ImageBytes          = C->ImageBytes;
    P->TelemetryAgeMs      = INGEST_APP_AgeMs(C->LastTelemetryMono, Now);
    P->ImageAgeMs          = INGEST_APP_AgeMs(C->LastImageMono, Now);

    CFE_SB_TimeStampMsg(CFE_MSG_PTR(INGEST_APP_Data.HkTlm.TelemetryHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(INGEST_APP_Data.HkTlm.TelemetryHeader), true);
    return CFE_SUCCESS;
}

/* Staleness during an active session (ADR-020 section 4). Before the first
 * sample or image of a session, the age counts from the session start. */
CFE_Status_t INGEST_APP_WakeupCmd(const INGEST_APP_WakeupCmd_t *Msg)
{
    const INGEST_APP_Counters_t *C   = &INGEST_APP_Data.Counters;
    const int64                  Now = OST_TimeMonoNs();
    int64                        LastTlm;
    int64                        LastImg;

    (void)Msg;
    if (INGEST_APP_Data.State != INGEST_APP_SessionState_ACTIVE)
    {
        INGEST_APP_FaultResolve(OST_FaultId_TELEMETRY_STALE);
        INGEST_APP_FaultResolve(OST_FaultId_IMAGE_STREAM_STALE);
        return CFE_SUCCESS;
    }
    LastTlm = (C->LastTelemetryMono > C->SessionStartMono) ? C->LastTelemetryMono : C->SessionStartMono;
    LastImg = (C->LastImageMono > C->SessionStartMono) ? C->LastImageMono : C->SessionStartMono;

    if (Now - LastTlm > (int64)INGEST_APP_Data.Config.TelemetryStaleMs * 1000000)
    {
        INGEST_APP_FaultRaise(OST_FaultId_TELEMETRY_STALE, "no telemetry sample");
    }
    else
    {
        INGEST_APP_FaultResolve(OST_FaultId_TELEMETRY_STALE);
    }
    if (Now - LastImg > (int64)INGEST_APP_Data.Config.ImageStaleMs * 1000000)
    {
        INGEST_APP_FaultRaise(OST_FaultId_IMAGE_STREAM_STALE, "no image");
    }
    else
    {
        INGEST_APP_FaultResolve(OST_FaultId_IMAGE_STREAM_STALE);
    }
    return CFE_SUCCESS;
}

/* ------------------------------------------------------------------ main */

void INGEST_APP_Main(void)
{
    CFE_Status_t     Status;
    CFE_SB_Buffer_t *Buf = NULL;

    Status = INGEST_APP_Init();
    if (Status != CFE_SUCCESS)
    {
        INGEST_APP_Data.RunStatus = CFE_ES_RunStatus_APP_ERROR;
    }

    while (CFE_ES_RunLoop(&INGEST_APP_Data.RunStatus))
    {
        Status = CFE_SB_ReceiveBuffer(&Buf, INGEST_APP_Data.CommandPipe, INGEST_APP_PIPE_TIMEOUT_MS);
        if (Status == CFE_SUCCESS)
        {
            INGEST_APP_ProcessPacket(Buf);
        }
        else if (Status != CFE_SB_TIME_OUT && Status != CFE_SB_NO_MESSAGE)
        {
            CFE_EVS_SendEvent(INGEST_APP_PIPE_ERR_EID, CFE_EVS_EventType_ERROR, "pipe read error 0x%08lX",
                              (unsigned long)Status);
            INGEST_APP_Data.RunStatus = CFE_ES_RunStatus_APP_ERROR;
        }
    }

    /* The receivers close their sockets when they see this. */
    INGEST_APP_Data.SocketsWanted = false;
    CFE_ES_ExitApp(INGEST_APP_Data.RunStatus);
}
