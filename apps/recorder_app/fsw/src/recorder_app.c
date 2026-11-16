/*
 * RECORDER_APP main task: initialisation, table and subscriptions,
 * commands, housekeeping and the recording of every subscribed message
 * (recorder_app.h).
 */
#include "recorder_app.h"
#include "recorder_app_eventids.h"
#include "recorder_app_version.h"

#include "cfe_config.h"
#include "cfe_core_api_base_msgids.h"
#include "cfe_mission_eds_designparameters.h"
#include "recorder_app_eds_designparameters.h"
#include "recorder_app_eds_dictionary.h"
#include "recorder_app_eds_dispatcher.h"

#include <stdio.h>
#include <string.h>

RECORDER_APP_Data_t RECORDER_APP_Data;

static const OST_FaultId_Enum_t RECORDER_APP_FaultIds[RECORDER_APP_FAULT_COUNT] = {
    OST_FaultId_RECORDING_FAILED, /* 60 */
    OST_FaultId_DISK_SPACE_LOW,   /* 61 */
    OST_FaultId_TABLE_REJECTED,   /* 62 */
};

static CFE_EVS_BinFilter_t RECORDER_APP_EventFilters[] = {
    {RECORDER_APP_WRITE_ERR_EID, CFE_EVS_FIRST_8_STOP},
    {RECORDER_APP_ENCODE_ERR_EID, CFE_EVS_FIRST_8_STOP},
    {RECORDER_APP_SUB_ERR_EID, CFE_EVS_FIRST_8_STOP},
};

/* clang-format off */
static const EdsDispatchTable_EdsComponent_RECORDER_APP_Application_CFE_SB_Telecommand_t RECORDER_APP_TC_DISPATCH_TABLE = {
    .CMD = {
        .NoopCmd_indication               = RECORDER_APP_NoopCmd,
        .ResetCountersCmd_indication      = RECORDER_APP_ResetCountersCmd,
        .ClearLatchedFaultsCmd_indication = RECORDER_APP_ClearLatchedFaultsCmd,
        .StartRecordingCmd_indication     = RECORDER_APP_StartRecordingCmd,
        .StopRecordingCmd_indication      = RECORDER_APP_StopRecordingCmd,
    },
    .SEND_HK = { .indication = RECORDER_APP_SendHkCmd },
    .WAKEUP  = { .indication = RECORDER_APP_WakeupCmd },
};
/* clang-format on */

static CFE_SB_MsgId_t RECORDER_APP_CmdMid(uint16 TopicId)
{
    return CFE_SB_ValueToMsgId(CFE_PLATFORM_CMD_TOPICID_TO_MIDV(TopicId));
}

static CFE_SB_MsgId_t RECORDER_APP_TlmMid(uint16 TopicId)
{
    return CFE_SB_ValueToMsgId(CFE_PLATFORM_TLM_TOPICID_TO_MIDV(TopicId));
}

/* ----------------------------------------------------------------- table */

static CFE_Status_t RECORDER_APP_ValidateConfig(void *TblData)
{
    const RECORDER_APP_ConfigTable_t *T = (const RECORDER_APP_ConfigTable_t *)TblData;
    uint32                            i;
    bool                              End = false;

    if (T->OutputDir[0] == '\0' || memchr(T->OutputDir, '\0', sizeof(T->OutputDir)) == NULL || T->AutoStart > 1 ||
        T->MinFreeMb > T->WarnFreeMb)
    {
        CFE_EVS_SendEvent(RECORDER_APP_TBL_ERR_EID, CFE_EVS_EventType_ERROR,
                          "ConfigTable: output directory, AutoStart 0/1 or MinFreeMb <= WarnFreeMb invalid");
        return CFE_STATUS_VALIDATION_FAILURE;
    }
    for (i = 0; i < EdsParam_RECORDER_APP_MAX_SUBSCRIPTIONS; ++i)
    {
        const RECORDER_APP_Subscription_t *S = &T->Subscriptions[i];

        if (S->MsgId == 0)
        {
            End = true;
            continue;
        }
        if (End || S->MsgLimit < 1 || S->MsgLimit > RECORDER_APP_PIPE_DEPTH ||
            !CFE_SB_IsValidMsgId(CFE_SB_ValueToMsgId(S->MsgId)))
        {
            CFE_EVS_SendEvent(RECORDER_APP_TBL_ERR_EID, CFE_EVS_EventType_ERROR,
                              "ConfigTable: subscription %lu invalid (message 0x%04lX, limit %u; no gaps)",
                              (unsigned long)i, (unsigned long)S->MsgId, (unsigned)S->MsgLimit);
            return CFE_STATUS_VALIDATION_FAILURE;
        }
    }
    return CFE_SUCCESS;
}

static void RECORDER_APP_LoadConfig(void)
{
    CFE_Status_t Status;
    void        *Addr = NULL;

    Status = CFE_TBL_Register(&RECORDER_APP_Data.TblHandle, "ConfigTable", sizeof(RECORDER_APP_ConfigTable_t),
                              CFE_TBL_OPT_DEFAULT, RECORDER_APP_ValidateConfig);
    if (Status == CFE_SUCCESS)
    {
        Status = CFE_TBL_Load(RECORDER_APP_Data.TblHandle, CFE_TBL_SRC_FILE, RECORDER_APP_TABLE_FILE);
    }
    if (Status == CFE_SUCCESS)
    {
        Status = CFE_TBL_GetAddress(&Addr, RECORDER_APP_Data.TblHandle);
        if (Status == CFE_TBL_INFO_UPDATED)
        {
            Status = CFE_SUCCESS;
        }
    }
    if (Status == CFE_SUCCESS && Addr != NULL)
    {
        memcpy(&RECORDER_APP_Data.Config, Addr, sizeof(RECORDER_APP_Data.Config));
        CFE_TBL_ReleaseAddress(RECORDER_APP_Data.TblHandle);
        RECORDER_APP_Data.ConfigValid = true;
        return;
    }
    CFE_EVS_SendEvent(RECORDER_APP_TBL_ERR_EID, CFE_EVS_EventType_ERROR,
                      "ConfigTable %s not loaded (0x%08lX); nothing is recorded", RECORDER_APP_TABLE_FILE,
                      (unsigned long)Status);
    OST_FaultSet_Raise(&RECORDER_APP_Data.Faults, OST_FaultId_TABLE_REJECTED, "ConfigTable not loaded");
}

static void RECORDER_APP_Subscribe(void)
{
    const RECORDER_APP_ConfigTable_t *T = &RECORDER_APP_Data.Config;
    uint32                            i;
    CFE_Status_t                      Status;

    for (i = 0; i < EdsParam_RECORDER_APP_MAX_SUBSCRIPTIONS && T->Subscriptions[i].MsgId != 0; ++i)
    {
        Status = CFE_SB_SubscribeEx(CFE_SB_ValueToMsgId(T->Subscriptions[i].MsgId),
                                    RECORDER_APP_Data.Pipes[i % RECORDER_APP_PIPES], CFE_SB_DEFAULT_QOS,
                                    T->Subscriptions[i].MsgLimit);
        if (Status == CFE_SUCCESS)
        {
            ++RECORDER_APP_Data.Subscriptions;
        }
        else
        {
            CFE_EVS_SendEvent(RECORDER_APP_SUB_ERR_EID, CFE_EVS_EventType_ERROR,
                              "message 0x%04lX not subscribed: 0x%08lX", (unsigned long)T->Subscriptions[i].MsgId,
                              (unsigned long)Status);
        }
    }
}

/* ------------------------------------------------------------------ init */

static bool RECORDER_APP_WriterStarted = false;

static CFE_Status_t RECORDER_APP_Init(void)
{
    CFE_Status_t    Status;
    CFE_ES_TaskId_t WriterId;
    uint32          i;
    char            PipeName[OS_MAX_API_NAME];
    char            VersionString[RECORDER_APP_MAX_VERSION_STR_LEN];

    memset(&RECORDER_APP_Data, 0, sizeof(RECORDER_APP_Data));
    RECORDER_APP_Data.RunStatus = CFE_ES_RunStatus_APP_RUN;

    Status = CFE_EVS_Register(RECORDER_APP_EventFilters,
                              sizeof(RECORDER_APP_EventFilters) / sizeof(RECORDER_APP_EventFilters[0]),
                              CFE_EVS_EventFilter_BINARY);
    if (Status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("RECORDER_APP: EVS register failed 0x%08lX\n", (unsigned long)Status);
        return Status;
    }
    if (OS_MutSemCreate(&RECORDER_APP_Data.BufferMutex, "REC_BUF", 0) != OS_SUCCESS ||
        OS_MutSemCreate(&RECORDER_APP_Data.FileMutex, "REC_FILE", 0) != OS_SUCCESS ||
        OS_BinSemCreate(&RECORDER_APP_Data.DataSem, "REC_DATA", 0, 0) != OS_SUCCESS ||
        OS_BinSemCreate(&RECORDER_APP_Data.WriterDone, "REC_DONE", 0, 0) != OS_SUCCESS)
    {
        CFE_EVS_SendEvent(RECORDER_APP_INIT_ERR_EID, CFE_EVS_EventType_ERROR, "mutexes or semaphores not created");
        return CFE_STATUS_EXTERNAL_RESOURCE_FAIL;
    }
    Status = OST_FaultSet_Init(&RECORDER_APP_Data.Faults, OST_AppId_RECORDER_APP, RECORDER_APP_FaultIds,
                               RECORDER_APP_FAULT_COUNT);
    if (Status != CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(RECORDER_APP_INIT_ERR_EID, CFE_EVS_EventType_ERROR, "fault set: 0x%08lX",
                          (unsigned long)Status);
        return Status;
    }
    CFE_MSG_Init(CFE_MSG_PTR(RECORDER_APP_Data.HkTlm.TelemetryHeader),
                 RECORDER_APP_TlmMid(EdsParam_CFE_MISSION_RECORDER_APP_HK_TLM_TOPICID),
                 sizeof(RECORDER_APP_Data.HkTlm));

    for (i = 0, Status = CFE_SUCCESS; i < RECORDER_APP_PIPES && Status == CFE_SUCCESS; ++i)
    {
        snprintf(PipeName, sizeof(PipeName), "RECORDER_PIPE%u", (unsigned)i);
        Status = CFE_SB_CreatePipe(&RECORDER_APP_Data.Pipes[i], RECORDER_APP_PIPE_DEPTH, PipeName);
    }
    if (Status == CFE_SUCCESS)
    {
        Status = CFE_SB_SubscribeEx(RECORDER_APP_CmdMid(EdsParam_CFE_MISSION_RECORDER_APP_CMD_TOPICID),
                                    RECORDER_APP_Data.Pipes[0], CFE_SB_DEFAULT_QOS, 4);
    }
    if (Status == CFE_SUCCESS)
    {
        Status = CFE_SB_SubscribeEx(RECORDER_APP_CmdMid(EdsParam_CFE_MISSION_RECORDER_APP_SEND_HK_TOPICID),
                                    RECORDER_APP_Data.Pipes[0], CFE_SB_DEFAULT_QOS, 2);
    }
    if (Status == CFE_SUCCESS)
    {
        Status = CFE_SB_SubscribeEx(RECORDER_APP_CmdMid(EdsParam_CFE_MISSION_RECORDER_APP_WAKEUP_TOPICID),
                                    RECORDER_APP_Data.Pipes[0], CFE_SB_DEFAULT_QOS, 2);
    }
    if (Status != CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(RECORDER_APP_INIT_ERR_EID, CFE_EVS_EventType_ERROR, "pipe or subscription: 0x%08lX",
                          (unsigned long)Status);
        return Status;
    }

    Status = CFE_ES_CreateChildTask(&WriterId, "REC_WRITE", RECORDER_APP_WriterTask, NULL,
                                    RECORDER_APP_WRITER_STACK_SIZE, RECORDER_APP_WRITER_PRIORITY, 0);
    if (Status != CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(RECORDER_APP_INIT_ERR_EID, CFE_EVS_EventType_ERROR, "writer task: 0x%08lX",
                          (unsigned long)Status);
        return Status;
    }
    RECORDER_APP_WriterStarted = true;

    RECORDER_APP_LoadConfig();
    if (RECORDER_APP_Data.ConfigValid)
    {
        RECORDER_APP_Subscribe();
        RECORDER_APP_CheckSpace();
    }

    CFE_Config_GetVersionString(VersionString, RECORDER_APP_MAX_VERSION_STR_LEN, "RECORDER_APP",
                                RECORDER_APP_VERSION, RECORDER_APP_BUILD_CODENAME, RECORDER_APP_LAST_OFFICIAL);
    CFE_EVS_SendEvent(RECORDER_APP_INIT_INF_EID, CFE_EVS_EventType_INFORMATION,
                      "RECORDER_APP initialised: %u messages to %.128s, %.0f MB free, auto start %s. %s",
                      (unsigned)RECORDER_APP_Data.Subscriptions, RECORDER_APP_Data.Config.OutputDir,
                      RECORDER_APP_Data.FreeMb, RECORDER_APP_Data.Config.AutoStart ? "on" : "off", VersionString);

    if (RECORDER_APP_Data.ConfigValid && RECORDER_APP_Data.Config.AutoStart)
    {
        RECORDER_APP_Start(NULL);
    }
    return CFE_SUCCESS;
}

/* -------------------------------------------------------------- commands */

static bool RECORDER_APP_IsCommand(CFE_SB_MsgId_t MsgId)
{
    return CFE_SB_MsgId_Equal(MsgId, RECORDER_APP_CmdMid(EdsParam_CFE_MISSION_RECORDER_APP_CMD_TOPICID)) ||
           CFE_SB_MsgId_Equal(MsgId, RECORDER_APP_CmdMid(EdsParam_CFE_MISSION_RECORDER_APP_SEND_HK_TOPICID)) ||
           CFE_SB_MsgId_Equal(MsgId, RECORDER_APP_CmdMid(EdsParam_CFE_MISSION_RECORDER_APP_WAKEUP_TOPICID));
}

static void RECORDER_APP_ProcessCommand(const CFE_SB_Buffer_t *Buf, CFE_SB_MsgId_t MsgId)
{
    CFE_Status_t      Status;
    CFE_MSG_Size_t    Size = 0;
    CFE_MSG_FcnCode_t Fc   = 0;

    Status = EdsDispatch_EdsComponent_RECORDER_APP_Application_Telecommand(Buf, &RECORDER_APP_TC_DISPATCH_TABLE);
    if (Status == CFE_SUCCESS)
    {
        return;
    }
    CFE_MSG_GetSize(&Buf->Msg, &Size);
    CFE_MSG_GetFcnCode(&Buf->Msg, &Fc);
    ++RECORDER_APP_Data.CommandErrorCounter;
    if (Status == CFE_STATUS_UNKNOWN_MSG_ID)
    {
        CFE_EVS_SendEvent(RECORDER_APP_MID_ERR_EID, CFE_EVS_EventType_ERROR, "invalid message ID 0x%x",
                          (unsigned)CFE_SB_MsgIdToValue(MsgId));
    }
    else if (Status == CFE_STATUS_WRONG_MSG_LENGTH)
    {
        CFE_EVS_SendEvent(RECORDER_APP_CMD_LEN_ERR_EID, CFE_EVS_EventType_ERROR,
                          "invalid length: MID 0x%x, CC %u, length %u", (unsigned)CFE_SB_MsgIdToValue(MsgId),
                          (unsigned)Fc, (unsigned)Size);
    }
    else
    {
        CFE_EVS_SendEvent(RECORDER_APP_CC_ERR_EID, CFE_EVS_EventType_ERROR, "invalid command code %u", (unsigned)Fc);
    }
}

CFE_Status_t RECORDER_APP_NoopCmd(const RECORDER_APP_NoopCmd_t *Msg)
{
    (void)Msg;
    ++RECORDER_APP_Data.CommandCounter;
    CFE_EVS_SendEvent(RECORDER_APP_NOOP_INF_EID, CFE_EVS_EventType_INFORMATION, "no-op, version %s",
                      RECORDER_APP_VERSION);
    return CFE_SUCCESS;
}

CFE_Status_t RECORDER_APP_ResetCountersCmd(const RECORDER_APP_ResetCountersCmd_t *Msg)
{
    (void)Msg;
    RECORDER_APP_Data.CommandCounter      = 0;
    RECORDER_APP_Data.CommandErrorCounter = 0;
    CFE_EVS_SendEvent(RECORDER_APP_RESET_INF_EID, CFE_EVS_EventType_INFORMATION,
                      "command counters reset; run counters belong to the run");
    return CFE_SUCCESS;
}

CFE_Status_t RECORDER_APP_ClearLatchedFaultsCmd(const RECORDER_APP_ClearLatchedFaultsCmd_t *Msg)
{
    uint32 Cleared;

    (void)Msg;
    Cleared = OST_FaultSet_ClearLatched(&RECORDER_APP_Data.Faults);
    ++RECORDER_APP_Data.CommandCounter;
    CFE_EVS_SendEvent(RECORDER_APP_CLEAR_INF_EID, CFE_EVS_EventType_INFORMATION, "%lu latched faults cleared",
                      (unsigned long)Cleared);
    return CFE_SUCCESS;
}

CFE_Status_t RECORDER_APP_StartRecordingCmd(const RECORDER_APP_StartRecordingCmd_t *Msg)
{
    char RunId[RECORDER_APP_RUN_ID_MAX + 1];

    snprintf(RunId, sizeof(RunId), "%.*s", (int)sizeof(Msg->Payload.RunId), Msg->Payload.RunId);
    if (!RECORDER_APP_Data.ConfigValid || (RunId[0] != '\0' && !RECORDER_APP_ValidRunId(RunId)))
    {
        ++RECORDER_APP_Data.CommandErrorCounter;
        CFE_EVS_SendEvent(RECORDER_APP_RUN_ID_ERR_EID, CFE_EVS_EventType_ERROR,
                          "start recording: %s", RECORDER_APP_Data.ConfigValid
                                                     ? "run identifier with letters, digits, '-', '_', '.' only"
                                                     : "no configuration table");
        return CFE_SUCCESS;
    }
    if (RECORDER_APP_Start(RunId) == CFE_SUCCESS)
    {
        ++RECORDER_APP_Data.CommandCounter;
    }
    else
    {
        ++RECORDER_APP_Data.CommandErrorCounter;
    }
    return CFE_SUCCESS;
}

CFE_Status_t RECORDER_APP_StopRecordingCmd(const RECORDER_APP_StopRecordingCmd_t *Msg)
{
    (void)Msg;
    ++RECORDER_APP_Data.CommandCounter;
    if (!RECORDER_APP_Data.Recording)
    {
        CFE_EVS_SendEvent(RECORDER_APP_STOP_INF_EID, CFE_EVS_EventType_INFORMATION, "not recording");
        return CFE_SUCCESS;
    }
    RECORDER_APP_Stop("command");
    return CFE_SUCCESS;
}

CFE_Status_t RECORDER_APP_WakeupCmd(const RECORDER_APP_WakeupCmd_t *Msg)
{
    (void)Msg;
    RECORDER_APP_CheckWriter();
    if (RECORDER_APP_Data.ConfigValid)
    {
        RECORDER_APP_CheckSpace();
    }
    return CFE_SUCCESS;
}

CFE_Status_t RECORDER_APP_SendHkCmd(const RECORDER_APP_SendHkCmd_t *Msg)
{
    RECORDER_APP_HkTlm_Payload_t *P = &RECORDER_APP_Data.HkTlm.Payload;
    const RECORDER_APP_Data_t    *D = &RECORDER_APP_Data;

    (void)Msg;
    P->Common.CommandCounter      = D->CommandCounter;
    P->Common.CommandErrorCounter = D->CommandErrorCounter;
    OST_FaultSet_Summarise(&RECORDER_APP_Data.Faults, &P->Common);
    P->Recording     = D->Recording ? 1 : 0;
    P->Subscriptions = D->Subscriptions;
    P->Runs          = D->Runs;
    P->Records       = D->Records;
    P->EncodeErrors  = D->EncodeErrors;
    P->WriteErrors   = D->WriteErrors;
    P->Dropped       = D->Dropped;
    P->BufferMaxKb   = D->BufferMax / 1024u;
    P->MaxWriteMs    = (float)((double)D->MaxWriteNs * 1e-6);
    P->RecordingMb   = (float)((double)D->Bytes / (1024.0 * 1024.0));
    P->FreeMb        = D->FreeMb;
    P->StartUtc      = D->StartUtc;
    memset(P->RunId, 0, sizeof(P->RunId));
    memcpy(P->RunId, D->RunId, strlen(D->RunId) < sizeof(P->RunId) ? strlen(D->RunId) : sizeof(P->RunId));

    CFE_SB_TimeStampMsg(CFE_MSG_PTR(RECORDER_APP_Data.HkTlm.TelemetryHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(RECORDER_APP_Data.HkTlm.TelemetryHeader), true);
    return CFE_SUCCESS;
}

/* ------------------------------------------------------------------ main */

static void RECORDER_APP_Handle(const CFE_SB_Buffer_t *Buf)
{
    CFE_SB_MsgId_t MsgId = CFE_SB_INVALID_MSG_ID;

    CFE_MSG_GetMsgId(&Buf->Msg, &MsgId);
    if (RECORDER_APP_IsCommand(MsgId))
    {
        RECORDER_APP_ProcessCommand(Buf, MsgId);
    }
    else
    {
        RECORDER_APP_Record(Buf);
    }
}

void RECORDER_APP_Main(void)
{
    CFE_Status_t     Status = CFE_SUCCESS;
    CFE_SB_Buffer_t *Buf    = NULL;
    uint32           i;
    bool             Received;

    Status = RECORDER_APP_Init();
    if (Status != CFE_SUCCESS)
    {
        RECORDER_APP_Data.RunStatus = CFE_ES_RunStatus_APP_ERROR;
    }

    while (CFE_ES_RunLoop(&RECORDER_APP_Data.RunStatus))
    {
        /* Empty every pipe; when all are empty, wait on pipe 0. */
        Received = false;
        for (i = 0; i < RECORDER_APP_PIPES; ++i)
        {
            while ((Status = CFE_SB_ReceiveBuffer(&Buf, RECORDER_APP_Data.Pipes[i], CFE_SB_POLL)) == CFE_SUCCESS)
            {
                RECORDER_APP_Handle(Buf);
                Received = true;
            }
            if (Status != CFE_SB_NO_MESSAGE)
            {
                break;
            }
        }
        if (Status == CFE_SB_NO_MESSAGE && !Received)
        {
            Status = CFE_SB_ReceiveBuffer(&Buf, RECORDER_APP_Data.Pipes[0], RECORDER_APP_PIPE_TIMEOUT_MS);
            if (Status == CFE_SUCCESS)
            {
                RECORDER_APP_Handle(Buf);
            }
        }
        if (Status != CFE_SUCCESS && Status != CFE_SB_TIME_OUT && Status != CFE_SB_NO_MESSAGE)
        {
            CFE_EVS_SendEvent(RECORDER_APP_PIPE_ERR_EID, CFE_EVS_EventType_ERROR, "pipe read error 0x%08lX",
                              (unsigned long)Status);
            RECORDER_APP_Data.RunStatus = CFE_ES_RunStatus_APP_ERROR;
        }
    }
    RECORDER_APP_Stop("application stopped");
    /* Child tasks end with the application: the writer finishes first. */
    if (RECORDER_APP_WriterStarted)
    {
        RECORDER_APP_Data.WriterStop = true;
        OS_BinSemGive(RECORDER_APP_Data.DataSem);
        OS_BinSemTimedWait(RECORDER_APP_Data.WriterDone, RECORDER_APP_DRAIN_WAIT_MS);
    }
    CFE_ES_ExitApp(RECORDER_APP_Data.RunStatus);
}
