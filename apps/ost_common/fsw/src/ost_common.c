/*
 * ost_common library: entry point, time helpers, shared message identifiers
 * and the mapping of library diagnostics to EVS events.
 */

/* clock_gettime() with CLOCK_MONOTONIC; the applications are built as
 * strict C99. The native Linux target (ADR-002) is POSIX. */
#define _POSIX_C_SOURCE 200809L

#include "ost_common.h"
#include "ost_common_version.h"

#include "cfe_config.h"
#include "cfe_core_api_base_msgids.h"
#include "cfe_mission_eds_designparameters.h"

#include <string.h>
#include <time.h>

int32 OST_Common_Init(void)
{
    char VersionString[OST_COMMON_MAX_VERSION_STR_LEN];

    CFE_Config_GetVersionString(VersionString, sizeof(VersionString), "OST", OST_COMMON_VERSION,
                                OST_COMMON_BUILD_CODENAME, OST_COMMON_LAST_OFFICIAL);
    OS_printf("OST_COMMON initialised. %s\n", VersionString);
    return CFE_SUCCESS;
}

static int64 OST_ClockNs(clockid_t Clock)
{
    struct timespec Ts;

    if (clock_gettime(Clock, &Ts) != 0)
    {
        return 0;
    }
    return (int64)Ts.tv_sec * 1000000000 + (int64)Ts.tv_nsec;
}

int64 OST_TimeUtcNs(void)
{
    return OST_ClockNs(CLOCK_REALTIME);
}

int64 OST_TimeMonoNs(void)
{
    return OST_ClockNs(CLOCK_MONOTONIC);
}

CFE_SB_MsgId_t OST_FaultEventMsgId(void)
{
    return CFE_SB_ValueToMsgId(CFE_PLATFORM_TLM_TOPICID_TO_MIDV(EdsParam_CFE_MISSION_OST_FAULT_EVENT_TLM_TOPICID));
}

CFE_SB_MsgId_t OST_BatchSelectedMsgId(void)
{
    return CFE_SB_ValueToMsgId(CFE_PLATFORM_TLM_TOPICID_TO_MIDV(EdsParam_CFE_MISSION_OST_BATCH_SELECTED_TLM_TOPICID));
}

CFE_SB_MsgId_t OST_BatchReadyMsgId(void)
{
    return CFE_SB_ValueToMsgId(CFE_PLATFORM_TLM_TOPICID_TO_MIDV(EdsParam_CFE_MISSION_OST_BATCH_READY_TLM_TOPICID));
}

void OST_CopyText(char *Dst, size_t DstSize, const char *Src)
{
    size_t Len;

    if (Dst == NULL || DstSize == 0)
    {
        return;
    }
    if (Src == NULL)
    {
        Dst[0] = '\0';
        return;
    }
    Len = strlen(Src);
    if (Len >= DstSize)
    {
        Len = DstSize - 1;
    }
    memcpy(Dst, Src, Len);
    memset(Dst + Len, 0, DstSize - Len);
}

void OST_DiagToEvs(void *Context, OST_DiagLevel_t Level, OST_Status_t Status, const char *Text)
{
    const OST_DiagEvents_t *Ids = (const OST_DiagEvents_t *)Context;
    const char             *Msg = (Text != NULL) ? Text : "";

    if (Ids == NULL)
    {
        return;
    }
    switch (Level)
    {
        case OST_DIAG_ERROR:
            CFE_EVS_SendEvent(Ids->ErrorEid, CFE_EVS_EventType_ERROR, "%s: %s", OST_StatusName(Status), Msg);
            break;
        case OST_DIAG_WARNING:
            CFE_EVS_SendEvent(Ids->WarnEid, CFE_EVS_EventType_INFORMATION, "warning %s: %s", OST_StatusName(Status),
                              Msg);
            break;
        default:
            CFE_EVS_SendEvent(Ids->DebugEid, CFE_EVS_EventType_DEBUG, "%s", Msg);
            break;
    }
}
