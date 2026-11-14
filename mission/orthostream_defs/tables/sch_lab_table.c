/*
 * SCH_LAB schedule of the OrthoStream reference mission (ADR-015 section 1):
 * housekeeping requests and the 1 Hz wake-up of the OrthoStream
 * applications, and the housekeeping requests of the cFE core services.
 * Replaces the empty default table of SCH_LAB; a host mission uses its own
 * scheduler.
 *
 * TickRate 100: a PacketRate of 100 sends the message once per second.
 */
#include "cfe_tbl_filedef.h"
#include "sch_lab_tbl.h"
#include "cfe_sb_api_typedefs.h"
#include "cfe_msgids.h"
#include "cfe_mission_eds_designparameters.h"

#define OST_SCH_CMD(topic) CFE_SB_MSGID_WRAP_VALUE(CFE_PLATFORM_CMD_TOPICID_TO_MIDV(EdsParam_CFE_MISSION_##topic))

SCH_LAB_ScheduleTable_t Schedule = {
    .TickRate = 100,
    .Config   = {
        /* cFE core housekeeping */
        {CFE_SB_MSGID_WRAP_VALUE(CFE_ES_SEND_HK_MID), 100, 0},
        {CFE_SB_MSGID_WRAP_VALUE(CFE_EVS_SEND_HK_MID), 100, 0},
        {CFE_SB_MSGID_WRAP_VALUE(CFE_SB_SEND_HK_MID), 100, 0},
        {CFE_SB_MSGID_WRAP_VALUE(CFE_TBL_SEND_HK_MID), 100, 0},
        {CFE_SB_MSGID_WRAP_VALUE(CFE_TIME_SEND_HK_MID), 100, 0},

        /* INGEST_APP */
        {OST_SCH_CMD(INGEST_APP_SEND_HK_TOPICID), 100, 0},
        {OST_SCH_CMD(INGEST_APP_WAKEUP_TOPICID), 100, 0},

        /* ALIGN_APP: no wake-up, its pipe timeout checks the waiting images */
        {OST_SCH_CMD(ALIGN_APP_SEND_HK_TOPICID), 100, 0},

        /* SELECT_APP */
        {OST_SCH_CMD(SELECT_APP_SEND_HK_TOPICID), 100, 0},

        /* REFINE_APP */
        {OST_SCH_CMD(REFINE_APP_SEND_HK_TOPICID), 100, 0},

        /* DENSE_APP */
        {OST_SCH_CMD(DENSE_APP_SEND_HK_TOPICID), 100, 0},
    }
};

CFE_TBL_FILEDEF(Schedule, SCH_LAB.Schedule, OrthoStream schedule, sch_lab_table.tbl)
