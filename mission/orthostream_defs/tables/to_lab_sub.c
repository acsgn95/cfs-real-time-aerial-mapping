/*
 * TO_LAB subscriptions of the OrthoStream reference mission (ADR-021
 * section 2): housekeeping of cFE and the OrthoStream applications, fault
 * events, EVS event messages, and the small status messages of the data
 * path. Bulk data never goes to the ground.
 *
 * During development the small data path messages (telemetry samples at
 * 10 Hz, one ImageArrived per image) are sent as well; they are removed
 * here for measurement runs if the link load matters.
 */
#include "cfe_tbl_filedef.h"
#include "cfe_sb_api_typedefs.h"
#include "to_lab_tbl.h"
#include "cfe_msgids.h"
#include "cfe_mission_eds_designparameters.h"

#define OST_TO_TLM(topic, depth) \
    {CFE_SB_MSGID_WRAP_VALUE(CFE_PLATFORM_TLM_TOPICID_TO_MIDV(EdsParam_CFE_MISSION_##topic)), {0, 0}, depth}

TO_LAB_Subs_t Subscriptions = {.Subs = {
    /* cFE core */
    {CFE_SB_MSGID_WRAP_VALUE(CFE_ES_HK_TLM_MID), {0, 0}, 4},
    {CFE_SB_MSGID_WRAP_VALUE(CFE_EVS_HK_TLM_MID), {0, 0}, 4},
    {CFE_SB_MSGID_WRAP_VALUE(CFE_SB_HK_TLM_MID), {0, 0}, 4},
    {CFE_SB_MSGID_WRAP_VALUE(CFE_TBL_HK_TLM_MID), {0, 0}, 4},
    {CFE_SB_MSGID_WRAP_VALUE(CFE_TIME_HK_TLM_MID), {0, 0}, 4},
    {CFE_SB_MSGID_WRAP_VALUE(CFE_EVS_LONG_EVENT_MSG_MID), {0, 0}, 32},
    {CFE_SB_MSGID_WRAP_VALUE(CFE_ES_APP_TLM_MID), {0, 0}, 4},

    /* OrthoStream, shared */
    OST_TO_TLM(OST_FAULT_EVENT_TLM_TOPICID, 16),

    /* INGEST_APP */
    OST_TO_TLM(INGEST_APP_HK_TLM_TOPICID, 4),
    OST_TO_TLM(INGEST_APP_SESSION_CALIBRATION_TLM_TOPICID, 4),
    OST_TO_TLM(INGEST_APP_IMAGE_ARRIVED_TLM_TOPICID, 8),
    OST_TO_TLM(INGEST_APP_TELEMETRY_SAMPLE_TLM_TOPICID, 16),

    /* ALIGN_APP */
    OST_TO_TLM(ALIGN_APP_HK_TLM_TOPICID, 4),
    OST_TO_TLM(ALIGN_APP_IMAGE_POSE_TLM_TOPICID, 8),

    /* End of the list */
    {CFE_SB_MSGID_RESERVED, {0, 0}, 0}}};

CFE_TBL_FILEDEF(Subscriptions, TO_LAB.Subscriptions, OrthoStream subscriptions, to_lab_sub.tbl)
