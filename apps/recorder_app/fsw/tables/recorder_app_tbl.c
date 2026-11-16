/*
 * Default configuration table of RECORDER_APP (ADR-016 section 5, ADR-017):
 * the data path messages with their trace times, the housekeeping of cFE
 * and of the OrthoStream applications, fault events and EVS event
 * messages. Bulk data never goes into a recording (ADR-001). A mission
 * replaces the list to record more or less.
 */
#include "cfe_tbl_filedef.h"
#include "cfe_msgids.h"
#include "cfe_mission_eds_designparameters.h"
#include "recorder_app_eds_typedefs.h"

#define OST_REC_TLM(topic, limit) \
    {CFE_PLATFORM_TLM_TOPICID_TO_MIDV(EdsParam_CFE_MISSION_##topic), limit, 0}

RECORDER_APP_ConfigTable_t ConfigTable = {
    .OutputDir      = "/cf/runs",
    .AutoStart      = 1,
    .MinFreeMb      = 1024,
    .WarnFreeMb     = 10240,
    .MaxRecordingMb = 0,
    .Subscriptions  = {
        /* cFE core */
        {CFE_ES_HK_TLM_MID, 4, 0},
        {CFE_EVS_HK_TLM_MID, 4, 0},
        {CFE_SB_HK_TLM_MID, 4, 0},
        {CFE_TIME_HK_TLM_MID, 4, 0},
        {CFE_EVS_LONG_EVENT_MSG_MID, 32, 0},

        /* OrthoStream, shared */
        OST_REC_TLM(OST_FAULT_EVENT_TLM_TOPICID, 16),
        OST_REC_TLM(OST_BATCH_SELECTED_TLM_TOPICID, 16),
        OST_REC_TLM(OST_BATCH_READY_TLM_TOPICID, 16),

        /* Data path and housekeeping of the applications */
        OST_REC_TLM(INGEST_APP_HK_TLM_TOPICID, 4),
        OST_REC_TLM(INGEST_APP_SESSION_CALIBRATION_TLM_TOPICID, 4),
        OST_REC_TLM(INGEST_APP_IMAGE_ARRIVED_TLM_TOPICID, 8),
        OST_REC_TLM(INGEST_APP_TELEMETRY_SAMPLE_TLM_TOPICID, 16),
        OST_REC_TLM(ALIGN_APP_HK_TLM_TOPICID, 4),
        OST_REC_TLM(ALIGN_APP_IMAGE_POSE_TLM_TOPICID, 8),
        OST_REC_TLM(SELECT_APP_HK_TLM_TOPICID, 4),
        OST_REC_TLM(REFINE_APP_HK_TLM_TOPICID, 4),
        OST_REC_TLM(DENSE_APP_HK_TLM_TOPICID, 4),
        OST_REC_TLM(DENSE_APP_DEPTH_READY_TLM_TOPICID, 16),
        OST_REC_TLM(DENSE_APP_DENSE_STATUS_TLM_TOPICID, 16),
        OST_REC_TLM(GEOREF_APP_HK_TLM_TOPICID, 4),
        OST_REC_TLM(GEOREF_APP_POINTS_READY_TLM_TOPICID, 8),
        OST_REC_TLM(MAPPROD_APP_HK_TLM_TOPICID, 4),
        OST_REC_TLM(MAPPROD_APP_TILE_UPDATED_TLM_TOPICID, 8),
        OST_REC_TLM(MAPPROD_APP_TILE_PUBLISHED_TLM_TOPICID, 8),
        OST_REC_TLM(RECORDER_APP_HK_TLM_TOPICID, 4),
    },
};

CFE_TBL_FILEDEF(ConfigTable, RECORDER_APP.ConfigTable, RECORDER_APP configuration, recorder_tbl.tbl)
