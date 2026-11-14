/*
 * Event identifiers of SELECT_APP (ADR-020 section 3), listed in
 * docs/interfaces/events.md. Identifiers are never reused or renumbered.
 * 60-64 are the fault events of the fault set (select_app.c, list order).
 */
#ifndef SELECT_APP_EVENTIDS_H
#define SELECT_APP_EVENTIDS_H

/* 1-19: life cycle and commands */
#define SELECT_APP_INIT_INF_EID    1
#define SELECT_APP_NOOP_INF_EID    2
#define SELECT_APP_RESET_INF_EID   3
#define SELECT_APP_MID_ERR_EID     4
#define SELECT_APP_CMD_LEN_ERR_EID 5
#define SELECT_APP_CC_ERR_EID      6
#define SELECT_APP_PIPE_ERR_EID    7
#define SELECT_APP_TBL_ERR_EID     8
#define SELECT_APP_CLEAR_INF_EID   9
#define SELECT_APP_INIT_ERR_EID    10

/* 20-59: data path */
#define SELECT_APP_SESSION_INF_EID     20
#define SELECT_APP_CALIBRATION_ERR_EID 21
#define SELECT_APP_TERRAIN_ERR_EID     22
#define SELECT_APP_DECODE_ERR_EID      23
#define SELECT_APP_QUOTA_ERR_EID       24
#define SELECT_APP_QUEUE_FULL_EID      25
#define SELECT_APP_MSG_LEN_ERR_EID     26
#define SELECT_APP_LIB_ERR_EID         27
#define SELECT_APP_KEYFRAME_DBG_EID    28
#define SELECT_APP_DATASTORE_ERR_EID   29

#endif /* SELECT_APP_EVENTIDS_H */
