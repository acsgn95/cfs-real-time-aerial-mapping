/*
 * Event identifiers of GEOREF_APP (ADR-020 section 3), listed in
 * docs/interfaces/events.md. Identifiers are never reused or renumbered.
 * 60-64 are the fault events of the fault set (georef_app.c, list order).
 */
#ifndef GEOREF_APP_EVENTIDS_H
#define GEOREF_APP_EVENTIDS_H

/* 1-19: life cycle and commands */
#define GEOREF_APP_INIT_INF_EID    1
#define GEOREF_APP_NOOP_INF_EID    2
#define GEOREF_APP_RESET_INF_EID   3
#define GEOREF_APP_MID_ERR_EID     4
#define GEOREF_APP_CMD_LEN_ERR_EID 5
#define GEOREF_APP_CC_ERR_EID      6
#define GEOREF_APP_PIPE_ERR_EID    7
#define GEOREF_APP_TBL_ERR_EID     8
#define GEOREF_APP_CLEAR_INF_EID   9
#define GEOREF_APP_INIT_ERR_EID    10

/* 20-59: data path */
#define GEOREF_APP_FAILED_EID        20
#define GEOREF_APP_QUOTA_EID         21
#define GEOREF_APP_MSG_LEN_ERR_EID   22
#define GEOREF_APP_LIB_ERR_EID       23
#define GEOREF_APP_DATASTORE_ERR_EID 24
#define GEOREF_APP_POINTS_DBG_EID    25
#define GEOREF_APP_LAZ_ERR_EID       26
#define GEOREF_APP_READY_INF_EID     27

#endif /* GEOREF_APP_EVENTIDS_H */
