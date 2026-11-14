/*
 * Event identifiers of MAPPROD_APP (ADR-020 section 3), listed in
 * docs/interfaces/events.md. Identifiers are never reused or renumbered.
 * 60-64 are the fault events of the fault set (mapprod_app.c, list order).
 */
#ifndef MAPPROD_APP_EVENTIDS_H
#define MAPPROD_APP_EVENTIDS_H

/* 1-19: life cycle and commands */
#define MAPPROD_APP_INIT_INF_EID     1
#define MAPPROD_APP_NOOP_INF_EID     2
#define MAPPROD_APP_RESET_INF_EID    3
#define MAPPROD_APP_MID_ERR_EID      4
#define MAPPROD_APP_CMD_LEN_ERR_EID  5
#define MAPPROD_APP_CC_ERR_EID       6
#define MAPPROD_APP_PIPE_ERR_EID     7
#define MAPPROD_APP_TBL_ERR_EID      8
#define MAPPROD_APP_CLEAR_INF_EID    9
#define MAPPROD_APP_INIT_ERR_EID     10
#define MAPPROD_APP_FINALISE_INF_EID 11

/* 20-59: data path */
#define MAPPROD_APP_FAILED_EID        20
#define MAPPROD_APP_MSG_LEN_ERR_EID   21
#define MAPPROD_APP_LIB_ERR_EID       22
#define MAPPROD_APP_DATASTORE_ERR_EID 23
#define MAPPROD_APP_UPDATE_DBG_EID    24
#define MAPPROD_APP_DECODE_ERR_EID    25
#define MAPPROD_APP_WRITE_ERR_EID     26
#define MAPPROD_APP_READY_INF_EID     27
#define MAPPROD_APP_CALIBRATION_EID   28
#define MAPPROD_APP_NO_ORTHO_EID      29
#define MAPPROD_APP_GRID_ERR_EID      30
#define MAPPROD_APP_PUBLISH_DBG_EID   31

#endif /* MAPPROD_APP_EVENTIDS_H */
