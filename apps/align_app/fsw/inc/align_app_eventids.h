/*
 * Event identifiers of ALIGN_APP (ADR-020 section 3), listed in
 * docs/interfaces/events.md. Identifiers are never reused or renumbered.
 * 60-63 are the fault events of the fault set (align_app.c, list order).
 */
#ifndef ALIGN_APP_EVENTIDS_H
#define ALIGN_APP_EVENTIDS_H

/* 1-19: life cycle and commands */
#define ALIGN_APP_INIT_INF_EID    1
#define ALIGN_APP_NOOP_INF_EID    2
#define ALIGN_APP_RESET_INF_EID   3
#define ALIGN_APP_MID_ERR_EID     4
#define ALIGN_APP_CMD_LEN_ERR_EID 5
#define ALIGN_APP_CC_ERR_EID      6
#define ALIGN_APP_PIPE_ERR_EID    7
#define ALIGN_APP_TBL_ERR_EID     8
#define ALIGN_APP_CLEAR_INF_EID   9
#define ALIGN_APP_INIT_ERR_EID    10

/* 20-59: data path */
#define ALIGN_APP_SESSION_INF_EID     20
#define ALIGN_APP_CALIBRATION_INF_EID 21
#define ALIGN_APP_CALIBRATION_ERR_EID 22
#define ALIGN_APP_IMAGE_REJECTED_EID  23
#define ALIGN_APP_MSG_LEN_ERR_EID     24
#define ALIGN_APP_DATASTORE_ERR_EID   25
#define ALIGN_APP_LIB_ERR_EID         26 /* a time_align call failed */

#endif /* ALIGN_APP_EVENTIDS_H */
