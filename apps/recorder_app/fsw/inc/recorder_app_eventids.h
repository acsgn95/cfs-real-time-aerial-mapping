/*
 * Event identifiers of RECORDER_APP (ADR-020 section 3), listed in
 * docs/interfaces/events.md. Identifiers are never reused or renumbered.
 * 60-62 are the fault events of the fault set (recorder_app.c, list order).
 */
#ifndef RECORDER_APP_EVENTIDS_H
#define RECORDER_APP_EVENTIDS_H

/* 1-19: life cycle and commands */
#define RECORDER_APP_INIT_INF_EID    1
#define RECORDER_APP_NOOP_INF_EID    2
#define RECORDER_APP_RESET_INF_EID   3
#define RECORDER_APP_MID_ERR_EID     4
#define RECORDER_APP_CMD_LEN_ERR_EID 5
#define RECORDER_APP_CC_ERR_EID      6
#define RECORDER_APP_PIPE_ERR_EID    7
#define RECORDER_APP_TBL_ERR_EID     8
#define RECORDER_APP_CLEAR_INF_EID   9
#define RECORDER_APP_INIT_ERR_EID    10
#define RECORDER_APP_START_INF_EID   11
#define RECORDER_APP_STOP_INF_EID    12
#define RECORDER_APP_RUN_ID_ERR_EID  13

/* 20-59: data path */
#define RECORDER_APP_WRITE_ERR_EID  20
#define RECORDER_APP_ENCODE_ERR_EID 21
#define RECORDER_APP_DISK_EID       22
#define RECORDER_APP_SUB_ERR_EID    23
#define RECORDER_APP_FILE_ERR_EID   24

#endif /* RECORDER_APP_EVENTIDS_H */
