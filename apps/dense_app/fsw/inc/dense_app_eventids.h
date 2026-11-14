/*
 * Event identifiers of DENSE_APP (ADR-020 section 3), listed in
 * docs/interfaces/events.md. Identifiers are never reused or renumbered.
 * 60-66 are the fault events of the fault set (dense_app.c, list order).
 */
#ifndef DENSE_APP_EVENTIDS_H
#define DENSE_APP_EVENTIDS_H

/* 1-19: life cycle and commands */
#define DENSE_APP_INIT_INF_EID    1
#define DENSE_APP_NOOP_INF_EID    2
#define DENSE_APP_RESET_INF_EID   3
#define DENSE_APP_MID_ERR_EID     4
#define DENSE_APP_CMD_LEN_ERR_EID 5
#define DENSE_APP_CC_ERR_EID      6
#define DENSE_APP_PIPE_ERR_EID    7
#define DENSE_APP_TBL_ERR_EID     8
#define DENSE_APP_CLEAR_INF_EID   9
#define DENSE_APP_INIT_ERR_EID    10

/* 20-59: data path */
#define DENSE_APP_SKIPPED_EID       20
#define DENSE_APP_FAILED_EID        21
#define DENSE_APP_QUOTA_EID         22
#define DENSE_APP_MSG_LEN_ERR_EID   23
#define DENSE_APP_LIB_ERR_EID       24
#define DENSE_APP_DATASTORE_ERR_EID 25
#define DENSE_APP_BATCH_DBG_EID     26
#define DENSE_APP_TERRAIN_EID       27
#define DENSE_APP_GPU_EID           28
#define DENSE_APP_READY_INF_EID     29

#endif /* DENSE_APP_EVENTIDS_H */
