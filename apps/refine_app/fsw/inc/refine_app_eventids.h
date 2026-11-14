/*
 * Event identifiers of REFINE_APP (ADR-020 section 3), listed in
 * docs/interfaces/events.md. Identifiers are never reused or renumbered.
 * 60-63 are the fault events of the fault set (refine_app.c, list order).
 */
#ifndef REFINE_APP_EVENTIDS_H
#define REFINE_APP_EVENTIDS_H

/* 1-19: life cycle and commands */
#define REFINE_APP_INIT_INF_EID    1
#define REFINE_APP_NOOP_INF_EID    2
#define REFINE_APP_RESET_INF_EID   3
#define REFINE_APP_MID_ERR_EID     4
#define REFINE_APP_CMD_LEN_ERR_EID 5
#define REFINE_APP_CC_ERR_EID      6
#define REFINE_APP_PIPE_ERR_EID    7
#define REFINE_APP_TBL_ERR_EID     8
#define REFINE_APP_CLEAR_INF_EID   9
#define REFINE_APP_INIT_ERR_EID    10
#define REFINE_APP_FLUSH_INF_EID   11

/* 20-59: data path */
#define REFINE_APP_SESSION_INF_EID    20
#define REFINE_APP_FAILED_EID         21
#define REFINE_APP_LATENCY_EID        22
#define REFINE_APP_QUEUE_FULL_EID     23
#define REFINE_APP_MSG_LEN_ERR_EID    24
#define REFINE_APP_LIB_ERR_EID        25
#define REFINE_APP_DATASTORE_ERR_EID  26
#define REFINE_APP_BATCH_DBG_EID      27
#define REFINE_APP_HELD_FULL_EID      28

#endif /* REFINE_APP_EVENTIDS_H */
