/*
 * Event identifiers of INGEST_APP (ADR-020 section 3), listed in
 * docs/interfaces/events.md. Identifiers are never reused or renumbered.
 * 60-64 are the fault events of the fault set (ingest_app.c, list order).
 */
#ifndef INGEST_APP_EVENTIDS_H
#define INGEST_APP_EVENTIDS_H

/* 1-19: life cycle and commands */
#define INGEST_APP_INIT_INF_EID     1
#define INGEST_APP_NOOP_INF_EID     2
#define INGEST_APP_RESET_INF_EID    3
#define INGEST_APP_MID_ERR_EID      4
#define INGEST_APP_CMD_LEN_ERR_EID  5
#define INGEST_APP_CC_ERR_EID       6
#define INGEST_APP_PIPE_ERR_EID     7
#define INGEST_APP_TBL_ERR_EID      8
#define INGEST_APP_CLEAR_INF_EID    9
#define INGEST_APP_INIT_ERR_EID     10
#define INGEST_APP_CMD_REFUSED_EID  11

/* 20-59: data path */
#define INGEST_APP_START_INF_EID        20
#define INGEST_APP_STOP_INF_EID         21
#define INGEST_APP_SOCKET_ERR_EID       22
#define INGEST_APP_CONNECTED_INF_EID    23
#define INGEST_APP_DISCONNECTED_INF_EID 24
#define INGEST_APP_SESSION_INF_EID      25
#define INGEST_APP_SESSION_END_INF_EID  26
#define INGEST_APP_IMAGE_DROPPED_EID    27
#define INGEST_APP_FRAME_REJECTED_EID   28
#define INGEST_APP_TLM_REJECTED_EID     29

#endif /* INGEST_APP_EVENTIDS_H */
