/*
 * Platform configuration of INGEST_APP. A mission overrides it with an
 * ingest_app_platform_cfg.h in its *_defs directory. Values that an
 * experiment changes are in the configuration table instead (ADR-016).
 */
#ifndef INGEST_APP_PLATFORM_CFG_H
#define INGEST_APP_PLATFORM_CFG_H

#define INGEST_APP_PIPE_DEPTH      16
#define INGEST_APP_PIPE_TIMEOUT_MS 100 /* ADR-015 section 1 */
#define INGEST_APP_TABLE_FILE      "/cf/ingest_app_tbl.tbl"

/* Receiver child tasks (ADR-015 section 2). INGEST_APP has the highest
 * priority of the OrthoStream applications (section 4); a lower number is a
 * higher priority. */
#define INGEST_APP_NET_TASK_PRIORITY   40
#define INGEST_APP_NET_TASK_STACK_SIZE 32768
#define INGEST_APP_SOCKET_TIMEOUT_MS   200 /* how often the receivers check for stop */

#endif /* INGEST_APP_PLATFORM_CFG_H */
