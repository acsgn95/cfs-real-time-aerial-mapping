/*
 * Platform configuration of SELECT_APP. A mission overrides it with a
 * select_app_platform_cfg.h in its *_defs directory. Values that an
 * experiment changes are in the configuration table instead (ADR-016).
 */
#ifndef SELECT_APP_PLATFORM_CFG_H
#define SELECT_APP_PLATFORM_CFG_H

#define SELECT_APP_PIPE_DEPTH      32
#define SELECT_APP_PIPE_TIMEOUT_MS 100 /* ADR-015 section 1 */
#define SELECT_APP_TABLE_FILE      "/cf/select_app_tbl.tbl"

/* Worker child task (ADR-015 section 2): selection, decoding to the
 * working resolution, batches. Below INGEST_APP and ALIGN_APP (section 4);
 * a lower number is a higher priority. */
#define SELECT_APP_WORKER_PRIORITY   60
#define SELECT_APP_WORKER_STACK_SIZE 262144
#define SELECT_APP_WORKER_TIMEOUT_MS 200 /* how often the worker checks for stop */

/* Items (images and calibrations) waiting for the worker, in a ring of the
 * application. Images arrive about every 1.5 s at the nominal point. */
#define SELECT_APP_QUEUE_DEPTH 32

/* Images held in the pool at the same time; at least MaxPoolImages of the
 * table. */
#define SELECT_APP_MAX_POOL 256

/* Pool images released to make room for one working image before giving up. */
#define SELECT_APP_MAX_QUOTA_EVICTIONS 8

#endif /* SELECT_APP_PLATFORM_CFG_H */
