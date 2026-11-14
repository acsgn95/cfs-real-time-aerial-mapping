/*
 * Platform configuration of DENSE_APP. A mission overrides it with a
 * dense_app_platform_cfg.h in its *_defs directory. Values that an
 * experiment changes are in the configuration table instead (ADR-016).
 */
#ifndef DENSE_APP_PLATFORM_CFG_H
#define DENSE_APP_PLATFORM_CFG_H

#define DENSE_APP_PIPE_DEPTH      32
#define DENSE_APP_PIPE_TIMEOUT_MS 100 /* ADR-015 section 1 */
#define DENSE_APP_TABLE_FILE      "/cf/dense_app_tbl.tbl"

/* Worker child task: one depth job at a time (ADR-015 sections 2 and 4:
 * priority level 4, the lowest of the data path; a lower number is a
 * higher priority). OpenCV and COLMAP allocate on the heap and run their
 * own threads; the stack covers their recursion. */
#define DENSE_APP_WORKER_PRIORITY   90
#define DENSE_APP_WORKER_STACK_SIZE (4 * 1024 * 1024)

/* Upper limit of the table value MaxWaitingBatches (a ring of the
 * application). REFINE_APP publishes the batches one window solution
 * covers together; Dorfberg gives bursts of up to 7. */
#define DENSE_APP_QUEUE_DEPTH 16

/* Largest working image in pixels (2048 x 2048); the grey copies of a
 * batch take (1 + MAX_SOURCE_VIEWS) times this. */
#define DENSE_APP_MAX_PIXELS (2048u * 2048u)

/* Retry period while waiting for depth set quota. */
#define DENSE_APP_QUOTA_RETRY_MS 100

#endif /* DENSE_APP_PLATFORM_CFG_H */
