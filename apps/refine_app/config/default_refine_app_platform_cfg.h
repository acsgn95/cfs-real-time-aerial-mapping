/*
 * Platform configuration of REFINE_APP. A mission overrides it with a
 * refine_app_platform_cfg.h in its *_defs directory. Values that an
 * experiment changes are in the configuration table instead (ADR-016).
 */
#ifndef REFINE_APP_PLATFORM_CFG_H
#define REFINE_APP_PLATFORM_CFG_H

#define REFINE_APP_PIPE_DEPTH      32
#define REFINE_APP_PIPE_TIMEOUT_MS 100 /* ADR-015 section 1 */
#define REFINE_APP_TABLE_FILE      "/cf/refine_app_tbl.tbl"

/* Worker child task: feature extraction and adjustment (ADR-019 section 6:
 * priority level 3, below SELECT_APP; a lower number is a higher priority).
 * GTSAM and OpenCV allocate on the heap; the stack covers their recursion. */
#define REFINE_APP_WORKER_PRIORITY   80
#define REFINE_APP_WORKER_STACK_SIZE (4 * 1024 * 1024)
#define REFINE_APP_WORKER_TIMEOUT_MS 500 /* how often waiting batches are checked against the waiting limit */

/* Batches waiting for the worker (a ring of the application). */
#define REFINE_APP_QUEUE_DEPTH 16

/* Batches held for refinement at the same time. MaxWaitS / keyframe
 * interval: 100 s / 2.9 s = 35 at the nominal point. When all slots are in
 * use, every waiting batch is solved at once. */
#define REFINE_APP_MAX_HELD 64

/* Least time between two BATCH_READY messages. One window solution
 * publishes all batches it covers (12 at once on Dorfberg), and OSAL limits
 * every Software Bus pipe on Linux to /proc/sys/fs/mqueue/msg_max messages
 * (10 without privileges): a burst would overflow the pipes of DENSE_APP,
 * which would lose a batch with its data store counts, and of the
 * recorder. Spaced by a few milliseconds, every subscriber takes each batch
 * off its pipe in time. 0: no spacing. */
#define REFINE_APP_PUBLISH_GAP_MS 5

/* Grey copy of one working image for feature extraction. */
#define REFINE_APP_MAX_GREY_BYTES (16u * 1024u * 1024u)

#endif /* REFINE_APP_PLATFORM_CFG_H */
