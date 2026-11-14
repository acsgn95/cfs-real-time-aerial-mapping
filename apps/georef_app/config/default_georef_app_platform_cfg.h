/*
 * Platform configuration of GEOREF_APP. A mission overrides it with a
 * georef_app_platform_cfg.h in its *_defs directory. Values that an
 * experiment changes are in the configuration table instead (ADR-016).
 */
#ifndef GEOREF_APP_PLATFORM_CFG_H
#define GEOREF_APP_PLATFORM_CFG_H

#define GEOREF_APP_PIPE_DEPTH      32
#define GEOREF_APP_PIPE_TIMEOUT_MS 100 /* ADR-015 section 1 */
#define GEOREF_APP_TABLE_FILE      "/cf/georef_app_tbl.tbl"

/* Worker child task: conversion, publication and the point cloud files
 * (ADR-015 sections 2 and 4: priority level 3, above DENSE_APP's worker;
 * a lower number is a higher priority). PROJ and LASzip allocate on the
 * heap. */
#define GEOREF_APP_WORKER_PRIORITY   85
#define GEOREF_APP_WORKER_STACK_SIZE (1024 * 1024)

/* Depth sets waiting for the worker. GEOREF_APP does not drop data
 * (ADR-015 section 3); a depth set arriving at a full queue is reported as
 * an error. DENSE_APP publishes at most one depth set per job. */
#define GEOREF_APP_QUEUE_DEPTH 16

/* Largest depth map in pixels (2048 x 2048), as DENSE_APP. */
#define GEOREF_APP_MAX_PIXELS (2048u * 2048u)

/* Retry period while waiting for point set quota. */
#define GEOREF_APP_QUOTA_RETRY_MS 100

#endif /* GEOREF_APP_PLATFORM_CFG_H */
