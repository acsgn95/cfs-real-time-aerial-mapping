/*
 * Platform configuration of MAPPROD_APP. A mission overrides it with a
 * mapprod_app_platform_cfg.h in its *_defs directory. Values that an
 * experiment changes are in the configuration table instead (ADR-016).
 */
#ifndef MAPPROD_APP_PLATFORM_CFG_H
#define MAPPROD_APP_PLATFORM_CFG_H

#define MAPPROD_APP_PIPE_DEPTH      32
#define MAPPROD_APP_PIPE_TIMEOUT_MS 100 /* ADR-015 section 1 */
#define MAPPROD_APP_TABLE_FILE      "/cf/mapprod_app_tbl.tbl"

/* Worker child task: fusion, orthomosaic, lifecycle and publication
 * (ADR-015 sections 2 and 4: priority level 3; a lower number is a higher
 * priority). The library is not thread-safe, so one task owns the tiles.
 * GDAL and PROJ allocate on the heap. */
#define MAPPROD_APP_WORKER_PRIORITY   85
#define MAPPROD_APP_WORKER_STACK_SIZE (2 * 1024 * 1024)

/* Publisher child task: writes copies of the changed tiles (GDAL). Below
 * the worker: publication is not on the path of the map update (ADR-007
 * section 6). */
#define MAPPROD_APP_PUBLISHER_PRIORITY   95
#define MAPPROD_APP_PUBLISHER_STACK_SIZE (1024 * 1024)

/* Items waiting for the worker: point sets, the session calibration and
 * publication requests. MAPPROD_APP does not drop point sets (ADR-015
 * section 3); one finding the queue full is reported as an error. */
#define MAPPROD_APP_QUEUE_DEPTH 16

/* Largest keyframe decoded for the orthomosaic, in pixels (RGB):
 * 4096 x 3072. Dorfberg (7952 x 5304) decodes at 1/2 to 3976 x 2652. */
#define MAPPROD_APP_MAX_ORTHO_PIXELS (4096u * 3072u)

/* Time the main task waits for the worker to finish its last work (final
 * tiles) when the application stops. */
#define MAPPROD_APP_STOP_WAIT_MS 60000

#endif /* MAPPROD_APP_PLATFORM_CFG_H */
