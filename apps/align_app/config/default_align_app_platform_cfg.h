/*
 * Platform configuration of ALIGN_APP. A mission overrides it with an
 * align_app_platform_cfg.h in its *_defs directory. Values that an
 * experiment changes are in the configuration table instead (ADR-016).
 */
#ifndef ALIGN_APP_PLATFORM_CFG_H
#define ALIGN_APP_PLATFORM_CFG_H

/* One pipe for commands and input messages, at most
 * CFE_PLATFORM_SB_MAX_PIPE_DEPTH (50). Per message: telemetry (10 Hz) up to
 * 3 s, images up to 16. An ImageArrived lost to a full pipe also loses its
 * JPEG reference count (ADR-013), so the image limit is generous. */
#define ALIGN_APP_PIPE_DEPTH       50
#define ALIGN_APP_TELEMETRY_LIMIT  30
#define ALIGN_APP_IMAGE_LIMIT      16

/* Receive timeout: how often waiting images are checked against the
 * waiting limit (ADR-014 section 4, ADR-015 section 1). */
#define ALIGN_APP_PIPE_TIMEOUT_MS 50

/* Images held by the application at the same time: waiting for telemetry
 * and decided but not yet published. Must be at least MaxPendingImages of
 * the table. */
#define ALIGN_APP_MAX_IMAGES 128

#define ALIGN_APP_TABLE_FILE "/cf/align_app_tbl.tbl"

#endif /* ALIGN_APP_PLATFORM_CFG_H */
