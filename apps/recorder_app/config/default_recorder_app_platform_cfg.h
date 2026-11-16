/*
 * Platform configuration of RECORDER_APP. A mission overrides it with a
 * recorder_app_platform_cfg.h in its *_defs directory. Values that an
 * experiment changes are in the configuration table instead (ADR-016).
 */
#ifndef RECORDER_APP_PLATFORM_CFG_H
#define RECORDER_APP_PLATFORM_CFG_H

/* cFE limits a pipe to 50 messages (CFE_PLATFORM_SB_MAX_PIPE_DEPTH), and
 * OSAL on Linux truncates it to /proc/sys/fs/mqueue/msg_max (10 without
 * privileges). SCH_LAB requests all housekeeping in the same tick, so about
 * 15 messages arrive at once: the subscriptions are spread over several
 * pipes (subscription i on pipe i mod RECORDER_APP_PIPES). The commands are
 * on pipe 0. */
#define RECORDER_APP_PIPES           3
#define RECORDER_APP_PIPE_DEPTH      50
#define RECORDER_APP_PIPE_TIMEOUT_MS 20 /* waiting on pipe 0 when all are empty */
#define RECORDER_APP_TABLE_FILE      "/cf/recorder_tbl.tbl"

/* Buffer between the main task, which takes messages off the pipe, and
 * the writer task: a write to the disk may block for seconds (dirty page
 * writeback under memory pressure), which must not stop the pipe from
 * being emptied. A message that finds the buffer full is not recorded
 * and counted. */
#define RECORDER_APP_BUFFER_BYTES (4u * 1024u * 1024u)

/* Writer child task (below the data path). */
#define RECORDER_APP_WRITER_PRIORITY   95
#define RECORDER_APP_WRITER_STACK_SIZE (64 * 1024)

/* Time a stop waits for the writer to empty the buffer. */
#define RECORDER_APP_DRAIN_WAIT_MS 5000

/* Name of the recording file and of the manifest in a run directory. */
#define RECORDER_APP_RECORDING_NAME "recording.ostrec"
#define RECORDER_APP_MANIFEST_NAME  "manifest.json"

#endif /* RECORDER_APP_PLATFORM_CFG_H */
