/*
 * Public interface of ost_common, the shared interface library of the
 * OrthoStream applications (ADR-020 section 1).
 *
 * ost_common is a cFS library module (CFE_LIB). It is loaded before the
 * OrthoStream applications and provides:
 *   * the EDS types of package OST (ost_eds_typedefs.h);
 *   * the status codes and diagnostics of the libraries (ost_status.h,
 *     ost_diag.h);
 *   * the fault catalog and fault sets (ost_fault.h);
 *   * time helpers and the message identifiers of the shared topics.
 */
#ifndef OST_COMMON_H
#define OST_COMMON_H

#include "cfe.h"
#include "ost_eds_typedefs.h"
#include "ost_status.h"
#include "ost_diag.h"
#include "ost_fault.h"

/* Version of the PayloadHeader.SchemaVersion field for the OST payloads. */
#define OST_FAULT_EVENT_SCHEMA_VERSION    1
#define OST_KEYFRAME_BATCH_SCHEMA_VERSION 1

/* Library entry point, named in the startup script. */
int32 OST_Common_Init(void);

/* UTC time, nanoseconds since the Unix epoch (CLOCK_REALTIME, ADR-014). */
int64 OST_TimeUtcNs(void);

/* CLOCK_MONOTONIC time in nanoseconds, for stage timings (ADR-014, ADR-017). */
int64 OST_TimeMonoNs(void);

/* Message identifiers of the shared topics. */
CFE_SB_MsgId_t OST_FaultEventMsgId(void);
CFE_SB_MsgId_t OST_BatchSelectedMsgId(void);
CFE_SB_MsgId_t OST_BatchReadyMsgId(void);

/* Copies Src into a fixed-size EDS string field and terminates it, also when
 * Src is longer. Src may be NULL. */
void OST_CopyText(char *Dst, size_t DstSize, const char *Src);

/*
 * Maps a library diagnostic to an EVS event (ADR-020 section 6). Context must
 * point to an OST_DiagEvents_t. Warnings and errors use the event identifiers
 * WarnEid and ErrorEid of the application (range 80-99); debug and
 * information messages use DebugEid with type DEBUG.
 */
typedef struct
{
    uint16 DebugEid;
    uint16 WarnEid;
    uint16 ErrorEid;
} OST_DiagEvents_t;

void OST_DiagToEvs(void *Context, OST_DiagLevel_t Level, OST_Status_t Status, const char *Text);

#endif /* OST_COMMON_H */
