/*
 * Library diagnostics (ADR-011 section 3, ADR-020 section 6).
 *
 * Libraries never print. They report diagnostics through a callback that the
 * caller provides when it creates a handle. A cFS application maps them to
 * EVS events: warnings and errors to its event range 80-99, debug and
 * information messages to EVS DEBUG events, disabled by default. Host tools
 * may print them.
 *
 * The callback runs on the task that called the library function, during
 * that call. The text is valid only during the callback.
 */
#ifndef OST_DIAG_H
#define OST_DIAG_H

#include "ost_status.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    OST_DIAG_DEBUG       = 0,
    OST_DIAG_INFORMATION = 1,
    OST_DIAG_WARNING     = 2,
    OST_DIAG_ERROR       = 3
} OST_DiagLevel_t;

/* Status is OST_SUCCESS for messages that do not report a failure. */
typedef void (*OST_DiagFn_t)(void *Context, OST_DiagLevel_t Level, OST_Status_t Status, const char *Text);

typedef struct
{
    OST_DiagFn_t Fn;      /* NULL: diagnostics are discarded */
    void        *Context; /* passed unchanged to Fn */
} OST_DiagSink_t;

#ifdef __cplusplus
}
#endif

#endif /* OST_DIAG_H */
