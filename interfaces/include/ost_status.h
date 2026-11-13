/*
 * OrthoStream status codes (ADR-020 section 5).
 *
 * Every C interface of an OrthoStream library (ADR-011) returns an
 * OST_Status_t: zero for success, negative values for errors, as cFE does.
 * Codes are never renumbered. The header has no dependency on cFE or OSAL,
 * so that libraries, applications and host tools share it.
 */
#ifndef OST_STATUS_H
#define OST_STATUS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef int32_t OST_Status_t;

#define OST_SUCCESS                ((OST_Status_t)0)   /* success */
#define OST_ERR_INVALID_ARGUMENT   ((OST_Status_t)-1)  /* invalid argument or configuration */
#define OST_ERR_NOT_INITIALISED    ((OST_Status_t)-2)  /* handle missing or not ready */
#define OST_ERR_NO_MEMORY          ((OST_Status_t)-3)  /* allocation or memory budget exceeded */
#define OST_ERR_QUOTA              ((OST_Status_t)-4)  /* data store quota exhausted */
#define OST_ERR_IO                 ((OST_Status_t)-5)  /* file or device error */
#define OST_ERR_FORMAT             ((OST_Status_t)-6)  /* malformed input data */
#define OST_ERR_NUMERIC            ((OST_Status_t)-7)  /* numerical failure (no convergence, degenerate geometry) */
#define OST_ERR_GPU                ((OST_Status_t)-8)  /* GPU or CUDA error */
#define OST_ERR_TIMEOUT            ((OST_Status_t)-9)  /* time limit exceeded */
#define OST_ERR_UNSUPPORTED        ((OST_Status_t)-10) /* feature not built or not supported */
#define OST_ERR_INTERNAL           ((OST_Status_t)-11) /* unexpected error, including caught C++ exceptions */

/* Name of a status code, for events and logs; "OST_ERR_UNKNOWN" for codes
 * outside the list. */
static inline const char *OST_StatusName(OST_Status_t Status)
{
    switch (Status)
    {
        case OST_SUCCESS:              return "OST_SUCCESS";
        case OST_ERR_INVALID_ARGUMENT: return "OST_ERR_INVALID_ARGUMENT";
        case OST_ERR_NOT_INITIALISED:  return "OST_ERR_NOT_INITIALISED";
        case OST_ERR_NO_MEMORY:        return "OST_ERR_NO_MEMORY";
        case OST_ERR_QUOTA:            return "OST_ERR_QUOTA";
        case OST_ERR_IO:               return "OST_ERR_IO";
        case OST_ERR_FORMAT:           return "OST_ERR_FORMAT";
        case OST_ERR_NUMERIC:          return "OST_ERR_NUMERIC";
        case OST_ERR_GPU:              return "OST_ERR_GPU";
        case OST_ERR_TIMEOUT:          return "OST_ERR_TIMEOUT";
        case OST_ERR_UNSUPPORTED:      return "OST_ERR_UNSUPPORTED";
        case OST_ERR_INTERNAL:         return "OST_ERR_INTERNAL";
        default:                       return "OST_ERR_UNKNOWN";
    }
}

#ifdef __cplusplus
}
#endif

#endif /* OST_STATUS_H */
