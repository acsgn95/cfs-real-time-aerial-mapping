/*
 * C interface of the OrthoStream data store (ADR-011, ADR-013).
 *
 * Write-once, reference-counted objects in one preallocated arena, with
 * quotas per object type:
 *
 *   producer: Allocate -> write -> Seal -> publish the reference
 *   reader:   Acquire -> read -> Release
 *
 * A producer that fails before sealing frees the object with Abort. An object
 * is freed when its last reference is released.
 *
 * All functions are thread-safe. Data of a sealed object is read without a
 * lock while a reference is held. No memory is allocated after Create.
 *
 * Status codes (ost_status.h):
 * - OST_ERR_QUOTA: the quota of the type would be exceeded;
 * - OST_ERR_NO_MEMORY: no free block is large enough, or the object table is
 *   full; from Create: the arena cannot be allocated;
 * - OST_ERR_INVALID_ARGUMENT: invalid argument or configuration, a stale
 *   reference (the object was freed; also counted in the statistics), or an
 *   operation in the wrong state (for example Acquire before Seal).
 */
#ifndef ORTHOSTREAM_DATASTORE_C_API_H
#define ORTHOSTREAM_DATASTORE_C_API_H

#include <stdint.h>

#include "ost_status.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    OST_DATASTORE_JPEG       = 0, /* original JPEG data */
    OST_DATASTORE_WORK_IMAGE = 1, /* working-resolution image */
    OST_DATASTORE_DEPTH_SET  = 2, /* depth, normal and confidence maps */
    OST_DATASTORE_POINT_SET  = 3, /* georeferenced points */
    OST_DATASTORE_TYPE_COUNT = 4
} OST_Datastore_Type_t;

/* Same layout as the EDS type OST/ObjectRef. object_id 0 means no object. */
typedef struct
{
    uint32_t object_id;
    uint32_t generation;
} OST_Datastore_Ref_t;

typedef struct
{
    uint64_t arena_bytes;                            /* multiple of 64 */
    uint64_t quota_bytes[OST_DATASTORE_TYPE_COUNT];  /* per type */
    uint32_t max_objects;                            /* size of the object table */
} OST_Datastore_Config_t;

typedef struct
{
    uint64_t bytes_in_use; /* sizes rounded up to 64 bytes */
    uint64_t peak_bytes;
    uint32_t objects_in_use;
    uint32_t failed_allocations;
} OST_Datastore_TypeStats_t;

typedef struct
{
    OST_Datastore_TypeStats_t types[OST_DATASTORE_TYPE_COUNT];
    uint64_t                  arena_bytes;
    uint64_t                  free_bytes;
    uint64_t                  largest_free_block;
    uint32_t                  max_objects;
    uint32_t                  objects_in_use;
    uint32_t                  stale_references;
} OST_Datastore_Stats_t;

typedef struct OST_Datastore OST_Datastore_t;

OST_Status_t OST_Datastore_Create(const OST_Datastore_Config_t *config, OST_Datastore_t **store);
void         OST_Datastore_Destroy(OST_Datastore_t *store);

/* Allocates a writable object of size bytes; the caller holds one
 * reference. */
OST_Status_t OST_Datastore_Allocate(OST_Datastore_t     *store,
                                    OST_Datastore_Type_t type,
                                    uint64_t             size,
                                    OST_Datastore_Ref_t *ref,
                                    void               **data);

/* Makes an allocated object immutable and readable. */
OST_Status_t OST_Datastore_Seal(OST_Datastore_t *store, OST_Datastore_Ref_t ref);

/* Frees an allocated object that was not sealed. */
OST_Status_t OST_Datastore_Abort(OST_Datastore_t *store, OST_Datastore_Ref_t ref);

/* Adds a reference to a sealed object. type may be NULL. */
OST_Status_t OST_Datastore_Acquire(OST_Datastore_t      *store,
                                   OST_Datastore_Ref_t   ref,
                                   const void          **data,
                                   uint64_t             *size,
                                   OST_Datastore_Type_t *type);

/* Removes a reference to a sealed object. */
OST_Status_t OST_Datastore_Release(OST_Datastore_t *store, OST_Datastore_Ref_t ref);

OST_Status_t OST_Datastore_GetStats(const OST_Datastore_t *store, OST_Datastore_Stats_t *stats);

#ifdef __cplusplus
}
#endif

#endif /* ORTHOSTREAM_DATASTORE_C_API_H */
