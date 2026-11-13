/*
 * Public interface of ost_datastore, the shared data store of the
 * OrthoStream applications (ADR-013).
 *
 * ost_datastore is a cFS library module (CFE_LIB). OST_DS_Init creates the
 * one store of the mission with the sizes of ost_datastore_platform_cfg.h;
 * the applications use it through the functions below. References are the
 * EDS type OST_ObjectRef_t, so message fields are passed directly.
 *
 *   producer: OST_DS_Allocate -> write -> OST_DS_Seal -> publish the reference
 *   reader:   OST_DS_Acquire -> read -> OST_DS_Release
 *
 * Hand-over along the data path (ADR-013 section 3): a data path message
 * carries one reference count for each object it names, owned by the
 * application that consumes the message. That application releases it, or
 * hands it on in its own output message. A publisher that keeps using an
 * object acquires its own count before publishing.
 *
 * All functions are thread-safe. Status codes are those of datastore_c_api.h;
 * every function returns OST_ERR_NOT_INITIALISED if the store was not
 * created.
 */
#ifndef OST_DATASTORE_H
#define OST_DATASTORE_H

#include "common_types.h"
#include "ost_eds_typedefs.h"
#include "ost_status.h"
#include "datastore_c_api.h"

/* Library entry point, named in the startup script. */
int32 OST_DS_Init(void);

OST_Status_t OST_DS_Allocate(OST_Datastore_Type_t Type, uint64 Size, OST_ObjectRef_t *Ref, void **Data);
OST_Status_t OST_DS_Seal(const OST_ObjectRef_t *Ref);
OST_Status_t OST_DS_Abort(const OST_ObjectRef_t *Ref);
OST_Status_t OST_DS_Acquire(const OST_ObjectRef_t *Ref, const void **Data, uint64 *Size);
OST_Status_t OST_DS_Release(const OST_ObjectRef_t *Ref);
OST_Status_t OST_DS_GetStats(OST_Datastore_Stats_t *Stats);

#endif /* OST_DATASTORE_H */
