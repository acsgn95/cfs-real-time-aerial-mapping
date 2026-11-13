/*
 * ost_datastore library module: creates the store of the mission and
 * forwards the calls of the applications to it (ADR-013).
 */
#include "ost_datastore.h"
#include "ost_datastore_platform_cfg.h"

#include "cfe.h"

#include <string.h>

static OST_Datastore_t *OST_DS_Store = NULL;

int32 OST_DS_Init(void)
{
    OST_Datastore_Config_t Config;
    OST_Status_t           Status;

    memset(&Config, 0, sizeof(Config));
    Config.arena_bytes                            = OST_DATASTORE_ARENA_BYTES;
    Config.quota_bytes[OST_DATASTORE_JPEG]        = OST_DATASTORE_QUOTA_JPEG;
    Config.quota_bytes[OST_DATASTORE_WORK_IMAGE]  = OST_DATASTORE_QUOTA_WORK_IMAGE;
    Config.quota_bytes[OST_DATASTORE_DEPTH_SET]   = OST_DATASTORE_QUOTA_DEPTH_SET;
    Config.quota_bytes[OST_DATASTORE_POINT_SET]   = OST_DATASTORE_QUOTA_POINT_SET;
    Config.max_objects                            = OST_DATASTORE_MAX_OBJECTS;

    Status = OST_Datastore_Create(&Config, &OST_DS_Store);
    if (Status != OST_SUCCESS)
    {
        CFE_ES_WriteToSysLog("OST_DATASTORE: store not created (%s), arena %lu MiB\n", OST_StatusName(Status),
                             (unsigned long)(Config.arena_bytes >> 20));
        return CFE_STATUS_EXTERNAL_RESOURCE_FAIL;
    }
    OS_printf("OST_DATASTORE initialised: arena %lu MiB, %lu objects\n", (unsigned long)(Config.arena_bytes >> 20),
              (unsigned long)Config.max_objects);
    return CFE_SUCCESS;
}

static OST_Datastore_Ref_t OST_DS_ToRef(const OST_ObjectRef_t *Ref)
{
    OST_Datastore_Ref_t R;

    R.object_id  = Ref->ObjectId;
    R.generation = Ref->Generation;
    return R;
}

OST_Status_t OST_DS_Allocate(OST_Datastore_Type_t Type, uint64 Size, OST_ObjectRef_t *Ref, void **Data)
{
    OST_Datastore_Ref_t R;
    OST_Status_t        Status;

    if (OST_DS_Store == NULL)
    {
        return OST_ERR_NOT_INITIALISED;
    }
    if (Ref == NULL)
    {
        return OST_ERR_INVALID_ARGUMENT;
    }
    Status = OST_Datastore_Allocate(OST_DS_Store, Type, Size, &R, Data);
    if (Status == OST_SUCCESS)
    {
        Ref->ObjectId   = R.object_id;
        Ref->Generation = R.generation;
    }
    return Status;
}

OST_Status_t OST_DS_Seal(const OST_ObjectRef_t *Ref)
{
    if (OST_DS_Store == NULL)
    {
        return OST_ERR_NOT_INITIALISED;
    }
    return (Ref == NULL) ? OST_ERR_INVALID_ARGUMENT : OST_Datastore_Seal(OST_DS_Store, OST_DS_ToRef(Ref));
}

OST_Status_t OST_DS_Abort(const OST_ObjectRef_t *Ref)
{
    if (OST_DS_Store == NULL)
    {
        return OST_ERR_NOT_INITIALISED;
    }
    return (Ref == NULL) ? OST_ERR_INVALID_ARGUMENT : OST_Datastore_Abort(OST_DS_Store, OST_DS_ToRef(Ref));
}

OST_Status_t OST_DS_Acquire(const OST_ObjectRef_t *Ref, const void **Data, uint64 *Size)
{
    uint64_t     Bytes = 0;
    OST_Status_t Status;

    if (OST_DS_Store == NULL)
    {
        return OST_ERR_NOT_INITIALISED;
    }
    if (Ref == NULL || Size == NULL)
    {
        return OST_ERR_INVALID_ARGUMENT;
    }
    Status = OST_Datastore_Acquire(OST_DS_Store, OST_DS_ToRef(Ref), Data, &Bytes, NULL);
    *Size  = Bytes;
    return Status;
}

OST_Status_t OST_DS_Release(const OST_ObjectRef_t *Ref)
{
    if (OST_DS_Store == NULL)
    {
        return OST_ERR_NOT_INITIALISED;
    }
    return (Ref == NULL) ? OST_ERR_INVALID_ARGUMENT : OST_Datastore_Release(OST_DS_Store, OST_DS_ToRef(Ref));
}

OST_Status_t OST_DS_GetStats(OST_Datastore_Stats_t *Stats)
{
    if (OST_DS_Store == NULL)
    {
        return OST_ERR_NOT_INITIALISED;
    }
    return OST_Datastore_GetStats(OST_DS_Store, Stats);
}
