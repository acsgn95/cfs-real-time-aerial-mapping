/*
 * Uses the C interface from C99, as a cFS application does: a producer
 * writes and seals a JPEG object, a reader acquires and releases it.
 * Returns 0 on success or the number of the failed check.
 */

#include <string.h>

#include "datastore_c_api.h"

int ost_datastore_c_api_usage(void)
{
    OST_Datastore_Config_t config;
    OST_Datastore_t       *store = NULL;
    OST_Datastore_Ref_t    ref;
    OST_Datastore_Stats_t  stats;
    OST_Datastore_Type_t   type = OST_DATASTORE_POINT_SET;
    void                  *data = NULL;
    const void            *view = NULL;
    uint64_t               size = 0;
    int                    result = 0;

    memset(&config, 0, sizeof(config));
    config.arena_bytes                         = 1024 * 1024;
    config.quota_bytes[OST_DATASTORE_JPEG]     = 512 * 1024;
    config.quota_bytes[OST_DATASTORE_POINT_SET] = 512 * 1024;
    config.max_objects                         = 8;

    if (OST_Datastore_Create(&config, &store) != OST_SUCCESS || store == NULL)
    {
        return 1;
    }
    if (OST_Datastore_Allocate(store, OST_DATASTORE_JPEG, 100, &ref, &data) != OST_SUCCESS || ref.object_id == 0)
    {
        result = 2;
    }
    else
    {
        memcpy(data, "OSTL", 4);
        if (OST_Datastore_Seal(store, ref) != OST_SUCCESS)
        {
            result = 3;
        }
        else if (OST_Datastore_Acquire(store, ref, &view, &size, &type) != OST_SUCCESS || size != 100 ||
                 type != OST_DATASTORE_JPEG || memcmp(view, "OSTL", 4) != 0)
        {
            result = 4;
        }
        else if (OST_Datastore_Release(store, ref) != OST_SUCCESS || OST_Datastore_Release(store, ref) != OST_SUCCESS)
        {
            result = 5;
        }
        else if (OST_Datastore_Acquire(store, ref, &view, &size, NULL) != OST_ERR_INVALID_ARGUMENT)
        {
            result = 6; /* stale */
        }
        else if (OST_Datastore_Allocate(store, OST_DATASTORE_WORK_IMAGE, 64, &ref, &data) != OST_ERR_QUOTA)
        {
            result = 7; /* quota 0 */
        }
        else if (OST_Datastore_GetStats(store, &stats) != OST_SUCCESS || stats.objects_in_use != 0 ||
                 stats.stale_references != 1 || stats.types[OST_DATASTORE_WORK_IMAGE].failed_allocations != 1)
        {
            result = 8;
        }
    }
    OST_Datastore_Destroy(store);
    return result;
}
