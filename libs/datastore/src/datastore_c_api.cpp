#include "datastore_c_api.h"

#include <new>

#include "orthostream/datastore/store.hpp"

namespace ds = orthostream::datastore;

struct OST_Datastore
{
    explicit OST_Datastore(const ds::Config &config) : store(config) {}
    ds::Store store;
};

namespace {

ds::Ref toRef(OST_Datastore_Ref_t ref)
{
    return {ref.object_id, ref.generation};
}

OST_Status_t toStatus(ds::Result result)
{
    switch (result)
    {
        case ds::Result::Ok:
            return OST_SUCCESS;
        case ds::Result::Quota:
            return OST_ERR_QUOTA;
        case ds::Result::NoSpace:
            return OST_ERR_NO_MEMORY;
        case ds::Result::InvalidArgument:
        case ds::Result::Stale:
        case ds::Result::WrongState:
            return OST_ERR_INVALID_ARGUMENT;
    }
    return OST_ERR_INTERNAL;
}

}  // namespace

extern "C" {

OST_Status_t OST_Datastore_Create(const OST_Datastore_Config_t *config, OST_Datastore_t **store)
{
    if (config == nullptr || store == nullptr)
    {
        return OST_ERR_INVALID_ARGUMENT;
    }
    *store = nullptr;
    ds::Config c;
    c.arena_bytes = config->arena_bytes;
    c.max_objects = config->max_objects;
    for (std::size_t i = 0; i < ds::kTypeCount; ++i)
    {
        c.quota_bytes[i] = config->quota_bytes[i];
    }
    try
    {
        *store = new OST_Datastore(c);
        return OST_SUCCESS;
    }
    catch (const ds::DatastoreError &)
    {
        return OST_ERR_INVALID_ARGUMENT;
    }
    catch (const std::bad_alloc &)
    {
        return OST_ERR_NO_MEMORY;
    }
    catch (...)
    {
        return OST_ERR_INTERNAL;
    }
}

void OST_Datastore_Destroy(OST_Datastore_t *store)
{
    delete store;
}

OST_Status_t OST_Datastore_Allocate(OST_Datastore_t     *store,
                                    OST_Datastore_Type_t type,
                                    uint64_t             size,
                                    OST_Datastore_Ref_t *ref,
                                    void               **data)
{
    if (store == nullptr || ref == nullptr || data == nullptr || type < OST_DATASTORE_JPEG ||
        type >= OST_DATASTORE_TYPE_COUNT)
    {
        return OST_ERR_INVALID_ARGUMENT;
    }
    ds::Ref r;
    const ds::Result result = store->store.allocate(static_cast<ds::ObjectType>(type), size, &r, data);
    if (result == ds::Result::Ok)
    {
        ref->object_id  = r.id;
        ref->generation = r.generation;
    }
    return toStatus(result);
}

OST_Status_t OST_Datastore_Seal(OST_Datastore_t *store, OST_Datastore_Ref_t ref)
{
    return store == nullptr ? OST_ERR_INVALID_ARGUMENT : toStatus(store->store.seal(toRef(ref)));
}

OST_Status_t OST_Datastore_Abort(OST_Datastore_t *store, OST_Datastore_Ref_t ref)
{
    return store == nullptr ? OST_ERR_INVALID_ARGUMENT : toStatus(store->store.abort(toRef(ref)));
}

OST_Status_t OST_Datastore_Acquire(OST_Datastore_t      *store,
                                   OST_Datastore_Ref_t   ref,
                                   const void          **data,
                                   uint64_t             *size,
                                   OST_Datastore_Type_t *type)
{
    if (store == nullptr || data == nullptr || size == nullptr)
    {
        return OST_ERR_INVALID_ARGUMENT;
    }
    ds::View         view;
    const ds::Result result = store->store.acquire(toRef(ref), &view);
    if (result == ds::Result::Ok)
    {
        *data = view.data;
        *size = view.size;
        if (type != nullptr)
        {
            *type = static_cast<OST_Datastore_Type_t>(view.type);
        }
    }
    return toStatus(result);
}

OST_Status_t OST_Datastore_Release(OST_Datastore_t *store, OST_Datastore_Ref_t ref)
{
    return store == nullptr ? OST_ERR_INVALID_ARGUMENT : toStatus(store->store.release(toRef(ref)));
}

OST_Status_t OST_Datastore_GetStats(const OST_Datastore_t *store, OST_Datastore_Stats_t *stats)
{
    if (store == nullptr || stats == nullptr)
    {
        return OST_ERR_INVALID_ARGUMENT;
    }
    const ds::Stats s = store->store.stats();
    for (std::size_t i = 0; i < ds::kTypeCount; ++i)
    {
        stats->types[i].bytes_in_use       = s.types[i].bytes_in_use;
        stats->types[i].peak_bytes         = s.types[i].peak_bytes;
        stats->types[i].objects_in_use     = s.types[i].objects_in_use;
        stats->types[i].failed_allocations = s.types[i].failed_allocations;
    }
    stats->arena_bytes        = s.arena_bytes;
    stats->free_bytes         = s.free_bytes;
    stats->largest_free_block = s.largest_free_block;
    stats->max_objects        = s.max_objects;
    stats->objects_in_use     = s.objects_in_use;
    stats->stale_references   = s.stale_references;
    return OST_SUCCESS;
}

}  // extern "C"
