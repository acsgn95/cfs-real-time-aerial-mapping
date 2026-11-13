/*
 * Default platform configuration of the OrthoStream data store (ADR-013
 * section 4, ADR-016 section 6).
 *
 * A mission overrides these values by placing ost_datastore_platform_cfg.h
 * in its *_defs directory. Sizes are bytes; the arena size is a multiple of
 * 64. Memory is reserved when the library starts and never grows.
 */
#ifndef DEFAULT_OST_DATASTORE_PLATFORM_CFG_H
#define DEFAULT_OST_DATASTORE_PLATFORM_CFG_H

#define OST_DATASTORE_GIB(n) ((uint64)(n) * 1024 * 1024 * 1024)

/* Arena: 8 GB within the 16 GB RAM budget of ADR-002. */
#define OST_DATASTORE_ARENA_BYTES OST_DATASTORE_GIB(8)

/* Quotas per object type. */
#define OST_DATASTORE_QUOTA_JPEG       OST_DATASTORE_GIB(1)
#define OST_DATASTORE_QUOTA_WORK_IMAGE OST_DATASTORE_GIB(2)
#define OST_DATASTORE_QUOTA_DEPTH_SET  OST_DATASTORE_GIB(4)
#define OST_DATASTORE_QUOTA_POINT_SET  OST_DATASTORE_GIB(1)

/* Size of the object table: objects that can exist at the same time. */
#define OST_DATASTORE_MAX_OBJECTS 4096

#endif /* DEFAULT_OST_DATASTORE_PLATFORM_CFG_H */
