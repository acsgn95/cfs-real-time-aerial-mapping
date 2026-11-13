/*
 * Fault catalog (ADR-020 section 4).
 *
 * One entry per identifier of the EDS enumeration OST/FaultId. Severity and
 * policy are fixed here; the raising application is the source of the fault
 * set that reports it. docs/interfaces/faults.md lists the catalog; the test
 * tests/interfaces/check_interfaces.py compares the two.
 */
#include "ost_fault.h"

#include <stddef.h>

static const OST_FaultCatalogEntry_t OST_FAULT_CATALOG[] = {
    {OST_FaultId_TELEMETRY_STALE, OST_FaultSeverity_ERROR, OST_FAULT_POLICY_AUTO_CLEAR, "TELEMETRY_STALE"},
    {OST_FaultId_IMAGE_STREAM_STALE, OST_FaultSeverity_ERROR, OST_FAULT_POLICY_AUTO_CLEAR, "IMAGE_STREAM_STALE"},
    {OST_FaultId_LINK_FORMAT, OST_FaultSeverity_CRITICAL, OST_FAULT_POLICY_LATCH, "LINK_FORMAT"},
    {OST_FaultId_POSE_NO_TELEMETRY, OST_FaultSeverity_WARNING, OST_FAULT_POLICY_LATCH, "POSE_NO_TELEMETRY"},
    {OST_FaultId_POSE_ACCURACY_LOW, OST_FaultSeverity_DEGRADED, OST_FAULT_POLICY_AUTO_CLEAR, "POSE_ACCURACY_LOW"},
    {OST_FaultId_DATASTORE_QUOTA, OST_FaultSeverity_ERROR, OST_FAULT_POLICY_LATCH, "DATASTORE_QUOTA"},
    {OST_FaultId_KEYFRAMES_SKIPPED, OST_FaultSeverity_DEGRADED, OST_FAULT_POLICY_LATCH, "KEYFRAMES_SKIPPED"},
    {OST_FaultId_GPU_FAILURE, OST_FaultSeverity_CRITICAL, OST_FAULT_POLICY_LATCH, "GPU_FAILURE"},
    {OST_FaultId_REFINE_REJECTED, OST_FaultSeverity_WARNING, OST_FAULT_POLICY_LATCH, "REFINE_REJECTED"},
    {OST_FaultId_REFINE_LATENCY, OST_FaultSeverity_DEGRADED, OST_FAULT_POLICY_LATCH, "REFINE_LATENCY"},
    {OST_FaultId_TILE_WRITE, OST_FaultSeverity_ERROR, OST_FAULT_POLICY_LATCH, "TILE_WRITE"},
    {OST_FaultId_DISK_SPACE_LOW, OST_FaultSeverity_WARNING, OST_FAULT_POLICY_AUTO_CLEAR, "DISK_SPACE_LOW"},
    {OST_FaultId_TABLE_REJECTED, OST_FaultSeverity_ERROR, OST_FAULT_POLICY_LATCH, "TABLE_REJECTED"},
    {OST_FaultId_LIBRARY_ERROR, OST_FaultSeverity_ERROR, OST_FAULT_POLICY_LATCH, "LIBRARY_ERROR"},
};

#define OST_FAULT_CATALOG_SIZE (sizeof(OST_FAULT_CATALOG) / sizeof(OST_FAULT_CATALOG[0]))

const OST_FaultCatalogEntry_t *OST_Fault_Lookup(OST_FaultId_Enum_t Id)
{
    size_t i;

    for (i = 0; i < OST_FAULT_CATALOG_SIZE; ++i)
    {
        if (OST_FAULT_CATALOG[i].Id == Id)
        {
            return &OST_FAULT_CATALOG[i];
        }
    }
    return NULL;
}

const OST_FaultCatalogEntry_t *OST_Fault_Catalog(uint32 *Count)
{
    if (Count != NULL)
    {
        *Count = (uint32)OST_FAULT_CATALOG_SIZE;
    }
    return OST_FAULT_CATALOG;
}
