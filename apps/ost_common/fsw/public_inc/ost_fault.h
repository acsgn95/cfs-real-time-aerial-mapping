/*
 * Fault catalog and fault state handling (ADR-020 section 4).
 *
 * The identifiers, severities and states are EDS enumerations of package OST
 * (eds/ost.xml). This header adds the catalog properties (severity and
 * clear policy of each identifier) and a fault set that an application keeps
 * for the faults it can raise.
 *
 * State changes:
 *
 *   CLEARED --Raise--> ACTIVE --Resolve--> CLEARED   (policy AUTO_CLEAR)
 *                             --Resolve--> LATCHED   (policy LATCH)
 *   LATCHED --Raise--> ACTIVE
 *   LATCHED --ClearLatched--> CLEARED
 *
 * Report() records a momentary occurrence, for faults detected per item
 * (an image without telemetry, a skipped keyframe, a failed tile write):
 *
 *   CLEARED --Report--> LATCHED
 *
 * Further occurrences while the fault is ACTIVE or LATCHED only increase its
 * occurrence count. Momentary faults therefore have the policy LATCH.
 *
 * Every change of state is published once as a FaultEvent message and raised
 * once as an EVS event. Repeated occurrences go to the occurrence count, not
 * to new messages (ADR-020 section 3: events on changes of state).
 *
 * A fault set is used by one task only. Applications whose child tasks detect
 * faults pass the condition to the main task, which owns the set.
 */
#ifndef OST_FAULT_H
#define OST_FAULT_H

#include "common_types.h"
#include "cfe_error.h"
#include "ost_eds_typedefs.h"

/* Maximum number of faults one application can raise. */
#define OST_FAULT_MAX_PER_APP 20

/* EVS event identifiers of fault changes: first + index of the fault in the
 * set, within the range 60-79 of ADR-020 section 3. */
#define OST_FAULT_FIRST_EID 60

typedef enum
{
    OST_FAULT_POLICY_AUTO_CLEAR = 0, /* cleared when the condition is gone */
    OST_FAULT_POLICY_LATCH      = 1  /* kept until a clear command */
} OST_FaultPolicy_t;

typedef struct
{
    OST_FaultId_Enum_t       Id;
    OST_FaultSeverity_Enum_t Severity;
    OST_FaultPolicy_t        Policy;
    const char              *Name; /* label of the EDS enumeration */
} OST_FaultCatalogEntry_t;

/* Catalog entry of an identifier, or NULL if the identifier is unknown. */
const OST_FaultCatalogEntry_t *OST_Fault_Lookup(OST_FaultId_Enum_t Id);

/* All catalog entries; for tests and documentation tools. */
const OST_FaultCatalogEntry_t *OST_Fault_Catalog(uint32 *Count);

typedef struct
{
    const OST_FaultCatalogEntry_t *Entry;
    OST_FaultState_Enum_t          State;
    uint32                         OccurrenceCount;
} OST_FaultSlot_t;

typedef struct
{
    OST_AppId_Enum_t Source;
    uint32           SlotCount;
    OST_FaultSlot_t  Slots[OST_FAULT_MAX_PER_APP];
    uint32           Sequence;
    OST_FaultEventTlm_t Msg;
} OST_FaultSet_t;

/*
 * Prepares a fault set for the faults Ids[0..Count-1]. The index of an
 * identifier in Ids gives its event identifier (OST_FAULT_FIRST_EID + index),
 * so the list of an application must only grow at its end.
 * Returns CFE_SUCCESS, or CFE_STATUS_RANGE_ERROR for an unknown or repeated
 * identifier or more than OST_FAULT_MAX_PER_APP entries.
 */
CFE_Status_t OST_FaultSet_Init(OST_FaultSet_t *Set, OST_AppId_Enum_t Source, const OST_FaultId_Enum_t *Ids,
                               uint32 Count);

/* The condition of Id is present. Text describes it (may be NULL). */
void OST_FaultSet_Raise(OST_FaultSet_t *Set, OST_FaultId_Enum_t Id, const char *Text);

/* The condition of Id is gone. */
void OST_FaultSet_Resolve(OST_FaultSet_t *Set, OST_FaultId_Enum_t Id);

/* A momentary occurrence of Id (see above). */
void OST_FaultSet_Report(OST_FaultSet_t *Set, OST_FaultId_Enum_t Id, const char *Text);

/* Clears all LATCHED faults (command "clear latched faults"). Returns the
 * number of cleared faults. ACTIVE faults stay active. */
uint32 OST_FaultSet_ClearLatched(OST_FaultSet_t *Set);

/* Current state of Id; CLEARED for an identifier that is not in the set. */
OST_FaultState_Enum_t OST_FaultSet_State(const OST_FaultSet_t *Set, OST_FaultId_Enum_t Id);

/* Fills the fault fields of the common housekeeping part. */
void OST_FaultSet_Summarise(const OST_FaultSet_t *Set, OST_HkCommon_t *Hk);

#endif /* OST_FAULT_H */
