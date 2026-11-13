/*
 * Fault sets: state changes, FaultEvent messages and EVS events
 * (ADR-020 section 4). See ost_fault.h for the state diagram.
 */
#include "ost_common.h"

#include <string.h>

static OST_FaultSlot_t *OST_FaultSet_Find(OST_FaultSet_t *Set, OST_FaultId_Enum_t Id, uint16 *EventId)
{
    uint32 i;

    for (i = 0; i < Set->SlotCount; ++i)
    {
        if (Set->Slots[i].Entry->Id == Id)
        {
            if (EventId != NULL)
            {
                *EventId = (uint16)(OST_FAULT_FIRST_EID + i);
            }
            return &Set->Slots[i];
        }
    }
    return NULL;
}

static const char *OST_FaultStateName(OST_FaultState_Enum_t State)
{
    switch (State)
    {
        case OST_FaultState_ACTIVE:
            return "ACTIVE";
        case OST_FaultState_LATCHED:
            return "LATCHED";
        default:
            return "CLEARED";
    }
}

/* EVS type of a state change: the severity of the fault while it is present,
 * INFORMATION when it is cleared. */
static uint16 OST_FaultEventType(const OST_FaultSlot_t *Slot)
{
    if (Slot->State == OST_FaultState_CLEARED)
    {
        return CFE_EVS_EventType_INFORMATION;
    }
    switch (Slot->Entry->Severity)
    {
        case OST_FaultSeverity_CRITICAL:
            return CFE_EVS_EventType_CRITICAL;
        case OST_FaultSeverity_ERROR:
            return CFE_EVS_EventType_ERROR;
        default:
            return CFE_EVS_EventType_INFORMATION;
    }
}

static void OST_FaultSet_Publish(OST_FaultSet_t *Set, const OST_FaultSlot_t *Slot, uint16 EventId,
                                 const char *Text)
{
    OST_FaultEvent_Payload_t *P = &Set->Msg.Payload;

    P->Header.SchemaVersion = OST_FAULT_EVENT_SCHEMA_VERSION;
    P->Header.Flags         = 0;
    P->Header.Sequence      = ++Set->Sequence;
    P->Time                 = OST_TimeUtcNs();
    P->OccurrenceCount      = Slot->OccurrenceCount;
    P->FaultId              = Slot->Entry->Id;
    P->Severity             = Slot->Entry->Severity;
    P->State                = Slot->State;
    P->Source               = Set->Source;
    OST_CopyText(P->Text, sizeof(P->Text), Text);

    CFE_SB_TimeStampMsg(CFE_MSG_PTR(Set->Msg.TelemetryHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(Set->Msg.TelemetryHeader), true);

    CFE_EVS_SendEvent(EventId, OST_FaultEventType(Slot), "Fault %s %s (%lu): %s", Slot->Entry->Name,
                      OST_FaultStateName(Slot->State), (unsigned long)Slot->OccurrenceCount,
                      (Text != NULL) ? Text : "");
}

CFE_Status_t OST_FaultSet_Init(OST_FaultSet_t *Set, OST_AppId_Enum_t Source, const OST_FaultId_Enum_t *Ids,
                               uint32 Count)
{
    uint32 i;

    memset(Set, 0, sizeof(*Set));
    Set->Source = Source;

    if (Count > OST_FAULT_MAX_PER_APP || (Count > 0 && Ids == NULL))
    {
        return CFE_STATUS_RANGE_ERROR;
    }
    for (i = 0; i < Count; ++i)
    {
        const OST_FaultCatalogEntry_t *Entry = OST_Fault_Lookup(Ids[i]);

        if (Entry == NULL || OST_FaultSet_Find(Set, Ids[i], NULL) != NULL)
        {
            Set->SlotCount = 0;
            return CFE_STATUS_RANGE_ERROR;
        }
        Set->Slots[i].Entry = Entry;
        Set->Slots[i].State = OST_FaultState_CLEARED;
        Set->SlotCount      = i + 1;
    }

    CFE_MSG_Init(CFE_MSG_PTR(Set->Msg.TelemetryHeader), OST_FaultEventMsgId(), sizeof(Set->Msg));
    return CFE_SUCCESS;
}

void OST_FaultSet_Raise(OST_FaultSet_t *Set, OST_FaultId_Enum_t Id, const char *Text)
{
    uint16           EventId;
    OST_FaultSlot_t *Slot = OST_FaultSet_Find(Set, Id, &EventId);

    if (Slot == NULL || Slot->State == OST_FaultState_ACTIVE)
    {
        return;
    }
    Slot->State = OST_FaultState_ACTIVE;
    ++Slot->OccurrenceCount;
    OST_FaultSet_Publish(Set, Slot, EventId, Text);
}

void OST_FaultSet_Resolve(OST_FaultSet_t *Set, OST_FaultId_Enum_t Id)
{
    uint16           EventId;
    OST_FaultSlot_t *Slot = OST_FaultSet_Find(Set, Id, &EventId);

    if (Slot == NULL || Slot->State != OST_FaultState_ACTIVE)
    {
        return;
    }
    Slot->State = (Slot->Entry->Policy == OST_FAULT_POLICY_LATCH) ? OST_FaultState_LATCHED : OST_FaultState_CLEARED;
    OST_FaultSet_Publish(Set, Slot, EventId, "condition gone");
}

void OST_FaultSet_Report(OST_FaultSet_t *Set, OST_FaultId_Enum_t Id, const char *Text)
{
    uint16           EventId;
    OST_FaultSlot_t *Slot = OST_FaultSet_Find(Set, Id, &EventId);

    if (Slot == NULL)
    {
        return;
    }
    ++Slot->OccurrenceCount;
    if (Slot->State == OST_FaultState_CLEARED)
    {
        Slot->State = OST_FaultState_LATCHED;
        OST_FaultSet_Publish(Set, Slot, EventId, Text);
    }
}

uint32 OST_FaultSet_ClearLatched(OST_FaultSet_t *Set)
{
    uint32 i;
    uint32 Cleared = 0;

    for (i = 0; i < Set->SlotCount; ++i)
    {
        OST_FaultSlot_t *Slot = &Set->Slots[i];

        if (Slot->State == OST_FaultState_LATCHED)
        {
            Slot->State = OST_FaultState_CLEARED;
            OST_FaultSet_Publish(Set, Slot, (uint16)(OST_FAULT_FIRST_EID + i), "cleared by command");
            ++Cleared;
        }
    }
    return Cleared;
}

OST_FaultState_Enum_t OST_FaultSet_State(const OST_FaultSet_t *Set, OST_FaultId_Enum_t Id)
{
    uint32 i;

    for (i = 0; i < Set->SlotCount; ++i)
    {
        if (Set->Slots[i].Entry->Id == Id)
        {
            return Set->Slots[i].State;
        }
    }
    return OST_FaultState_CLEARED;
}

void OST_FaultSet_Summarise(const OST_FaultSet_t *Set, OST_HkCommon_t *Hk)
{
    uint32 i;

    Hk->FaultsActive    = 0;
    Hk->FaultsLatched   = 0;
    Hk->HighestSeverity = OST_FaultSeverity_NONE;

    for (i = 0; i < Set->SlotCount; ++i)
    {
        const OST_FaultSlot_t *Slot = &Set->Slots[i];

        if (Slot->State == OST_FaultState_CLEARED)
        {
            continue;
        }
        if (Slot->State == OST_FaultState_ACTIVE)
        {
            ++Hk->FaultsActive;
        }
        else
        {
            ++Hk->FaultsLatched;
        }
        if (Slot->Entry->Severity > Hk->HighestSeverity)
        {
            Hk->HighestSeverity = Slot->Entry->Severity;
        }
    }
}
