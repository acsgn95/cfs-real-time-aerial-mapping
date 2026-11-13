#include "orthostream/datastore/store.hpp"

#include <algorithm>
#include <new>

namespace orthostream::datastore {

namespace {

enum class SlotState : std::uint8_t
{
    Free,
    Writing,
    Sealed,
};

std::uint64_t roundUp(std::uint64_t size)
{
    return (size + kAlignment - 1) / kAlignment * kAlignment;
}

std::byte *allocateArena(std::uint64_t bytes)
{
    return static_cast<std::byte *>(::operator new(static_cast<std::size_t>(bytes), std::align_val_t(kAlignment)));
}

void freeArena(std::byte *arena)
{
    ::operator delete(arena, std::align_val_t(kAlignment));
}

}  // namespace

struct Store::Slot
{
    std::uint64_t offset     = 0;
    std::uint64_t size       = 0;  // requested size
    std::uint64_t reserved   = 0;  // rounded size taken from the arena
    std::uint32_t generation = 1;
    std::uint32_t refcount   = 0;
    ObjectType    type       = ObjectType::Jpeg;
    SlotState     state      = SlotState::Free;
};

Store::Store(const Config &config)
    : config_(config), arena_(nullptr, freeArena)
{
    if (config.arena_bytes == 0 || config.arena_bytes % kAlignment != 0)
    {
        throw DatastoreError("arena size must be a positive multiple of 64 bytes");
    }
    if (config.max_objects == 0)
    {
        throw DatastoreError("max_objects must be positive");
    }
    arena_.reset(allocateArena(config.arena_bytes));

    slots_.resize(config.max_objects);
    free_ids_.reserve(config.max_objects);
    for (std::uint32_t i = config.max_objects; i > 0; --i)
    {
        free_ids_.push_back(i - 1);
    }
    // Every allocated object can split one free extent into two.
    free_list_.reserve(static_cast<std::size_t>(config.max_objects) + 1);
    free_list_.push_back({0, config.arena_bytes});

    stats_.arena_bytes        = config.arena_bytes;
    stats_.free_bytes         = config.arena_bytes;
    stats_.largest_free_block = config.arena_bytes;
    stats_.max_objects        = config.max_objects;
}

Store::~Store() = default;

Store::Slot *Store::find(Ref ref)
{
    if (ref.id == 0 || ref.id > slots_.size())
    {
        return nullptr;
    }
    Slot &slot = slots_[ref.id - 1];
    if (slot.state == SlotState::Free || slot.generation != ref.generation)
    {
        return nullptr;
    }
    return &slot;
}

bool Store::takeExtent(std::uint64_t size, std::uint64_t *offset)
{
    // First fit keeps long-lived objects at low addresses.
    for (std::size_t i = 0; i < free_list_.size(); ++i)
    {
        Extent &e = free_list_[i];
        if (e.size < size)
        {
            continue;
        }
        *offset = e.offset;
        if (e.size == size)
        {
            free_list_.erase(free_list_.begin() + static_cast<std::ptrdiff_t>(i));
        }
        else
        {
            e.offset += size;
            e.size -= size;
        }
        return true;
    }
    return false;
}

void Store::returnExtent(std::uint64_t offset, std::uint64_t size)
{
    auto it = std::lower_bound(free_list_.begin(), free_list_.end(), offset,
                               [](const Extent &e, std::uint64_t o) { return e.offset < o; });
    const bool merge_prev = it != free_list_.begin() && (it - 1)->offset + (it - 1)->size == offset;
    const bool merge_next = it != free_list_.end() && offset + size == it->offset;

    if (merge_prev && merge_next)
    {
        (it - 1)->size += size + it->size;
        free_list_.erase(it);
    }
    else if (merge_prev)
    {
        (it - 1)->size += size;
    }
    else if (merge_next)
    {
        it->offset = offset;
        it->size += size;
    }
    else
    {
        // Capacity is reserved for the worst case; no allocation happens.
        free_list_.insert(it, {offset, size});
    }
}

void Store::freeSlot(Slot &slot)
{
    TypeStats &ts = stats_.types[static_cast<std::size_t>(slot.type)];
    ts.bytes_in_use -= slot.reserved;
    ts.objects_in_use -= 1;
    stats_.objects_in_use -= 1;
    stats_.free_bytes += slot.reserved;

    returnExtent(slot.offset, slot.reserved);

    slot.state    = SlotState::Free;
    slot.refcount = 0;
    slot.generation += 1;
    if (slot.generation == 0)
    {
        slot.generation = 1;
    }
    free_ids_.push_back(static_cast<std::uint32_t>(&slot - slots_.data()));
}

Result Store::allocate(ObjectType type, std::uint64_t size, Ref *ref, void **data)
{
    const auto type_index = static_cast<std::size_t>(type);
    if (type_index >= kTypeCount || size == 0 || ref == nullptr || data == nullptr)
    {
        return Result::InvalidArgument;
    }
    const std::uint64_t reserved = roundUp(size);

    std::lock_guard<std::mutex> lock(mutex_);
    TypeStats &ts = stats_.types[type_index];

    if (ts.bytes_in_use + reserved > config_.quota_bytes[type_index])
    {
        ts.failed_allocations += 1;
        return Result::Quota;
    }
    std::uint64_t offset = 0;
    if (free_ids_.empty() || !takeExtent(reserved, &offset))
    {
        ts.failed_allocations += 1;
        return Result::NoSpace;
    }
    const std::uint32_t index = free_ids_.back();
    free_ids_.pop_back();

    Slot &slot    = slots_[index];
    slot.offset   = offset;
    slot.size     = size;
    slot.reserved = reserved;
    slot.refcount = 1;
    slot.type     = type;
    slot.state    = SlotState::Writing;

    ts.bytes_in_use += reserved;
    ts.peak_bytes = std::max(ts.peak_bytes, ts.bytes_in_use);
    ts.objects_in_use += 1;
    stats_.objects_in_use += 1;
    stats_.free_bytes -= reserved;

    ref->id    = index + 1;
    ref->generation = slot.generation;
    *data      = arena_.get() + offset;
    return Result::Ok;
}

Result Store::seal(Ref ref)
{
    std::lock_guard<std::mutex> lock(mutex_);
    Slot *slot = find(ref);
    if (slot == nullptr)
    {
        stats_.stale_references += 1;
        return Result::Stale;
    }
    if (slot->state != SlotState::Writing)
    {
        return Result::WrongState;
    }
    slot->state = SlotState::Sealed;
    return Result::Ok;
}

Result Store::abort(Ref ref)
{
    std::lock_guard<std::mutex> lock(mutex_);
    Slot *slot = find(ref);
    if (slot == nullptr)
    {
        stats_.stale_references += 1;
        return Result::Stale;
    }
    if (slot->state != SlotState::Writing)
    {
        return Result::WrongState;
    }
    freeSlot(*slot);
    return Result::Ok;
}

Result Store::acquire(Ref ref, View *view)
{
    if (view == nullptr)
    {
        return Result::InvalidArgument;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    Slot *slot = find(ref);
    if (slot == nullptr)
    {
        stats_.stale_references += 1;
        return Result::Stale;
    }
    if (slot->state != SlotState::Sealed)
    {
        return Result::WrongState;
    }
    slot->refcount += 1;
    view->data = arena_.get() + slot->offset;
    view->size = slot->size;
    view->type = slot->type;
    return Result::Ok;
}

Result Store::release(Ref ref)
{
    std::lock_guard<std::mutex> lock(mutex_);
    Slot *slot = find(ref);
    if (slot == nullptr)
    {
        stats_.stale_references += 1;
        return Result::Stale;
    }
    if (slot->state != SlotState::Sealed)
    {
        // An unsealed object is freed with abort().
        return Result::WrongState;
    }
    slot->refcount -= 1;
    if (slot->refcount == 0)
    {
        freeSlot(*slot);
    }
    return Result::Ok;
}

Stats Store::stats() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    Stats s              = stats_;
    s.largest_free_block = 0;
    for (const Extent &e : free_list_)
    {
        s.largest_free_block = std::max(s.largest_free_block, e.size);
    }
    return s;
}

const char *resultName(Result result)
{
    switch (result)
    {
        case Result::Ok:
            return "ok";
        case Result::InvalidArgument:
            return "invalid argument";
        case Result::Quota:
            return "quota exceeded";
        case Result::NoSpace:
            return "no space";
        case Result::Stale:
            return "stale reference";
        case Result::WrongState:
            return "wrong object state";
    }
    return "unknown";
}

}  // namespace orthostream::datastore
