// Shared data store for bulk data (ADR-013).
//
// One preallocated arena holds write-once, reference-counted objects with
// quotas per type. A producer allocates an object, writes it and seals it;
// readers acquire and release it. A reference {id, generation} detects use
// after free. Allocation, acquisition and release take a mutex; reading a
// sealed object needs no lock. No memory is allocated after construction.
#ifndef ORTHOSTREAM_DATASTORE_STORE_HPP
#define ORTHOSTREAM_DATASTORE_STORE_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <vector>

namespace orthostream::datastore {

enum class ObjectType : std::uint32_t
{
    Jpeg      = 0,  // original JPEG data
    WorkImage = 1,  // working-resolution image
    DepthSet  = 2,  // depth, normal and confidence maps
    PointSet  = 3,  // georeferenced points
};

inline constexpr std::size_t kTypeCount = 4;

// Object sizes are rounded up to this alignment.
inline constexpr std::uint64_t kAlignment = 64;

struct Config
{
    std::uint64_t                            arena_bytes = 0;
    std::array<std::uint64_t, kTypeCount>    quota_bytes{};  // per type; their sum may exceed the arena
    std::uint32_t                            max_objects = 0;
};

struct Ref
{
    std::uint32_t id         = 0;  // 0: no object
    std::uint32_t generation = 0;
};

enum class Result
{
    Ok,
    InvalidArgument,  // bad type, size or reference form
    Quota,            // the quota of the type would be exceeded
    NoSpace,          // no free block large enough, or the object table is full
    Stale,            // the reference names a freed or reused object
    WrongState,       // for example acquire before seal, or seal twice
};

struct View
{
    const void   *data = nullptr;
    std::uint64_t size = 0;
    ObjectType    type = ObjectType::Jpeg;
};

struct TypeStats
{
    std::uint64_t bytes_in_use       = 0;  // rounded to kAlignment
    std::uint64_t peak_bytes         = 0;
    std::uint32_t objects_in_use     = 0;
    std::uint32_t failed_allocations = 0;  // quota or space
};

struct Stats
{
    std::array<TypeStats, kTypeCount> types{};
    std::uint64_t                     arena_bytes        = 0;
    std::uint64_t                     free_bytes         = 0;
    std::uint64_t                     largest_free_block = 0;
    std::uint32_t                     max_objects        = 0;
    std::uint32_t                     objects_in_use     = 0;
    std::uint32_t                     stale_references   = 0;
};

class DatastoreError : public std::runtime_error
{
public:
    using std::runtime_error::runtime_error;
};

class Store
{
public:
    // Allocates the arena and the object table. Throws DatastoreError for an
    // invalid configuration and std::bad_alloc if the arena cannot be
    // allocated.
    explicit Store(const Config &config);
    ~Store();

    Store(const Store &)            = delete;
    Store &operator=(const Store &) = delete;

    // Allocates a writable object. The caller holds one reference.
    Result allocate(ObjectType type, std::uint64_t size, Ref *ref, void **data);

    // Makes an allocated object readable and immutable.
    Result seal(Ref ref);

    // Frees an allocated object that was not sealed (producer error path).
    Result abort(Ref ref);

    // Adds a reference to a sealed object and returns its data.
    Result acquire(Ref ref, View *view);

    // Removes a reference; the object is freed when none is left.
    Result release(Ref ref);

    Stats stats() const;

private:
    struct Slot;
    struct Extent
    {
        std::uint64_t offset;
        std::uint64_t size;
    };

    Slot *find(Ref ref);
    bool  takeExtent(std::uint64_t size, std::uint64_t *offset);
    void  returnExtent(std::uint64_t offset, std::uint64_t size);
    void  freeSlot(Slot &slot);

    Config                    config_;
    std::unique_ptr<std::byte, void (*)(std::byte *)> arena_;
    std::vector<Slot>         slots_;       // index id - 1
    std::vector<std::uint32_t> free_ids_;   // stack of free slot indices
    std::vector<Extent>       free_list_;   // sorted by offset, coalesced
    Stats                     stats_;
    mutable std::mutex        mutex_;
};

const char *resultName(Result result);

}  // namespace orthostream::datastore

#endif  // ORTHOSTREAM_DATASTORE_STORE_HPP
