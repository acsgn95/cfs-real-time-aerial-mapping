#include <gtest/gtest.h>

#include <atomic>
#include <cstring>
#include <thread>
#include <vector>

#include "orthostream/datastore/store.hpp"

namespace ds = orthostream::datastore;

namespace {

ds::Config smallConfig()
{
    ds::Config c;
    c.arena_bytes = 64 * 1024;
    c.quota_bytes = {16 * 1024, 32 * 1024, 64 * 1024, 64 * 1024};
    c.max_objects = 16;
    return c;
}

ds::Ref makeSealed(ds::Store &store, ds::ObjectType type, std::uint64_t size, std::uint8_t fill)
{
    ds::Ref ref;
    void   *data = nullptr;
    EXPECT_EQ(store.allocate(type, size, &ref, &data), ds::Result::Ok);
    std::memset(data, fill, size);
    EXPECT_EQ(store.seal(ref), ds::Result::Ok);
    return ref;
}

}  // namespace

TEST(DatastoreStore, RejectsInvalidConfiguration)
{
    ds::Config c = smallConfig();
    c.arena_bytes = 100;  // not a multiple of 64
    EXPECT_THROW(ds::Store{c}, ds::DatastoreError);
    c             = smallConfig();
    c.max_objects = 0;
    EXPECT_THROW(ds::Store{c}, ds::DatastoreError);
}

TEST(DatastoreStore, LifecycleFreesAfterLastRelease)
{
    ds::Store store(smallConfig());
    const ds::Ref ref = makeSealed(store, ds::ObjectType::Jpeg, 1000, 0xAB);

    ds::View view;
    ASSERT_EQ(store.acquire(ref, &view), ds::Result::Ok);
    EXPECT_EQ(view.size, 1000u);
    EXPECT_EQ(view.type, ds::ObjectType::Jpeg);
    EXPECT_EQ(static_cast<const std::uint8_t *>(view.data)[999], 0xAB);

    ds::Stats s = store.stats();
    EXPECT_EQ(s.objects_in_use, 1u);
    EXPECT_EQ(s.types[0].bytes_in_use, 1024u);  // rounded to 64 bytes

    EXPECT_EQ(store.release(ref), ds::Result::Ok);  // reader
    EXPECT_EQ(store.stats().objects_in_use, 1u);
    EXPECT_EQ(store.release(ref), ds::Result::Ok);  // producer
    s = store.stats();
    EXPECT_EQ(s.objects_in_use, 0u);
    EXPECT_EQ(s.free_bytes, s.arena_bytes);
    EXPECT_EQ(s.largest_free_block, s.arena_bytes);
    EXPECT_EQ(s.types[0].peak_bytes, 1024u);
}

TEST(DatastoreStore, StaleReferenceIsDetectedAfterReuse)
{
    ds::Store     store(smallConfig());
    const ds::Ref old_ref = makeSealed(store, ds::ObjectType::Jpeg, 64, 1);
    ASSERT_EQ(store.release(old_ref), ds::Result::Ok);

    const ds::Ref new_ref = makeSealed(store, ds::ObjectType::Jpeg, 64, 2);
    EXPECT_EQ(new_ref.id, old_ref.id);  // slot reused
    EXPECT_NE(new_ref.generation, old_ref.generation);

    ds::View view;
    EXPECT_EQ(store.acquire(old_ref, &view), ds::Result::Stale);
    EXPECT_EQ(store.release(old_ref), ds::Result::Stale);
    EXPECT_EQ(store.stats().stale_references, 2u);
    EXPECT_EQ(store.acquire(ds::Ref{}, &view), ds::Result::Stale);
}

TEST(DatastoreStore, StateRules)
{
    ds::Store store(smallConfig());
    ds::Ref   ref;
    void     *data = nullptr;
    ASSERT_EQ(store.allocate(ds::ObjectType::WorkImage, 128, &ref, &data), ds::Result::Ok);

    ds::View view;
    EXPECT_EQ(store.acquire(ref, &view), ds::Result::WrongState);  // not sealed
    EXPECT_EQ(store.release(ref), ds::Result::WrongState);         // unsealed: abort
    EXPECT_EQ(store.seal(ref), ds::Result::Ok);
    EXPECT_EQ(store.seal(ref), ds::Result::WrongState);
    EXPECT_EQ(store.abort(ref), ds::Result::WrongState);
    EXPECT_EQ(store.release(ref), ds::Result::Ok);

    ASSERT_EQ(store.allocate(ds::ObjectType::WorkImage, 128, &ref, &data), ds::Result::Ok);
    EXPECT_EQ(store.abort(ref), ds::Result::Ok);
    EXPECT_EQ(store.stats().objects_in_use, 0u);

    EXPECT_EQ(store.allocate(ds::ObjectType::WorkImage, 0, &ref, &data), ds::Result::InvalidArgument);
    EXPECT_EQ(store.allocate(static_cast<ds::ObjectType>(7), 64, &ref, &data), ds::Result::InvalidArgument);
}

TEST(DatastoreStore, QuotaPerType)
{
    ds::Store store(smallConfig());  // JPEG quota 16 KiB
    ds::Ref   a;
    ds::Ref   b;
    void     *data = nullptr;
    ASSERT_EQ(store.allocate(ds::ObjectType::Jpeg, 12 * 1024, &a, &data), ds::Result::Ok);
    EXPECT_EQ(store.allocate(ds::ObjectType::Jpeg, 8 * 1024, &b, &data), ds::Result::Quota);
    // Another type still has room.
    EXPECT_EQ(store.allocate(ds::ObjectType::DepthSet, 8 * 1024, &b, &data), ds::Result::Ok);
    EXPECT_EQ(store.stats().types[0].failed_allocations, 1u);
}

TEST(DatastoreStore, FragmentationAndCoalescing)
{
    ds::Config c = smallConfig();
    c.quota_bytes.fill(2 * c.arena_bytes);  // space, not the quota, is the limit
    ds::Store store(c);

    // Four blocks of 16 KiB fill the arena.
    ds::Ref refs[4];
    for (int i = 0; i < 4; ++i)
    {
        refs[i] = makeSealed(store, ds::ObjectType::DepthSet, 16 * 1024, static_cast<std::uint8_t>(i));
    }
    ds::Ref extra;
    void   *data = nullptr;
    EXPECT_EQ(store.allocate(ds::ObjectType::DepthSet, 64, &extra, &data), ds::Result::NoSpace);

    // Freeing blocks 0 and 2 leaves 32 KiB free but no 32 KiB block.
    ASSERT_EQ(store.release(refs[0]), ds::Result::Ok);
    ASSERT_EQ(store.release(refs[2]), ds::Result::Ok);
    ds::Stats s = store.stats();
    EXPECT_EQ(s.free_bytes, 32u * 1024);
    EXPECT_EQ(s.largest_free_block, 16u * 1024);
    EXPECT_EQ(store.allocate(ds::ObjectType::DepthSet, 32 * 1024, &extra, &data), ds::Result::NoSpace);

    // Freeing block 1 merges 0, 1 and 2.
    ASSERT_EQ(store.release(refs[1]), ds::Result::Ok);
    EXPECT_EQ(store.stats().largest_free_block, 48u * 1024);
    ASSERT_EQ(store.allocate(ds::ObjectType::DepthSet, 48 * 1024, &extra, &data), ds::Result::Ok);

    // Block 3 kept its content.
    ds::View view;
    ASSERT_EQ(store.acquire(refs[3], &view), ds::Result::Ok);
    EXPECT_EQ(static_cast<const std::uint8_t *>(view.data)[0], 3);
}

TEST(DatastoreStore, ObjectTableLimit)
{
    ds::Config c  = smallConfig();
    c.max_objects = 2;
    ds::Store store(c);
    ds::Ref   ref;
    void     *data = nullptr;
    ASSERT_EQ(store.allocate(ds::ObjectType::PointSet, 64, &ref, &data), ds::Result::Ok);
    ASSERT_EQ(store.allocate(ds::ObjectType::PointSet, 64, &ref, &data), ds::Result::Ok);
    EXPECT_EQ(store.allocate(ds::ObjectType::PointSet, 64, &ref, &data), ds::Result::NoSpace);
}

TEST(DatastoreStore, ConcurrentReaders)
{
    ds::Config c = smallConfig();
    c.quota_bytes.fill(c.arena_bytes);
    ds::Store     store(c);
    const ds::Ref ref = makeSealed(store, ds::ObjectType::WorkImage, 4096, 0x5A);

    std::atomic<int>         errors{0};
    std::vector<std::thread> readers;
    for (int t = 0; t < 4; ++t)
    {
        readers.emplace_back([&] {
            for (int i = 0; i < 10000; ++i)
            {
                ds::View view;
                if (store.acquire(ref, &view) != ds::Result::Ok ||
                    static_cast<const std::uint8_t *>(view.data)[4095] != 0x5A ||
                    store.release(ref) != ds::Result::Ok)
                {
                    ++errors;
                }
            }
        });
    }
    // A producer allocates and frees other objects meanwhile.
    for (int i = 0; i < 10000; ++i)
    {
        ds::Ref r;
        void   *data = nullptr;
        if (store.allocate(ds::ObjectType::PointSet, 256, &r, &data) == ds::Result::Ok)
        {
            store.abort(r);
        }
    }
    for (auto &t : readers)
    {
        t.join();
    }
    EXPECT_EQ(errors.load(), 0);
    const ds::Stats s = store.stats();
    EXPECT_EQ(s.objects_in_use, 1u);
    EXPECT_EQ(s.free_bytes, s.arena_bytes - 4096);
}
