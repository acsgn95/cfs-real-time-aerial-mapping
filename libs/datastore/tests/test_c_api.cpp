#include <gtest/gtest.h>

#include "datastore_c_api.h"

extern "C" int ost_datastore_c_api_usage(void);

TEST(DatastoreCApi, UsageFromC)
{
    EXPECT_EQ(ost_datastore_c_api_usage(), 0);
}

TEST(DatastoreCApi, InvalidArguments)
{
    OST_Datastore_t       *store = nullptr;
    OST_Datastore_Config_t config{};
    EXPECT_EQ(OST_Datastore_Create(nullptr, &store), OST_ERR_INVALID_ARGUMENT);
    EXPECT_EQ(OST_Datastore_Create(&config, &store), OST_ERR_INVALID_ARGUMENT);  // size 0
    EXPECT_EQ(store, nullptr);

    config.arena_bytes = 4096;
    config.max_objects = 4;
    ASSERT_EQ(OST_Datastore_Create(&config, &store), OST_SUCCESS);
    OST_Datastore_Ref_t ref{};
    void               *data = nullptr;
    EXPECT_EQ(OST_Datastore_Allocate(store, OST_DATASTORE_TYPE_COUNT, 64, &ref, &data), OST_ERR_INVALID_ARGUMENT);
    EXPECT_EQ(OST_Datastore_Seal(nullptr, ref), OST_ERR_INVALID_ARGUMENT);
    EXPECT_EQ(OST_Datastore_GetStats(store, nullptr), OST_ERR_INVALID_ARGUMENT);
    OST_Datastore_Destroy(store);
    OST_Datastore_Destroy(nullptr);
}
