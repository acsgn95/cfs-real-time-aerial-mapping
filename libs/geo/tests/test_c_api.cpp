#include <gtest/gtest.h>

#include "geo_c_api.h"

extern "C" int ost_geo_c_api_usage(const char *proj_data_dir);

namespace
{

TEST(GeoCApi, UsageFromC)
{
    EXPECT_EQ(ost_geo_c_api_usage(OST_GEO_TEST_PROJ_DATA_DIR), 0);
}

TEST(GeoCApi, CreateWithBadDataDirectoryFailsCleanly)
{
    OST_Geo_Context_t *ctx = nullptr;
    EXPECT_EQ(OST_Geo_Create(&ctx, "/nonexistent"), OST_GEO_ERR_PROJ);
    EXPECT_EQ(ctx, nullptr);
    EXPECT_EQ(OST_Geo_Create(nullptr, "/tmp"), OST_GEO_ERR_INVALID_ARGUMENT);
}

}  // namespace
