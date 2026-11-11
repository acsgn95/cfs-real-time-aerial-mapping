#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <string>

#include <gtest/gtest.h>

#include "dataset.hpp"
#include "error_stats.hpp"
#include "raster.hpp"

namespace orthostream::bench
{
namespace
{

TEST(ErrorStats, KnownValues)
{
    const ErrorStats s = errorStats({ -1.0, 0.0, 0.2, 0.4, 3.0 });
    EXPECT_EQ(s.count, 5u);
    EXPECT_NEAR(s.mean, 0.52, 1e-12);
    EXPECT_NEAR(s.median, 0.2, 1e-12);
    EXPECT_NEAR(s.mae, 0.92, 1e-12);
    EXPECT_NEAR(s.rmse, std::sqrt((1.0 + 0.04 + 0.16 + 9.0) / 5.0), 1e-12);
    // |e - median| = 1.2, 0.2, 0, 0.2, 2.8: median 0.2.
    EXPECT_NEAR(s.nmad, 1.4826 * 0.2, 1e-12);
    EXPECT_NEAR(s.within_05, 0.6, 1e-12);
    EXPECT_NEAR(s.within_1, 0.8, 1e-12);
    // Sorted |e|: 0, 0.2, 0.4, 1, 3; position 3.8: 1 + 0.8 * 2.
    EXPECT_NEAR(s.p95_abs, 2.6, 1e-12);
    EXPECT_EQ(errorStats({}).count, 0u);
}

TEST(RasterWindow, BilinearBetweenPixelCentres)
{
    RasterWindow w;
    w.origin_x = 100.0;
    w.origin_y = 200.0;
    w.pixel_x  = 1.0;
    w.pixel_y  = -1.0;
    w.width    = 3;
    w.height   = 2;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    w.values        = { 0.0f, 1.0f, nan, 2.0f, 3.0f, 4.0f };
    EXPECT_NEAR(*w.sample(100.5, 199.5), 0.0, 1e-12);  // centre of the first pixel
    EXPECT_NEAR(*w.sample(101.0, 199.0), 1.5, 1e-12);  // between four centres
    EXPECT_NEAR(*w.sample(101.5, 198.5), 3.0, 1e-12);  // last row
    EXPECT_FALSE(w.sample(102.0, 199.0));               // touches the missing value
    EXPECT_FALSE(w.sample(100.2, 199.5));               // outside the centres
}

TEST(Dataset, ParsesPosesSortedByName)
{
    const std::string path = ::testing::TempDir() + "poses.txt";
    {
        std::ofstream out(path);
        out << "# comment\n"
               "camera 100 50 80 80 49.5 24.5 0 0 0 0 0\n"
               "pose b.jpg 1 2 3 1 0 0 0\n"
               "pose a.jpg 4 5 6 0 1 0 0\n";
    }
    const Dataset d = loadPoses(path);
    EXPECT_EQ(d.camera.width, 100);
    EXPECT_DOUBLE_EQ(d.camera.cx, 49.5);
    ASSERT_EQ(d.images.size(), 2u);
    EXPECT_EQ(d.images[0].name, "a.jpg");
    EXPECT_TRUE(d.images[0].T_E_C.translation().isApprox(Eigen::Vector3d(4, 5, 6)));
    EXPECT_NEAR(d.images[0].T_E_C.linear()(1, 1), -1.0, 1e-12);  // 180° about x
    std::remove(path.c_str());
}

}  // namespace
}  // namespace orthostream::bench
