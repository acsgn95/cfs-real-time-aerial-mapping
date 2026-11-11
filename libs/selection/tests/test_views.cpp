#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "orthostream/selection/views.hpp"
#include "test_support.hpp"

namespace orthostream::selection
{
namespace
{

using test::camera;
using test::flat;
using test::nadir;

constexpr double kPi = 3.14159265358979323846;

View view(std::uint64_t id, const Eigen::Vector3d &centre, int line = 0, double tilt_rad = 0.0)
{
    View v;
    v.id          = id;
    v.T_L_C       = nadir(centre, 0.0, tilt_rad);
    v.footprint   = *computeFootprint(camera(), v.T_L_C, flat(0.0));
    v.flight_line = line;
    return v;
}

TEST(KeyframeSelector, KeepsTheTargetOverlap)
{
    // Steps of 16 m along a 200 m wide footprint: 8 % per image, so the
    // overlap with the last keyframe drops below 70 % after four images.
    KeyframeSelector selector(KeyframeConfig {});
    std::vector<int> keyframes;
    for (int i = 0; i < 10; ++i)
    {
        const Eigen::Isometry3d T = nadir({ 16.0 * i, 0.0, 100.0 });
        if (selector.consider(T, computeFootprint(camera(), T, flat(0.0))))
        {
            keyframes.push_back(i);
        }
    }
    EXPECT_EQ(keyframes, (std::vector<int> { 0, 4, 8 }));
}

TEST(KeyframeSelector, RejectsTiltedImagesAndMissingFootprints)
{
    KeyframeSelector        selector(KeyframeConfig {});
    const Eigen::Isometry3d tilted = nadir({ 0, 0, 100 }, 0.0, 30.0 * kPi / 180.0);
    EXPECT_FALSE(selector.consider(tilted, computeFootprint(camera(), tilted, flat(0.0))));
    EXPECT_FALSE(selector.consider(nadir({ 0, 0, 100 }), std::nullopt));
    EXPECT_TRUE(selector.consider(nadir({ 0, 0, 100 }), computeFootprint(camera(), nadir({ 0, 0, 100 }), flat(0.0))));
}

TEST(SourceViews, AppliesTheCriteriaAndRanks)
{
    const View        key = view(0, { 0, 0, 100 });
    std::vector<View> candidates {
        view(1, { 2, 0, 100 }),                         // baseline ratio 0.02: too short
        view(2, { 10, 0, 100 }),                        // ratio 0.1, angle 5.7°
        view(3, { 20, 0, 100 }),                        // ratio 0.2, angle 11.3°
        view(4, { 30, 0, 100 }),                        // ratio 0.3
        view(5, { 150, 0, 100 }),                       // overlap 25 %
        view(6, { 20, 0, 100 }, 0, 40.0 * kPi / 180.0), // optical axes 40° apart
        view(0, { 0, 0, 100 }),                         // the keyframe itself
    };
    SourceViewConfig config;
    config.max_sources                   = 5;
    const std::vector<SourceView> chosen = selectSourceViews(key, candidates, config);
    ASSERT_EQ(chosen.size(), 3u);
    // Ranked by overlap x min(1, angle / 10°).
    EXPECT_EQ(chosen[0].id, 3u);  // 0.9 x 1
    EXPECT_EQ(chosen[1].id, 4u);  // 0.85 x 1
    EXPECT_EQ(chosen[2].id, 2u);  // 0.95 x 0.57
    EXPECT_NEAR(chosen[0].baseline_ratio, 0.2, 1e-12);
    EXPECT_NEAR(chosen[0].triangulation_angle_deg, std::atan(0.2) * 180.0 / kPi, 1e-9);
    EXPECT_NEAR(chosen[0].overlap, 0.9, 1e-12);
}

TEST(SourceViews, IncludesTheNeighbouringLine)
{
    const View        key = view(0, { 0, 0, 100 }, 1);
    std::vector<View> candidates { view(1, { 20, 0, 100 }, 1), view(2, { 30, 0, 100 }, 1),
                                   view(3, { 0, 45, 100 }, 0) };  // overlap 55 %, lowest score
    SourceViewConfig  config;
    config.max_sources                   = 2;
    const std::vector<SourceView> chosen = selectSourceViews(key, candidates, config);
    ASSERT_EQ(chosen.size(), 2u);
    EXPECT_EQ(chosen[0].id, 1u);
    EXPECT_EQ(chosen[1].id, 3u);
    EXPECT_TRUE(chosen[1].other_line);
}

TEST(FlightLines, SplitAtTurns)
{
    std::vector<Eigen::Vector3d> p;
    for (int i = 0; i < 5; ++i)
    {
        p.emplace_back(20.0 * i, 0.0, 100.0);  // east
    }
    p.emplace_back(100.0, 60.0, 100.0);  // turn north
    for (int i = 0; i < 5; ++i)
    {
        p.emplace_back(80.0 - 20.0 * i, 60.0, 100.0);  // west
    }
    const std::vector<int> lines = assignFlightLines(p);
    EXPECT_EQ(lines, (std::vector<int> { 0, 0, 0, 0, 0, 1, 2, 2, 2, 2, 2 }));
}

}  // namespace
}  // namespace orthostream::selection
