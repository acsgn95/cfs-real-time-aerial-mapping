// Keyframe and source view selection over a whole recorded flight, as the
// mission does it (ADR-006), for the host tools: footprints on a terrain
// prior, flight lines, keyframes and causal (or delayed) source views.

#ifndef OST_TOOLS_FLIGHT_HPP
#define OST_TOOLS_FLIGHT_HPP

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "dataset.hpp"
#include "orthostream/geo/crs.hpp"
#include "orthostream/geo/frames.hpp"
#include "orthostream/selection/views.hpp"
#include "raster.hpp"

namespace orthostream::tools
{

struct FlightConfig
{
    std::string proj_data_dir;
    std::string grid_dir;
    std::string terrain_dem;                         ///< terrain prior raster
    std::string terrain_crs = "EPSG:4326+3855";      ///< its CRS with heights (Copernicus: EGM2008)
    int         terrain_decimation = 1;              ///< read every n-th pixel
    double      nominal_agl        = 250.0;
    double      target_overlap     = 0.7;
    int         sources            = 5;
    bool        delayed            = false;          ///< sources also up to the next keyframe
};

class Flight
{
public:
    Flight(const Dataset &data, const FlightConfig &config);

    const geo::LocalFrame                &frame() const { return *frame_; }
    const std::vector<std::size_t>       &keyframes() const { return keyframes_; }
    const selection::View                &view(std::size_t i) const { return views_[i]; }
    bool                                  hasFootprint(std::size_t i) const { return has_footprint_[i]; }
    std::size_t                           withFootprint() const { return with_footprint_; }
    int                                   flightLines() const { return lines_; }

    /// Source views of image k (causal: images before k).
    std::vector<selection::SourceView> sources(std::size_t k) const;

    /// Depth range along the optical axis of a camera with pose T_F_C (in
    /// the flight frame) over its footprint's terrain, widened by a margin.
    std::pair<double, double> depthRange(const geo::CameraModel &camera, std::size_t k, double margin) const;

    /// Terrain prior height (flight frame z) at a horizontal position.
    std::optional<double> terrainHeight(double x, double y) const;

private:
    const Dataset                       &data_;
    FlightConfig                         config_;
    std::unique_ptr<geo::LocalFrame>     frame_;
    std::unique_ptr<geo::CrsTransformer> to_terrain_;
    std::unique_ptr<RasterWindow>        terrain_;
    std::vector<selection::View>         views_;
    std::vector<bool>                    has_footprint_;
    std::vector<std::size_t>             keyframes_;
    std::size_t                          with_footprint_ = 0;
    int                                  lines_          = 0;
};

}  // namespace orthostream::tools

#endif  // OST_TOOLS_FLIGHT_HPP
