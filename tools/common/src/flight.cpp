#include "flight.hpp"

#include <algorithm>
#include <stdexcept>

namespace orthostream::tools
{

Flight::Flight(const Dataset &data, const FlightConfig &config) : data_(data), config_(config)
{
    if (config.terrain_dem.empty())
    {
        throw std::runtime_error("a terrain prior is required");
    }
    // Flight frame at the mean camera position.
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (const PosedImage &im : data.images)
    {
        mean += im.T_E_C.translation();
    }
    mean /= static_cast<double>(data.images.size());
    const geo::GeodeticConverter geodetic(config.proj_data_dir);
    const geo::Geodetic          g = geodetic.ecefToGeodetic(mean);
    frame_                         = std::make_unique<geo::LocalFrame>(mean, g.latitude_rad, g.longitude_rad);

    // Terrain prior around the flight: camera positions plus a margin.
    to_terrain_ = std::make_unique<geo::CrsTransformer>("EPSG:4978", config.terrain_crs, config.proj_data_dir,
                                                        std::vector<std::string> { config.grid_dir });
    double x0 = 1e300, y0 = 1e300, x1 = -1e300, y1 = -1e300;
    for (const PosedImage &im : data.images)
    {
        const Eigen::Vector3d p = to_terrain_->forward(im.T_E_C.translation());
        x0 = std::min(x0, p.x()), y0 = std::min(y0, p.y());
        x1 = std::max(x1, p.x()), y1 = std::max(y1, p.y());
    }
    const double margin = (x1 - x0 < 10.0) ? 0.05 : 0.2 * std::max(x1 - x0, y1 - y0);  // degrees or metres
    const Raster dem(config.terrain_dem);
    terrain_ = std::make_unique<RasterWindow>(
        dem.read(x0 - margin, y0 - margin, x1 + margin, y1 + margin, config.terrain_decimation));

    // Footprints, flight lines and keyframes.
    const std::size_t            n = data.images.size();
    std::vector<Eigen::Vector3d> positions(n);
    views_.resize(n);
    for (std::size_t i = 0; i < n; ++i)
    {
        views_[i].id    = i;
        views_[i].T_L_C = frame_->poseInLocal(data.images[i].T_E_C);
        positions[i]    = views_[i].T_L_C.translation();
    }
    const std::vector<int> lines = selection::assignFlightLines(positions);
    lines_                       = lines.empty() ? 0 : lines.back() + 1;
    const selection::TerrainHeight terrain = [this](double x, double y) { return terrainHeight(x, y); };
    selection::FootprintConfig     fp_config;
    fp_config.nominal_height_agl = config.nominal_agl;
    selection::KeyframeConfig kf_config;
    kf_config.target_overlap = config.target_overlap;
    selection::KeyframeSelector selector(kf_config);
    has_footprint_.assign(n, false);
    for (std::size_t i = 0; i < n; ++i)
    {
        views_[i].flight_line = lines[i];
        const std::optional<selection::Footprint> fp =
            selection::computeFootprint(data.camera, views_[i].T_L_C, terrain, fp_config);
        if (fp)
        {
            views_[i].footprint = *fp;
            has_footprint_[i]   = true;
            ++with_footprint_;
        }
        if (selector.consider(views_[i].T_L_C, fp))
        {
            keyframes_.push_back(i);
        }
    }
}

std::optional<double> Flight::terrainHeight(double x, double y) const
{
    try
    {
        const Eigen::Vector3d p = to_terrain_->forward(frame_->toEcef({ x, y, -config_.nominal_agl }));
        const std::optional<double> h = terrain_->sample(p.x(), p.y());
        if (!h)
        {
            return std::nullopt;
        }
        return frame_->toLocal(to_terrain_->inverse({ p.x(), p.y(), *h })).z();
    }
    catch (const geo::GeoError &)
    {
        return std::nullopt;
    }
}

std::vector<selection::SourceView> Flight::sources(std::size_t k) const
{
    std::size_t limit = k;
    if (config_.delayed)
    {
        const auto next = std::upper_bound(keyframes_.begin(), keyframes_.end(), k);
        limit           = next == keyframes_.end() ? views_.size() : *next;
    }
    std::vector<selection::View> candidates;
    for (std::size_t j = 0; j < limit; ++j)
    {
        if (j != k && has_footprint_[j])
        {
            candidates.push_back(views_[j]);
        }
    }
    selection::SourceViewConfig sv;
    sv.max_sources = config_.sources;
    return selection::selectSourceViews(views_[k], candidates, sv);
}

std::pair<double, double> Flight::depthRange(const geo::CameraModel &cam, std::size_t k, double margin) const
{
    const selection::Footprint &f  = views_[k].footprint;
    double                      lo = f.centre.z(), hi = f.centre.z();
    for (const Eigen::Vector3d &c : f.corners)
    {
        lo = std::min(lo, c.z());
        hi = std::max(hi, c.z());
    }
    // Depth of a ground point at height h on the ray of pixel r: (C.z - h) / (-(R r).z).
    double       down_min = 1e9, down_max = 0.0;
    const double w = cam.width, h = cam.height;
    for (const Eigen::Vector2d &p : { Eigen::Vector2d(-0.5, -0.5), Eigen::Vector2d(w - 0.5, -0.5),
                                      Eigen::Vector2d(w - 0.5, h - 0.5), Eigen::Vector2d(-0.5, h - 0.5),
                                      Eigen::Vector2d(cam.cx, cam.cy) })
    {
        const Eigen::Vector3d r((p.x() - cam.cx) / cam.fx, (p.y() - cam.cy) / cam.fy, 1.0);
        const double          down = -(views_[k].T_L_C.linear() * r).z();
        down_min                   = std::min(down_min, down);
        down_max                   = std::max(down_max, down);
    }
    if (!(down_min > 0.05))
    {
        throw std::runtime_error("camera looks too far from nadir");
    }
    const double cz = views_[k].T_L_C.translation().z();
    return { std::max(1.0, (cz - hi - margin) / down_max), (cz - lo + margin) / down_min };
}

}  // namespace orthostream::tools
