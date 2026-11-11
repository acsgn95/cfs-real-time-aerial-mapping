// dense_benchmark: runs dense reconstruction on keyframe batches of a posed
// image set and measures time, peak GPU memory and depth accuracy against a
// reference DSM (ADR-005 section 6, ADR-018).
//
// Per run, one configuration (working width, K, iterations, causal or
// delayed mode). Keyframes and source views are chosen with the selection
// library (ADR-006); footprints use the reference DSM instead of SRTM.
// Results are written as one CSV row per keyframe.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "dataset.hpp"
#include "error_stats.hpp"
#include "gpu_memory.hpp"
#include "orthostream/dense/patch_match.hpp"
#include "orthostream/geo/crs.hpp"
#include "orthostream/geo/frames.hpp"
#include "orthostream/imaging/working_image.hpp"
#include "orthostream/selection/footprint.hpp"
#include "orthostream/selection/views.hpp"
#include "raster.hpp"

namespace
{

using namespace orthostream;
using bench::RasterWindow;
using Clock = std::chrono::steady_clock;

struct Options
{
    std::string              poses;
    std::string              images;
    std::string              dsm;
    std::string              out;
    std::string              dsm_crs   = "EPSG:2056+5729";  // LV95 with LHN95 heights (ADR-008)
    std::string              proj_data = OST_BENCH_PROJ_DATA_DIR;
    std::string              grid_dir  = OST_BENCH_GRID_DIR;
    std::string              save_depth;
    std::vector<std::string> keyframes;
    int                      width        = 2048;
    int                      sources      = 5;
    int                      iterations   = 5;
    int                      count        = 20;
    int                      eval_stride  = 2;
    int                      gpu          = 0;
    double                   nominal_agl  = 250.0;
    double                   depth_margin = 50.0;
    double                   target_overlap = 0.7;
    bool                     delayed      = false;
    bool                     dry_run      = false;
    bool                     warmup       = true;
};

const char *kUsage = R"(usage: dense_benchmark --poses FILE --images DIR --dsm FILE --out FILE.csv [options]

  --poses FILE          camera and poses (tools/dataset/extract_metashape_poses.py)
  --images DIR          directory with the JPEG images named in the pose file
  --dsm FILE            reference DSM (GeoTIFF)
  --out FILE            CSV results, one row per keyframe
  --dsm-crs CRS         CRS of the DSM including heights (default EPSG:2056+5729)
  --width N             dense working width in pixels (default 2048)
  --sources K           source views per keyframe (default 5)
  --iterations N        PatchMatch iterations (default 5)
  --mode causal|delayed source views from images before the keyframe, or also
                        up to the next keyframe (default causal)
  --count N             keyframes, evenly spread over the flight (default 20)
  --keyframe NAME       a given keyframe instead (repeatable)
  --target-overlap F    forward overlap between keyframes (default 0.7)
  --nominal-agl M       nominal height above ground for footprints (default 250)
  --depth-margin M      margin of the depth range around the terrain (default 50)
  --eval-stride N       evaluate every N-th pixel in both directions (default 2)
  --gpu N               GPU index (default 0)
  --no-warmup           do not run the first batch once before measuring
  --dry-run             no PatchMatch: render depth from the DSM instead; checks
                        frames, selection and evaluation without a GPU
  --save-depth DIR      write depth maps as PFM files
  --proj-data DIR       directory with proj.db (default: from the build)
  --grid-dir DIR        PROJ grid directory (default: build-deps/proj-grids)
)";

Options parseOptions(int argc, char **argv)
{
    Options o;
    for (int i = 1; i < argc; ++i)
    {
        const std::string a     = argv[i];
        auto              value = [&]() -> std::string
        {
            if (i + 1 >= argc)
            {
                throw std::runtime_error("missing value for " + a);
            }
            return argv[++i];
        };
        if (a == "--poses") o.poses = value();
        else if (a == "--images") o.images = value();
        else if (a == "--dsm") o.dsm = value();
        else if (a == "--out") o.out = value();
        else if (a == "--dsm-crs") o.dsm_crs = value();
        else if (a == "--width") o.width = std::stoi(value());
        else if (a == "--sources") o.sources = std::stoi(value());
        else if (a == "--iterations") o.iterations = std::stoi(value());
        else if (a == "--count") o.count = std::stoi(value());
        else if (a == "--keyframe") o.keyframes.push_back(value());
        else if (a == "--target-overlap") o.target_overlap = std::stod(value());
        else if (a == "--nominal-agl") o.nominal_agl = std::stod(value());
        else if (a == "--depth-margin") o.depth_margin = std::stod(value());
        else if (a == "--eval-stride") o.eval_stride = std::stoi(value());
        else if (a == "--gpu") o.gpu = std::stoi(value());
        else if (a == "--no-warmup") o.warmup = false;
        else if (a == "--dry-run") o.dry_run = true;
        else if (a == "--save-depth") o.save_depth = value();
        else if (a == "--proj-data") o.proj_data = value();
        else if (a == "--grid-dir") o.grid_dir = value();
        else if (a == "--mode")
        {
            const std::string m = value();
            if (m != "causal" && m != "delayed")
            {
                throw std::runtime_error("--mode must be causal or delayed");
            }
            o.delayed = m == "delayed";
        }
        else if (a == "--help" || a == "-h")
        {
            std::cout << kUsage;
            std::exit(0);
        }
        else
        {
            throw std::runtime_error("unknown option " + a);
        }
    }
    if (o.poses.empty() || o.images.empty() || o.dsm.empty() || o.out.empty())
    {
        throw std::runtime_error("--poses, --images, --dsm and --out are required");
    }
    if (o.width <= 0 || o.sources <= 0 || o.iterations <= 0 || o.count <= 0 || o.eval_stride <= 0)
    {
        throw std::runtime_error("invalid numeric option");
    }
    return o;
}

std::vector<std::uint8_t> readFile(const std::string &path)
{
    std::ifstream in(path, std::ios::binary);
    if (!in)
    {
        throw std::runtime_error("cannot open " + path);
    }
    return std::vector<std::uint8_t>((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

void writePfm(const std::string &path, const std::vector<float> &values, int width, int height)
{
    std::ofstream out(path, std::ios::binary);
    out << "Pf\n" << width << " " << height << "\n-1\n";  // negative scale: little-endian
    for (int r = height - 1; r >= 0; --r)                  // PFM stores the bottom row first
    {
        out.write(reinterpret_cast<const char *>(values.data() + static_cast<std::size_t>(r) * width),
                  static_cast<std::streamsize>(width * sizeof(float)));
    }
}

// Ground heights of a footprint in the flight frame: min and max of the five
// points.
std::pair<double, double> footprintHeightRange(const selection::Footprint &f)
{
    double lo = f.centre.z(), hi = f.centre.z();
    for (const Eigen::Vector3d &c : f.corners)
    {
        lo = std::min(lo, c.z());
        hi = std::max(hi, c.z());
    }
    return { lo, hi };
}

// Depth range along the optical axis for terrain between h_lo and h_hi
// (flight frame z), widened by a margin, over all pixels of the image.
std::pair<double, double> depthRange(const geo::CameraModel &cam, const Eigen::Isometry3d &T_F_C, double h_lo,
                                     double h_hi, double margin)
{
    // Depth of a ground point at height h on the ray of pixel r: (C.z - h) / (-(R r).z).
    double       down_min = 1e9, down_max = 0.0;
    const double w = cam.width, h = cam.height;
    for (const Eigen::Vector2d &p : { Eigen::Vector2d(-0.5, -0.5), Eigen::Vector2d(w - 0.5, -0.5),
                                      Eigen::Vector2d(w - 0.5, h - 0.5), Eigen::Vector2d(-0.5, h - 0.5),
                                      Eigen::Vector2d(cam.cx, cam.cy) })
    {
        const Eigen::Vector3d r((p.x() - cam.cx) / cam.fx, (p.y() - cam.cy) / cam.fy, 1.0);
        const double          down = -(T_F_C.linear() * r).z();
        down_min                   = std::min(down_min, down);
        down_max                   = std::max(down_max, down);
    }
    const double cz = T_F_C.translation().z();
    if (!(down_min > 0.05))
    {
        throw std::runtime_error("reference camera looks too far from nadir");
    }
    const double d_min = std::max(1.0, (cz - h_hi - margin) / down_max);
    const double d_max = (cz - h_lo + margin) / down_min;
    return { d_min, d_max };
}

struct BatchResult
{
    std::string           status = "ok";
    double                dense_s = 0.0;
    bench::GpuMemorySampler::Result gpu;
    double                valid_fraction = 0.0;
    bench::ErrorStats     errors;
};

class Benchmark
{
public:
    explicit Benchmark(const Options &o)
        : o_(o),
          data_(bench::loadPoses(o.poses)),
          geodetic_(o.proj_data),
          to_dsm_("EPSG:4978", o.dsm_crs, o.proj_data, { o.grid_dir }),
          dsm_(o.dsm),
          coarse_(dsm_.readAll(16)),
          converter_(data_.camera, o.width, 1),
          wcam_(converter_.workingCamera()),
          gpu_(static_cast<unsigned>(o.gpu))
    {
        Eigen::Vector3d mean = Eigen::Vector3d::Zero();
        for (const bench::PosedImage &im : data_.images)
        {
            mean += im.T_E_C.translation();
        }
        mean /= static_cast<double>(data_.images.size());
        const geo::Geodetic g = geodetic_.ecefToGeodetic(mean);
        flight_frame_         = std::make_unique<geo::LocalFrame>(mean, g.latitude_rad, g.longitude_rad);
        if (!o.dry_run)
        {
            dense::PatchMatchConfig config;
            config.gpu_index      = o.gpu;
            config.num_iterations = o.iterations;
            estimator_            = std::make_unique<dense::PatchMatchEstimator>(config);
        }
    }

    int run()
    {
        selectViews();
        const std::vector<std::size_t> chosen = chooseKeyframes();
        std::cout << "images " << data_.images.size() << ", with footprint " << with_footprint_ << ", flight lines "
                  << num_lines_ << ", keyframes " << keyframes_.size() << ", benchmarked " << chosen.size()
                  << "\nworking image " << wcam_.width << " x " << wcam_.height << ", DCT scale "
                  << converter_.dctScale().num << "/" << converter_.dctScale().denom << ", mode "
                  << (o_.delayed ? "delayed" : "causal") << (o_.dry_run ? ", dry run" : "") << "\n";
        if (!gpu_.available() && !o_.dry_run)
        {
            std::cout << "NVML not available: GPU memory is not measured\n";
        }

        std::ofstream csv(o_.out);
        if (!csv)
        {
            throw std::runtime_error("cannot write " + o_.out);
        }
        csv << "image,index,flight_line,mode,method,width,height,iterations,sources,other_line_sources,source_images,"
               "depth_min,depth_max,decode_ms_mean,decoded_images,dense_s,gpu_process_peak_mb,gpu_device_peak_mb,"
               "gpu_device_baseline_mb,valid_fraction,eval_points,mean,median,mae,rmse,nmad,p95_abs,within_05,"
               "within_1,warmup,status\n";

        std::vector<std::pair<std::size_t, bool>> runs;
        if (o_.warmup && !o_.dry_run && !chosen.empty())
        {
            runs.emplace_back(chosen.front(), true);
        }
        for (std::size_t k : chosen)
        {
            runs.emplace_back(k, false);
        }
        for (const auto &[k, warmup] : runs)
        {
            runBatch(k, warmup, csv);
        }
        return 0;
    }

private:
    std::optional<double> terrainInFlightFrame(double x, double y) const
    {
        try
        {
            const Eigen::Vector3d p = to_dsm_.forward(flight_frame_->toEcef({ x, y, -o_.nominal_agl }));
            const std::optional<double> h = coarse_.sample(p.x(), p.y());
            if (!h)
            {
                return std::nullopt;
            }
            return flight_frame_->toLocal(to_dsm_.inverse({ p.x(), p.y(), *h })).z();
        }
        catch (const geo::GeoError &)
        {
            return std::nullopt;
        }
    }

    void selectViews()
    {
        const std::size_t            n = data_.images.size();
        std::vector<Eigen::Vector3d> positions(n);
        views_.resize(n);
        for (std::size_t i = 0; i < n; ++i)
        {
            views_[i].id    = i;
            views_[i].T_L_C = flight_frame_->poseInLocal(data_.images[i].T_E_C);
            positions[i]    = views_[i].T_L_C.translation();
        }
        const std::vector<int> lines = selection::assignFlightLines(positions);
        num_lines_                   = lines.empty() ? 0 : lines.back() + 1;
        const selection::TerrainHeight terrain = [this](double x, double y) { return terrainInFlightFrame(x, y); };
        selection::FootprintConfig     fp_config;
        fp_config.nominal_height_agl = o_.nominal_agl;
        selection::KeyframeConfig kf_config;
        kf_config.target_overlap = o_.target_overlap;
        selection::KeyframeSelector keyframe_selector(kf_config);
        has_footprint_.assign(n, false);
        for (std::size_t i = 0; i < n; ++i)
        {
            views_[i].flight_line = lines[i];
            const std::optional<selection::Footprint> fp =
                selection::computeFootprint(data_.camera, views_[i].T_L_C, terrain, fp_config);
            if (fp)
            {
                views_[i].footprint = *fp;
                has_footprint_[i]   = true;
                ++with_footprint_;
            }
            if (keyframe_selector.consider(views_[i].T_L_C, fp))
            {
                keyframes_.push_back(i);
            }
        }
    }

    std::vector<selection::SourceView> sourcesFor(std::size_t k) const
    {
        std::size_t limit = k;  // causal: images received before the keyframe
        if (o_.delayed)
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
        selection::SourceViewConfig config;
        config.max_sources = o_.sources;
        return selection::selectSourceViews(views_[k], candidates, config);
    }

    std::vector<std::size_t> chooseKeyframes() const
    {
        std::vector<std::size_t> chosen;
        if (!o_.keyframes.empty())
        {
            for (const std::string &name : o_.keyframes)
            {
                const auto it = std::find_if(data_.images.begin(), data_.images.end(),
                                             [&](const bench::PosedImage &im) { return im.name == name; });
                if (it == data_.images.end())
                {
                    throw std::runtime_error("unknown image " + name);
                }
                const std::size_t i = static_cast<std::size_t>(it - data_.images.begin());
                if (!has_footprint_[i])
                {
                    throw std::runtime_error("no footprint for " + name);
                }
                chosen.push_back(i);
            }
            return chosen;
        }
        // Keyframes with the full number of source views; the first ones of
        // the flight have too few earlier images.
        std::vector<std::size_t> eligible;
        for (std::size_t k : keyframes_)
        {
            if (static_cast<int>(sourcesFor(k).size()) == o_.sources)
            {
                eligible.push_back(k);
            }
        }
        const std::size_t m = eligible.size();
        const std::size_t c = std::min<std::size_t>(static_cast<std::size_t>(o_.count), m);
        std::set<std::size_t> picked;
        for (std::size_t j = 0; j < c; ++j)
        {
            const std::size_t idx = c == 1 ? m / 2 : static_cast<std::size_t>(std::lround(
                                                         static_cast<double>(j) * static_cast<double>(m - 1) /
                                                         static_cast<double>(c - 1)));
            picked.insert(eligible[idx]);
        }
        return { picked.begin(), picked.end() };
    }

    const std::vector<std::uint8_t> &workingImage(std::size_t i, double &decode_ms, int &decoded)
    {
        auto it = cache_.find(i);
        if (it != cache_.end())
        {
            return it->second;
        }
        const std::vector<std::uint8_t> jpeg = readFile(o_.images + "/" + data_.images[i].name);
        std::vector<std::uint8_t>       img(static_cast<std::size_t>(wcam_.width) * wcam_.height);
        imaging::ImageView              view { img.data(), wcam_.width, wcam_.height, 1,
                                  static_cast<std::size_t>(wcam_.width) };
        const auto                      t0 = Clock::now();
        converter_.convert(jpeg.data(), jpeg.size(), view);
        decode_ms += std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
        ++decoded;
        return cache_.emplace(i, std::move(img)).first->second;
    }

    // Depth map of the reference camera rendered from the DSM (dry run).
    std::vector<float> renderDsmDepth(const RasterWindow &win, const Eigen::Isometry3d &T_E_C) const
    {
        const std::size_t       n = static_cast<std::size_t>(wcam_.width) * wcam_.height;
        std::vector<float>      depth(n, 0.0f);
        const Eigen::Isometry3d T_C_E = T_E_C.inverse();
        for (int r = 0; r < win.height; ++r)
        {
            for (int c = 0; c < win.width; ++c)
            {
                const float h = win.at(c, r);
                if (std::isnan(h))
                {
                    continue;
                }
                const Eigen::Vector3d p_C = T_C_E * to_dsm_.inverse({ win.centreX(c), win.centreY(r), h });
                if (p_C.z() <= 0.0)
                {
                    continue;
                }
                const long u = std::lround(wcam_.fx * p_C.x() / p_C.z() + wcam_.cx);
                const long v = std::lround(wcam_.fy * p_C.y() / p_C.z() + wcam_.cy);
                if (u < 0 || v < 0 || u >= wcam_.width || v >= wcam_.height)
                {
                    continue;
                }
                float &d = depth[static_cast<std::size_t>(v) * wcam_.width + static_cast<std::size_t>(u)];
                if (d == 0.0f || p_C.z() < d)
                {
                    d = static_cast<float>(p_C.z());
                }
            }
        }
        return depth;
    }

    // DSM window covering a footprint given in the flight frame.
    RasterWindow dsmWindow(const selection::Footprint &f) const
    {
        double x0 = 1e300, y0 = 1e300, x1 = -1e300, y1 = -1e300;
        for (const Eigen::Vector3d &c : f.corners)
        {
            const Eigen::Vector3d p = to_dsm_.forward(flight_frame_->toEcef(c));
            x0 = std::min(x0, p.x()), y0 = std::min(y0, p.y());
            x1 = std::max(x1, p.x()), y1 = std::max(y1, p.y());
        }
        const double margin = 30.0;
        return dsm_.read(x0 - margin, y0 - margin, x1 + margin, y1 + margin, 1);
    }

    void evaluate(const std::vector<float> &depth, const Eigen::Isometry3d &T_E_C, const RasterWindow &win,
                  BatchResult &result) const
    {
        std::size_t valid = 0;
        for (float d : depth)
        {
            valid += d > 0.0f ? 1 : 0;
        }
        result.valid_fraction = static_cast<double>(valid) / static_cast<double>(depth.size());
        std::vector<double> errors;
        for (int v = 0; v < wcam_.height; v += o_.eval_stride)
        {
            for (int u = 0; u < wcam_.width; u += o_.eval_stride)
            {
                const double d = depth[static_cast<std::size_t>(v) * wcam_.width + static_cast<std::size_t>(u)];
                if (!(d > 0.0))
                {
                    continue;
                }
                const Eigen::Vector3d p_C(d * (u - wcam_.cx) / wcam_.fx, d * (v - wcam_.cy) / wcam_.fy, d);
                const Eigen::Vector3d p           = to_dsm_.forward(T_E_C * p_C);
                const std::optional<double> h_ref = win.sample(p.x(), p.y());
                if (h_ref)
                {
                    errors.push_back(p.z() - *h_ref);
                }
            }
        }
        result.errors = bench::errorStats(std::move(errors));
    }

    void runBatch(std::size_t k, bool warmup, std::ofstream &csv)
    {
        const bench::PosedImage                 &ref     = data_.images[k];
        const std::vector<selection::SourceView> sources = sourcesFor(k);
        BatchResult                              result;
        double                                   decode_ms = 0.0;
        int                                      decoded   = 0;
        std::pair<double, double>                range { 0.0, 0.0 };
        std::string                              source_names;
        int                                      other_line = 0;
        for (const selection::SourceView &s : sources)
        {
            source_names += (source_names.empty() ? "" : ";") + data_.images[s.id].name;
            other_line += s.other_line ? 1 : 0;
        }
        try
        {
            if (sources.empty())
            {
                throw std::runtime_error("no source views");
            }
            const auto [h_lo, h_hi] = footprintHeightRange(views_[k].footprint);
            range                   = depthRange(wcam_, views_[k].T_L_C, h_lo, h_hi, o_.depth_margin);
            const RasterWindow win  = dsmWindow(views_[k].footprint);

            // Keep only the images of this batch in the cache.
            std::set<std::size_t> needed { k };
            for (const selection::SourceView &s : sources)
            {
                needed.insert(s.id);
            }
            for (auto it = cache_.begin(); it != cache_.end();)
            {
                it = needed.count(it->first) ? std::next(it) : cache_.erase(it);
            }

            const std::size_t  n = static_cast<std::size_t>(wcam_.width) * wcam_.height;
            std::vector<float> depth(n, 0.0f);
            if (o_.dry_run)
            {
                const auto t0  = Clock::now();
                depth          = renderDsmDepth(win, ref.T_E_C);
                result.dense_s = std::chrono::duration<double>(Clock::now() - t0).count();
            }
            else
            {
                const geo::Geodetic   g = geodetic_.ecefToGeodetic(ref.T_E_C.translation());
                const geo::LocalFrame L(ref.T_E_C.translation(), g.latitude_rad, g.longitude_rad);
                dense::DepthBatch     batch;
                batch.camera          = wcam_;
                batch.depth_min       = range.first;
                batch.depth_max       = range.second;
                batch.reference.T_L_C = L.poseInLocal(ref.T_E_C);
                batch.reference.image = { workingImage(k, decode_ms, decoded).data(),
                                          static_cast<std::size_t>(wcam_.width) };
                for (const selection::SourceView &s : sources)
                {
                    dense::BatchView v;
                    v.T_L_C = L.poseInLocal(data_.images[s.id].T_E_C);
                    v.image = { workingImage(s.id, decode_ms, decoded).data(), static_cast<std::size_t>(wcam_.width) };
                    batch.sources.push_back(v);
                }
                std::vector<float> normals(3 * n), confidence(n);
                gpu_.start();
                const auto t0 = Clock::now();
                estimator_->estimate(batch, { depth.data(), normals.data(), confidence.data() });
                result.dense_s = std::chrono::duration<double>(Clock::now() - t0).count();
                result.gpu     = gpu_.stop();
            }
            evaluate(depth, ref.T_E_C, win, result);
            if (!o_.save_depth.empty() && !warmup)
            {
                const std::string stem = ref.name.substr(0, ref.name.find_last_of('.'));
                writePfm(o_.save_depth + "/" + stem + "_depth.pfm", depth, wcam_.width, wcam_.height);
            }
        }
        catch (const std::exception &e)
        {
            gpu_.stop();
            result.status = std::string("error: ") + e.what();
            std::replace(result.status.begin(), result.status.end(), ',', ';');
        }

        const bench::ErrorStats &s = result.errors;
        csv << ref.name << ',' << k << ',' << views_[k].flight_line << ',' << (o_.delayed ? "delayed" : "causal") << ','
            << (o_.dry_run ? "dsm_render" : "patchmatch") << ',' << wcam_.width << ',' << wcam_.height << ','
            << o_.iterations << ',' << sources.size() << ',' << other_line << ',' << source_names << ','
            << range.first << ',' << range.second << ',' << (decoded > 0 ? decode_ms / decoded : 0.0) << ','
            << decoded << ',' << result.dense_s << ',' << result.gpu.process_peak_mb << ','
            << result.gpu.device_peak_mb << ',' << result.gpu.device_baseline_mb << ',' << result.valid_fraction
            << ',' << s.count << ',' << s.mean << ',' << s.median << ',' << s.mae << ',' << s.rmse << ',' << s.nmad
            << ',' << s.p95_abs << ',' << s.within_05 << ',' << s.within_1 << ',' << (warmup ? 1 : 0) << ','
            << result.status << '\n';
        csv.flush();  // keep the rows written so far if COLMAP ends the process

        char line[256];
        std::snprintf(line, sizeof(line), "%s%-36s K=%zu (%d other line)  %6.2f s  gpu %6.0f MB  valid %4.2f  "
                      "median %+6.2f m  NMAD %5.2f m  RMSE %6.2f m  %s\n",
                      warmup ? "[warmup] " : "", ref.name.c_str(), sources.size(), other_line, result.dense_s,
                      result.gpu.process_peak_mb, result.valid_fraction, s.median, s.nmad, s.rmse,
                      result.status.c_str());
        std::cout << line << std::flush;
    }

    const Options                                o_;
    const bench::Dataset                         data_;
    geo::GeodeticConverter                       geodetic_;
    geo::CrsTransformer                          to_dsm_;
    bench::Raster                                dsm_;
    const RasterWindow                           coarse_;
    imaging::WorkingImageConverter               converter_;
    const geo::CameraModel                       wcam_;
    bench::GpuMemorySampler                      gpu_;
    std::unique_ptr<geo::LocalFrame>             flight_frame_;
    std::unique_ptr<dense::PatchMatchEstimator>  estimator_;
    std::vector<selection::View>                 views_;
    std::vector<bool>                            has_footprint_;
    std::vector<std::size_t>                     keyframes_;
    std::size_t                                  with_footprint_ = 0;
    int                                          num_lines_      = 0;
    std::map<std::size_t, std::vector<std::uint8_t>> cache_;
};

}  // namespace

int main(int argc, char **argv)
{
    try
    {
        const Options o = parseOptions(argc, argv);
        Benchmark     benchmark(o);
        return benchmark.run();
    }
    catch (const std::exception &e)
    {
        std::cerr << "dense_benchmark: " << e.what() << "\n";
        return 1;
    }
}
