// offline_pipeline: the OrthoStream mapping pipeline on a recorded flight,
// without cFS (ADR-011 section 7). Keyframes are processed in capture
// order: selection (ADR-006), PatchMatch (ADR-005), DSM fusion,
// orthomosaic and point cloud with the tile lifecycle (ADR-007). Stage
// times go to a CSV file; at the end the DSM is compared with a reference
// DSM, tile by tile.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "dataset.hpp"
#include "error_stats.hpp"
#include "flight.hpp"
#include "orthostream/dense/patch_match.hpp"
#include "orthostream/dense/sgbm.hpp"
#include "orthostream/geo/camera.hpp"
#include "orthostream/imaging/jpeg.hpp"
#include "orthostream/imaging/working_image.hpp"
#include "orthostream/map_products/lifecycle.hpp"
#include "orthostream/map_products/point_cloud.hpp"
#include "raster.hpp"
#include "visual.hpp"

#include <opencv2/imgproc.hpp>

namespace
{

using namespace orthostream;
namespace fs = std::filesystem;
namespace mp = orthostream::map_products;
using Clock  = std::chrono::steady_clock;

double since(Clock::time_point t0)
{
    return std::chrono::duration<double>(Clock::now() - t0).count();
}

struct Options
{
    std::string poses, selection_poses, images, terrain_dem, out, reference_dsm;
    std::string terrain_crs   = "EPSG:4326+3855";
    std::string product_crs   = "EPSG:2056";
    std::string vertical_crs  = "EPSG:5729";
    std::string proj_data     = OST_TOOLS_PROJ_DATA_DIR;
    std::string grid_dir      = OST_TOOLS_GRID_DIR;
    int         first         = 0;     // first keyframe (index in the keyframe list)
    int         count         = 20;    // keyframes to process
    int         width         = 2048;
    int         sources       = 5;
    int         iterations    = 3;
    int         window_radius = 3;
    int         window_step   = 1;
    int         samples       = 7;
    double      filter_min_ncc        = 0.3;
    int         filter_min_consistent = 2;
    double      filter_min_angle      = 3.0;
    double      dsm_cell      = 0.26;
    int         tile_cells    = 512;
    double      ortho_pixel   = 0.066;
    int         ortho_width   = 0;     // decode width for the orthomosaic; 0: half the camera width
    double      depth_margin  = 50.0;
    double      min_confidence = 0.0;
    double      terrain_gate   = 30.0;  // metres; 0 disables
    std::size_t max_tiles     = 64;
    double      finalise_distance = 600.0;
    bool        points        = false;
    bool        delayed       = true;   // ADR-005 section 2
    std::string method        = "sgbm";
    bool        evaluate_only = false;
};

const char *kUsage = R"(usage: offline_pipeline --poses FILE --images DIR --terrain-dem FILE --out DIR [options]

  --poses FILE            camera and poses (tools/dataset/extract_metashape_poses.py)
  --selection-poses FILE  poses for keyframe and source view selection, as the telemetry
                          poses in ADR-019 section 2 (default: --poses); depth range,
                          terrain gate and reconstruction always use --poses
  --images DIR            JPEG images named in the pose file
  --terrain-dem FILE      terrain prior (tools/dataset/fetch_dem.py)
  --terrain-crs CRS       its CRS with heights (default EPSG:4326+3855)
  --out DIR               products, timings and evaluation
  --reference-dsm FILE    reference DSM for the evaluation, in the product CRS
  --product-crs CRS       projected CRS of the products (default EPSG:2056)
  --vertical-crs CRS      vertical CRS of the products, empty for ellipsoidal (default EPSG:5729)
  --first N, --count N    keyframes to process (default 0, 20)
  --method M              sgbm (CPU, two views; default, ADR-005 section 1) or
                          patchmatch (GPU, ADR-005 section 4)
  --width N               dense working width (default 2048)
  --sources K             source views (default 5)
  --mode causal|delayed   source views from images before the keyframe, or also up to the
                          next keyframe (default delayed; ADR-005 section 2)
  --iterations N          PatchMatch iterations (default 3)
  --window-radius N       PatchMatch NCC half window (default 3)
  --window-step N         PatchMatch NCC window step, 1 or 2 (default 1)
  --samples N             PatchMatch view selection samples (default 7)
  --filter-min-ncc F      photometric filter: minimum NCC (default 0.3)
  --filter-min-consistent N  photometric filter: minimum consistent sources (default 2)
  --filter-min-angle D    photometric filter: minimum triangulation angle (default 3)
  --dsm-cell M            DSM cell size (default 0.26)
  --tile-cells N          cells per tile side (default 512)
  --ortho-pixel M         orthomosaic pixel size (default 0.066)
  --ortho-width N         keyframe decode width for the orthomosaic (default: half)
  --min-confidence F      depth pixels below it are ignored (default 0)
  --terrain-gate M        drop depth pixels farther than M from the terrain prior (default 30; 0: off)
  --max-tiles N           active tile budget (default 64)
  --finalise-distance M   finalise tiles farther from the vehicle (default 600)
  --points                also write the LAZ point cloud
  --evaluate-only         only evaluate the products already in --out (needs --reference-dsm)
  --proj-data DIR, --grid-dir DIR
)";

Options parse(int argc, char **argv)
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
        else if (a == "--selection-poses") o.selection_poses = value();
        else if (a == "--images") o.images = value();
        else if (a == "--terrain-dem") o.terrain_dem = value();
        else if (a == "--terrain-crs") o.terrain_crs = value();
        else if (a == "--out") o.out = value();
        else if (a == "--reference-dsm") o.reference_dsm = value();
        else if (a == "--product-crs") o.product_crs = value();
        else if (a == "--vertical-crs") o.vertical_crs = value();
        else if (a == "--first") o.first = std::stoi(value());
        else if (a == "--count") o.count = std::stoi(value());
        else if (a == "--width") o.width = std::stoi(value());
        else if (a == "--sources") o.sources = std::stoi(value());
        else if (a == "--iterations") o.iterations = std::stoi(value());
        else if (a == "--window-radius") o.window_radius = std::stoi(value());
        else if (a == "--window-step") o.window_step = std::stoi(value());
        else if (a == "--samples") o.samples = std::stoi(value());
        else if (a == "--filter-min-ncc") o.filter_min_ncc = std::stod(value());
        else if (a == "--filter-min-consistent") o.filter_min_consistent = std::stoi(value());
        else if (a == "--filter-min-angle") o.filter_min_angle = std::stod(value());
        else if (a == "--dsm-cell") o.dsm_cell = std::stod(value());
        else if (a == "--tile-cells") o.tile_cells = std::stoi(value());
        else if (a == "--ortho-pixel") o.ortho_pixel = std::stod(value());
        else if (a == "--ortho-width") o.ortho_width = std::stoi(value());
        else if (a == "--min-confidence") o.min_confidence = std::stod(value());
        else if (a == "--terrain-gate") o.terrain_gate = std::stod(value());
        else if (a == "--max-tiles") o.max_tiles = static_cast<std::size_t>(std::stoul(value()));
        else if (a == "--finalise-distance") o.finalise_distance = std::stod(value());
        else if (a == "--points") o.points = true;
        else if (a == "--method")
        {
            o.method = value();
            if (o.method != "patchmatch" && o.method != "sgbm")
            {
                throw std::runtime_error("--method must be patchmatch or sgbm");
            }
        }
        else if (a == "--mode")
        {
            const std::string m = value();
            if (m != "causal" && m != "delayed")
            {
                throw std::runtime_error("--mode must be causal or delayed");
            }
            o.delayed = m == "delayed";
        }
        else if (a == "--evaluate-only") o.evaluate_only = true;
        else if (a == "--proj-data") o.proj_data = value();
        else if (a == "--grid-dir") o.grid_dir = value();
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
    if (o.evaluate_only)
    {
        if (o.out.empty() || o.reference_dsm.empty())
        {
            throw std::runtime_error("--evaluate-only needs --out and --reference-dsm");
        }
        return o;
    }
    if (o.poses.empty() || o.images.empty() || o.terrain_dem.empty() || o.out.empty())
    {
        throw std::runtime_error("--poses, --images, --terrain-dem and --out are required");
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

// DSM products against the reference DSM, tile by tile.
void evaluate(const Options &o, std::ostream &report)
{
    const tools::Raster   reference(o.reference_dsm);
    std::vector<double>   diff;
    std::size_t           cells = 0;
    double                cell_area = 0.0;
    for (const auto &entry : fs::directory_iterator(fs::path(o.out) / "dsm"))
    {
        if (entry.path().extension() != ".tif")
        {
            continue;
        }
        const tools::Raster       tile(entry.path().string());
        const tools::RasterWindow ours = tile.readAll(1);
        const double              x0 = ours.origin_x, y1 = ours.origin_y;
        const double              x1 = x0 + ours.width * ours.pixel_x, y0 = y1 + ours.height * ours.pixel_y;
        const tools::RasterWindow ref  = reference.read(x0 - 1, y0 - 1, x1 + 1, y1 + 1, 1);
        cell_area = ours.pixel_x * -ours.pixel_y;
        for (int r = 0; r < ours.height; ++r)
        {
            for (int c = 0; c < ours.width; ++c)
            {
                const float h = ours.at(c, r);
                if (std::isnan(h))
                {
                    continue;
                }
                ++cells;
                if (const std::optional<double> hr = ref.sample(ours.centreX(c), ours.centreY(r)))
                {
                    diff.push_back(h - *hr);
                }
            }
        }
    }
    const tools::ErrorStats s = tools::errorStats(diff);
    char line[512];
    std::snprintf(line, sizeof(line),
                  "DSM against the reference: %zu cells (%.0f m2), %zu compared; mean %+.3f m, median %+.3f m, "
                  "NMAD %.3f m, RMSE %.3f m, MAE %.3f m, 95%% |e| %.3f m, within 0.5 m %.1f %%, within 1 m %.1f %%\n",
                  cells, static_cast<double>(cells) * cell_area, s.count, s.mean, s.median, s.nmad, s.rmse, s.mae,
                  s.p95_abs, 100.0 * s.within_05, 100.0 * s.within_1);
    report << line;
    std::cout << line;
}

// Overview of the whole area: orthomosaic, our DSM, reference DSM and their
// difference, plus the difference as a GeoTIFF for GIS viewers.
void writeOverview(const Options &o)
{
    const fs::path      out = o.out;
    const tools::Raster ours((out / "dsm.vrt").string());
    const int           dec = std::max(1, (std::max(ours.width(), ours.height()) + 1599) / 1600);
    const tools::RasterWindow dsm = ours.readAll(dec);
    const double x0 = dsm.origin_x, y1 = dsm.origin_y;
    const double x1 = x0 + dsm.width * dsm.pixel_x, y0 = y1 + dsm.height * dsm.pixel_y;

    // Reference and difference on the overview grid.
    const tools::Raster       reference(o.reference_dsm);
    const tools::RasterWindow ref_win = reference.read(x0 - 1, y0 - 1, x1 + 1, y1 + 1, 1);
    tools::RasterWindow       ref = dsm, diff = dsm;
    for (int r = 0; r < dsm.height; ++r)
    {
        for (int c = 0; c < dsm.width; ++c)
        {
            const std::size_t           i = static_cast<std::size_t>(r) * dsm.width + c;
            const std::optional<double> h = ref_win.sample(dsm.centreX(c), dsm.centreY(r));
            ref.values[i]  = h ? static_cast<float>(*h) : std::numeric_limits<float>::quiet_NaN();
            diff.values[i] = h && !std::isnan(dsm.values[i]) ? static_cast<float>(dsm.values[i] - *h)
                                                             : std::numeric_limits<float>::quiet_NaN();
        }
    }
    tools::writeGeoTiff((out / "dsm_difference.tif").string(), diff, ours.wkt());

    // Orthomosaic on the same grid.
    cv::Mat ortho(dsm.height, dsm.width, CV_8UC3, cv::Scalar(0, 0, 0));
    if (fs::exists(out / "ortho.vrt"))
    {
        const tools::Raster o_r((out / "ortho.vrt").string());
        const double        o_px = (x1 - x0) / o_r.width();
        (void)o_px;
        std::vector<tools::RasterWindow> band;
        for (int b = 1; b <= 4; ++b)
        {
            band.push_back(o_r.read(x0, y0, x1, y1, std::max(1, o_r.width() / dsm.width), b));
        }
        cv::Mat bgr(band[0].height, band[0].width, CV_8UC3);
        for (int r = 0; r < bgr.rows; ++r)
        {
            for (int c = 0; c < bgr.cols; ++c)
            {
                auto v = [&](int b) { return band[static_cast<std::size_t>(b)].at(c, r); };
                const bool ok = v(3) > 0.0f;
                bgr.at<cv::Vec3b>(r, c) = ok ? cv::Vec3b(cv::saturate_cast<std::uint8_t>(v(2)),
                                                         cv::saturate_cast<std::uint8_t>(v(1)),
                                                         cv::saturate_cast<std::uint8_t>(v(0)))
                                             : cv::Vec3b(0, 0, 0);
            }
        }
        cv::resize(bgr, ortho, cv::Size(dsm.width, dsm.height), 0, 0, cv::INTER_AREA);
    }

    const auto [lo, hi] = tools::robustRange(ref.values.data(), ref.values.size(), -1e9);
    cv::Mat    a = ortho, b = tools::colourize(dsm.values.data(), dsm.width, dsm.height, lo, hi, -1e9),
            c = tools::colourize(ref.values.data(), ref.width, ref.height, lo, hi, -1e9),
            d = tools::diverging(diff.values.data(), diff.width, diff.height, 2.0);
    char text[160];
    tools::caption(a, "OrthoStream orthomosaic");
    std::snprintf(text, sizeof(text), "OrthoStream DSM, %.0f-%.0f m (LHN95)", lo, hi);
    tools::caption(b, text);
    tools::caption(c, "reference DSM (Metashape)");
    tools::caption(d, "OrthoStream - reference, +-2 m (red: higher)");
    tools::writeJpeg((out / "compare_overview.jpg").string(), tools::grid({ a, b, c, d }, 2, 1.0));
    std::cout << "comparison: " << (out / "compare_overview.jpg").string() << ", "
              << (out / "dsm_difference.tif").string() << "\n";
}

int run(const Options &o)
{
    if (o.evaluate_only)
    {
        std::ofstream report(fs::path(o.out) / "evaluation.txt");
        evaluate(o, report);
        writeOverview(o);
        return 0;
    }
    if (fs::exists(o.out) && !fs::is_empty(o.out))
    {
        throw std::runtime_error("output directory is not empty: " + o.out);
    }
    fs::create_directories(o.out);
    tools::Dataset data = tools::loadPoses(o.poses);
    for (const std::string &name : tools::dropMissingImages(data, o.images))
    {
        std::cout << "image missing, left out: " << name << "\n";
    }

    tools::FlightConfig fc;
    fc.proj_data_dir = o.proj_data;
    fc.grid_dir      = o.grid_dir;
    fc.terrain_dem   = o.terrain_dem;
    fc.terrain_crs   = o.terrain_crs;
    fc.sources       = o.sources;
    fc.delayed       = o.delayed;
    // Selection uses its own poses when given (ADR-019 section 2: the
    // telemetry poses), so that a refined block keeps its keyframes and
    // source views.
    tools::Dataset selection = data;
    if (!o.selection_poses.empty())
    {
        selection = tools::loadPoses(o.selection_poses);
        tools::dropMissingImages(selection, o.images);
        bool same = selection.images.size() == data.images.size();
        for (std::size_t i = 0; same && i < data.images.size(); ++i)
        {
            same = selection.images[i].name == data.images[i].name;
        }
        if (!same)
        {
            throw std::runtime_error("--selection-poses must list the same images as --poses");
        }
    }
    const tools::Flight flight(selection, fc);
    // Depth range, terrain gate and footprint follow the poses of the batch
    // (DENSE_APP computes them from the refined poses it receives).
    std::unique_ptr<tools::Flight> refined_geometry;
    if (!o.selection_poses.empty())
    {
        refined_geometry = std::make_unique<tools::Flight>(data, fc);
    }
    const tools::Flight &geometry = refined_geometry ? *refined_geometry : flight;
    const auto         &kf = flight.keyframes();
    const std::size_t   first = std::min<std::size_t>(static_cast<std::size_t>(o.first), kf.size());
    const std::size_t   last  = std::min<std::size_t>(first + static_cast<std::size_t>(o.count), kf.size());
    std::cout << "images " << data.images.size() << ", keyframes " << kf.size() << ", processing " << first << " to "
              << last << "\n";

    // Dense stage.
    imaging::WorkingImageConverter converter(data.camera, o.width, 1);
    const geo::CameraModel         wcam = converter.workingCamera();
    dense::PatchMatchConfig        pm;
    pm.num_iterations = o.iterations;
    pm.window_radius  = o.window_radius;
    pm.window_step    = o.window_step;
    pm.num_samples    = o.samples;
    pm.filter_min_ncc                 = o.filter_min_ncc;
    pm.filter_min_num_consistent      = o.filter_min_consistent;
    pm.filter_min_triangulation_angle = o.filter_min_angle;
    std::unique_ptr<dense::DepthEstimator> estimator;
    if (o.method == "sgbm")
    {
        estimator = std::make_unique<dense::SgbmEstimator>();
    }
    else
    {
        estimator = std::make_unique<dense::PatchMatchEstimator>(pm);
    }

    // Map products.
    const std::vector<std::string> grids { o.grid_dir };
    const mp::ProductFrame         frame(o.product_crs, o.proj_data, grids);
    mp::DsmFusionConfig            dc;
    dc.grid.cell_size  = o.dsm_cell;
    dc.grid.tile_cells = o.tile_cells;
    dc.min_confidence  = o.min_confidence;
    mp::DsmFusion     dsm(dc, frame);
    mp::OrthoRenderer ortho({ o.ortho_pixel }, dsm, frame);
    mp::DsmWriter     dsm_writer({ o.out, o.product_crs, o.vertical_crs, o.proj_data, grids }, frame);
    mp::OrthoWriter   ortho_writer({ o.out, o.product_crs, o.proj_data });
    mp::TileLifecycleConfig lc;
    lc.max_active_tiles  = o.max_tiles;
    lc.finalise_distance = o.finalise_distance;
    lc.state_directory   = (fs::path(o.out) / "state").string();
    mp::TileManager manager(lc, dsm, ortho, dsm_writer, ortho_writer);
    std::unique_ptr<mp::PointCloudWriter> points;
    if (o.points)
    {
        mp::PointCloudWriterConfig pc;
        pc.directory      = o.out;
        pc.projected_crs  = o.product_crs;
        pc.vertical_crs   = o.vertical_crs;
        pc.proj_data_dir  = o.proj_data;
        pc.grid_dirs      = grids;
        pc.grid           = dc.grid;
        pc.min_confidence = static_cast<float>(o.min_confidence);
        points            = std::make_unique<mp::PointCloudWriter>(pc, frame);
    }

    // Orthomosaic image of a keyframe: DCT-scaled RGB, full camera model.
    imaging::JpegDecoder decoder;
    const int            ortho_width = o.ortho_width > 0 ? o.ortho_width : data.camera.width / 2;

    std::ofstream csv(fs::path(o.out) / "timings.csv");
    csv << "keyframe,image,sources,decode_s,dense_s,gate_s,fusion_s,ortho_decode_s,ortho_s,points_s,lifecycle_s,total_s,"
           "valid_fraction,gated,fused,rejected,tiles,active_tiles\n";
    std::map<std::size_t, std::vector<std::uint8_t>> cache;  // working images of the current batch
    const std::size_t                                n = static_cast<std::size_t>(wcam.width) * wcam.height;
    std::vector<float>                               depth(n), normals(3 * n), confidence(n);
    for (std::size_t q = first; q < last; ++q)
    {
        const auto        t_start = Clock::now();
        const std::size_t k       = kf[q];
        const auto        sources = flight.sources(k);
        if (sources.empty())
        {
            std::cout << data.images[k].name << ": no source views, skipped\n";
            continue;
        }

        // Working images (grey), cached within the batch sequence.
        auto t0 = Clock::now();
        std::map<std::size_t, std::vector<std::uint8_t>> next;
        auto working = [&](std::size_t i) -> const std::vector<std::uint8_t> &
        {
            if (auto it = cache.find(i); it != cache.end())
            {
                return next.emplace(i, std::move(it->second)).first->second;
            }
            const std::vector<std::uint8_t> jpeg = readFile(o.images + "/" + data.images[i].name);
            std::vector<std::uint8_t>       img(n);
            converter.convert(jpeg.data(), jpeg.size(), { img.data(), wcam.width, wcam.height, 1,
                                                          static_cast<std::size_t>(wcam.width) });
            return next.emplace(i, std::move(img)).first->second;
        };
        const geo::Geodetic g = geo::GeodeticConverter(o.proj_data).ecefToGeodetic(data.images[k].T_E_C.translation());
        const geo::LocalFrame L(data.images[k].T_E_C.translation(), g.latitude_rad, g.longitude_rad);
        dense::DepthBatch     batch;
        batch.camera = wcam;
        std::tie(batch.depth_min, batch.depth_max) = geometry.depthRange(wcam, k, o.depth_margin);
        batch.reference.T_L_C = L.poseInLocal(data.images[k].T_E_C);
        batch.reference.image = { working(k).data(), static_cast<std::size_t>(wcam.width) };
        for (const auto &s : sources)
        {
            dense::BatchView v;
            v.T_L_C = L.poseInLocal(data.images[s.id].T_E_C);
            v.image = { working(s.id).data(), static_cast<std::size_t>(wcam.width) };
            batch.sources.push_back(v);
        }
        cache.swap(next);
        const double t_decode = since(t0);

        t0 = Clock::now();
        estimator->estimate(batch, { depth.data(), normals.data(), confidence.data() });
        const double t_dense = since(t0);
        t0                   = Clock::now();
        std::size_t gated    = 0;
        if (o.terrain_gate > 0.0)
        {
            gated = dense::applyTerrainGate(batch, geometry.heightGrid(k, L), o.terrain_gate,
                                            { depth.data(), normals.data(), confidence.data() });
        }
        const double t_gate = since(t0);
        const double valid =
            static_cast<double>(std::count_if(depth.begin(), depth.end(), [](float d) { return d > 0.0f; }))
            / static_cast<double>(n);

        t0 = Clock::now();
        const mp::FusionStats fused = dsm.addDepthMap({ depth.data(), confidence.data() }, wcam, data.images[k].T_E_C);
        const double          t_fusion = since(t0);

        t0 = Clock::now();
        const std::vector<std::uint8_t> jpeg  = readFile(o.images + "/" + data.images[k].name);
        const imaging::JpegInfo         info  = decoder.readHeader(jpeg.data(), jpeg.size());
        const imaging::DctScale         scale = imaging::chooseDctScale(info, ortho_width, 0);
        std::vector<std::uint8_t>       rgb(static_cast<std::size_t>(3 * scale.width * scale.height));
        decoder.decode(jpeg.data(), jpeg.size(), scale,
                       { rgb.data(), scale.width, scale.height, 3, static_cast<std::size_t>(3 * scale.width) });
        const geo::CameraModel   icam = geo::scaledCamera(data.camera, scale.width, scale.height);
        const mp::RgbImageView   view { rgb.data(), scale.width, scale.height, static_cast<std::size_t>(3 * scale.width) };
        const double             t_ortho_decode = since(t0);

        // Render around the keyframe's footprint only: outliers can stretch
        // the extent of the fused points far beyond it.
        Eigen::AlignedBox2d footprint;
        for (const Eigen::Vector3d &c : geometry.view(k).footprint.corners)
        {
            footprint.extend(frame.fromEcef(geometry.frame().toEcef(c)).head<2>());
        }
        footprint.min().array() -= 30.0;
        footprint.max().array() += 30.0;
        const Eigen::AlignedBox2d render_extent = footprint.intersection(fused.extent);

        t0 = Clock::now();
        const mp::RenderStats drawn = ortho.render(view, icam, data.images[k].T_E_C, render_extent);
        const double          t_ortho = since(t0);

        t0 = Clock::now();
        if (points)
        {
            points->write(static_cast<std::uint32_t>(k), { depth.data(), confidence.data() }, wcam, view, icam,
                          data.images[k].T_E_C);
        }
        const double t_points = since(t0);

        t0 = Clock::now();
        manager.markChanged(fused.tiles);
        manager.markChanged(drawn.tiles);
        manager.update(frame.fromEcef(data.images[k].T_E_C.translation()).head<2>());
        const double t_life  = since(t0);
        const double t_total = since(t_start);

        csv << q << ',' << data.images[k].name << ',' << sources.size() << ',' << t_decode << ',' << t_dense << ','
            << t_gate << ',' << t_fusion << ',' << t_ortho_decode << ',' << t_ortho << ',' << t_points << ',' << t_life << ','
            << t_total << ',' << valid << ',' << gated << ',' << fused.added << ',' << fused.rejected << ',' << fused.tiles.size()
            << ',' << manager.activeTiles() << '\n';
        csv.flush();
        char line[256];
        std::snprintf(line, sizeof(line),
                      "%3zu %s  dense %5.2f s  fusion %4.2f s  ortho %4.2f s (+%4.2f decode)  points %4.2f s  "
                      "total %5.2f s  valid %.2f  tiles %zu\n",
                      q, data.images[k].name.c_str(), t_dense, t_fusion, t_ortho, t_ortho_decode, t_points, t_total,
                      valid, manager.activeTiles());
        std::cout << line << std::flush;
    }
    manager.finaliseAll();

    if (!o.reference_dsm.empty())
    {
        std::ofstream report(fs::path(o.out) / "evaluation.txt");
        evaluate(o, report);
        writeOverview(o);
    }
    return 0;
}

}  // namespace

int main(int argc, char **argv)
{
    try
    {
        return run(parse(argc, argv));
    }
    catch (const std::exception &e)
    {
        std::cerr << "offline_pipeline: " << e.what() << "\n";
        return 1;
    }
}
