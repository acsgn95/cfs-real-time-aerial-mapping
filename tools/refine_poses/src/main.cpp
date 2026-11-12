// refine_poses: pose refinement (ADR-019) over a block of a recorded flight,
// without cFS. Selects keyframes and source views as the mission does
// (ADR-006), extracts features from the working images, matches the images
// of every batch with each other and adjusts all poses of the block
// together. Writes a pose file in the format of the input, with the refined
// poses replacing those of the block, so that offline_pipeline can use it
// unchanged. With --reference-poses, reports the pose errors before and
// after refinement.
//
// This is block mode: all batches of the block in one adjustment, the upper
// bound of what the sliding window of ADR-019 section 3 can reach.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "dataset.hpp"
#include "flight.hpp"
#include "orthostream/geo/crs.hpp"
#include "orthostream/geo/frames.hpp"
#include "orthostream/imaging/working_image.hpp"
#include "orthostream/refine/refine.hpp"

namespace
{

using namespace orthostream;
using Clock = std::chrono::steady_clock;

double since(Clock::time_point t0)
{
    return std::chrono::duration<double>(Clock::now() - t0).count();
}

struct Options
{
    std::string poses, images, terrain_dem, out, reference_poses;
    std::string terrain_crs = "EPSG:4326+3855";
    std::string proj_data   = OST_TOOLS_PROJ_DATA_DIR;
    std::string grid_dir    = OST_TOOLS_GRID_DIR;
    int         first       = 0;
    int         count       = 40;
    int         width       = 2048;
    int         sources     = 5;
    bool        delayed     = true;
    refine::RefineConfig refine;
};

const char *kUsage = R"(usage: refine_poses --poses FILE --images DIR --terrain-dem FILE --out FILE [options]

  --poses FILE            prior poses (normally the non-optimized poses)
  --images DIR            image directory
  --terrain-dem FILE      terrain prior for the selection (ADR-006)
  --terrain-crs CRS       its CRS with heights (default EPSG:4326+3855)
  --out FILE              refined pose file (all images; the block refined)
  --reference-poses FILE  report pose errors against these poses (Metashape)
  --first N, --count N    keyframes of the block (default 0, 40)
  --width N               working image width (default 2048)
  --sources N             source views per keyframe (default 5)
  --mode causal|delayed   source views (default delayed)
  --precise-position      PPK/RTK positions: held by a tight prior
  --position-sigma M      position prior, m (default 2; precise: 0.02)
  --attitude-sigma DEG    attitude prior per image (default 3)
  --no-boresight          no shared mounting rotation
  --max-features N        features per image (default 4000)
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
        else if (a == "--images") o.images = value();
        else if (a == "--terrain-dem") o.terrain_dem = value();
        else if (a == "--terrain-crs") o.terrain_crs = value();
        else if (a == "--out") o.out = value();
        else if (a == "--reference-poses") o.reference_poses = value();
        else if (a == "--first") o.first = std::stoi(value());
        else if (a == "--count") o.count = std::stoi(value());
        else if (a == "--width") o.width = std::stoi(value());
        else if (a == "--sources") o.sources = std::stoi(value());
        else if (a == "--precise-position") o.refine.precise_position = true;
        else if (a == "--position-sigma")
        {
            const double s = std::stod(value());
            o.refine.position_sigma = o.refine.precise_position_sigma = s;
        }
        else if (a == "--attitude-sigma") o.refine.attitude_sigma_deg = std::stod(value());
        else if (a == "--no-boresight") o.refine.estimate_boresight = false;
        else if (a == "--max-features") o.refine.max_features = std::stoi(value());
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
            throw std::runtime_error("unknown option " + a + "\n" + kUsage);
        }
    }
    if (o.poses.empty() || o.images.empty() || o.terrain_dem.empty() || o.out.empty())
    {
        throw std::runtime_error(std::string("missing required option\n") + kUsage);
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

double angleDeg(const Eigen::Matrix3d &A, const Eigen::Matrix3d &B)
{
    return std::acos(std::clamp(((A.transpose() * B).trace() - 1.0) / 2.0, -1.0, 1.0)) * 180.0 / M_PI;
}

struct Summary
{
    double median = 0, p95 = 0, max = 0, rms = 0;
};

Summary summarise(std::vector<double> v)
{
    Summary s;
    if (v.empty())
    {
        return s;
    }
    std::sort(v.begin(), v.end());
    s.median = v[v.size() / 2];
    s.p95    = v[static_cast<std::size_t>(0.95 * static_cast<double>(v.size() - 1))];
    s.max    = v.back();
    double sq = 0;
    for (double x : v)
    {
        sq += x * x;
    }
    s.rms = std::sqrt(sq / static_cast<double>(v.size()));
    return s;
}

/// Pose errors of a set of poses (ECEF) against reference poses, in the
/// local ENU frame: attitude angle and position components.
void reportErrors(std::ostream &out, const char *label, const std::vector<std::string> &names,
                  const std::vector<Eigen::Isometry3d> &T_E_C, const std::map<std::string, Eigen::Isometry3d> &ref,
                  const geo::LocalFrame &L)
{
    std::vector<double> att, horiz, up, dist;
    Eigen::Vector3d     mean = Eigen::Vector3d::Zero();
    for (std::size_t i = 0; i < names.size(); ++i)
    {
        const auto it = ref.find(names[i]);
        if (it == ref.end())
        {
            continue;
        }
        att.push_back(angleDeg(T_E_C[i].linear(), it->second.linear()));
        const Eigen::Vector3d d = L.toLocal(T_E_C[i].translation()) - L.toLocal(it->second.translation());
        mean += d;
        horiz.push_back(d.head<2>().norm());
        up.push_back(std::abs(d.z()));
        dist.push_back(d.norm());
    }
    if (att.empty())
    {
        out << label << ": no image in the reference\n";
        return;
    }
    mean /= static_cast<double>(att.size());
    const Summary a = summarise(att), h = summarise(horiz), u = summarise(up);
    out << std::fixed << std::setprecision(3);
    out << label << " (" << att.size() << " images)\n"
        << "  attitude  median " << a.median << " deg, 95% " << a.p95 << ", max " << a.max << ", RMS " << a.rms << "\n"
        << "  position  mean E/N/U " << std::showpos << mean.x() << " " << mean.y() << " " << mean.z()
        << std::noshowpos << " m; horizontal median " << h.median << ", max " << h.max << "; vertical median "
        << u.median << ", max " << u.max << "\n";
}

void writePoses(const std::string &path, const tools::Dataset &data, const std::string &source,
                const std::string &note)
{
    std::ofstream f(path);
    if (!f)
    {
        throw std::runtime_error("cannot write " + path);
    }
    const geo::CameraModel &c = data.camera;
    f << "# Camera poses refined by tools/refine_poses (ADR-019); do not edit.\n"
      << "# Prior poses: " << source << "\n# " << note << "\n#\n"
      << "# camera width height fx fy cx cy k1 k2 k3 p1 p2\n"
      << "# pose image x y z qw qx qy qz\n"
      << "#   camera centre in ECEF (m) and q_E_C (camera to ECEF), Hamilton, w >= 0.\n";
    f << std::setprecision(17) << "camera " << c.width << ' ' << c.height << ' ' << c.fx << ' ' << c.fy << ' ' << c.cx
      << ' ' << c.cy << ' ' << c.k1 << ' ' << c.k2 << ' ' << c.k3 << ' ' << c.p1 << ' ' << c.p2 << "\n";
    for (const auto &img : data.images)
    {
        const Eigen::Quaterniond q = geo::canonicalQuaternion(Eigen::Quaterniond(img.T_E_C.linear()));
        const Eigen::Vector3d    t = img.T_E_C.translation();
        f << "pose " << img.name << std::fixed << std::setprecision(4) << ' ' << t.x() << ' ' << t.y() << ' ' << t.z()
          << std::setprecision(12) << ' ' << q.w() << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << "\n";
        f << std::defaultfloat;
    }
}

int run(const Options &o)
{
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
    const tools::Flight flight(data, fc);
    const auto         &kf    = flight.keyframes();
    const std::size_t   first = std::min<std::size_t>(static_cast<std::size_t>(o.first), kf.size());
    const std::size_t   last  = std::min<std::size_t>(first + static_cast<std::size_t>(o.count), kf.size());
    if (first >= last)
    {
        throw std::runtime_error("no keyframes in the block");
    }

    // Images of the block and the pairs to match: every pair within a batch
    // (keyframe and its source views).
    std::vector<std::vector<std::size_t>> batches;
    std::set<std::size_t>                 members;
    for (std::size_t q = first; q < last; ++q)
    {
        std::vector<std::size_t> b { kf[q] };
        for (const auto &s : flight.sources(kf[q]))
        {
            b.push_back(static_cast<std::size_t>(s.id));
        }
        members.insert(b.begin(), b.end());
        batches.push_back(b);
    }
    const std::vector<std::size_t> block(members.begin(), members.end());
    std::map<std::size_t, std::size_t> index;  // dataset index -> refiner index
    for (std::size_t i = 0; i < block.size(); ++i)
    {
        index[block[i]] = i;
    }
    std::set<std::pair<std::size_t, std::size_t>> pair_set;
    for (const auto &b : batches)
    {
        for (std::size_t x = 0; x < b.size(); ++x)
        {
            for (std::size_t y = x + 1; y < b.size(); ++y)
            {
                const std::size_t p = index[b[x]], q = index[b[y]];
                pair_set.emplace(std::min(p, q), std::max(p, q));
            }
        }
    }
    const std::vector<std::pair<std::size_t, std::size_t>> pairs(pair_set.begin(), pair_set.end());
    std::cout << "keyframes " << first << " to " << last << ": " << block.size() << " images, " << pairs.size()
              << " pairs\n";

    // Local working frame at the first keyframe (ADR-004 section 4).
    const Eigen::Vector3d origin = data.images[kf[first]].T_E_C.translation();
    const geo::Geodetic   g      = geo::GeodeticConverter(o.proj_data).ecefToGeodetic(origin);
    const geo::LocalFrame L(origin, g.latitude_rad, g.longitude_rad);

    imaging::WorkingImageConverter converter(data.camera, o.width, 1);
    const geo::CameraModel         wcam = converter.workingCamera();
    refine::Refiner                refiner(wcam, o.refine);
    std::vector<std::uint8_t>      img(static_cast<std::size_t>(wcam.width) * wcam.height);
    double                         decode_s = 0.0;
    for (std::size_t i : block)
    {
        const auto                      t0   = Clock::now();
        const std::vector<std::uint8_t> jpeg = readFile(o.images + "/" + data.images[i].name);
        converter.convert(jpeg.data(), jpeg.size(),
                          { img.data(), wcam.width, wcam.height, 1, static_cast<std::size_t>(wcam.width) });
        decode_s += since(t0);
        refiner.addImage({ img.data(), wcam.width, wcam.height, static_cast<std::size_t>(wcam.width) },
                         L.poseInLocal(data.images[i].T_E_C));
    }
    refiner.addPairs(pairs);
    const refine::RefineResult r = refiner.solve();
    const auto                &a = r.adjustment;

    std::ostringstream rep;
    rep << std::fixed << std::setprecision(3);
    rep << "refine_poses: " << o.poses << "\n"
        << "block: keyframes " << first << " to " << last << ", " << block.size() << " images, " << pairs.size()
        << " pairs\n"
        << "settings: precise_position " << (o.refine.precise_position ? "true" : "false") << ", position sigma "
        << (o.refine.precise_position ? o.refine.precise_position_sigma : o.refine.position_sigma)
        << " m, attitude sigma " << o.refine.attitude_sigma_deg << " deg, boresight "
        << (o.refine.estimate_boresight ? "estimated" : "off") << ", features " << o.refine.max_features
        << ", width " << wcam.width << "\n"
        << "result: " << (r.source == refine::PoseSource::refined ? "refined" : "refine_failed")
        << (r.reason.empty() ? "" : " (" + r.reason + ")") << "\n"
        << "pairs used " << r.pairs_used << " of " << r.pairs_tried << ", tracks " << r.tracks << ", points "
        << a.points << ", observations " << a.observations << ", rejected " << a.rejected << "\n"
        << "reprojection RMS " << a.rms_before_px << " px at the priors, " << a.rms_after_px << " px refined\n";
    const Eigen::AngleAxisd bore(a.boresight);
    rep << "boresight " << bore.angle() * 180.0 / M_PI << " deg about camera axis (" << bore.axis().x() << ", "
        << bore.axis().y() << ", " << bore.axis().z() << ")\n"
        << "largest correction per image: " << a.max_rotation_correction_deg << " deg (boresight excluded), "
        << a.max_position_correction_m << " m\n";
    std::size_t min_obs = a.observations_per_image.empty() ? 0 : *std::min_element(a.observations_per_image.begin(),
                                                                                     a.observations_per_image.end());
    rep << "observations per image: min " << min_obs << "\n"
        << "time: decode " << decode_s << " s, features " << r.features_s << " s, matching " << r.matching_s
        << " s, adjustment " << r.adjustment_s << " s\n";

    // Replace the poses of the block (refined, or the priors if refinement
    // failed) and write the pose file.
    std::vector<std::string>       names;
    std::vector<Eigen::Isometry3d> prior_E, refined_E;
    for (std::size_t i = 0; i < block.size(); ++i)
    {
        names.push_back(data.images[block[i]].name);
        prior_E.push_back(data.images[block[i]].T_E_C);
        refined_E.push_back(a.T_W_C.empty() ? data.images[block[i]].T_E_C : L.ecefFromLocal() * a.T_W_C[i]);
    }
    if (r.source == refine::PoseSource::refined)
    {
        for (std::size_t i = 0; i < block.size(); ++i)
        {
            data.images[block[i]].T_E_C = L.ecefFromLocal() * r.T_W_C[i];
        }
    }
    if (!o.reference_poses.empty())
    {
        const tools::Dataset                     ref_data = tools::loadPoses(o.reference_poses);
        std::map<std::string, Eigen::Isometry3d> ref;
        for (const auto &im : ref_data.images)
        {
            ref[im.name] = im.T_E_C;
        }
        reportErrors(rep, "prior poses against the reference", names, prior_E, ref, L);
        if (!a.T_W_C.empty())
        {
            reportErrors(rep, "refined poses against the reference", names, refined_E, ref, L);
        }
    }
    std::ostringstream note;
    note << "Block keyframes " << first << " to " << last << " (" << block.size() << " images): "
         << (r.source == refine::PoseSource::refined ? "refined" : "refine_failed, priors kept")
         << "; other images unchanged.";
    writePoses(o.out, data, o.poses, note.str());
    std::ofstream(o.out + ".report.txt") << rep.str();
    std::cout << rep.str() << "poses: " << o.out << "\n";
    return r.source == refine::PoseSource::refined ? 0 : 2;
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
        std::cerr << "refine_poses: " << e.what() << "\n";
        return 1;
    }
}
