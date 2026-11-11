// Reader for tests/data/dorfberg_gcp.txt, written by
// tools/fixtures/extract_dorfberg_gcp.py (format described in the file).

#ifndef ORTHOSTREAM_GEO_TESTS_DORFBERG_FIXTURE_HPP
#define ORTHOSTREAM_GEO_TESTS_DORFBERG_FIXTURE_HPP

#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "orthostream/geo/camera.hpp"
#include "orthostream/geo/metashape.hpp"

namespace orthostream::geo::test
{

struct ReportCheckPoint
{
    double image_error_px = 0.0;
    double easting_cm     = 0.0;
    double northing_cm    = 0.0;
    double altitude_cm    = 0.0;
};

struct MetashapeCamera
{
    Eigen::Matrix3d rotation;     ///< camera to chunk
    Eigen::Vector3d translation;  ///< camera centre in the chunk frame
};

struct MarkerObservation
{
    std::string     gcp;
    std::string     camera;
    Eigen::Vector2d metashape_pixel;  ///< Metashape convention (top-left pixel centre at 0.5)
};

struct DorfbergFixture
{
    MetashapeCalibration                    calibration;
    MetashapeChunkTransform                 chunk;
    std::map<std::string, Eigen::Vector3d>  gcp_ecef;
    std::map<std::string, ReportCheckPoint> report;
    std::map<std::string, MetashapeCamera>  cameras;
    std::vector<MarkerObservation>          markers;
};

inline DorfbergFixture loadDorfbergFixture(const std::string &path)
{
    std::ifstream in(path);
    if (!in)
    {
        throw std::runtime_error("cannot open fixture " + path);
    }
    DorfbergFixture fx;
    std::string     line;
    while (std::getline(in, line))
    {
        if (line.empty() || line[0] == '#')
        {
            continue;
        }
        std::istringstream ss(line);
        std::string        kind;
        ss >> kind;
        if (kind == "metashape_calibration")
        {
            MetashapeCalibration &c = fx.calibration;
            ss >> c.width >> c.height >> c.f >> c.cx >> c.cy >> c.k1 >> c.k2 >> c.k3 >> c.p1 >> c.p2;
        }
        else if (kind == "chunk_transform")
        {
            ss >> fx.chunk.scale;
            for (int i = 0; i < 9; ++i)
            {
                ss >> fx.chunk.rotation(i / 3, i % 3);
            }
            ss >> fx.chunk.translation.x() >> fx.chunk.translation.y() >> fx.chunk.translation.z();
        }
        else if (kind == "towgs84")
        {
            ss >> fx.chunk.datum_shift.x() >> fx.chunk.datum_shift.y() >> fx.chunk.datum_shift.z();
        }
        else if (kind == "gcp")
        {
            std::string     label;
            Eigen::Vector3d p;
            ss >> label >> p.x() >> p.y() >> p.z();
            fx.gcp_ecef[label] = p;
        }
        else if (kind == "report")
        {
            std::string      label;
            ReportCheckPoint r;
            ss >> label >> r.image_error_px >> r.easting_cm >> r.northing_cm >> r.altitude_cm;
            fx.report[label] = r;
        }
        else if (kind == "camera")
        {
            std::string     label;
            MetashapeCamera cam;
            ss >> label;
            for (int i = 0; i < 9; ++i)
            {
                ss >> cam.rotation(i / 3, i % 3);
            }
            ss >> cam.translation.x() >> cam.translation.y() >> cam.translation.z();
            fx.cameras[label] = cam;
        }
        else if (kind == "marker")
        {
            MarkerObservation m;
            ss >> m.gcp >> m.camera >> m.metashape_pixel.x() >> m.metashape_pixel.y();
            fx.markers.push_back(m);
        }
        else
        {
            throw std::runtime_error("unknown fixture record: " + kind);
        }
        if (ss.fail())
        {
            throw std::runtime_error("malformed fixture line: " + line);
        }
    }
    return fx;
}

}  // namespace orthostream::geo::test

#endif  // ORTHOSTREAM_GEO_TESTS_DORFBERG_FIXTURE_HPP
