#include "dataset.hpp"

#include <algorithm>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace orthostream::bench
{

Dataset loadPoses(const std::string &path)
{
    std::ifstream in(path);
    if (!in)
    {
        throw std::runtime_error("cannot open " + path);
    }
    Dataset     data;
    bool        have_camera = false;
    std::string line;
    int         line_no = 0;
    while (std::getline(in, line))
    {
        ++line_no;
        if (line.empty() || line[0] == '#')
        {
            continue;
        }
        std::istringstream s(line);
        std::string        kind;
        s >> kind;
        if (kind == "camera")
        {
            geo::CameraModel &c = data.camera;
            s >> c.width >> c.height >> c.fx >> c.fy >> c.cx >> c.cy >> c.k1 >> c.k2 >> c.k3 >> c.p1 >> c.p2;
            have_camera = static_cast<bool>(s);
        }
        else if (kind == "pose")
        {
            PosedImage image;
            double     x, y, z, qw, qx, qy, qz;
            s >> image.name >> x >> y >> z >> qw >> qx >> qy >> qz;
            if (!s)
            {
                throw std::runtime_error(path + ":" + std::to_string(line_no) + ": malformed pose");
            }
            image.T_E_C               = Eigen::Isometry3d::Identity();
            image.T_E_C.linear()      = Eigen::Quaterniond(qw, qx, qy, qz).normalized().toRotationMatrix();
            image.T_E_C.translation() = Eigen::Vector3d(x, y, z);
            data.images.push_back(image);
        }
        else
        {
            throw std::runtime_error(path + ":" + std::to_string(line_no) + ": unknown record " + kind);
        }
    }
    if (!have_camera || data.images.empty())
    {
        throw std::runtime_error(path + ": camera or poses missing");
    }
    std::sort(data.images.begin(), data.images.end(),
              [](const PosedImage &a, const PosedImage &b) { return a.name < b.name; });
    return data;
}

}  // namespace orthostream::bench
