// Camera and poses written by tools/dataset/extract_metashape_poses.py.

#ifndef OST_TOOLS_DATASET_HPP
#define OST_TOOLS_DATASET_HPP

#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "orthostream/geo/camera.hpp"

namespace orthostream::tools
{

struct PosedImage
{
    std::string       name;
    Eigen::Isometry3d T_E_C;  ///< camera to ECEF
};

struct Dataset
{
    geo::CameraModel        camera;
    std::vector<PosedImage> images;  ///< sorted by name, which is the capture order
};

/// Reads "camera W H fx fy cx cy k1 k2 k3 p1 p2" and
/// "pose name x y z qw qx qy qz" lines; '#' starts a comment line. Throws
/// std::runtime_error on malformed input.
Dataset loadPoses(const std::string &path);

/// Removes the images whose file is missing in the image directory and
/// returns their names, so that tools can report them.
std::vector<std::string> dropMissingImages(Dataset &data, const std::string &image_dir);

}  // namespace orthostream::tools

#endif  // OST_TOOLS_DATASET_HPP
