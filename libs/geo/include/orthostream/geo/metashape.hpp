// Agisoft Metashape project frames (ADR-008 section 3).
//
// A Metashape chunk stores camera poses in its own chunk frame. The chunk
// transform maps that frame to the geocentric frame of the project CRS's
// datum: X = scale * R * x + t. For a project in CH1903+ / LV95 this is the
// CH1903+ (Bessel) geocentric frame, not WGS 84; the datum shift (the TOWGS84
// translation of the project CRS) completes the step to ETRS89 / WGS 84 ECEF.

#ifndef ORTHOSTREAM_GEO_METASHAPE_HPP
#define ORTHOSTREAM_GEO_METASHAPE_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace orthostream::geo
{

struct MetashapeChunkTransform
{
    double          scale = 1.0;
    Eigen::Matrix3d rotation    = Eigen::Matrix3d::Identity();
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();
    /// Translation from the datum's geocentric frame to WGS 84 ECEF (zero if
    /// the project CRS is already based on WGS 84).
    Eigen::Vector3d datum_shift = Eigen::Vector3d::Zero();
};

/// Chunk frame point to WGS 84 ECEF.
Eigen::Vector3d chunkToEcef(const MetashapeChunkTransform &chunk, const Eigen::Vector3d &p_chunk);

/// WGS 84 ECEF point to the chunk frame.
Eigen::Vector3d ecefToChunk(const MetashapeChunkTransform &chunk, const Eigen::Vector3d &p_E);

/// Camera pose T_E_C from a Metashape camera transform (camera to chunk:
/// rotation and translation). Metashape's camera axes are those of OpenCV.
Eigen::Isometry3d cameraPoseEcef(const MetashapeChunkTransform &chunk,
                                 const Eigen::Matrix3d         &rotation_chunk_camera,
                                 const Eigen::Vector3d         &translation_chunk_camera);

}  // namespace orthostream::geo

#endif  // ORTHOSTREAM_GEO_METASHAPE_HPP
