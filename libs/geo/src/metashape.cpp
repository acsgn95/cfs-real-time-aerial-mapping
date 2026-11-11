#include "orthostream/geo/metashape.hpp"

namespace orthostream::geo
{

Eigen::Vector3d chunkToEcef(const MetashapeChunkTransform &chunk, const Eigen::Vector3d &p_chunk)
{
    return chunk.scale * (chunk.rotation * p_chunk) + chunk.translation + chunk.datum_shift;
}

Eigen::Vector3d ecefToChunk(const MetashapeChunkTransform &chunk, const Eigen::Vector3d &p_E)
{
    return chunk.rotation.transpose() * (p_E - chunk.datum_shift - chunk.translation) / chunk.scale;
}

Eigen::Isometry3d cameraPoseEcef(const MetashapeChunkTransform &chunk,
                                 const Eigen::Matrix3d         &rotation_chunk_camera,
                                 const Eigen::Vector3d         &translation_chunk_camera)
{
    // The scale of the chunk transform applies to positions only; rotations
    // compose directly.
    Eigen::Isometry3d T_E_C = Eigen::Isometry3d::Identity();
    T_E_C.linear()          = chunk.rotation * rotation_chunk_camera;
    T_E_C.translation()     = chunkToEcef(chunk, translation_chunk_camera);
    return T_E_C;
}

}  // namespace orthostream::geo
