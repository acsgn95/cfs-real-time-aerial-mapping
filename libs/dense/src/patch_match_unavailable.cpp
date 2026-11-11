// PatchMatch in a build without COLMAP (ORTHOSTREAM_WITH_COLMAP=OFF):
// reports "not available" (ADR-011 section 5).

#include "orthostream/dense/patch_match.hpp"

namespace orthostream::dense
{

bool patchMatchAvailable()
{
    return false;
}

PatchMatchEstimator::PatchMatchEstimator(const PatchMatchConfig &config) : config_(config)
{
    throw NotAvailableError("PatchMatch is not available: built without COLMAP");
}

void PatchMatchEstimator::estimate(const DepthBatch &, const DepthOutput &)
{
    throw NotAvailableError("PatchMatch is not available: built without COLMAP");
}

}  // namespace orthostream::dense
