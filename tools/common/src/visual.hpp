// Comparison images for the host tools: colour-coded depth and height maps,
// difference maps and labelled panel grids, written as JPEG.

#ifndef OST_TOOLS_VISUAL_HPP
#define OST_TOOLS_VISUAL_HPP

#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>

namespace orthostream::tools
{

/// Values to colours (turbo map) between lo and hi; values that are not
/// finite or are <= invalid_below are black.
cv::Mat colourize(const float *values, int width, int height, double lo, double hi, double invalid_below = 0.0);

/// Differences to blue (negative), white (zero) and red (positive),
/// saturated at +-limit; values that are not finite are grey.
cv::Mat diverging(const float *values, int width, int height, double limit);

/// 2nd and 98th percentiles of the finite values above invalid_below.
std::pair<double, double> robustRange(const float *values, std::size_t n, double invalid_below = 0.0);

/// Draws a caption with a dark band at the top of an image.
void caption(cv::Mat &image, const std::string &text);

/// Panels of equal size in a grid of the given number of columns, each
/// scaled by the factor.
cv::Mat grid(const std::vector<cv::Mat> &panels, int columns, double scale);

void writeJpeg(const std::string &path, const cv::Mat &image, int quality = 90);

}  // namespace orthostream::tools

#endif  // OST_TOOLS_VISUAL_HPP
