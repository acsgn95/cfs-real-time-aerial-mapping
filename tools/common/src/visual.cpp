#include "visual.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace orthostream::tools
{

cv::Mat colourize(const float *values, int width, int height, double lo, double hi, double invalid_below)
{
    cv::Mat grey(height, width, CV_8UC1);
    cv::Mat valid(height, width, CV_8UC1);
    const double span = hi > lo ? hi - lo : 1.0;
    for (int r = 0; r < height; ++r)
    {
        for (int c = 0; c < width; ++c)
        {
            const float v  = values[static_cast<std::size_t>(r) * width + c];
            const bool  ok = std::isfinite(v) && v > invalid_below;
            valid.at<std::uint8_t>(r, c) = ok ? 255 : 0;
            grey.at<std::uint8_t>(r, c) =
                ok ? static_cast<std::uint8_t>(std::lround(std::clamp((v - lo) / span, 0.0, 1.0) * 255.0)) : 0;
        }
    }
    cv::Mat colour;
    cv::applyColorMap(grey, colour, cv::COLORMAP_TURBO);
    colour.setTo(cv::Scalar(0, 0, 0), valid == 0);
    return colour;
}

cv::Mat diverging(const float *values, int width, int height, double limit)
{
    cv::Mat out(height, width, CV_8UC3);
    for (int r = 0; r < height; ++r)
    {
        for (int c = 0; c < width; ++c)
        {
            const float v = values[static_cast<std::size_t>(r) * width + c];
            cv::Vec3b  &p = out.at<cv::Vec3b>(r, c);
            if (!std::isfinite(v))
            {
                p = cv::Vec3b(60, 60, 60);
                continue;
            }
            const double t    = std::clamp(v / limit, -1.0, 1.0);
            const auto   fade = static_cast<std::uint8_t>(std::lround(255.0 * (1.0 - std::abs(t))));
            p = t >= 0 ? cv::Vec3b(fade, fade, 255) : cv::Vec3b(255, fade, fade);  // BGR: red positive, blue negative
        }
    }
    return out;
}

std::pair<double, double> robustRange(const float *values, std::size_t n, double invalid_below)
{
    std::vector<float> v;
    v.reserve(n);
    for (std::size_t i = 0; i < n; ++i)
    {
        if (std::isfinite(values[i]) && values[i] > invalid_below)
        {
            v.push_back(values[i]);
        }
    }
    if (v.empty())
    {
        return { 0.0, 1.0 };
    }
    auto q = [&](double f)
    {
        const std::size_t k = static_cast<std::size_t>(f * static_cast<double>(v.size() - 1));
        std::nth_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(k), v.end());
        return static_cast<double>(v[k]);
    };
    return { q(0.02), q(0.98) };
}

void caption(cv::Mat &image, const std::string &text)
{
    const double scale = std::max(0.5, image.cols / 1400.0);
    const int    band  = static_cast<int>(std::lround(36 * scale));
    cv::rectangle(image, cv::Rect(0, 0, image.cols, band), cv::Scalar(0, 0, 0), cv::FILLED);
    cv::putText(image, text, cv::Point(static_cast<int>(10 * scale), static_cast<int>(26 * scale)),
                cv::FONT_HERSHEY_SIMPLEX, 0.8 * scale, cv::Scalar(255, 255, 255), std::max(1, static_cast<int>(2 * scale)),
                cv::LINE_AA);
}

cv::Mat grid(const std::vector<cv::Mat> &panels, int columns, double scale)
{
    if (panels.empty() || columns <= 0)
    {
        throw std::runtime_error("no panels");
    }
    std::vector<cv::Mat> scaled;
    for (const cv::Mat &p : panels)
    {
        cv::Mat s, bgr;
        cv::resize(p, s, cv::Size(), scale, scale, cv::INTER_AREA);
        if (s.channels() == 1)
        {
            cv::cvtColor(s, bgr, cv::COLOR_GRAY2BGR);
        }
        else
        {
            bgr = s;
        }
        scaled.push_back(bgr);
    }
    const int rows = static_cast<int>((scaled.size() + columns - 1) / columns);
    const int w = scaled[0].cols, h = scaled[0].rows;
    cv::Mat   out(rows * h, columns * w, CV_8UC3, cv::Scalar(30, 30, 30));
    for (std::size_t i = 0; i < scaled.size(); ++i)
    {
        cv::Mat s;
        cv::resize(scaled[i], s, cv::Size(w, h), 0, 0, cv::INTER_AREA);
        s.copyTo(out(cv::Rect(static_cast<int>(i % columns) * w, static_cast<int>(i / columns) * h, w, h)));
    }
    return out;
}

void writeJpeg(const std::string &path, const cv::Mat &image, int quality)
{
    if (!cv::imwrite(path, image, { cv::IMWRITE_JPEG_QUALITY, quality }))
    {
        throw std::runtime_error("cannot write " + path);
    }
}

}  // namespace orthostream::tools
