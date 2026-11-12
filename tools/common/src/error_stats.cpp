#include "error_stats.hpp"

#include <algorithm>
#include <cmath>

namespace orthostream::tools
{
namespace
{

// Linear interpolation between order statistics of a sorted vector.
double quantile(const std::vector<double> &sorted, double q)
{
    const double      pos = q * static_cast<double>(sorted.size() - 1);
    const std::size_t i   = static_cast<std::size_t>(pos);
    const std::size_t j   = std::min(i + 1, sorted.size() - 1);
    return sorted[i] + (pos - static_cast<double>(i)) * (sorted[j] - sorted[i]);
}

}  // namespace

ErrorStats errorStats(std::vector<double> errors)
{
    ErrorStats s;
    s.count = errors.size();
    if (errors.empty())
    {
        return s;
    }
    const double n = static_cast<double>(errors.size());
    std::vector<double> abs_err;
    abs_err.reserve(errors.size());
    double sum = 0.0, sum_abs = 0.0, sum_sq = 0.0, w05 = 0.0, w1 = 0.0;
    for (double e : errors)
    {
        const double a = std::abs(e);
        sum += e;
        sum_abs += a;
        sum_sq += e * e;
        w05 += a <= 0.5 ? 1.0 : 0.0;
        w1 += a <= 1.0 ? 1.0 : 0.0;
        abs_err.push_back(a);
    }
    s.mean      = sum / n;
    s.mae       = sum_abs / n;
    s.rmse      = std::sqrt(sum_sq / n);
    s.within_05 = w05 / n;
    s.within_1  = w1 / n;

    std::sort(errors.begin(), errors.end());
    s.median = quantile(errors, 0.5);
    std::sort(abs_err.begin(), abs_err.end());
    s.p95_abs = quantile(abs_err, 0.95);
    for (double &e : errors)
    {
        e = std::abs(e - s.median);
    }
    std::sort(errors.begin(), errors.end());
    s.nmad = 1.4826 * quantile(errors, 0.5);
    return s;
}

}  // namespace orthostream::tools
