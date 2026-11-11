// Statistics of height differences (ADR-018 section 3).

#ifndef OST_BENCH_ERROR_STATS_HPP
#define OST_BENCH_ERROR_STATS_HPP

#include <cstddef>
#include <vector>

namespace orthostream::bench
{

struct ErrorStats
{
    std::size_t count     = 0;
    double      mean      = 0.0;
    double      median    = 0.0;
    double      mae       = 0.0;  ///< mean absolute error
    double      rmse      = 0.0;
    double      nmad      = 0.0;  ///< 1.4826 x median of |e - median|
    double      p95_abs   = 0.0;  ///< 95th percentile of |e|
    double      within_05 = 0.0;  ///< share with |e| <= 0.5 m
    double      within_1  = 0.0;  ///< share with |e| <= 1 m
};

/// Statistics of the errors; all zero for an empty input.
ErrorStats errorStats(std::vector<double> errors);

}  // namespace orthostream::bench

#endif  // OST_BENCH_ERROR_STATS_HPP
