// Peak GPU memory during a call, sampled through NVML.

#ifndef OST_BENCH_GPU_MEMORY_HPP
#define OST_BENCH_GPU_MEMORY_HPP

#include <atomic>
#include <thread>

namespace orthostream::bench
{

class GpuMemorySampler
{
public:
    struct Result
    {
        double process_peak_mb    = -1.0;  ///< this process; -1 if NVML does not report it
        double device_peak_mb     = -1.0;  ///< whole device, all processes
        double device_baseline_mb = -1.0;  ///< whole device when sampling started
    };

    explicit GpuMemorySampler(unsigned gpu_index, int period_ms = 5);
    ~GpuMemorySampler();
    GpuMemorySampler(const GpuMemorySampler &)            = delete;
    GpuMemorySampler &operator=(const GpuMemorySampler &) = delete;

    bool available() const { return device_ != nullptr; }

    void   start();
    Result stop();

private:
    void   sample();
    double deviceUsedMb() const;
    double processUsedMb() const;

    void             *device_ = nullptr;  // nvmlDevice_t
    int               period_ms_;
    std::atomic<bool> running_ { false };
    std::thread       thread_;
    Result            result_;
    std::atomic<double> process_peak_ { -1.0 };
    std::atomic<double> device_peak_ { -1.0 };
};

}  // namespace orthostream::bench

#endif  // OST_BENCH_GPU_MEMORY_HPP
