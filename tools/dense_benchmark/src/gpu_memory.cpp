#include "gpu_memory.hpp"

#include <algorithm>
#include <chrono>

#include <nvml.h>
#include <unistd.h>

namespace orthostream::bench
{
namespace
{

constexpr double kMb = 1024.0 * 1024.0;

void raiseTo(std::atomic<double> &peak, double v)
{
    double cur = peak.load();
    while (v > cur && !peak.compare_exchange_weak(cur, v))
    {
    }
}

}  // namespace

GpuMemorySampler::GpuMemorySampler(unsigned gpu_index, int period_ms) : period_ms_(period_ms)
{
    if (nvmlInit_v2() != NVML_SUCCESS)
    {
        return;
    }
    nvmlDevice_t device = nullptr;
    if (nvmlDeviceGetHandleByIndex_v2(gpu_index, &device) == NVML_SUCCESS)
    {
        device_ = device;
    }
}

GpuMemorySampler::~GpuMemorySampler()
{
    if (running_)
    {
        stop();
    }
    nvmlShutdown();
}

double GpuMemorySampler::deviceUsedMb() const
{
    nvmlMemory_t memory {};
    if (nvmlDeviceGetMemoryInfo(static_cast<nvmlDevice_t>(device_), &memory) != NVML_SUCCESS)
    {
        return -1.0;
    }
    return static_cast<double>(memory.used) / kMb;
}

double GpuMemorySampler::processUsedMb() const
{
    nvmlProcessInfo_t infos[64];
    unsigned int      count = 64;
    if (nvmlDeviceGetComputeRunningProcesses(static_cast<nvmlDevice_t>(device_), &count, infos) != NVML_SUCCESS)
    {
        return -1.0;
    }
    const unsigned int pid = static_cast<unsigned int>(getpid());
    for (unsigned int i = 0; i < count; ++i)
    {
        if (infos[i].pid == pid && infos[i].usedGpuMemory != static_cast<unsigned long long>(NVML_VALUE_NOT_AVAILABLE))
        {
            return static_cast<double>(infos[i].usedGpuMemory) / kMb;
        }
    }
    return -1.0;
}

void GpuMemorySampler::sample()
{
    raiseTo(device_peak_, deviceUsedMb());
    raiseTo(process_peak_, processUsedMb());
}

void GpuMemorySampler::start()
{
    if (!available() || running_)
    {
        return;
    }
    result_                    = Result {};
    result_.device_baseline_mb = deviceUsedMb();
    process_peak_              = -1.0;
    device_peak_               = -1.0;
    running_                   = true;
    thread_                    = std::thread(
        [this]
        {
            while (running_)
            {
                sample();
                std::this_thread::sleep_for(std::chrono::milliseconds(period_ms_));
            }
        });
}

GpuMemorySampler::Result GpuMemorySampler::stop()
{
    if (!running_)
    {
        return result_;
    }
    running_ = false;
    thread_.join();
    sample();
    result_.process_peak_mb = process_peak_;
    result_.device_peak_mb  = device_peak_;
    return result_;
}

}  // namespace orthostream::bench
