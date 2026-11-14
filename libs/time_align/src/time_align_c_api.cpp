// C interface of the time alignment (time_align_c_api.h).

#include "time_align_c_api.h"

#include <cstring>
#include <deque>
#include <exception>
#include <memory>
#include <new>
#include <stdexcept>
#include <string>

#include "orthostream/time_align/aligner.hpp"

namespace ta = orthostream::time_align;

struct OST_Align
{
    std::unique_ptr<ta::Aligner>      aligner;
    std::deque<OST_Align_ImagePose_t> queue;  ///< decided, not yet returned
    std::string                       error;
};

namespace
{

ta::Accuracy toAccuracy(const OST_Align_Accuracy_t &a)
{
    ta::Accuracy r;
    r.position_std_north_m   = a.position_std_north_m;
    r.position_std_east_m    = a.position_std_east_m;
    r.position_std_down_m    = a.position_std_down_m;
    r.attitude_std_roll_deg  = a.attitude_std_roll_deg;
    r.attitude_std_pitch_deg = a.attitude_std_pitch_deg;
    r.attitude_std_yaw_deg   = a.attitude_std_yaw_deg;
    r.fix                    = a.fix;
    return r;
}

OST_Align_Accuracy_t fromAccuracy(const ta::Accuracy &a)
{
    OST_Align_Accuracy_t r;
    std::memset(&r, 0, sizeof(r));
    r.position_std_north_m   = a.position_std_north_m;
    r.position_std_east_m    = a.position_std_east_m;
    r.position_std_down_m    = a.position_std_down_m;
    r.attitude_std_roll_deg  = a.attitude_std_roll_deg;
    r.attitude_std_pitch_deg = a.attitude_std_pitch_deg;
    r.attitude_std_yaw_deg   = a.attitude_std_yaw_deg;
    r.fix                    = a.fix;
    return r;
}

OST_Align_ImagePose_t fromPose(const ta::ImagePose &p)
{
    OST_Align_ImagePose_t r;
    std::memset(&r, 0, sizeof(r));
    r.id         = p.id;
    r.capture_ns = p.capture_ns;
    r.result     = static_cast<uint8_t>(p.result);
    if (p.result == ta::Result::valid)
    {
        for (int i = 0; i < 3; ++i)
        {
            r.position_ecef[i] = p.p_E[i];
        }
        r.q_ecef_from_camera[0] = p.q_E_C.w();
        r.q_ecef_from_camera[1] = p.q_E_C.x();
        r.q_ecef_from_camera[2] = p.q_E_C.y();
        r.q_ecef_from_camera[3] = p.q_E_C.z();
    }
    r.accuracy  = fromAccuracy(p.accuracy);
    r.before_ns = p.before_ns;
    r.after_ns  = p.after_ns;
    r.waited_ns = p.waited_ns;
    return r;
}

/// Runs f and maps exceptions to status codes and the handle's error text.
template <typename F>
OST_Status_t guarded(OST_Align_t *handle, F &&f)
{
    try
    {
        handle->error.clear();
        return f();
    }
    catch (const std::invalid_argument &e)
    {
        handle->error = e.what();
        return OST_ERR_INVALID_ARGUMENT;
    }
    catch (const std::bad_alloc &)
    {
        handle->error = "out of memory";
        return OST_ERR_NO_MEMORY;
    }
    catch (const std::exception &e)
    {
        handle->error = e.what();
        return OST_ERR_INTERNAL;
    }
    catch (...)
    {
        handle->error = "unknown exception";
        return OST_ERR_INTERNAL;
    }
}

}  // namespace

extern "C" {

void OST_Align_DefaultConfig(OST_Align_Config_t *config)
{
    if (config == nullptr)
    {
        return;
    }
    const ta::AlignConfig d;
    std::memset(config, 0, sizeof(*config));
    config->max_gap_ns  = d.max_gap_ns;
    config->max_wait_ns = d.max_wait_ns;
    config->buffer_ns   = d.buffer_ns;
    config->max_pending = static_cast<uint32_t>(d.max_pending);
}

OST_Status_t OST_Align_Create(const OST_Align_Config_t *config, OST_Align_t **handle)
{
    if (config == nullptr || handle == nullptr)
    {
        return OST_ERR_INVALID_ARGUMENT;
    }
    *handle = nullptr;
    try
    {
        ta::AlignConfig c;
        c.max_gap_ns  = config->max_gap_ns;
        c.max_wait_ns = config->max_wait_ns;
        c.buffer_ns   = config->buffer_ns;
        c.max_pending = config->max_pending;
        auto h        = std::make_unique<OST_Align>();
        h->aligner    = std::make_unique<ta::Aligner>(c);
        *handle       = h.release();
        return OST_SUCCESS;
    }
    catch (const std::invalid_argument &)
    {
        return OST_ERR_INVALID_ARGUMENT;
    }
    catch (const std::bad_alloc &)
    {
        return OST_ERR_NO_MEMORY;
    }
    catch (...)
    {
        return OST_ERR_INTERNAL;
    }
}

void OST_Align_Destroy(OST_Align_t *handle)
{
    delete handle;
}

const char *OST_Align_LastError(const OST_Align_t *handle)
{
    return handle == nullptr ? "no handle" : handle->error.c_str();
}

OST_Status_t OST_Align_SetMounting(OST_Align_t *handle, const double q_body_from_camera[4],
                                   const double camera_in_body[3])
{
    if (handle == nullptr || q_body_from_camera == nullptr || camera_in_body == nullptr)
    {
        return OST_ERR_INVALID_ARGUMENT;
    }
    return guarded(handle, [&] {
        ta::Mounting m;
        m.q_B_C = Eigen::Quaterniond(q_body_from_camera[0], q_body_from_camera[1], q_body_from_camera[2],
                                     q_body_from_camera[3]);
        m.t_B_C = Eigen::Vector3d(camera_in_body[0], camera_in_body[1], camera_in_body[2]);
        handle->aligner->setMounting(m);
        return OST_SUCCESS;
    });
}

OST_Status_t OST_Align_AddSample(OST_Align_t *handle, const OST_Align_Sample_t *sample, int *verdict)
{
    if (handle == nullptr || sample == nullptr || verdict == nullptr)
    {
        return OST_ERR_INVALID_ARGUMENT;
    }
    return guarded(handle, [&] {
        ta::Sample s;
        s.time_ns       = sample->time_ns;
        s.latitude_deg  = sample->latitude_deg;
        s.longitude_deg = sample->longitude_deg;
        s.height_m      = sample->height_m;
        s.q_N_B = Eigen::Quaterniond(sample->q_ned_from_body[0], sample->q_ned_from_body[1],
                                     sample->q_ned_from_body[2], sample->q_ned_from_body[3]);
        s.accuracy = toAccuracy(sample->accuracy);
        switch (handle->aligner->addSample(s))
        {
            case ta::SampleVerdict::accepted:     *verdict = OST_ALIGN_SAMPLE_ACCEPTED; break;
            case ta::SampleVerdict::out_of_order: *verdict = OST_ALIGN_SAMPLE_OUT_OF_ORDER; break;
            case ta::SampleVerdict::duplicate:    *verdict = OST_ALIGN_SAMPLE_DUPLICATE; break;
            case ta::SampleVerdict::invalid:      *verdict = OST_ALIGN_SAMPLE_INVALID; break;
        }
        return OST_SUCCESS;
    });
}

OST_Status_t OST_Align_AddImage(OST_Align_t *handle, uint64_t id, int64_t capture_ns, int64_t arrival_ns)
{
    if (handle == nullptr)
    {
        return OST_ERR_INVALID_ARGUMENT;
    }
    return guarded(handle, [&] {
        handle->aligner->addImage(id, capture_ns, arrival_ns);
        return OST_SUCCESS;
    });
}

OST_Status_t OST_Align_Poll(OST_Align_t *handle, int64_t now_ns, OST_Align_ImagePose_t *poses, uint32_t capacity,
                            uint32_t *count)
{
    if (handle == nullptr || count == nullptr || (poses == nullptr && capacity > 0))
    {
        return OST_ERR_INVALID_ARGUMENT;
    }
    *count = 0;
    return guarded(handle, [&] {
        for (const ta::ImagePose &p : handle->aligner->poll(now_ns))
        {
            handle->queue.push_back(fromPose(p));
        }
        while (*count < capacity && !handle->queue.empty())
        {
            poses[(*count)++] = handle->queue.front();
            handle->queue.pop_front();
        }
        return OST_SUCCESS;
    });
}

OST_Status_t OST_Align_Clear(OST_Align_t *handle)
{
    if (handle == nullptr)
    {
        return OST_ERR_INVALID_ARGUMENT;
    }
    return guarded(handle, [&] {
        handle->aligner->clear();
        return OST_SUCCESS;
    });
}

OST_Status_t OST_Align_GetStats(const OST_Align_t *handle, OST_Align_Stats_t *stats)
{
    if (handle == nullptr || stats == nullptr)
    {
        return OST_ERR_INVALID_ARGUMENT;
    }
    const ta::AlignStats s = handle->aligner->stats();
    std::memset(stats, 0, sizeof(*stats));
    stats->samples_accepted     = s.samples_accepted;
    stats->samples_out_of_order = s.samples_out_of_order;
    stats->samples_duplicate    = s.samples_duplicate;
    stats->samples_invalid      = s.samples_invalid;
    stats->buffered_samples     = static_cast<uint32_t>(s.buffered_samples);
    stats->pending_images       = static_cast<uint32_t>(s.pending_images + handle->queue.size());
    stats->buffer_span_ns       = s.buffer_span_ns;
    return OST_SUCCESS;
}

const char *OST_Align_ResultName(int result)
{
    if (result < 0 || result > OST_ALIGN_RESULT_CLEARED)
    {
        return "unknown";
    }
    return ta::resultName(static_cast<ta::Result>(result));
}

}  // extern "C"
