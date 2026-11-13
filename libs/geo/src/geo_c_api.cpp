// C interface of the geo library. No C++ exception crosses this boundary.

#include "geo_c_api.h"

#include <cmath>
#include <new>
#include <optional>
#include <string>

#include "orthostream/geo/camera.hpp"
#include "orthostream/geo/crs.hpp"
#include "orthostream/geo/frames.hpp"

struct OST_Geo_Context
{
    std::optional<orthostream::geo::GeodeticConverter> geodetic;
    std::string                                        last_error;
};

namespace
{

using orthostream::geo::CameraModel;

CameraModel toModel(const OST_Geo_Camera_t &c)
{
    CameraModel m;
    m.width  = c.width;
    m.height = c.height;
    m.fx     = c.fx;
    m.fy     = c.fy;
    m.cx     = c.cx;
    m.cy     = c.cy;
    m.k1     = c.k1;
    m.k2     = c.k2;
    m.k3     = c.k3;
    m.p1     = c.p1;
    m.p2     = c.p2;
    return m;
}

OST_Geo_Camera_t fromModel(const CameraModel &m)
{
    OST_Geo_Camera_t c;
    c.width  = m.width;
    c.height = m.height;
    c.fx     = m.fx;
    c.fy     = m.fy;
    c.cx     = m.cx;
    c.cy     = m.cy;
    c.k1     = m.k1;
    c.k2     = m.k2;
    c.k3     = m.k3;
    c.p1     = m.p1;
    c.p2     = m.p2;
    return c;
}

bool validCamera(const OST_Geo_Camera_t *c)
{
    return c != nullptr && c->width > 0 && c->height > 0 && c->fx > 0.0 && c->fy > 0.0;
}

// Runs f, converting exceptions to status codes and recording the message.
// geo_error is returned for GeoError, the errors reported by PROJ.
template <typename F>
OST_Status_t guarded(OST_Geo_Context_t *context, OST_Status_t geo_error, F &&f)
{
    try
    {
        if (context != nullptr)
        {
            context->last_error.clear();
        }
        return f();
    }
    catch (const orthostream::geo::GeoError &e)
    {
        if (context != nullptr)
        {
            context->last_error = e.what();
        }
        return geo_error;
    }
    catch (const std::bad_alloc &)
    {
        return OST_ERR_NO_MEMORY;
    }
    catch (const std::exception &e)
    {
        if (context != nullptr)
        {
            context->last_error = e.what();
        }
        return OST_ERR_INTERNAL;
    }
    catch (...)
    {
        return OST_ERR_INTERNAL;
    }
}

}  // namespace

extern "C" {

OST_Status_t OST_Geo_Create(OST_Geo_Context_t **context, const char *proj_data_dir)
{
    if (context == nullptr || proj_data_dir == nullptr)
    {
        return OST_ERR_INVALID_ARGUMENT;
    }
    *context = nullptr;
    OST_Geo_Context_t *created = new (std::nothrow) OST_Geo_Context_t();
    if (created == nullptr)
    {
        return OST_ERR_NO_MEMORY;
    }
    const OST_Status_t status = guarded(created, OST_ERR_INVALID_ARGUMENT,
                                            [&]
                                            {
                                                created->geodetic.emplace(proj_data_dir);
                                                return OST_SUCCESS;
                                            });
    if (status != OST_SUCCESS)
    {
        delete created;
        return status;
    }
    *context = created;
    return OST_SUCCESS;
}

void OST_Geo_Destroy(OST_Geo_Context_t *context)
{
    delete context;
}

const char *OST_Geo_LastError(const OST_Geo_Context_t *context)
{
    return context != nullptr ? context->last_error.c_str() : "";
}

OST_Status_t OST_Geo_EcefToGeodetic(OST_Geo_Context_t *context,
                                        const double       ecef[3],
                                        double            *latitude_rad,
                                        double            *longitude_rad,
                                        double            *height_m)
{
    if (context == nullptr || ecef == nullptr || latitude_rad == nullptr || longitude_rad == nullptr
        || height_m == nullptr)
    {
        return OST_ERR_INVALID_ARGUMENT;
    }
    return guarded(context, OST_ERR_NUMERIC,
                   [&]
                   {
                       const auto g   = context->geodetic->ecefToGeodetic({ ecef[0], ecef[1], ecef[2] });
                       *latitude_rad  = g.latitude_rad;
                       *longitude_rad = g.longitude_rad;
                       *height_m      = g.height_m;
                       return OST_SUCCESS;
                   });
}

OST_Status_t OST_Geo_GeodeticToEcef(OST_Geo_Context_t *context,
                                        double             latitude_rad,
                                        double             longitude_rad,
                                        double             height_m,
                                        double             ecef[3])
{
    if (context == nullptr || ecef == nullptr)
    {
        return OST_ERR_INVALID_ARGUMENT;
    }
    return guarded(context, OST_ERR_NUMERIC,
                   [&]
                   {
                       const Eigen::Vector3d p =
                           context->geodetic->geodeticToEcef({ latitude_rad, longitude_rad, height_m });
                       ecef[0] = p.x();
                       ecef[1] = p.y();
                       ecef[2] = p.z();
                       return OST_SUCCESS;
                   });
}

OST_Status_t OST_Geo_RotationEcefFromEnu(double latitude_rad, double longitude_rad, double rotation[9])
{
    if (rotation == nullptr || !std::isfinite(latitude_rad) || !std::isfinite(longitude_rad))
    {
        return OST_ERR_INVALID_ARGUMENT;
    }
    const Eigen::Matrix3d R = orthostream::geo::rotationEcefFromEnu(latitude_rad, longitude_rad);
    for (int r = 0; r < 3; ++r)
    {
        for (int c = 0; c < 3; ++c)
        {
            rotation[3 * r + c] = R(r, c);
        }
    }
    return OST_SUCCESS;
}

OST_Status_t OST_Geo_CameraFromMetashape(const OST_Geo_MetashapeCalibration_t *calibration,
                                             OST_Geo_Camera_t                     *camera)
{
    if (calibration == nullptr || camera == nullptr || calibration->width <= 0 || calibration->height <= 0
        || !(calibration->f > 0.0))
    {
        return OST_ERR_INVALID_ARGUMENT;
    }
    orthostream::geo::MetashapeCalibration m;
    m.width  = calibration->width;
    m.height = calibration->height;
    m.f      = calibration->f;
    m.cx     = calibration->cx;
    m.cy     = calibration->cy;
    m.k1     = calibration->k1;
    m.k2     = calibration->k2;
    m.k3     = calibration->k3;
    m.p1     = calibration->p1;
    m.p2     = calibration->p2;
    *camera  = fromModel(orthostream::geo::cameraFromMetashape(m));
    return OST_SUCCESS;
}

OST_Status_t OST_Geo_ProjectPoint(const OST_Geo_Camera_t *camera, const double point_camera[3], double pixel[2])
{
    if (!validCamera(camera) || point_camera == nullptr || pixel == nullptr)
    {
        return OST_ERR_INVALID_ARGUMENT;
    }
    const auto uv =
        orthostream::geo::projectPoint(toModel(*camera), { point_camera[0], point_camera[1], point_camera[2] });
    if (!uv)
    {
        return OST_ERR_NUMERIC;
    }
    pixel[0] = uv->x();
    pixel[1] = uv->y();
    return OST_SUCCESS;
}

OST_Status_t OST_Geo_UndistortPixel(const OST_Geo_Camera_t *camera, const double pixel[2], double normalized[2])
{
    if (!validCamera(camera) || pixel == nullptr || normalized == nullptr)
    {
        return OST_ERR_INVALID_ARGUMENT;
    }
    const auto xy = orthostream::geo::undistortPixel(toModel(*camera), { pixel[0], pixel[1] });
    if (!xy)
    {
        return OST_ERR_NUMERIC;
    }
    normalized[0] = xy->x();
    normalized[1] = xy->y();
    return OST_SUCCESS;
}

}  // extern "C"
