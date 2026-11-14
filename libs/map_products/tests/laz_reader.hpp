// Test helper: reads a LAZ file written by the point cloud writer.

#ifndef ORTHOSTREAM_MAP_PRODUCTS_LAZ_READER_HPP
#define ORTHOSTREAM_MAP_PRODUCTS_LAZ_READER_HPP

#include <cstdint>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <laszip/laszip_api.h>

namespace orthostream::map_products::test
{

struct LazPoint
{
    double        x, y, z;
    std::uint16_t intensity, source, r, g, b;
};

struct LazFile
{
    std::vector<LazPoint>      points;
    std::vector<std::uint16_t> geokeys;
    double                     min_x = 0, max_x = 0, min_y = 0, max_y = 0;
};

inline LazFile readLaz(const std::string &path)
{
    LazFile         f;
    laszip_POINTER  r = nullptr;
    laszip_BOOL     compressed = 0;
    laszip_header  *h = nullptr;
    laszip_point   *p = nullptr;
    EXPECT_EQ(laszip_create(&r), 0);
    EXPECT_EQ(laszip_open_reader(r, path.c_str(), &compressed), 0) << path;
    EXPECT_TRUE(compressed);
    laszip_get_header_pointer(r, &h);
    laszip_get_point_pointer(r, &p);
    f.min_x = h->min_x, f.max_x = h->max_x, f.min_y = h->min_y, f.max_y = h->max_y;
    for (laszip_U32 i = 0; i < h->number_of_variable_length_records; ++i)
    {
        if (h->vlrs[i].record_id == 34735)
        {
            for (laszip_U16 k = 0; k + 1 < h->vlrs[i].record_length_after_header; k += 2)
            {
                f.geokeys.push_back(static_cast<std::uint16_t>(h->vlrs[i].data[k] | (h->vlrs[i].data[k + 1] << 8)));
            }
        }
    }
    for (laszip_U32 i = 0; i < h->number_of_point_records; ++i)
    {
        laszip_read_point(r);
        f.points.push_back({ h->x_offset + p->X * h->x_scale_factor, h->y_offset + p->Y * h->y_scale_factor,
                             h->z_offset + p->Z * h->z_scale_factor, p->intensity, p->point_source_ID, p->rgb[0],
                             p->rgb[1], p->rgb[2] });
    }
    laszip_close_reader(r);
    laszip_destroy(r);
    return f;
}

inline bool hasKey(const std::vector<std::uint16_t> &keys, std::uint16_t id, std::uint16_t value)
{
    for (std::size_t i = 4; i + 3 < keys.size(); i += 4)
    {
        if (keys[i] == id && keys[i + 3] == value)
        {
            return true;
        }
    }
    return false;
}

}  // namespace orthostream::map_products::test

#endif  // ORTHOSTREAM_MAP_PRODUCTS_LAZ_READER_HPP
