// Binary state of DSM and orthomosaic tiles, written when a tile is evicted
// and read when it becomes active again. These files are a cache for one
// run on one machine (native byte order), not map products.

#include <cstring>
#include <filesystem>
#include <fstream>

#include "orthostream/map_products/dsm.hpp"
#include "orthostream/map_products/ortho.hpp"

namespace orthostream::map_products
{
namespace
{

constexpr std::uint32_t kVersion = 1;

template <typename T>
void put(std::ofstream &out, const T &v)
{
    out.write(reinterpret_cast<const char *>(&v), sizeof(T));
}

template <typename T>
void putVector(std::ofstream &out, const std::vector<T> &v)
{
    out.write(reinterpret_cast<const char *>(v.data()), static_cast<std::streamsize>(v.size() * sizeof(T)));
}

template <typename T>
T get(std::ifstream &in)
{
    T v {};
    in.read(reinterpret_cast<char *>(&v), sizeof(T));
    return v;
}

template <typename T>
void getVector(std::ifstream &in, std::vector<T> &v, std::size_t n)
{
    v.resize(n);
    in.read(reinterpret_cast<char *>(v.data()), static_cast<std::streamsize>(n * sizeof(T)));
}

void checkMagic(std::ifstream &in, const char magic[4], const std::string &path)
{
    char m[4] = {};
    in.read(m, 4);
    if (!in || std::memcmp(m, magic, 4) != 0 || get<std::uint32_t>(in) != kVersion)
    {
        throw MapProductsError("not a tile state file: " + path);
    }
}

// Writes under a temporary name and renames, like the products.
template <typename Fn>
void writeAtomically(const std::string &path, Fn &&fn)
{
    const std::string tmp = path + ".tmp";
    {
        std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
        if (!out)
        {
            throw MapProductsError("cannot write " + tmp);
        }
        fn(out);
        if (!out)
        {
            throw MapProductsError("writing " + tmp + " failed");
        }
    }
    std::filesystem::rename(tmp, path);
}

}  // namespace

void DsmTile::save(const std::string &path) const
{
    writeAtomically(path,
                    [this](std::ofstream &out)
                    {
                        out.write("OSTD", 4);
                        put(out, kVersion);
                        put(out, id_.ix);
                        put(out, id_.iy);
                        put(out, cells_);
                        put(out, cell_size_);
                        put(out, reference_height_);
                        put(out, origin_e_);
                        put(out, origin_n_);
                        putVector(out, weight_);
                        putVector(out, mean_);
                        putVector(out, m2_);
                        putVector(out, count_);
                    });
}

std::unique_ptr<DsmTile> DsmTile::load(const std::string &path)
{
    std::ifstream in(path, std::ios::binary);
    if (!in)
    {
        throw MapProductsError("cannot read " + path);
    }
    checkMagic(in, "OSTD", path);
    std::unique_ptr<DsmTile> t(new DsmTile());
    t->id_.ix            = get<std::int64_t>(in);
    t->id_.iy            = get<std::int64_t>(in);
    t->cells_            = get<int>(in);
    t->cell_size_        = get<double>(in);
    t->reference_height_ = get<double>(in);
    t->origin_e_         = get<double>(in);
    t->origin_n_         = get<double>(in);
    if (!in || t->cells_ <= 0 || t->cells_ > 65536)
    {
        throw MapProductsError("corrupt tile state " + path);
    }
    const std::size_t n = static_cast<std::size_t>(t->cells_) * t->cells_;
    getVector(in, t->weight_, n);
    getVector(in, t->mean_, n);
    getVector(in, t->m2_, n);
    getVector(in, t->count_, n);
    if (!in)
    {
        throw MapProductsError("truncated tile state " + path);
    }
    return t;
}

void OrthoTile::save(const std::string &path) const
{
    writeAtomically(path,
                    [this](std::ofstream &out)
                    {
                        out.write("OSTO", 4);
                        put(out, kVersion);
                        put(out, id_.ix);
                        put(out, id_.iy);
                        put(out, pixels_);
                        put(out, pixel_size_);
                        put(out, origin_e_);
                        put(out, origin_n_);
                        putVector(out, rgb_);
                        putVector(out, angle_);
                    });
}

std::unique_ptr<OrthoTile> OrthoTile::load(const std::string &path)
{
    std::ifstream in(path, std::ios::binary);
    if (!in)
    {
        throw MapProductsError("cannot read " + path);
    }
    checkMagic(in, "OSTO", path);
    std::unique_ptr<OrthoTile> t(new OrthoTile());
    t->id_.ix      = get<std::int64_t>(in);
    t->id_.iy      = get<std::int64_t>(in);
    t->pixels_     = get<int>(in);
    t->pixel_size_ = get<double>(in);
    t->origin_e_   = get<double>(in);
    t->origin_n_   = get<double>(in);
    if (!in || t->pixels_ <= 0 || t->pixels_ > 65536)
    {
        throw MapProductsError("corrupt tile state " + path);
    }
    const std::size_t n = static_cast<std::size_t>(t->pixels_) * t->pixels_;
    getVector(in, t->rgb_, 3 * n);
    getVector(in, t->angle_, n);
    if (!in)
    {
        throw MapProductsError("truncated tile state " + path);
    }
    return t;
}

}  // namespace orthostream::map_products
