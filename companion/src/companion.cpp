#include "companion/companion.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>

namespace companion {
namespace {
constexpr std::size_t max_pixels = 64u * 1024u * 1024u;
using Vector = std::array<double, 3>;
using Rotation = std::array<double, 9>;
bool dimensionsValid(int width, int height) {
    return width > 0 && height > 0 && width <= 16384 && height <= 16384
        && static_cast<std::size_t>(width) * height <= max_pixels;
}
bool poseValid(const CameraPose& p) {
    return std::isfinite(p.north_m) && std::isfinite(p.east_m) && std::isfinite(p.height_m)
        && std::isfinite(p.roll_rad) && std::isfinite(p.pitch_rad) && std::isfinite(p.yaw_rad);
}
Rotation rotation(const CameraPose& p) {
    const double cr = std::cos(p.roll_rad), sr = std::sin(p.roll_rad);
    const double cp = std::cos(p.pitch_rad), sp = std::sin(p.pitch_rad);
    const double cy = std::cos(p.yaw_rad), sy = std::sin(p.yaw_rad);
    return {cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
            sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
            -sp, cp * sr, cp * cr};
}
Vector rotate(const Rotation& r, const Vector& v) {
    return {r[0] * v[0] + r[1] * v[1] + r[2] * v[2],
            r[3] * v[0] + r[4] * v[1] + r[5] * v[2],
            r[6] * v[0] + r[7] * v[1] + r[8] * v[2]};
}
Vector inverseRotate(const Rotation& r, const Vector& v) {
    return {r[0] * v[0] + r[3] * v[1] + r[6] * v[2],
            r[1] * v[0] + r[4] * v[1] + r[7] * v[2],
            r[2] * v[0] + r[5] * v[1] + r[8] * v[2]};
}
Vector ray(const Calibration& k, const Rotation& r, double u, double v) {
    return rotate(r, {-(v - k.cy) / k.fy, (u - k.cx) / k.fx, 1.0});
}
bool project(const Calibration& k, const CameraPose& p, const Rotation& r,
             const GroundPoint& point, double& u, double& v) {
    const Vector b = inverseRotate(r, {point.north_m - p.north_m,
                                       point.east_m - p.east_m, p.height_m - point.height_m});
    if (!std::isfinite(b[2]) || b[2] <= 1e-8) return false;
    u = k.cx + k.fx * b[1] / b[2];
    v = k.cy - k.fy * b[0] / b[2];
    return std::isfinite(u) && std::isfinite(v);
}
bool bilinear(const Image& image, double x, double y, float& value) {
    if (!std::isfinite(x) || !std::isfinite(y) || x < -1e-8 || y < -1e-8
        || x > image.width - 1 + 1e-8 || y > image.height - 1 + 1e-8) return false;
    x = std::clamp(x, 0.0, static_cast<double>(image.width - 1));
    y = std::clamp(y, 0.0, static_cast<double>(image.height - 1));
    const int x0 = static_cast<int>(std::floor(x)), y0 = static_cast<int>(std::floor(y));
    const int x1 = std::min(x0 + 1, image.width - 1), y1 = std::min(y0 + 1, image.height - 1);
    const double ax = x - x0, ay = y - y0;
    value = static_cast<float>((1 - ay) * ((1 - ax) * image.at(x0, y0) + ax * image.at(x1, y0))
                            + ay * ((1 - ax) * image.at(x0, y1) + ax * image.at(x1, y1)));
    return true;
}
bool token(std::istream& stream, std::string& out) {
    while (stream >> out) {
        if (!out.empty() && out[0] == '#') { stream.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); continue; }
        return true;
    }
    return false;
}
bool integerToken(std::istream& stream, int& out) {
    std::string text;
    if (!token(stream, text)) return false;
    std::istringstream parse(text);
    parse >> out;
    return parse && parse.peek() == std::char_traits<char>::eof();
}
std::uint32_t rotr(std::uint32_t a, unsigned shift) { return (a >> shift) | (a << (32 - shift)); }
double dot(const Vector& a, const Vector& b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
Vector unit(Vector a) { const double length = std::sqrt(dot(a, a)); for (double& v : a) v /= length; return a; }
}  // namespace

bool Image::valid() const {
    return dimensionsValid(width, height) && pixels.size() == static_cast<std::size_t>(width) * height
        && std::all_of(pixels.begin(), pixels.end(), [](float p) { return std::isfinite(p) && p >= 0 && p <= 255; });
}
bool Calibration::valid() const {
    return std::isfinite(fx) && std::isfinite(fy) && std::isfinite(cx) && std::isfinite(cy) && fx > 0 && fy > 0;
}
bool readPgm(const std::string& path, Image& image, std::string& error) {
    image = {};
    std::ifstream stream(path, std::ios::binary);
    std::string magic;
    int width = 0, height = 0, maximum = 0;
    if (!stream || !token(stream, magic) || (magic != "P2" && magic != "P5")
        || !integerToken(stream, width) || !integerToken(stream, height) || !integerToken(stream, maximum)
        || !dimensionsValid(width, height) || maximum < 1 || maximum > 65535) {
        error = "invalid PGM header: " + path; return false;
    }
    Image out{width, height, std::vector<float>(static_cast<std::size_t>(width) * height)};
    if (magic == "P5") {
        const int delimiter = stream.get();
        if (delimiter != ' ' && delimiter != '\n' && delimiter != '\r' && delimiter != '\t') {
            error = "PGM binary header has no delimiter"; return false;
        }
        if (delimiter == '\r' && stream.peek() == '\n') stream.get();
        for (float& pixel : out.pixels) {
            const int high = stream.get();
            const int low = maximum > 255 ? stream.get() : 0;
            if (high == std::char_traits<char>::eof() || low == std::char_traits<char>::eof()) {
                error = "truncated PGM raster: " + path; return false;
            }
            const int raw = maximum > 255 ? high * 256 + low : high;
            if (raw > maximum) { error = "PGM sample exceeds maxval"; return false; }
            pixel = static_cast<float>(raw * 255.0 / maximum);
        }
        // 不吞掉首像素里的空白值；仅允许栅格之后的空白。
        int extra = stream.get();
        while (extra != std::char_traits<char>::eof()) {
            if (extra != ' ' && extra != '\n' && extra != '\r' && extra != '\t') { error = "extra PGM raster data"; return false; }
            extra = stream.get();
        }
    } else {
        for (float& pixel : out.pixels) {
            int raw = 0;
            if (!integerToken(stream, raw) || raw < 0 || raw > maximum) { error = "invalid PGM text sample"; return false; }
            pixel = static_cast<float>(raw * 255.0 / maximum);
        }
        std::string extra;
        if (token(stream, extra)) { error = "extra PGM text samples"; return false; }
    }
    image = std::move(out); error.clear(); return true;
}
bool writePgm(const std::string& path, const Image& image, std::string& error) {
    if (!image.valid()) { error = "invalid image"; return false; }
    std::ofstream stream(path, std::ios::binary);
    if (!stream) { error = "cannot write PGM: " + path; return false; }
    stream << "P5\n" << image.width << ' ' << image.height << "\n255\n";
    for (float pixel : image.pixels) stream.put(static_cast<char>(std::lround(pixel)));
    if (!stream) { error = "PGM write failed: " + path; return false; }
    error.clear(); return true;
}

std::string sha256(const std::string& bytes) {
    static constexpr std::uint32_t constants[64] = {
        0x428a2f98,0x71374491,0xb5c0fbcf,0xe9b5dba5,0x3956c25b,0x59f111f1,0x923f82a4,0xab1c5ed5,
        0xd807aa98,0x12835b01,0x243185be,0x550c7dc3,0x72be5d74,0x80deb1fe,0x9bdc06a7,0xc19bf174,
        0xe49b69c1,0xefbe4786,0x0fc19dc6,0x240ca1cc,0x2de92c6f,0x4a7484aa,0x5cb0a9dc,0x76f988da,
        0x983e5152,0xa831c66d,0xb00327c8,0xbf597fc7,0xc6e00bf3,0xd5a79147,0x06ca6351,0x14292967,
        0x27b70a85,0x2e1b2138,0x4d2c6dfc,0x53380d13,0x650a7354,0x766a0abb,0x81c2c92e,0x92722c85,
        0xa2bfe8a1,0xa81a664b,0xc24b8b70,0xc76c51a3,0xd192e819,0xd6990624,0xf40e3585,0x106aa070,
        0x19a4c116,0x1e376c08,0x2748774c,0x34b0bcb5,0x391c0cb3,0x4ed8aa4a,0x5b9cca4f,0x682e6ff3,
        0x748f82ee,0x78a5636f,0x84c87814,0x8cc70208,0x90befffa,0xa4506ceb,0xbef9a3f7,0xc67178f2};
    std::array<std::uint32_t, 8> h{0x6a09e667,0xbb67ae85,0x3c6ef372,0xa54ff53a,0x510e527f,0x9b05688c,0x1f83d9ab,0x5be0cd19};
    std::vector<std::uint8_t> data(bytes.begin(), bytes.end());
    const std::uint64_t bits = static_cast<std::uint64_t>(data.size()) * 8;
    data.push_back(0x80);
    while (data.size() % 64 != 56) data.push_back(0);
    for (int i = 7; i >= 0; --i) data.push_back(static_cast<std::uint8_t>(bits >> (8 * i)));
    for (std::size_t block = 0; block < data.size(); block += 64) {
        std::uint32_t w[64]{};
        for (int i = 0; i < 16; ++i) for (int j = 0; j < 4; ++j) w[i] = (w[i] << 8) | data[block + 4 * i + j];
        for (int i = 16; i < 64; ++i) {
            const auto s0 = rotr(w[i-15],7) ^ rotr(w[i-15],18) ^ (w[i-15] >> 3);
            const auto s1 = rotr(w[i-2],17) ^ rotr(w[i-2],19) ^ (w[i-2] >> 10);
            w[i] = w[i-16] + s0 + w[i-7] + s1;
        }
        auto a=h[0], b=h[1], c=h[2], d=h[3], e=h[4], f=h[5], g=h[6], z=h[7];
        for (int i = 0; i < 64; ++i) {
            const auto s1 = rotr(e,6) ^ rotr(e,11) ^ rotr(e,25);
            const auto t1 = z + s1 + ((e & f) ^ (~e & g)) + constants[i] + w[i];
            const auto s0 = rotr(a,2) ^ rotr(a,13) ^ rotr(a,22);
            const auto t2 = s0 + ((a & b) ^ (a & c) ^ (b & c));
            z=g; g=f; f=e; e=d+t1; d=c; c=b; b=a; a=t1+t2;
        }
        h[0]+=a; h[1]+=b; h[2]+=c; h[3]+=d; h[4]+=e; h[5]+=f; h[6]+=g; h[7]+=z;
    }
    std::ostringstream out;
    out << std::hex << std::setfill('0');
    for (auto word : h) out << std::setw(8) << word;
    return out.str();
}
bool fileSha256(const std::string& path, std::string& digest, std::string& error) {
    std::ifstream stream(path, std::ios::binary | std::ios::ate);
    if (!stream || stream.tellg() < 0 || stream.tellg() > 512 * 1024 * 1024) { error = "missing or oversized file: " + path; return false; }
    const auto size = static_cast<std::size_t>(stream.tellg());
    stream.seekg(0);
    std::string bytes(size, '\0');
    if (size && !stream.read(bytes.data(), static_cast<std::streamsize>(size))) { error = "cannot read file: " + path; return false; }
    digest = sha256(bytes); error.clear(); return true;
}
double textureStd(const Image& image) {
    if (!image.valid()) return 0;
    double sum = 0, square = 0;
    for (float p : image.pixels) { sum += p; square += p * static_cast<double>(p); }
    const double n = static_cast<double>(image.pixels.size());
    return std::sqrt(std::max(0.0, square / n - (sum / n) * (sum / n)));
}
double laplacianVariance(const Image& image) {
    if (!image.valid() || image.width < 3 || image.height < 3) return 0;
    double sum = 0, square = 0, n = 0;
    for (int y=1; y<image.height-1; ++y) for (int x=1; x<image.width-1; ++x) {
        const double value = image.at(x-1,y) + image.at(x+1,y) + image.at(x,y-1) + image.at(x,y+1) - 4.0 * image.at(x,y);
        sum += value; square += value * value; ++n;
    }
    return std::max(0.0, square / n - (sum / n) * (sum / n));
}

bool pixelToGround(const Calibration& k, const CameraPose& pose, double u, double v,
                   double plane_height_m, GroundPoint& point) {
    if (!k.valid() || !poseValid(pose) || !std::isfinite(u) || !std::isfinite(v)
        || !std::isfinite(plane_height_m) || pose.height_m <= plane_height_m) return false;
    const auto direction = ray(k, rotation(pose), u, v);
    if (direction[2] <= 1e-6) return false;
    const double scale = (pose.height_m - plane_height_m) / direction[2];
    point = {pose.north_m + scale * direction[0], pose.east_m + scale * direction[1], plane_height_m};
    return std::isfinite(point.north_m) && std::isfinite(point.east_m);
}
bool groundToPixel(const Calibration& k, const CameraPose& pose, const GroundPoint& point, double& u, double& v) {
    return k.valid() && poseValid(pose) && std::isfinite(point.north_m) && std::isfinite(point.east_m)
        && std::isfinite(point.height_m) && project(k, pose, rotation(pose), point, u, v);
}
OrthoImage orthorectify(const Image& input, const Calibration& k, const CameraPose& pose,
                       double plane_height_m, double gsd, int width, int height, Timestamp exposure) {
    OrthoImage result;
    if (!input.valid() || !k.valid() || !poseValid(pose) || !std::isfinite(plane_height_m)
        || pose.height_m <= plane_height_m || !std::isfinite(gsd) || gsd <= 0
        || !dimensionsValid(width, height) || exposure == 0) return result;
    result.image = {width, height, std::vector<float>(static_cast<std::size_t>(width) * height, 0)};
    result.mask.resize(result.image.pixels.size(), 0);
    result.center_north_m = pose.north_m; result.center_east_m = pose.east_m;
    result.camera_height_m = pose.height_m; result.gsd_m = gsd; result.t_us = exposure;
    const auto r = rotation(pose);
    const double cx = (width-1) * .5, cy = (height-1) * .5;
    for (int y=0; y<height; ++y) for (int x=0; x<width; ++x) {
        const GroundPoint ground{pose.north_m + (cy-y)*gsd, pose.east_m + (x-cx)*gsd, plane_height_m};
        double u = 0, v = 0;
        float value = 0;
        if (project(k, pose, r, ground, u, v) && bilinear(input, u, v, value)) {
            result.image.at(x,y) = value; result.mask[static_cast<std::size_t>(y)*width+x] = 1;
        }
    }
    return result;
}

const char* reasonName(RejectReason reason) {
    switch (reason) {
        case RejectReason::None: return "ok";
        case RejectReason::InvalidInput: return "invalid_input";
        case RejectReason::TimeOrder: return "time_order";
        case RejectReason::OutOfBounds: return "out_of_bounds";
        case RejectReason::LowTexture: return "low_texture";
        case RejectReason::LowCorrelation: return "low_correlation";
        case RejectReason::Ambiguous: return "ambiguous";
        case RejectReason::PoorGeometry: return "poor_geometry";
        case RejectReason::HeightDatum: return "height_datum";
        case RejectReason::Integrity: return "integrity";
        case RejectReason::MissingWeights: return "missing_weights";
        case RejectReason::BackendUnavailable: return "backend_unavailable";
    }
    return "invalid_input";
}

TriangulationResult triangulate(const Calibration& k, const CameraPose& first,
                                const CameraPose& second, const PixelObservation& a,
                                const PixelObservation& b) {
    TriangulationResult result;
    if (!k.valid() || !poseValid(first) || !poseValid(second) || !std::isfinite(a.u)
        || !std::isfinite(a.v) || !std::isfinite(b.u) || !std::isfinite(b.v)) {
        result.reason = RejectReason::InvalidInput; return result;
    }
    const Vector p{first.north_m, first.east_m, -first.height_m};
    const Vector q{second.north_m, second.east_m, -second.height_m};
    const Vector w{p[0]-q[0], p[1]-q[1], p[2]-q[2]};
    if (std::sqrt(dot(w,w)) < .5) return result;
    const Vector r1 = unit(ray(k, rotation(first), a.u, a.v));
    const Vector r2 = unit(ray(k, rotation(second), b.u, b.v));
    const double cosine = std::clamp(dot(r1,r2), -1.0, 1.0);
    result.ray_angle_rad = std::acos(cosine);
    if (result.ray_angle_rad < .008726646259971648 || result.ray_angle_rad > 3.13) return result;
    const double denominator = 1 - cosine * cosine;
    const double d = dot(r1,w), e = dot(r2,w);
    const double s = (cosine*e-d)/denominator, t = (e-cosine*d)/denominator;
    if (s <= 0 || t <= 0 || s > 1e6 || t > 1e6) return result;
    const Vector a3{p[0]+s*r1[0], p[1]+s*r1[1], p[2]+s*r1[2]};
    const Vector b3{q[0]+t*r2[0], q[1]+t*r2[1], q[2]+t*r2[2]};
    const Vector gap{a3[0]-b3[0], a3[1]-b3[1], a3[2]-b3[2]};
    result.ray_gap_m = std::sqrt(dot(gap,gap));
    if (result.ray_gap_m > 2) return result;
    result.point = {(a3[0]+b3[0])*.5, (a3[1]+b3[1])*.5, -(a3[2]+b3[2])*.5};
    double u1=0,v1=0,u2=0,v2=0;
    if (!groundToPixel(k, first, result.point, u1,v1) || !groundToPixel(k, second, result.point, u2,v2)) return result;
    result.reprojection_error_px = std::max(std::hypot(u1-a.u,v1-a.v), std::hypot(u2-b.u,v2-b.v));
    if (result.reprojection_error_px > 2) return result;
    result.valid = true; result.reason = RejectReason::None; return result;
}
std::vector<GroundPoint> reconstructSparse(const Calibration& k, const CameraPose& first,
                                           const CameraPose& second,
                                           const std::vector<StereoCorrespondence>& matches) {
    std::vector<GroundPoint> points;
    for (const auto& match : matches) {
        const auto point = triangulate(k, first, second, match.first, match.second);
        if (point.valid) points.push_back(point.point);
    }
    return points;
}
UnavailableLearnedMatcher::UnavailableLearnedMatcher(std::string path) : weights_path_(std::move(path)) {}
MatchResult UnavailableLearnedMatcher::match(const OrthoImage& image, const ImageMap&, const SearchPrior&) const {
    MatchResult out; out.t_us = image.t_us;
    std::ifstream weights(weights_path_, std::ios::binary | std::ios::ate);
    out.reason = weights_path_.empty() || !weights || weights.tellg() <= 0
        ? RejectReason::MissingWeights : RejectReason::BackendUnavailable;
    return out;
}
bool UnavailableDenseReconstructor::reconstruct(const std::vector<Frame>&, HeightGrid& output, std::string& error) {
    output = {}; error = "dense SfM/BA backend unavailable; provide calibrated poses, correspondence frontend and a validated dense backend"; return false;
}
}  // namespace companion
