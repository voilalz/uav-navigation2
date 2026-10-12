#include "companion/companion.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>

namespace {
using namespace companion;
int failures = 0;
int tests = 0;
#define CHECK(condition) do { if (!(condition)) { std::cerr << "FAIL " << __func__ << ":" << __LINE__ << " " << #condition << '\n'; ++failures; return; } } while (false)
bool close(double a, double b, double tolerance = 1e-6) { return std::abs(a - b) <= tolerance; }
const auto temp = std::filesystem::temp_directory_path() / "uav_companion_regression";

Image texture(int width, int height, bool smooth = false) {
    Image out{width, height, std::vector<float>(static_cast<std::size_t>(width) * height)};
    for (int y = 0; y < height; ++y) for (int x = 0; x < width; ++x) {
        std::uint32_t z = static_cast<std::uint32_t>(x * 73856093u) ^ static_cast<std::uint32_t>(y * 19349663u) ^ 92837111u;
        z ^= z >> 13; z *= 1274126177u;
        out.at(x, y) = smooth
            ? static_cast<float>(90 + 35 * std::sin(x * .27) + 30 * std::cos(y * .31)
                + 25 * std::sin(x * .13 + y * .21) + (z % 45))
            : static_cast<float>(40 + 15 * std::sin(x * .27) + 12 * std::cos(y * .31)
                + 8 * std::sin(x * .13 + y * .21) + (z % 160));
    }
    return out;
}
Image crop(const Image& image, int x0, int y0, int width, int height) {
    Image out{width, height, std::vector<float>(static_cast<std::size_t>(width) * height)};
    for (int y = 0; y < height; ++y) for (int x = 0; x < width; ++x) out.at(x, y) = image.at(x0 + x, y0 + y);
    return out;
}
HeightGrid terrain(int width, int height) {
    HeightGrid out;
    out.width = width; out.height = height; out.gsd_m = 2;
    out.origin_north_m = 500; out.origin_east_m = -150;
    out.values.resize(static_cast<std::size_t>(width) * height);
    for (int y = 0; y < height; ++y) for (int x = 0; x < width; ++x)
        out.values[static_cast<std::size_t>(y) * width + x] = 100 + 4 * std::sin(.23 * x)
            + 7 * std::cos(.14 * y) + 3 * std::sin(.15 * x + .3 * y);
    return out;
}
void textFile(const std::filesystem::path& path, const std::string& text) { std::ofstream(path, std::ios::binary) << text; }
std::string allBytes(const std::filesystem::path& path) { std::ifstream stream(path, std::ios::binary); return std::string(std::istreambuf_iterator<char>(stream), {}); }

void pgm_roundtrip_and_sha256() {
    std::string error;
    Image image{4, 3, {0, 10, 32, 255, 128, 64, 13, 1, 8, 20, 100, 240}};
    CHECK(writePgm((temp / "raw.pgm").string(), image, error));
    Image decoded;
    CHECK(readPgm((temp / "raw.pgm").string(), decoded, error));
    CHECK(decoded.width == 4 && decoded.height == 3 && decoded.pixels == image.pixels);
    textFile(temp / "text.pgm", "P2\n# comment\n2 2\n65535\n0 65535 32768 257\n");
    CHECK(readPgm((temp / "text.pgm").string(), decoded, error));
    CHECK(close(decoded.at(1, 0), 255) && close(decoded.at(1, 1), 1));
    textFile(temp / "truncated.pgm", "P5\n2 2\n255\nx");
    CHECK(!readPgm((temp / "truncated.pgm").string(), decoded, error));
    textFile(temp / "bad.pgm", "P2\n2 2\n255\n0 1 2 999\n");
    CHECK(!readPgm((temp / "bad.pgm").string(), decoded, error));
    CHECK(sha256("") == "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
    CHECK(sha256("abc") == "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
}

void calibrated_projection_and_rotation() {
    const Calibration cal{80, 100, 39.5, 29.5};
    CameraPose pose{100, 200, 150, 0, 0, 0};
    GroundPoint point;
    CHECK(pixelToGround(cal, pose, 43.5, 35.5, 100, point));
    CHECK(close(point.north_m, 97) && close(point.east_m, 202.5));
    pose.yaw_rad = 1.57079632679489661923;
    CHECK(pixelToGround(cal, pose, 43.5, 29.5, 100, point));
    CHECK(close(point.north_m, 97.5) && close(point.east_m, 200));
    pose.roll_rad = .13; pose.pitch_rad = -.2;
    CHECK(pixelToGround(cal, pose, 28.2, 19.4, 100, point));
    double u = 0, v = 0;
    CHECK(groundToPixel(cal, pose, point, u, v));
    CHECK(close(u, 28.2) && close(v, 19.4));
    pose.roll_rad = 3.14159265358979323846;
    CHECK(!pixelToGround(cal, pose, cal.cx, cal.cy, 100, point));
    CHECK(!pixelToGround({0, 100, 0, 0}, pose, 1, 1, 100, point));
}

void orthorectification_preserves_exposure_time() {
    const Image image = texture(64, 64);
    const Calibration cal{64, 64, 31.5, 31.5};
    const auto ortho = orthorectify(image, cal, {25, -40, 164, 0, 0, 0}, 100, 1, 64, 64, 123456789);
    CHECK(ortho.image.valid() && ortho.t_us == 123456789);
    CHECK(close(ortho.center_north_m, 25) && close(ortho.center_east_m, -40));
    CHECK(ortho.mask[32 * 64 + 31] && close(ortho.image.at(31, 32), image.at(31, 32), 1e-4));
    const auto invalid = orthorectify(image, cal, {0, 0, 80, 0, 0, 0}, 100, 1, 64, 64, 1);
    CHECK(!invalid.image.valid());
}

void ncc_recovers_shift_and_rejects_texture_and_ambiguity() {
    const Image reference = texture(128, 104);
    const Image query = crop(reference, 45, 37, 32, 24);
    const auto match = matchNcc(query, {}, reference, {10, 10, 80, 70});
    if (!match.valid) std::cerr << "NCC reason=" << reasonName(match.reason) << " score=" << match.quality.score << " second=" << match.quality.second_score << " PSR=" << match.quality.psr << " x=" << match.x << " y=" << match.y << '\n';
    CHECK(match.valid);
    CHECK(close(match.x, 45, .15) && close(match.y, 37, .15));
    CHECK(match.quality.score > .999 && match.quality.psr > 3 && match.sigma_px > 0);
    std::vector<std::uint8_t> mask(query.pixels.size(), 1);
    for (int y = 0; y < 5; ++y) for (int x = 0; x < query.width; ++x) mask[y * query.width + x] = 0;
    const auto masked = matchNcc(query, mask, reference, {10, 10, 80, 70});
    CHECK(masked.valid && close(masked.x, 45, .2) && close(masked.y, 37, .2));
    const Image flat{32, 24, std::vector<float>(32 * 24, 100)};
    const auto rejected = matchNcc(flat, {}, reference, {0, 0, 80, 70});
    CHECK(!rejected.valid && rejected.reason == RejectReason::LowTexture);
    Image repeat{80, 80, std::vector<float>(80 * 80)};
    for (int y = 0; y < 80; ++y) for (int x = 0; x < 80; ++x) repeat.at(x, y) = reference.at(x % 16, y % 16);
    const auto ambiguous = matchNcc(crop(repeat, 16, 16, 24, 24), {}, repeat, {0, 0, 56, 56});
    CHECK(!ambiguous.valid && ambiguous.reason == RejectReason::Ambiguous);
    const auto smooth = texture(128, 104, true);
    const auto low_psr = matchNcc(crop(smooth, 45, 37, 32, 24), {}, smooth, {10, 10, 80, 70});
    CHECK(!low_psr.valid && low_psr.reason == RejectReason::Ambiguous && low_psr.quality.psr < 3);
    CHECK(!matchNcc(query, {}, reference, {200, 200, 300, 300}).valid);
}

void map_match_outputs_local_position_covariance_and_timestamp() {
    const ImageMap map{texture(128, 104), 250, -100, 2};
    OrthoImage query{crop(map.image, 45, 37, 32, 24), {}, 0, 0, 2, 700, 1234500};
    const double north = 250 - (37 + 11.5) * 2;
    const double east = -100 + (45 + 15.5) * 2;
    const auto result = matchAbsolute(query, map, {north + 8, east - 6, 30});
    if (!result.valid) std::cerr << "MAP reason=" << reasonName(result.reason) << " score=" << result.quality.score << " second=" << result.quality.second_score << " PSR=" << result.quality.psr << '\n';
    CHECK(result.valid && result.t_us == 1234500 && result.dimensions == 2);
    CHECK(close(result.north_m, north, .3) && close(result.east_m, east, .3));
    CHECK(result.covariance[0] > 0 && result.covariance[4] > 0 && result.sigma_m > 0);
    query.gsd_m = 1;
    CHECK(!matchAbsolute(query, map, {north, east, 30}).valid);
    query.gsd_m = 2;
    CHECK(!matchAbsolute(query, map, {10000, 10000, 2}).valid);
}

void planar_vo_uses_image_translation_altitude_and_exposure_interval() {
    const Image world = texture(180, 160);
    Frame previous{crop(world, 45, 40, 64, 64), 1000200, 64, .2, 0, 0, 0};
    Frame current{crop(world, 49, 38, 64, 64), 1100200, 64, .2, 0, 0, 0};
    const Calibration cal{64, 64, 31.5, 31.5};
    const auto velocity = estimatePlanarVelocity(previous, current, cal);
    if (!velocity.valid) std::cerr << "VO reason=" << reasonName(velocity.reason) << " score=" << velocity.quality.score << " second=" << velocity.quality.second_score << " PSR=" << velocity.quality.psr << '\n';
    CHECK(velocity.valid && velocity.t_us == 1100200);
    CHECK(velocity.interval_begin_us == 1000200 && velocity.interval_end_us == 1100200);
    CHECK(close(velocity.velocity_ned[0], 20, 1) && close(velocity.velocity_ned[1], 40, 1));
    CHECK(close(velocity.scale_m_per_px, 1) && velocity.covariance[0] > 0);
    previous.altitude_agl_m = 128; current.altitude_agl_m = 128;
    const auto scaled = estimatePlanarVelocity(previous, current, cal);
    CHECK(scaled.valid && close(scaled.velocity_ned[0], 40, 2) && close(scaled.velocity_ned[1], 80, 2));
    previous.altitude_agl_m = 0;
    CHECK(!estimatePlanarVelocity(previous, current, cal).valid);
    previous.altitude_agl_m = 128; current.exposure_t_us = previous.exposure_t_us;
    CHECK(!estimatePlanarVelocity(previous, current, cal).valid);
    current.exposure_t_us = 1100200;
    previous.image = Image{64, 64, std::vector<float>(64 * 64, 50)}; current.image = previous.image;
    CHECK(!estimatePlanarVelocity(previous, current, cal).valid);
}

void terrain_registration_recovers_horizontal_shift_and_vertical_bias() {
    const auto reference = terrain(128, 100);
    HeightGrid observed;
    observed.width = 24; observed.height = 20; observed.gsd_m = 2;
    observed.origin_north_m = 500 - 30 * 2 + 6;
    observed.origin_east_m = -150 + 40 * 2 - 4;
    for (int y = 0; y < 20; ++y) for (int x = 0; x < 24; ++x) observed.values.push_back(reference.at(x + 40, y + 30) + 7.5);
    const auto result = registerTerrain(observed, reference, 12, 7654321);
    CHECK(result.valid && result.t_us == 7654321);
    CHECK(close(result.delta_north_m, -6, .15) && close(result.delta_east_m, 4, .15));
    CHECK(close(result.vertical_bias_m, 7.5, .01) && close(result.height_correction_m, -7.5, .01));
    CHECK(result.rmse_m < 1e-6 && result.covariance[0] > 0 && result.covariance[4] > 0);
    double sample = 0;
    CHECK(!reference.sample(10000, 10000, sample));
    CHECK(reference.sample(reference.origin_north_m, reference.origin_east_m, sample) && close(sample, reference.at(0, 0)));
    observed.origin_north_m = 10000;
    CHECK(!registerTerrain(observed, reference, 12, 1).valid);
    observed.origin_north_m = 446; observed.height_datum = "AMSL";
    CHECK(!registerTerrain(observed, reference, 12, 1).valid);
}

void flat_and_planar_terrain_have_no_horizontal_fix() {
    auto reference = terrain(80, 80);
    auto observed = terrain(20, 20);
    observed.origin_north_m = 460; observed.origin_east_m = -110;
    for (auto& value : reference.values) value = 123;
    for (auto& value : observed.values) value = 130;
    CHECK(!registerTerrain(observed, reference, 10, 1).valid);
    for (int y = 0; y < reference.height; ++y) for (int x = 0; x < reference.width; ++x) reference.values[y * reference.width + x] = 100 + .1 * x + .2 * y;
    for (int y = 0; y < observed.height; ++y) for (int x = 0; x < observed.width; ++x) observed.values[y * observed.width + x] = 100 + .1 * (x + 20) + .2 * (y + 20) + 7;
    CHECK(!registerTerrain(observed, reference, 10, 1).valid);
}

void triangulation_recovers_real_depth_and_rejects_zero_baseline() {
    const Calibration cal{100, 100, 50, 50};
    const CameraPose first{0, 0, 200, 0, 0, 0}, second{20, 0, 200, 0, 0, 0};
    const GroundPoint truth{5, 2, 100};
    PixelObservation a, b;
    CHECK(groundToPixel(cal, first, truth, a.u, a.v));
    CHECK(groundToPixel(cal, second, truth, b.u, b.v));
    const auto result = triangulate(cal, first, second, a, b);
    CHECK(result.valid && close(result.point.north_m, 5) && close(result.point.east_m, 2) && close(result.point.height_m, 100));
    CHECK(result.ray_angle_rad > .1 && result.reprojection_error_px < 1e-6);
    CHECK(!triangulate(cal, first, first, a, a).valid);
    CHECK(reconstructSparse(cal, first, second, {{a, b}}).size() == 1);
}

std::string makeMetadata(const std::string& id, const std::string& datum = "WGS84_ELLIPSOID") {
    std::ostringstream out;
    out.precision(17);
    out << "format=NAVTILE_1\nid=" << id << "\nimage_file=" << id << ".pgm\nheight_file=" << id << ".height\n"
        << "image_sha256=" << sha256(allBytes(temp / (id + ".pgm"))) << "\nheight_sha256=" << sha256(allBytes(temp / (id + ".height"))) << '\n'
        << "width=16\nheight=16\norigin_north_m=500\norigin_east_m=-150\ngsd_m=2\n"
        << "north_min_m=470\nnorth_max_m=500\neast_min_m=-150\neast_max_m=-120\n"
        << "origin_lat_deg=30\norigin_lon_deg=120\nheight_datum=" << datum << "\nacquired_utc=2026-01-01T00:00:00Z\n"
        << "texture_std=" << textureStd(texture(16, 16)) << "\nlaplacian_variance=" << laplacianVariance(texture(16, 16)) << "\nheight_std_m=5\n";
    return out.str();
}
void rebuildIndex(const std::vector<std::string>& ids) {
    std::string index = "NAVREF 1\n";
    for (const auto& id : ids) index += id + " " + id + ".meta " + sha256(allBytes(temp / (id + ".meta"))) + "\n";
    textFile(temp / "index.tsv", index); textFile(temp / "index.sha256", sha256(index) + "\n");
}

void tile_store_checks_integrity_bounds_datum_and_lru() {
    std::string error;
    for (const std::string id : {"a", "b"}) {
        CHECK(writePgm((temp / (id + ".pgm")).string(), texture(16, 16), error));
        CHECK(writeHeightGrid((temp / (id + ".height")).string(), terrain(16, 16), error));
        textFile(temp / (id + ".meta"), makeMetadata(id));
    }
    rebuildIndex({"a", "b"});
    TileStore store(1);
    CHECK(store.open(temp.string(), error));
    CHECK(store.tilesForWindow(480, 490, -145, -130).size() == 2);
    CHECK(store.tilesForWindow(0, 10, 0, 10).empty());
    const auto a = store.load("a", error);
    CHECK(a && a->dom.image.width == 16 && a->dsm.valid() && a->metadata.acquired_utc == "2026-01-01T00:00:00Z");
    CHECK(store.load("a", error) && store.cacheHits() == 1);
    CHECK(store.load("b", error) && store.residentCount() == 1);
    CHECK(a->dom.image.valid());  // 调用者持有的瓦片不因 LRU 淘汰而悬空。
    textFile(temp / "a.meta", makeMetadata("a") + "tampered=true\n");
    TileStore corrupted;
    CHECK(!corrupted.open(temp.string(), error));
    textFile(temp / "a.meta", makeMetadata("a", "AMSL")); rebuildIndex({"a", "b"});
    CHECK(!corrupted.open(temp.string(), error));
    auto bad_bounds = makeMetadata("a");
    bad_bounds.replace(bad_bounds.find("north_min_m=470"), std::string("north_min_m=470").size(), "north_min_m=400");
    textFile(temp / "a.meta", bad_bounds); rebuildIndex({"a", "b"});
    CHECK(!corrupted.open(temp.string(), error));
    textFile(temp / "a.meta", makeMetadata("a")); rebuildIndex({"a", "b"});
    CHECK(corrupted.open(temp.string(), error));
    std::ofstream(temp / "a.pgm", std::ios::binary | std::ios::app) << "corruption";
    CHECK(!corrupted.load("a", error));
}

void unavailable_learned_and_dense_backends_produce_no_measurement() {
    UnavailableLearnedMatcher absent;
    const auto missing = absent.match({}, {}, {});
    CHECK(!missing.valid && missing.reason == RejectReason::MissingWeights);
    textFile(temp / "weights.bin", "external weights");
    UnavailableLearnedMatcher no_runtime((temp / "weights.bin").string());
    CHECK(no_runtime.match({}, {}, {}).reason == RejectReason::BackendUnavailable);
    UnavailableDenseReconstructor dense;
    HeightGrid grid;
    std::string error;
    CHECK(!dense.reconstruct({}, grid, error) && !error.empty() && !grid.valid());
}

void run(void (*test)()) { ++tests; test(); }
}  // namespace

int main() {
    std::filesystem::remove_all(temp);
    std::filesystem::create_directories(temp);
    run(pgm_roundtrip_and_sha256);
    run(calibrated_projection_and_rotation);
    run(orthorectification_preserves_exposure_time);
    run(ncc_recovers_shift_and_rejects_texture_and_ambiguity);
    run(map_match_outputs_local_position_covariance_and_timestamp);
    run(planar_vo_uses_image_translation_altitude_and_exposure_interval);
    run(terrain_registration_recovers_horizontal_shift_and_vertical_bias);
    run(flat_and_planar_terrain_have_no_horizontal_fix);
    run(triangulation_recovers_real_depth_and_rejects_zero_baseline);
    run(tile_store_checks_integrity_bounds_datum_and_lru);
    run(unavailable_learned_and_dense_backends_produce_no_measurement);
    std::filesystem::remove_all(temp);
    std::cout << "companion: " << tests << " cases, " << failures << " failures\n";
    return failures ? 1 : 0;
}
