#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace companion {

using Timestamp = std::uint64_t;

// 本模块在局部切平面使用北/东（m）及向上的椭球高（m）。
struct Image {
    int width{};
    int height{};
    std::vector<float> pixels;  // 灰度归一到 [0,255]，输入必须已去畸变。
    float& at(int x, int y) { return pixels[static_cast<std::size_t>(y) * width + x]; }
    float at(int x, int y) const { return pixels[static_cast<std::size_t>(y) * width + x]; }
    bool valid() const;
};

bool readPgm(const std::string& path, Image& image, std::string& error);
bool writePgm(const std::string& path, const Image& image, std::string& error);
std::string sha256(const std::string& bytes);
bool fileSha256(const std::string& path, std::string& digest, std::string& error);
double textureStd(const Image& image);
double laplacianVariance(const Image& image);

struct Calibration { double fx{}, fy{}, cx{}, cy{}; bool valid() const; };
struct CameraPose {
    double north_m{}, east_m{}, height_m{};
    double roll_rad{}, pitch_rad{}, yaw_rad{};  // FRD 机体到 NED。
};
struct GroundPoint { double north_m{}, east_m{}, height_m{}; };

// 零姿态时图像右为东、图像上为北、光轴为下。已标定固定下视安装。
bool pixelToGround(const Calibration&, const CameraPose&, double u, double v,
                   double plane_height_m, GroundPoint& point);
bool groundToPixel(const Calibration&, const CameraPose&, const GroundPoint&,
                   double& u, double& v);

struct OrthoImage {
    Image image;
    std::vector<std::uint8_t> mask;
    double center_north_m{}, center_east_m{}, gsd_m{}, camera_height_m{};
    Timestamp t_us{};
};
OrthoImage orthorectify(const Image&, const Calibration&, const CameraPose&,
                       double plane_height_m, double gsd_m, int width, int height,
                       Timestamp exposure_t_us);

enum class RejectReason {
    None, InvalidInput, TimeOrder, OutOfBounds, LowTexture, LowCorrelation,
    Ambiguous, PoorGeometry, HeightDatum, Integrity, MissingWeights,
    BackendUnavailable
};
const char* reasonName(RejectReason);

struct MatchQuality {
    double score{}, second_score{}, psr{}, texture_std{}, valid_fraction{};
    double residual_m{};
    std::size_t samples{};
};
struct MatchOptions {
    double min_texture_std{6.0};
    double min_score{0.72};
    double min_peak_gap{0.06};
    double min_psr{3.0};
    double min_valid_fraction{0.70};
    int coarse_factor{4};
    int coarse_candidates{16};
    int peak_exclusion_px{3};
};
struct PixelWindow { int min_x{}, min_y{}, max_x{}, max_y{}; };
struct PixelMatch {
    bool valid{};
    double x{}, y{}, sigma_px{};  // 模板左上角在参考图内的位置。
    MatchQuality quality{};
    RejectReason reason{RejectReason::InvalidInput};
};
PixelMatch matchNcc(const Image& query, const std::vector<std::uint8_t>& mask,
                    const Image& reference, PixelWindow search,
                    const MatchOptions& options = {});

struct ImageMap {
    Image image;
    double origin_north_m{}, origin_east_m{}, gsd_m{};  // 左上像素中心。
};
struct SearchPrior { double north_m{}, east_m{}, radius_m{}; };
enum class FixSource : std::uint8_t { Vision = 1, Terrain3D = 2 };
struct MatchResult {
    bool valid{};
    Timestamp t_us{};  // 原始曝光中点，不是处理完成时刻。
    double north_m{}, east_m{}, height_m{}, sigma_m{};
    std::array<double, 9> covariance{};  // 局部 NED；未观测维度不应融合。
    std::uint8_t dimensions{2};
    FixSource source{FixSource::Vision};
    MatchQuality quality{};
    RejectReason reason{RejectReason::InvalidInput};
};
MatchResult matchAbsolute(const OrthoImage&, const ImageMap&, const SearchPrior&,
                          const MatchOptions& options = {});

struct Frame {
    Image image;
    Timestamp exposure_t_us{};
    double altitude_agl_m{}, altitude_sigma_m{1.0};
    double roll_rad{}, pitch_rad{}, yaw_rad{};
};
struct VoOptions {
    int max_shift_px{12};
    double min_dt_s{0.005}, max_dt_s{2.0};
    MatchOptions match{};
};
struct VelocityResult {
    bool valid{};
    Timestamp t_us{}, interval_begin_us{}, interval_end_us{};
    std::array<double, 3> velocity_ned{}, velocity_body{};
    std::array<double, 9> covariance{};  // NED 协方差，水平两维有效。
    std::uint8_t dimensions{2};
    double delta_north_m{}, delta_east_m{}, scale_m_per_px{};
    MatchQuality quality{};
    RejectReason reason{RejectReason::InvalidInput};
};
VelocityResult estimatePlanarVelocity(const Frame& previous, const Frame& current,
                                      const Calibration&, const VoOptions& options = {});

struct HeightGrid {
    int width{}, height{};
    std::vector<double> values;
    double origin_north_m{}, origin_east_m{}, gsd_m{};
    std::string height_datum{"WGS84_ELLIPSOID"};
    double at(int x, int y) const { return values[static_cast<std::size_t>(y) * width + x]; }
    bool valid() const;
    bool sample(double north_m, double east_m, double& height_m) const;
};
bool readHeightGrid(const std::string& path, HeightGrid&, std::string& error);
bool writeHeightGrid(const std::string& path, const HeightGrid&, std::string& error);
struct TerrainOptions {
    double step_m{};  // 0 采用参考网格分辨率。
    double min_height_std_m{0.5}, max_rmse_m{2.0}, min_rmse_gap_m{0.05};
    double min_gradient_eigenvalue{1e-4};
};
struct TerrainResult {
    bool valid{};
    Timestamp t_us{};
    double delta_north_m{}, delta_east_m{}, vertical_bias_m{}, height_correction_m{};
    double rmse_m{}, sigma_m{};
    std::array<double, 9> covariance{};  // 北/东/向下高度修正。
    MatchQuality quality{};
    RejectReason reason{RejectReason::InvalidInput};
};
TerrainResult registerTerrain(const HeightGrid& observed, const HeightGrid& reference,
                               double search_radius_m, Timestamp exposure_t_us,
                               const TerrainOptions& options = {});

struct PixelObservation { double u{}, v{}; };
struct TriangulationResult {
    bool valid{};
    GroundPoint point{};
    double ray_angle_rad{}, reprojection_error_px{}, ray_gap_m{};
    RejectReason reason{RejectReason::PoorGeometry};
};
TriangulationResult triangulate(const Calibration&, const CameraPose& first,
                                const CameraPose& second, const PixelObservation& a,
                                const PixelObservation& b);
struct StereoCorrespondence { PixelObservation first{}, second{}; };
std::vector<GroundPoint> reconstructSparse(const Calibration&, const CameraPose& first,
                                           const CameraPose& second,
                                           const std::vector<StereoCorrespondence>&);

// 未提供网络权重/运行时或稠密 BA 实现时显式无效，绝不制造量测。
class LearnedMatcher {
public:
    virtual ~LearnedMatcher() = default;
    virtual MatchResult match(const OrthoImage&, const ImageMap&, const SearchPrior&) const = 0;
};
class UnavailableLearnedMatcher final : public LearnedMatcher {
public:
    explicit UnavailableLearnedMatcher(std::string weights_path = {});
    MatchResult match(const OrthoImage&, const ImageMap&, const SearchPrior&) const override;
private:
    std::string weights_path_;
};
class DenseReconstructor {
public:
    virtual ~DenseReconstructor() = default;
    virtual bool reconstruct(const std::vector<Frame>&, HeightGrid&, std::string& error) = 0;
};
class UnavailableDenseReconstructor final : public DenseReconstructor {
public:
    bool reconstruct(const std::vector<Frame>&, HeightGrid&, std::string& error) override;
};

struct TileMetadata {
    std::string id, image_file, height_file, image_sha256, height_sha256;
    int width{}, height{};
    double origin_north_m{}, origin_east_m{}, gsd_m{};
    double north_min_m{}, north_max_m{}, east_min_m{}, east_max_m{};
    double origin_lat_deg{}, origin_lon_deg{};
    std::string height_datum, acquired_utc;
    double texture_std{}, laplacian_variance{}, height_std_m{};
};
struct Tile { TileMetadata metadata; ImageMap dom; HeightGrid dsm; };
class TileStore {
public:
    explicit TileStore(std::size_t capacity_tiles = 8);
    bool open(const std::string& directory, std::string& error);
    std::vector<TileMetadata> tilesForWindow(double north_min_m, double north_max_m,
                                             double east_min_m, double east_max_m) const;
    std::shared_ptr<const Tile> load(const std::string& id, std::string& error);
    std::size_t residentCount() const { return cache_.size(); }
    std::size_t cacheHits() const { return cache_hits_; }
    const std::vector<TileMetadata>& metadata() const { return metadata_; }
private:
    struct CachedTile { std::shared_ptr<const Tile> tile; std::list<std::string>::iterator lru; };
    std::string directory_;
    std::size_t capacity_, cache_hits_{};
    std::vector<TileMetadata> metadata_;
    std::list<std::string> lru_;
    std::unordered_map<std::string, CachedTile> cache_;
};

}  // namespace companion
