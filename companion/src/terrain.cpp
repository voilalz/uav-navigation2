#include "companion/companion.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>

namespace companion {
namespace {
struct Candidate { bool valid{}; double north{}, east{}, rmse{}, bias{}; };
Candidate evaluate(const HeightGrid& observed, const HeightGrid& reference, double dn, double de) {
    Candidate result;
    double sum=0,square=0;
    for(int y=0;y<observed.height;++y) for(int x=0;x<observed.width;++x) {
        double expected=0;
        if(!reference.sample(observed.origin_north_m-y*observed.gsd_m+dn,
                             observed.origin_east_m+x*observed.gsd_m+de,expected)) return result;
        const double difference=observed.at(x,y)-expected;
        sum+=difference; square+=difference*difference;
    }
    const double n=static_cast<double>(observed.values.size());
    result.valid=true; result.north=dn; result.east=de; result.bias=sum/n;
    result.rmse=std::sqrt(std::max(0.0,square/n-result.bias*result.bias));
    return result;
}
double standardDeviation(const std::vector<double>& values) {
    double mean=0; for(double v:values) mean+=v; mean/=values.size();
    double var=0; for(double v:values) var+=(v-mean)*(v-mean);
    return std::sqrt(var/values.size());
}
}  // namespace

bool HeightGrid::valid() const {
    return width>=2 && height>=2 && width<=16384 && height<=16384
        && static_cast<std::size_t>(width)*height<=64u*1024u*1024u
        && values.size()==static_cast<std::size_t>(width)*height
        && std::isfinite(origin_north_m) && std::isfinite(origin_east_m)
        && std::isfinite(gsd_m) && gsd_m>0 && height_datum=="WGS84_ELLIPSOID"
        && std::all_of(values.begin(),values.end(),[](double v){return std::isfinite(v);});
}
bool HeightGrid::sample(double north, double east, double& height_out) const {
    // 栅格内容在加载时校验一次；采样热路径不逐点扫描整个 DSM。
    if(width<2 || height<2 || values.size()!=static_cast<std::size_t>(width)*height
        || !std::isfinite(gsd_m) || gsd_m<=0 || !std::isfinite(north) || !std::isfinite(east)
        || !std::isfinite(origin_north_m) || !std::isfinite(origin_east_m)) return false;
    double x=(east-origin_east_m)/gsd_m,y=(origin_north_m-north)/gsd_m;
    if(!std::isfinite(x) || !std::isfinite(y) || x < -1e-8 || y < -1e-8
        || x>width-1+1e-8 || y>height-1+1e-8) return false;
    x=std::clamp(x,0.0,static_cast<double>(width-1)); y=std::clamp(y,0.0,static_cast<double>(height-1));
    const int x0=static_cast<int>(std::floor(x)),y0=static_cast<int>(std::floor(y));
    const int x1=std::min(x0+1,width-1),y1=std::min(y0+1,height-1);
    const double ax=x-x0,ay=y-y0;
    height_out=(1-ay)*((1-ax)*at(x0,y0)+ax*at(x1,y0))+ay*((1-ax)*at(x0,y1)+ax*at(x1,y1));
    return std::isfinite(height_out);
}
bool readHeightGrid(const std::string& path, HeightGrid& grid, std::string& error) {
    // 几何来自元数据；文件只保留行列与高程，避免双重地理信息冲突。
    const double north=grid.origin_north_m,east=grid.origin_east_m,gsd=grid.gsd_m;
    const std::string datum=grid.height_datum;
    grid={}; grid.origin_north_m=north; grid.origin_east_m=east; grid.gsd_m=gsd; grid.height_datum=datum;
    std::ifstream stream(path);
    std::string magic; int version=0,width=0,height=0;
    if(!(stream>>magic>>version>>width>>height) || magic!="NAVHEIGHT" || version!=1
        || width<2 || height<2 || width>16384 || height>16384
        || static_cast<std::size_t>(width)*height>64u*1024u*1024u) {
        error="invalid height grid header: "+path; return false;
    }
    std::vector<double> values(static_cast<std::size_t>(width)*height);
    for(auto& value:values) if(!(stream>>value) || !std::isfinite(value)) { error="invalid/nonfinite height samples: "+path; return false; }
    std::string extra;
    if(stream>>extra) { error="extra height samples: "+path; return false; }
    grid.width=width; grid.height=height; grid.values=std::move(values); error.clear(); return true;
}
bool writeHeightGrid(const std::string& path, const HeightGrid& grid, std::string& error) {
    if(!grid.valid()) { error="invalid height grid or height datum"; return false; }
    std::ofstream stream(path);
    if(!stream) { error="cannot write height grid: "+path; return false; }
    stream<<"NAVHEIGHT 1 "<<grid.width<<' '<<grid.height<<'\n'<<std::setprecision(17);
    for(int y=0;y<grid.height;++y) {
        for(int x=0;x<grid.width;++x) { if(x) stream<<' '; stream<<grid.at(x,y); }
        stream<<'\n';
    }
    if(!stream) { error="height grid write failed: "+path; return false; }
    error.clear(); return true;
}

TerrainResult registerTerrain(const HeightGrid& observed, const HeightGrid& reference,
                               double search_radius, Timestamp exposure, const TerrainOptions& options) {
    TerrainResult result; result.t_us=exposure;
    if(observed.height_datum!="WGS84_ELLIPSOID" || reference.height_datum!=observed.height_datum) {
        result.reason=RejectReason::HeightDatum; return result;
    }
    if(!observed.valid() || !reference.valid() || !std::isfinite(search_radius) || search_radius<0
        || !std::isfinite(options.step_m) || options.step_m<0 || exposure==0
        || !std::isfinite(options.min_height_std_m) || options.min_height_std_m<=0
        || !std::isfinite(options.max_rmse_m) || options.max_rmse_m<=0
        || !std::isfinite(options.min_rmse_gap_m) || options.min_rmse_gap_m<=0
        || !std::isfinite(options.min_gradient_eigenvalue) || options.min_gradient_eigenvalue<=0) return result;
    const double step=options.step_m>0?options.step_m:reference.gsd_m;
    if(search_radius/step>512) return result;  // 防止错误配置导致无界 CPU 搜索。
    const double height_std=standardDeviation(observed.values);
    result.quality.texture_std=height_std;
    result.quality.samples=observed.values.size();
    if(height_std<options.min_height_std_m) { result.reason=RejectReason::LowTexture; return result; }
    const int steps=static_cast<int>(std::floor(search_radius/step));
    Candidate best; best.rmse=std::numeric_limits<double>::infinity();
    std::vector<Candidate> candidates;
    for(int n=-steps;n<=steps;++n) for(int e=-steps;e<=steps;++e) {
        const auto c=evaluate(observed,reference,n*step,e*step);
        if(!c.valid) continue;
        candidates.push_back(c); if(c.rmse<best.rmse) best=c;
    }
    if(!best.valid) { result.reason=RejectReason::OutOfBounds; return result; }
    double second=std::numeric_limits<double>::infinity();
    for(const auto& c:candidates) if(std::hypot(c.north-best.north,c.east-best.east)>2*step)
        second=std::min(second,c.rmse);
    // 实际高程残差搜索细化至 1/8 网格，不凭空报告子像素精度。
    double fine=step*.5;
    for(int level=0;level<3;++level,fine*=.5) {
        const auto center=best;
        for(int n=-1;n<=1;++n) for(int e=-1;e<=1;++e) {
            const double dn=center.north+n*fine,de=center.east+e*fine;
            if(std::abs(dn)>search_radius || std::abs(de)>search_radius) continue;
            const auto c=evaluate(observed,reference,dn,de);
            if(c.valid && c.rmse<best.rmse) best=c;
        }
    }
    result.delta_north_m=best.north; result.delta_east_m=best.east;
    result.vertical_bias_m=best.bias; result.height_correction_m=-best.bias; result.rmse_m=best.rmse;
    result.quality.residual_m=best.rmse; result.quality.valid_fraction=1;
    result.quality.score=1/(1+best.rmse*best.rmse);
    result.quality.second_score=std::isfinite(second)?1/(1+second*second):0;
    if(best.rmse>options.max_rmse_m) { result.reason=RejectReason::LowCorrelation; return result; }
    if(std::isfinite(second) && second-best.rmse<options.min_rmse_gap_m) {
        result.reason=RejectReason::Ambiguous; return result;
    }
    double sn=0,se=0,snn=0,see=0,sne=0,count=0;
    const double h=reference.gsd_m;
    for(int y=0;y<observed.height;++y) for(int x=0;x<observed.width;++x) {
        const double n=observed.origin_north_m-y*observed.gsd_m+best.north;
        const double e=observed.origin_east_m+x*observed.gsd_m+best.east;
        double np=0,nm=0,ep=0,em=0;
        if(!reference.sample(n+h,e,np) || !reference.sample(n-h,e,nm)
            || !reference.sample(n,e+h,ep) || !reference.sample(n,e-h,em)) continue;
        const double gn=(np-nm)/(2*h),ge=(ep-em)/(2*h);
        sn+=gn; se+=ge; snn+=gn*gn; see+=ge*ge; sne+=gn*ge; ++count;
    }
    if(count<16) { result.reason=RejectReason::PoorGeometry; return result; }
    // 消去垂直偏置后须使用去均值梯度信息；斜平面也无法定位。
    const double a=snn-sn*sn/count,b=sne-sn*se/count,c=see-se*se/count;
    const double eig=(a+c-std::hypot(a-c,2*b))/(2*count);
    const double determinant=a*c-b*b;
    if(eig<options.min_gradient_eigenvalue || determinant<=1e-12) { result.reason=RejectReason::PoorGeometry; return result; }
    const double noise=std::max(.0625,best.rmse*best.rmse);
    const double resolution_floor=reference.gsd_m*reference.gsd_m/16;
    result.covariance[0]=noise*c/determinant+resolution_floor;
    result.covariance[4]=noise*a/determinant+resolution_floor;
    result.covariance[1]=result.covariance[3]=-noise*b/determinant;
    result.covariance[8]=noise/observed.values.size()+.0625;
    result.sigma_m=std::sqrt(std::max(result.covariance[0],result.covariance[4]));
    result.valid=true; result.reason=RejectReason::None; return result;
}
}  // namespace companion
