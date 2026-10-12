#include "companion/companion.hpp"

#include <algorithm>
#include <cmath>
#include <set>

namespace companion {
namespace {
struct TemplateSample { int x{}, y{}; double centered{}; };
struct TemplateData { std::vector<TemplateSample> samples; double variance{}, stddev{}; };
struct Candidate { int x{}, y{}; double score{-1}; };
TemplateData prepare(const Image& query, const std::vector<std::uint8_t>& mask) {
    TemplateData out;
    double sum=0;
    for (int y=0; y<query.height; ++y) for (int x=0; x<query.width; ++x) {
        if (!mask.empty() && !mask[static_cast<std::size_t>(y)*query.width+x]) continue;
        out.samples.push_back({x,y,query.at(x,y)}); sum += query.at(x,y);
    }
    if (out.samples.empty()) return out;
    const double mean = sum/out.samples.size();
    for (auto& s : out.samples) { s.centered -= mean; out.variance += s.centered*s.centered; }
    out.stddev = std::sqrt(out.variance/out.samples.size());
    return out;
}
double correlation(const TemplateData& q, const Image& reference, int x0, int y0) {
    if (q.samples.empty() || q.variance <= 1e-8) return -1;
    double sum=0, square=0, product=0;
    for (const auto& s : q.samples) {
        const double r = reference.at(x0+s.x,y0+s.y);
        sum += r; square += r*r; product += s.centered*r;
    }
    const double variance = std::max(0.0, square-sum*sum/q.samples.size());
    if (variance <= 1e-8) return -1;
    return std::clamp(product/std::sqrt(q.variance*variance), -1.0, 1.0);
}
Image pooled(const Image& input, int factor, const std::vector<std::uint8_t>& mask,
             std::vector<std::uint8_t>* output_mask) {
    Image out;
    out.width = (input.width+factor-1)/factor; out.height = (input.height+factor-1)/factor;
    out.pixels.resize(static_cast<std::size_t>(out.width)*out.height);
    if (output_mask) output_mask->resize(out.pixels.size(), 0);
    for (int y=0; y<out.height; ++y) for (int x=0; x<out.width; ++x) {
        double sum=0; int count=0, area=0;
        for (int dy=0; dy<factor && y*factor+dy<input.height; ++dy)
            for (int dx=0; dx<factor && x*factor+dx<input.width; ++dx) {
                ++area; const int u=x*factor+dx,v=y*factor+dy;
                if (!mask.empty() && !mask[static_cast<std::size_t>(v)*input.width+u]) continue;
                sum += input.at(u,v); ++count;
            }
        out.at(x,y) = count ? static_cast<float>(sum/count) : 0;
        if (output_mask) (*output_mask)[static_cast<std::size_t>(y)*out.width+x] = count*2 >= area && count > 0;
    }
    return out;
}
bool matchOptionsValid(const MatchOptions& o) {
    return std::isfinite(o.min_texture_std) && o.min_texture_std > 0 && std::isfinite(o.min_score)
        && o.min_score >= 0 && o.min_score <= 1 && std::isfinite(o.min_peak_gap) && o.min_peak_gap > 0
        && o.min_peak_gap <= 2 && std::isfinite(o.min_psr) && o.min_psr >= 0
        && std::isfinite(o.min_valid_fraction) && o.min_valid_fraction > 0 && o.min_valid_fraction <= 1
        && o.coarse_factor >= 1 && o.coarse_factor <= 16 && o.coarse_candidates >= 1
        && o.coarse_candidates <= 64 && o.peak_exclusion_px >= 1;
}
bool scalar(double a) { return std::isfinite(a); }
}  // namespace

PixelMatch matchNcc(const Image& query, const std::vector<std::uint8_t>& mask,
                    const Image& reference, PixelWindow window, const MatchOptions& options) {
    PixelMatch result;
    if (!query.valid() || !reference.valid() || !matchOptionsValid(options)
        || (!mask.empty() && mask.size() != query.pixels.size())) return result;
    window.min_x = std::max(0,window.min_x); window.min_y = std::max(0,window.min_y);
    window.max_x = std::min(reference.width-query.width,window.max_x);
    window.max_y = std::min(reference.height-query.height,window.max_y);
    if (window.max_x < window.min_x || window.max_y < window.min_y) {
        result.reason=RejectReason::OutOfBounds; return result;
    }
    const auto full = prepare(query,mask);
    result.quality.texture_std = full.stddev;
    result.quality.samples = full.samples.size();
    result.quality.valid_fraction = static_cast<double>(full.samples.size())/query.pixels.size();
    if (full.samples.size() < 16 || result.quality.valid_fraction < options.min_valid_fraction) {
        result.reason=RejectReason::PoorGeometry; return result;
    }
    if (full.stddev < options.min_texture_std) { result.reason=RejectReason::LowTexture; return result; }
    int factor = options.coarse_factor;
    while (factor > 1 && (query.width/factor < 6 || query.height/factor < 6)) factor /= 2;
    std::vector<std::uint8_t> coarse_mask;
    const Image qsmall = pooled(query,factor,mask,&coarse_mask);
    const Image rsmall = pooled(reference,factor,{},nullptr);
    const auto coarse = prepare(qsmall,coarse_mask);
    std::vector<Candidate> coarse_scores;
    for (int y=window.min_y/factor; y<=std::min(window.max_y/factor,rsmall.height-qsmall.height); ++y)
        for (int x=window.min_x/factor; x<=std::min(window.max_x/factor,rsmall.width-qsmall.width); ++x)
            coarse_scores.push_back({x*factor,y*factor,correlation(coarse,rsmall,x,y)});
    if (coarse_scores.empty()) { result.reason=RejectReason::OutOfBounds; return result; }
    std::sort(coarse_scores.begin(),coarse_scores.end(),[](const Candidate& a,const Candidate& b) {
        if (a.score != b.score) return a.score>b.score;
        return a.y<b.y || (a.y==b.y && a.x<b.x);
    });
    std::vector<Candidate> hypotheses;
    for (const auto& c : coarse_scores) {
        bool near=false;
        for (const auto& h : hypotheses) if (std::max(std::abs(c.x-h.x),std::abs(c.y-h.y)) <= factor*2) { near=true; break; }
        if (!near) hypotheses.push_back(c);
        if (static_cast<int>(hypotheses.size()) == options.coarse_candidates) break;
    }
    // 保留多个独立峰；精配准始终在原始像素上计算 NCC。
    std::set<std::pair<int,int>> evaluated;
    std::vector<Candidate> fine_scores;
    Candidate best;
    for (const auto& h : hypotheses) {
        for (int y=std::max(window.min_y,h.y-factor-1); y<=std::min(window.max_y,h.y+factor+1); ++y)
            for (int x=std::max(window.min_x,h.x-factor-1); x<=std::min(window.max_x,h.x+factor+1); ++x) {
                if (!evaluated.insert({x,y}).second) continue;
                Candidate c{x,y,correlation(full,reference,x,y)};
                fine_scores.push_back(c);
                if (c.score>best.score) best=c;
            }
    }
    double second=-1, sum=0,square=0;
    std::size_t side_count=0;
    for (const auto& c : fine_scores) {
        if (std::hypot(static_cast<double>(c.x-best.x),static_cast<double>(c.y-best.y)) <= options.peak_exclusion_px) continue;
        second=std::max(second,c.score);
    }
    // PSR 从整个搜索窗口等距采样；仅使用入选高峰会使旁瓣统计偏高。
    for (int y=window.min_y; y<=window.max_y; y+=factor) for (int x=window.min_x; x<=window.max_x; x+=factor) {
        if (std::hypot(static_cast<double>(x-best.x),static_cast<double>(y-best.y)) <= options.peak_exclusion_px) continue;
        const double score=correlation(full,reference,x,y);
        second=std::max(second,score); sum+=score; square+=score*score; ++side_count;
    }
    const double mean = side_count ? sum/side_count : 0;
    const double deviation = side_count ? std::sqrt(std::max(0.0,square/side_count-mean*mean)) : 0;
    result.quality.score=best.score; result.quality.second_score=second;
    result.quality.psr=deviation>1e-8 ? (best.score-mean)/deviation : (best.score>mean ? 100 : 0);
    result.x=best.x; result.y=best.y;
    if (best.score<options.min_score) { result.reason=RejectReason::LowCorrelation; return result; }
    if (best.score-second<options.min_peak_gap || result.quality.psr<options.min_psr) {
        result.reason=RejectReason::Ambiguous; return result;
    }
    double curvature=0;
    auto refine = [&](bool horizontal) {
        if (horizontal ? (best.x<=window.min_x || best.x>=window.max_x) : (best.y<=window.min_y || best.y>=window.max_y)) return 0.0;
        const double a=correlation(full,reference,best.x-(horizontal?1:0),best.y-(horizontal?0:1));
        const double b=correlation(full,reference,best.x+(horizontal?1:0),best.y+(horizontal?0:1));
        const double denominator=a-2*best.score+b;
        curvature+=std::max(0.0,-denominator);
        return denominator< -1e-8 ? std::clamp(.5*(a-b)/denominator,-.5,.5) : 0.0;
    };
    result.x+=refine(true); result.y+=refine(false);
    result.sigma_px=std::clamp(std::sqrt((std::max(0.0,1-best.score)+.005)/(curvature*.5+1e-6)),.25,5.0);
    result.valid=true; result.reason=RejectReason::None; return result;
}

MatchResult matchAbsolute(const OrthoImage& query, const ImageMap& map, const SearchPrior& prior,
                          const MatchOptions& options) {
    MatchResult result;
    result.t_us=query.t_us; result.height_m=query.camera_height_m;
    if (!scalar(query.gsd_m) || !scalar(map.gsd_m) || query.gsd_m<=0 || map.gsd_m<=0
        || std::abs(query.gsd_m-map.gsd_m)>1e-6*map.gsd_m || !scalar(map.origin_north_m)
        || !scalar(map.origin_east_m) || !scalar(prior.north_m) || !scalar(prior.east_m)
        || !scalar(prior.radius_m) || prior.radius_m<0 || prior.radius_m>1e7
        || !scalar(query.camera_height_m) || query.t_us==0 || !query.image.valid() || !map.image.valid()) return result;
    const double cx=(query.image.width-1)*.5,cy=(query.image.height-1)*.5;
    const double x=(prior.east_m-map.origin_east_m)/map.gsd_m-cx;
    const double y=(map.origin_north_m-prior.north_m)/map.gsd_m-cy;
    const double radius=prior.radius_m/map.gsd_m;
    if (x+radius<0 || y+radius<0 || x-radius>map.image.width-query.image.width
        || y-radius>map.image.height-query.image.height) { result.reason=RejectReason::OutOfBounds; return result; }
    // 先截断到图幅，防止极端有限先验导致浮点到整数溢出。
    PixelWindow window{static_cast<int>(std::ceil(std::max(0.0,x-radius))),static_cast<int>(std::ceil(std::max(0.0,y-radius))),
        static_cast<int>(std::floor(std::min(static_cast<double>(map.image.width-query.image.width),x+radius))),
        static_cast<int>(std::floor(std::min(static_cast<double>(map.image.height-query.image.height),y+radius)))};
    const auto match=matchNcc(query.image,query.mask,map.image,window,options);
    result.reason=match.reason; result.quality=match.quality;
    if (!match.valid) return result;
    result.north_m=map.origin_north_m-(match.y+cy)*map.gsd_m;
    result.east_m=map.origin_east_m+(match.x+cx)*map.gsd_m;
    result.sigma_m=match.sigma_px*map.gsd_m;
    result.covariance[0]=result.covariance[4]=result.sigma_m*result.sigma_m;
    result.covariance[8]=1e6;
    result.valid=true; return result;
}

VelocityResult estimatePlanarVelocity(const Frame& previous, const Frame& current,
                                      const Calibration& k, const VoOptions& options) {
    VelocityResult result;
    result.t_us=current.exposure_t_us; result.interval_begin_us=previous.exposure_t_us;
    result.interval_end_us=current.exposure_t_us;
    if (previous.exposure_t_us==0 || current.exposure_t_us<=previous.exposure_t_us) { result.reason=RejectReason::TimeOrder; return result; }
    const double dt=(current.exposure_t_us-previous.exposure_t_us)*1e-6;
    if (!scalar(options.min_dt_s) || !scalar(options.max_dt_s) || options.min_dt_s<=0
        || options.max_dt_s<options.min_dt_s || dt<options.min_dt_s || dt>options.max_dt_s) {
        result.reason=RejectReason::TimeOrder; return result;
    }
    if (!k.valid() || !previous.image.valid() || !current.image.valid() || !scalar(previous.altitude_agl_m)
        || !scalar(current.altitude_agl_m) || previous.altitude_agl_m<=1 || current.altitude_agl_m<=1
        || !scalar(previous.altitude_sigma_m) || !scalar(current.altitude_sigma_m)
        || previous.altitude_sigma_m<0 || current.altitude_sigma_m<0 || options.max_shift_px<1) return result;
    const int width=std::min(previous.image.width,current.image.width),height=std::min(previous.image.height,current.image.height);
    const int margin=options.max_shift_px+1;
    if (width-2*margin<8 || height-2*margin<8) return result;
    const double gsd=previous.altitude_agl_m/((k.fx+k.fy)*.5);
    const auto a=orthorectify(previous.image,k,{0,0,previous.altitude_agl_m,previous.roll_rad,previous.pitch_rad,previous.yaw_rad},0,gsd,width,height,previous.exposure_t_us);
    const auto b=orthorectify(current.image,k,{0,0,current.altitude_agl_m,current.roll_rad,current.pitch_rad,current.yaw_rad},0,gsd,width,height,current.exposure_t_us);
    if (!a.image.valid() || !b.image.valid()) return result;
    Image query{width-2*margin,height-2*margin,{}};
    query.pixels.resize(static_cast<std::size_t>(query.width)*query.height);
    std::vector<std::uint8_t> mask(query.pixels.size());
    for (int y=0;y<query.height;++y) for(int x=0;x<query.width;++x) {
        query.at(x,y)=a.image.at(x+margin,y+margin);
        mask[static_cast<std::size_t>(y)*query.width+x]=a.mask[static_cast<std::size_t>(y+margin)*width+x+margin];
    }
    const auto match=matchNcc(query,mask,b.image,{1,1,2*margin-1,2*margin-1},options.match);
    result.reason=match.reason; result.quality=match.quality;
    if (!match.valid) return result;
    const int ix=static_cast<int>(std::lround(match.x)),iy=static_cast<int>(std::lround(match.y));
    std::size_t supported=0,total=0;
    for(int y=0;y<query.height;++y) for(int x=0;x<query.width;++x) {
        if(!mask[static_cast<std::size_t>(y)*query.width+x]) continue;
        ++total; if(b.mask[static_cast<std::size_t>(y+iy)*width+x+ix]) ++supported;
    }
    result.quality.valid_fraction=total?static_cast<double>(supported)/total:0;
    if(result.quality.valid_fraction<options.match.min_valid_fraction) { result.reason=RejectReason::PoorGeometry; return result; }
    result.scale_m_per_px=gsd;
    // 地面特征的图像运动与相机运动方向相反；图像行向南。
    result.delta_north_m=(match.y-margin)*gsd; result.delta_east_m=-(match.x-margin)*gsd;
    result.velocity_ned={result.delta_north_m/dt,result.delta_east_m/dt,0};
    const double cr=std::cos(current.roll_rad),sr=std::sin(current.roll_rad);
    const double cp=std::cos(current.pitch_rad),sp=std::sin(current.pitch_rad);
    const double cy=std::cos(current.yaw_rad),sy=std::sin(current.yaw_rad);
    const double n=result.velocity_ned[0],e=result.velocity_ned[1];
    result.velocity_body={cy*cp*n+sy*cp*e,(cy*sp*sr-sy*cr)*n+(sy*sp*sr+cy*cr)*e,
        (cy*sp*cr+sy*sr)*n+(sy*sp*cr-cy*sr)*e};
    const double relative_variance=(previous.altitude_sigma_m*previous.altitude_sigma_m
        +current.altitude_sigma_m*current.altitude_sigma_m)/(4*previous.altitude_agl_m*previous.altitude_agl_m);
    const double image_variance=match.sigma_px*match.sigma_px*gsd*gsd/(dt*dt);
    result.covariance[0]=image_variance+n*n*relative_variance;
    result.covariance[4]=image_variance+e*e*relative_variance;
    result.covariance[1]=result.covariance[3]=n*e*relative_variance;
    result.covariance[8]=1e6;
    result.valid=true; result.reason=RejectReason::None; return result;
}
}  // namespace companion
