#include "companion/companion.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <sstream>

namespace {
using namespace companion;
template<std::size_t N> void array(const std::array<double,N>& a) {
    std::cout<<'['; for(std::size_t i=0;i<N;++i) { if(i) std::cout<<','; std::cout<<a[i]; } std::cout<<']';
}
void quality(const MatchQuality& q) {
    std::cout<<",\"quality\":{\"score\":"<<q.score<<",\"second_score\":"<<q.second_score
        <<",\"psr\":"<<q.psr<<",\"texture_std\":"<<q.texture_std<<",\"samples\":"<<q.samples
        <<",\"valid_fraction\":"<<q.valid_fraction<<'}';
}
void printFix(const MatchResult& m) {
    std::cout<<"{\"type\":\"image_fix\",\"valid\":"<<m.valid<<",\"t_us\":"<<m.t_us
        <<",\"source\":\"Vision\",\"frame\":\"local_NED\",\"dimensions\":"<<static_cast<int>(m.dimensions)
        <<",\"north_m\":"<<m.north_m<<",\"east_m\":"<<m.east_m<<",\"height_m\":"<<m.height_m
        <<",\"sigma_m\":"<<m.sigma_m<<",\"covariance\":";
    array(m.covariance); quality(m.quality); std::cout<<",\"reason\":\""<<reasonName(m.reason)<<"\"}\n";
}
void printVo(const VelocityResult& v) {
    std::cout<<"{\"type\":\"vo\",\"valid\":"<<v.valid<<",\"t_us\":"<<v.t_us
        <<",\"interval_begin_us\":"<<v.interval_begin_us<<",\"interval_end_us\":"<<v.interval_end_us
        <<",\"frame\":\"NED\",\"covariance_frame\":\"NED\",\"dimensions\":"<<static_cast<int>(v.dimensions)
        <<",\"velocity_ned\":";
    array(v.velocity_ned); std::cout<<",\"velocity_body\":"; array(v.velocity_body);
    std::cout<<",\"scale_m_per_px\":"<<v.scale_m_per_px<<",\"covariance\":"; array(v.covariance);
    quality(v.quality); std::cout<<",\"reason\":\""<<reasonName(v.reason)<<"\"}\n";
}
void printTerrain(const TerrainResult& t) {
    std::cout<<"{\"type\":\"terrain_registration\",\"valid\":"<<t.valid<<",\"t_us\":"<<t.t_us
        <<",\"delta_north_m\":"<<t.delta_north_m<<",\"delta_east_m\":"<<t.delta_east_m
        <<",\"vertical_bias_m\":"<<t.vertical_bias_m<<",\"height_correction_m\":"<<t.height_correction_m
        <<",\"rmse_m\":"<<t.rmse_m<<",\"sigma_m\":"<<t.sigma_m<<",\"covariance\":";
    array(t.covariance); quality(t.quality); std::cout<<",\"reason\":\""<<reasonName(t.reason)<<"\"}\n";
}
Image syntheticImage(int width,int height) {
    Image image{width,height,std::vector<float>(static_cast<std::size_t>(width)*height)};
    for(int y=0;y<height;++y) for(int x=0;x<width;++x) {
        std::uint32_t z=static_cast<std::uint32_t>(x*73856093u)^static_cast<std::uint32_t>(y*19349663u)^92837111u;
        z^=z>>13; z*=1274126177u;
        image.at(x,y)=static_cast<float>(std::lround(40+15*std::sin(x*.27)+12*std::cos(y*.31)+8*std::sin(x*.13+y*.21)+z%160));
    }
    return image;
}
Image crop(const Image& image,int x0,int y0,int width,int height) {
    Image out{width,height,std::vector<float>(static_cast<std::size_t>(width)*height)};
    for(int y=0;y<height;++y) for(int x=0;x<width;++x) out.at(x,y)=image.at(x+x0,y+y0);
    return out;
}
HeightGrid syntheticTerrain(int width,int height) {
    HeightGrid grid; grid.width=width; grid.height=height; grid.gsd_m=1;
    grid.origin_north_m=200; grid.origin_east_m=-100;
    for(int y=0;y<height;++y) for(int x=0;x<width;++x)
        grid.values.push_back(100+4*std::sin(.23*x)+7*std::cos(.14*y)+3*std::sin(.15*x+.3*y));
    return grid;
}
int demo() {
    const ImageMap map{syntheticImage(256,192),200,-100,1};
    const Calibration k{64,64,31.5,31.5};
    Frame previous; int failed=0,image_fixes=0,velocities=0;
    for(int i=0;i<3;++i) {
        const int x=72+5*i,y=50-2*i;
        const double north=200-(y+31.5),east=-100+(x+31.5);
        Frame frame{crop(map.image,x,y,64,64),static_cast<Timestamp>(1000200+i*100000),64,.2,0,0,0};
        const auto ortho=orthorectify(frame.image,k,{north+3,east-4,164,0,0,0},100,1,64,64,frame.exposure_t_us);
        const auto fix=matchAbsolute(ortho,map,{north+3,east-4,12}); printFix(fix);
        if(fix.valid) ++image_fixes;
        if(!fix.valid || std::abs(fix.north_m-north)>.2 || std::abs(fix.east_m-east)>.2) ++failed;
        if(i) {
            const auto v=estimatePlanarVelocity(previous,frame,k); printVo(v);
            if(v.valid) ++velocities;
            if(!v.valid || std::abs(v.velocity_ned[0]-20)>1 || std::abs(v.velocity_ned[1]-50)>1) ++failed;
        }
        previous=std::move(frame);
    }
    const auto reference=syntheticTerrain(256,192);
    HeightGrid observed; observed.width=24; observed.height=20; observed.gsd_m=1;
    observed.origin_north_m=141; observed.origin_east_m=-14;
    for(int y=0;y<20;++y) for(int x=0;x<24;++x) observed.values.push_back(reference.at(x+90,y+65)+7.5);
    const auto terrain=registerTerrain(observed,reference,12,previous.exposure_t_us); printTerrain(terrain);
    if(!terrain.valid || std::abs(terrain.delta_north_m+6)>.1 || std::abs(terrain.delta_east_m-4)>.1
        || std::abs(terrain.vertical_bias_m-7.5)>.01) ++failed;
    const CameraPose p1{0,0,200,0,0,0},p2{20,0,200,0,0,0};
    const GroundPoint truth{5,2,100}; PixelObservation a,b;
    groundToPixel(k,p1,truth,a.u,a.v); groundToPixel(k,p2,truth,b.u,b.v);
    const auto point=triangulate(k,p1,p2,a,b);
    std::cout<<"{\"type\":\"sparse_point\",\"valid\":"<<point.valid<<",\"north_m\":"<<point.point.north_m
        <<",\"east_m\":"<<point.point.east_m<<",\"height_m\":"<<point.point.height_m
        <<",\"ray_angle_rad\":"<<point.ray_angle_rad<<",\"reprojection_error_px\":"<<point.reprojection_error_px<<"}\n";
    if(!point.valid || std::abs(point.point.height_m-100)>1e-6) ++failed;
    std::cout<<"{\"type\":\"summary\",\"frames\":3,\"image_fixes\":"<<image_fixes<<",\"vo\":"<<velocities
        <<",\"terrain_fixes\":"<<(terrain.valid?1:0)<<",\"sparse_points\":"<<(point.valid?1:0)<<",\"failed\":"<<failed<<"}\n";
    if(failed) std::cerr<<"demo quantitative verification failed: "<<failed<<" cases\n";
    return failed?1:0;
}

template<typename T> bool number(const std::string& text,T& value) {
    std::istringstream stream(text); stream>>value;
    return stream && stream.peek()==std::char_traits<char>::eof() && std::isfinite(static_cast<double>(value));
}
bool timestamp(const std::string& text,Timestamp& value) {
    const auto result=std::from_chars(text.data(),text.data()+text.size(),value);
    return result.ec==std::errc{} && result.ptr==text.data()+text.size() && value>0;
}
bool relativePath(const std::string& text) {
    const std::filesystem::path path(text);
    if(text.empty() || path.is_absolute()) return false;
    for(const auto& part:path) if(part=="..") return false;
    return true;
}
struct ReplayFrame { std::string image; Timestamp exposure{}; CameraPose pose{}; double plane{},altitude_sigma{}; };
struct Manifest {
    std::filesystem::path root;
    std::string reference,terrain_observed;
    Calibration k{};
    int width{},height{};
    double gsd{},radius{},terrain_north{},terrain_east{},terrain_gsd{},terrain_radius{};
    std::vector<ReplayFrame> frames;
};
bool readManifest(const std::string& filename,Manifest& out,std::string& error) {
    out.root=std::filesystem::path(filename).parent_path();
    std::ifstream stream(filename); std::string line;
    if(!stream || !std::getline(stream,line) || line!="NAVREPLAY 1") { error="invalid replay manifest header"; return false; }
    std::map<std::string,std::string> fields;
    bool found=false;
    while(std::getline(stream,line)) {
        if(!line.empty() && line.back()=='\r') line.pop_back();
        if(line=="FRAMES") { found=true; break; }
        if(line.empty() || line[0]=='#') continue;
        const auto equal=line.find('=');
        if(equal==std::string::npos || equal==0 || equal+1==line.size()
            || !fields.emplace(line.substr(0,equal),line.substr(equal+1)).second) { error="invalid/duplicate replay header field"; return false; }
    }
    const std::set<std::string> allowed={"reference","fx","fy","cx","cy","ortho_gsd_m","ortho_width","ortho_height",
        "search_radius_m","height_datum","terrain_observed","terrain_north_m","terrain_east_m","terrain_gsd_m","terrain_search_radius_m"};
    for(const auto& field:fields) if(!allowed.count(field.first)) { error="unknown replay field: "+field.first; return false; }
    if(!found || !fields.count("reference") || !relativePath(fields["reference"])
        || fields["height_datum"]!="WGS84_ELLIPSOID" || !number(fields["fx"],out.k.fx)
        || !number(fields["fy"],out.k.fy) || !number(fields["cx"],out.k.cx) || !number(fields["cy"],out.k.cy)
        || !number(fields["ortho_width"],out.width) || !number(fields["ortho_height"],out.height)
        || !number(fields["ortho_gsd_m"],out.gsd) || !number(fields["search_radius_m"],out.radius)
        || !out.k.valid() || out.width<8 || out.height<8 || out.width>16384 || out.height>16384
        || static_cast<std::size_t>(out.width)*out.height>64u*1024u*1024u || out.gsd<=0 || out.radius<0) {
        error="invalid replay calibration, bounds or height datum"; return false;
    }
    out.reference=fields["reference"];
    if(fields.count("terrain_observed")) {
        if(!relativePath(fields["terrain_observed"]) || !number(fields["terrain_north_m"],out.terrain_north)
            || !number(fields["terrain_east_m"],out.terrain_east) || !number(fields["terrain_gsd_m"],out.terrain_gsd)
            || !number(fields["terrain_search_radius_m"],out.terrain_radius) || out.terrain_gsd<=0 || out.terrain_radius<0) {
            error="invalid observed terrain manifest fields"; return false;
        }
        out.terrain_observed=fields["terrain_observed"];
    }
    if(!std::getline(stream,line) || line!="image,t_us,north_m,east_m,height_m,plane_height_m,roll_rad,pitch_rad,yaw_rad,altitude_sigma_m") {
        error="invalid replay CSV schema"; return false;
    }
    Timestamp previous=0;
    while(std::getline(stream,line)) {
        if(!line.empty() && line.back()=='\r') line.pop_back();
        if(line.empty() || line[0]=='#') continue;
        std::istringstream row(line); std::vector<std::string> columns; std::string value;
        while(std::getline(row,value,',')) columns.push_back(value);
        ReplayFrame frame;
        if(columns.size()!=10 || !relativePath(columns[0]) || !timestamp(columns[1],frame.exposure)) {
            error="invalid replay image path/exposure row"; return false;
        }
        if(frame.exposure<=previous) { error="replay exposure timestamps must increase strictly"; return false; }
        previous=frame.exposure; frame.image=columns[0];
        if(!number(columns[2],frame.pose.north_m) || !number(columns[3],frame.pose.east_m)
            || !number(columns[4],frame.pose.height_m) || !number(columns[5],frame.plane)
            || !number(columns[6],frame.pose.roll_rad) || !number(columns[7],frame.pose.pitch_rad)
            || !number(columns[8],frame.pose.yaw_rad) || !number(columns[9],frame.altitude_sigma)
            || frame.pose.height_m<=frame.plane+1 || frame.altitude_sigma<0 || out.frames.size()>=100000) {
            error="invalid replay pose/AGL/uncertainty"; return false;
        }
        out.frames.push_back(frame);
    }
    if(out.frames.empty()) { error="replay has no frames"; return false; }
    return true;
}

bool mosaic(TileStore& store,const std::vector<TileMetadata>& selected,ImageMap& dom,HeightGrid& dsm,std::string& error) {
    if(selected.empty()) { error="no reference tiles for window"; return false; }
    double nmax=selected[0].north_max_m,nmin=selected[0].north_min_m;
    double emin=selected[0].east_min_m,emax=selected[0].east_max_m;
    const double gsd=selected[0].gsd_m;
    for(const auto& m:selected) {
        if(std::abs(m.gsd_m-gsd)>gsd*1e-6) { error="cannot mosaic different reference GSDs"; return false; }
        nmax=std::max(nmax,m.north_max_m); nmin=std::min(nmin,m.north_min_m);
        emin=std::min(emin,m.east_min_m); emax=std::max(emax,m.east_max_m);
    }
    const double wf=(emax-emin)/gsd+1,hf=(nmax-nmin)/gsd+1;
    if(!std::isfinite(wf) || !std::isfinite(hf) || wf<2 || hf<2 || wf>16384 || hf>16384
        || wf*hf>64u*1024u*1024u) { error="reference mosaic exceeds CPU memory bounds"; return false; }
    const int width=static_cast<int>(std::lround(wf)),height=static_cast<int>(std::lround(hf));
    dom={Image{width,height,std::vector<float>(static_cast<std::size_t>(width)*height)},nmax,emin,gsd};
    dsm.width=width; dsm.height=height; dsm.origin_north_m=nmax; dsm.origin_east_m=emin; dsm.gsd_m=gsd;
    dsm.values.resize(dom.image.pixels.size()); std::vector<std::uint8_t> covered(dom.image.pixels.size(),0);
    for(const auto& m:selected) {
        const auto tile=store.load(m.id,error); if(!tile) return false;
        const double xf=(m.origin_east_m-emin)/gsd,yf=(nmax-m.origin_north_m)/gsd;
        const int x0=static_cast<int>(std::lround(xf)),y0=static_cast<int>(std::lround(yf));
        if(std::abs(xf-x0)>1e-6 || std::abs(yf-y0)>1e-6 || x0<0 || y0<0 || x0+m.width>width || y0+m.height>height) {
            error="reference tiles are not aligned on one local grid"; return false;
        }
        for(int y=0;y<m.height;++y) for(int x=0;x<m.width;++x) {
            const auto index=static_cast<std::size_t>(y+y0)*width+x+x0;
            if(covered[index] && (dom.image.pixels[index]!=tile->dom.image.at(x,y) || dsm.values[index]!=tile->dsm.at(x,y))) {
                error="overlapping reference tiles disagree"; return false;
            }
            dom.image.pixels[index]=tile->dom.image.at(x,y); dsm.values[index]=tile->dsm.at(x,y); covered[index]=1;
        }
    }
    if(std::find(covered.begin(),covered.end(),0)!=covered.end()) { error="reference mosaic has uncovered cells"; return false; }
    return true;
}

int replay(const std::string& filename) {
    Manifest manifest; std::string error;
    if(!readManifest(filename,manifest,error)) { std::cerr<<error<<'\n'; return 2; }
    TileStore store(8);
    if(!store.open((manifest.root/manifest.reference).string(),error)) { std::cerr<<error<<'\n'; return 2; }
    Frame previous; int fixes=0,velocities=0,rejections=0;
    for(const auto& input:manifest.frames) {
        Frame frame;
        if(!readPgm((manifest.root/input.image).string(),frame.image,error)) { std::cerr<<error<<'\n'; return 2; }
        frame.exposure_t_us=input.exposure; frame.altitude_agl_m=input.pose.height_m-input.plane;
        frame.altitude_sigma_m=input.altitude_sigma; frame.roll_rad=input.pose.roll_rad;
        frame.pitch_rad=input.pose.pitch_rad; frame.yaw_rad=input.pose.yaw_rad;
        const auto ortho=orthorectify(frame.image,manifest.k,input.pose,input.plane,manifest.gsd,manifest.width,manifest.height,input.exposure);
        const double half_n=(manifest.height-1)*manifest.gsd*.5,half_e=(manifest.width-1)*manifest.gsd*.5;
        const auto selected=store.tilesForWindow(input.pose.north_m-manifest.radius-half_n,input.pose.north_m+manifest.radius+half_n,
            input.pose.east_m-manifest.radius-half_e,input.pose.east_m+manifest.radius+half_e);
        MatchResult fix; fix.t_us=input.exposure; fix.height_m=input.pose.height_m; fix.reason=RejectReason::OutOfBounds;
        if(!selected.empty()) {
            ImageMap dom; HeightGrid dsm;
            if(!mosaic(store,selected,dom,dsm,error)) { std::cerr<<error<<'\n'; return 2; }
            fix=matchAbsolute(ortho,dom,{input.pose.north_m,input.pose.east_m,manifest.radius});
        }
        printFix(fix); if(fix.valid) ++fixes; else ++rejections;
        if(previous.exposure_t_us) {
            const auto velocity=estimatePlanarVelocity(previous,frame,manifest.k); printVo(velocity);
            if(velocity.valid) ++velocities; else ++rejections;
        }
        previous=std::move(frame);
    }
    int terrain_fixes=0;
    if(!manifest.terrain_observed.empty()) {
        HeightGrid observed; observed.origin_north_m=manifest.terrain_north; observed.origin_east_m=manifest.terrain_east; observed.gsd_m=manifest.terrain_gsd;
        if(!readHeightGrid((manifest.root/manifest.terrain_observed).string(),observed,error) || !observed.valid()) {
            std::cerr<<"invalid observed terrain grid: "<<error<<'\n'; return 2;
        }
        const auto selected=store.tilesForWindow(observed.origin_north_m-(observed.height-1)*observed.gsd_m-manifest.terrain_radius,
            observed.origin_north_m+manifest.terrain_radius,observed.origin_east_m-manifest.terrain_radius,
            observed.origin_east_m+(observed.width-1)*observed.gsd_m+manifest.terrain_radius);
        TerrainResult terrain; terrain.t_us=previous.exposure_t_us; terrain.reason=RejectReason::OutOfBounds;
        if(!selected.empty()) {
            ImageMap dom; HeightGrid dsm;
            if(!mosaic(store,selected,dom,dsm,error)) { std::cerr<<error<<'\n'; return 2; }
            terrain=registerTerrain(observed,dsm,manifest.terrain_radius,previous.exposure_t_us);
        }
        printTerrain(terrain); if(terrain.valid) ++terrain_fixes; else ++rejections;
    }
    std::cout<<"{\"type\":\"summary\",\"frames\":"<<manifest.frames.size()<<",\"image_fixes\":"<<fixes
        <<",\"vo\":"<<velocities<<",\"terrain_fixes\":"<<terrain_fixes<<",\"rejected\":"<<rejections
        <<",\"cache_hits\":"<<store.cacheHits()<<"}\n";
    return 0;
}
void usage(std::ostream& stream) {
    stream<<"Usage: nav_companion --demo | --replay <frames.manifest>\n"
        <<"CPU file-based visual navigation baseline; stdout contains JSONL results.\n"
        <<"Frame timestamps are supplied MCU exposure-midpoint microseconds.\n";
}
}  // namespace

int main(int argc,char** argv) {
    std::cout<<std::boolalpha<<std::setprecision(12);
    try {
        if(argc==2 && std::string(argv[1])=="--demo") return demo();
        if(argc==3 && std::string(argv[1])=="--replay") return replay(argv[2]);
        if(argc==2 && (std::string(argv[1])=="--help" || std::string(argv[1])=="-h")) { usage(std::cout); return 0; }
        usage(std::cerr); return 2;
    } catch(const std::exception& error) {
        std::cerr<<"companion input/processing error: "<<error.what()<<'\n'; return 2;
    }
}
