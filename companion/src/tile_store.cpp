#include "companion/companion.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
#include <set>
#include <sstream>

namespace companion {
namespace {
bool readBytes(const std::string& path, std::string& bytes, std::string& error, std::size_t limit=1024*1024) {
    std::ifstream stream(path,std::ios::binary|std::ios::ate);
    if(!stream || stream.tellg()<0 || static_cast<std::size_t>(stream.tellg())>limit) { error="missing/oversized metadata: "+path; return false; }
    bytes.resize(static_cast<std::size_t>(stream.tellg())); stream.seekg(0);
    if(!bytes.empty() && !stream.read(bytes.data(),static_cast<std::streamsize>(bytes.size()))) { error="metadata read failed: "+path; return false; }
    return true;
}
bool safeName(const std::string& text) {
    return !text.empty() && text!="." && text!=".." && text.size()<128
        && std::all_of(text.begin(),text.end(),[](unsigned char c) {
            return (c>='a'&&c<='z') || (c>='A'&&c<='Z') || (c>='0'&&c<='9') || c=='_' || c=='-' || c=='.';
        });
}
bool digestValid(const std::string& digest) {
    return digest.size()==64 && std::all_of(digest.begin(),digest.end(),[](char c){return (c>='0'&&c<='9') || (c>='a'&&c<='f');});
}
template<typename T> bool number(const std::string& text,T& value) {
    std::istringstream stream(text); stream>>value;
    return stream && stream.peek()==std::char_traits<char>::eof() && std::isfinite(static_cast<double>(value));
}
bool timestampValid(const std::string& t) {
    if(t.size()!=20 || t[4]!='-' || t[7]!='-' || t[10]!='T' || t[13]!=':' || t[16]!=':' || t[19]!='Z') return false;
    int year=0,month=0,day=0,hour=0,minute=0,second=0;
    if(!number(t.substr(0,4),year) || !number(t.substr(5,2),month) || !number(t.substr(8,2),day)
        || !number(t.substr(11,2),hour) || !number(t.substr(14,2),minute) || !number(t.substr(17,2),second)) return false;
    if(year<1970 || month<1 || month>12 || hour<0 || hour>23 || minute<0 || minute>59 || second<0 || second>59) return false;
    const int days[]={31,28,31,30,31,30,31,31,30,31,30,31};
    const bool leap=year%4==0 && (year%100!=0 || year%400==0);
    return day>=1 && day<=days[month-1]+(month==2 && leap?1:0);
}
bool nearly(double a,double b) { return std::abs(a-b)<=1e-8*std::max({1.0,std::abs(a),std::abs(b)}); }
bool parseMetadata(const std::string& bytes,TileMetadata& m,std::string& error) {
    std::map<std::string,std::string> fields;
    std::istringstream stream(bytes); std::string line;
    while(std::getline(stream,line)) {
        if(!line.empty() && line.back()=='\r') line.pop_back();
        const auto equal=line.find('=');
        if(equal==std::string::npos || equal==0 || equal+1==line.size()
            || !fields.emplace(line.substr(0,equal),line.substr(equal+1)).second) { error="invalid/duplicate tile metadata key"; return false; }
    }
    const std::set<std::string> required={"format","id","image_file","height_file","image_sha256","height_sha256",
        "width","height","origin_north_m","origin_east_m","gsd_m","north_min_m","north_max_m","east_min_m","east_max_m",
        "origin_lat_deg","origin_lon_deg","height_datum","acquired_utc","texture_std","laplacian_variance","height_std_m"};
    if(fields.size()!=required.size()) { error="tile metadata schema mismatch"; return false; }
    for(const auto& key:required) if(!fields.count(key)) { error="missing tile metadata: "+key; return false; }
    if(fields["format"]!="NAVTILE_1") { error="unsupported tile metadata format"; return false; }
    m.id=fields["id"]; m.image_file=fields["image_file"]; m.height_file=fields["height_file"];
    m.image_sha256=fields["image_sha256"]; m.height_sha256=fields["height_sha256"];
    m.height_datum=fields["height_datum"]; m.acquired_utc=fields["acquired_utc"];
    if(!safeName(m.id) || !safeName(m.image_file) || !safeName(m.height_file)
        || !digestValid(m.image_sha256) || !digestValid(m.height_sha256) || !timestampValid(m.acquired_utc)) {
        error="unsafe tile path/hash/acquisition timestamp"; return false;
    }
    if(m.height_datum!="WGS84_ELLIPSOID") { error="height datum must be WGS84_ELLIPSOID; geoid conversion required offline"; return false; }
    if(!number(fields["width"],m.width) || !number(fields["height"],m.height)
        || !number(fields["origin_north_m"],m.origin_north_m) || !number(fields["origin_east_m"],m.origin_east_m)
        || !number(fields["gsd_m"],m.gsd_m) || !number(fields["north_min_m"],m.north_min_m)
        || !number(fields["north_max_m"],m.north_max_m) || !number(fields["east_min_m"],m.east_min_m)
        || !number(fields["east_max_m"],m.east_max_m) || !number(fields["origin_lat_deg"],m.origin_lat_deg)
        || !number(fields["origin_lon_deg"],m.origin_lon_deg) || !number(fields["texture_std"],m.texture_std)
        || !number(fields["laplacian_variance"],m.laplacian_variance) || !number(fields["height_std_m"],m.height_std_m)) {
        error="nonfinite or invalid tile numeric metadata"; return false;
    }
    if(m.width<2 || m.height<2 || m.width>16384 || m.height>16384 || static_cast<std::size_t>(m.width)*m.height>64u*1024u*1024u
        || m.gsd_m<=0 || m.gsd_m>10000 || m.origin_lat_deg<-90 || m.origin_lat_deg>90
        || m.origin_lon_deg<-180 || m.origin_lon_deg>180 || m.texture_std<0 || m.laplacian_variance<0 || m.height_std_m<0
        || !nearly(m.north_max_m,m.origin_north_m) || !nearly(m.north_min_m,m.origin_north_m-(m.height-1)*m.gsd_m)
        || !nearly(m.east_min_m,m.origin_east_m) || !nearly(m.east_max_m,m.origin_east_m+(m.width-1)*m.gsd_m)
        || m.north_min_m>=m.north_max_m || m.east_min_m>=m.east_max_m) { error="invalid tile dimensions, origin or bounds"; return false; }
    return true;
}
}  // namespace

TileStore::TileStore(std::size_t capacity) : capacity_(std::max<std::size_t>(1,capacity)) {}
bool TileStore::open(const std::string& directory,std::string& error) {
    directory_.clear(); metadata_.clear(); cache_.clear(); lru_.clear(); cache_hits_=0;
    const std::filesystem::path root(directory);
    std::string index,stored_hash;
    if(!readBytes((root/"index.tsv").string(),index,error) || !readBytes((root/"index.sha256").string(),stored_hash,error,128)) return false;
    std::istringstream hash_stream(stored_hash); std::string expected_hash,extra;
    hash_stream>>expected_hash;
    if(!digestValid(expected_hash) || (hash_stream>>extra) || sha256(index)!=expected_hash) { error="reference index checksum mismatch"; return false; }
    std::istringstream stream(index); std::string magic; int version=0;
    if(!(stream>>magic>>version) || magic!="NAVREF" || version!=1) { error="invalid reference index header"; return false; }
    std::vector<TileMetadata> metadata;
    std::set<std::string> seen;
    std::string id,name,digest;
    while(stream>>id) {
        if(!(stream>>name>>digest) || !safeName(id) || name!=id+".meta" || !digestValid(digest)
            || !seen.insert(id).second || metadata.size()>=10000) { error="invalid/duplicate reference index entry"; return false; }
        std::string bytes;
        if(!readBytes((root/name).string(),bytes,error,65536)) return false;
        if(sha256(bytes)!=digest) { error="tile metadata checksum mismatch: "+id; return false; }
        TileMetadata tile;
        if(!parseMetadata(bytes,tile,error)) return false;
        if(tile.id!=id) { error="tile id disagrees with reference index"; return false; }
        if(!metadata.empty() && (!nearly(tile.origin_lat_deg,metadata[0].origin_lat_deg)
            || !nearly(tile.origin_lon_deg,metadata[0].origin_lon_deg))) { error="reference tiles have different local tangent origins"; return false; }
        metadata.push_back(std::move(tile));
    }
    if(metadata.empty()) { error="empty reference index"; return false; }
    directory_=directory; metadata_=std::move(metadata); error.clear(); return true;
}
std::vector<TileMetadata> TileStore::tilesForWindow(double nmin,double nmax,double emin,double emax) const {
    std::vector<TileMetadata> result;
    if(!std::isfinite(nmin) || !std::isfinite(nmax) || !std::isfinite(emin) || !std::isfinite(emax) || nmin>nmax || emin>emax) return result;
    for(const auto& m:metadata_) if(m.north_max_m>=nmin && m.north_min_m<=nmax && m.east_max_m>=emin && m.east_min_m<=emax) result.push_back(m);
    return result;
}
std::shared_ptr<const Tile> TileStore::load(const std::string& id,std::string& error) {
    const auto cached=cache_.find(id);
    if(cached!=cache_.end()) {
        lru_.erase(cached->second.lru); lru_.push_front(id); cached->second.lru=lru_.begin();
        ++cache_hits_; error.clear(); return cached->second.tile;
    }
    const auto entry=std::find_if(metadata_.begin(),metadata_.end(),[&](const TileMetadata& m){return m.id==id;});
    if(entry==metadata_.end()) { error="tile not in verified index: "+id; return {}; }
    const auto root=std::filesystem::path(directory_);
    std::string digest;
    if(!fileSha256((root/entry->image_file).string(),digest,error) || digest!=entry->image_sha256) { error="DOM checksum mismatch: "+id; return {}; }
    if(!fileSha256((root/entry->height_file).string(),digest,error) || digest!=entry->height_sha256) { error="DSM checksum mismatch: "+id; return {}; }
    auto tile=std::make_shared<Tile>(); tile->metadata=*entry;
    tile->dom.origin_north_m=entry->origin_north_m; tile->dom.origin_east_m=entry->origin_east_m; tile->dom.gsd_m=entry->gsd_m;
    tile->dsm.origin_north_m=entry->origin_north_m; tile->dsm.origin_east_m=entry->origin_east_m;
    tile->dsm.gsd_m=entry->gsd_m; tile->dsm.height_datum=entry->height_datum;
    if(!readPgm((root/entry->image_file).string(),tile->dom.image,error)
        || !readHeightGrid((root/entry->height_file).string(),tile->dsm,error)) return {};
    if(tile->dom.image.width!=entry->width || tile->dom.image.height!=entry->height
        || tile->dsm.width!=entry->width || tile->dsm.height!=entry->height || !tile->dsm.valid()) {
        error="tile raster dimensions/height datum disagree with metadata"; return {};
    }
    while(cache_.size()>=capacity_) { cache_.erase(lru_.back()); lru_.pop_back(); }
    lru_.push_front(id); cache_.emplace(id,CachedTile{tile,lru_.begin()}); error.clear(); return tile;
}
}  // namespace companion
