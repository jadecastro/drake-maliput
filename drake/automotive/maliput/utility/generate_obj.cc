#include "generate_obj.h"

#include <cassert>
#include <fstream>
#include <functional>
#include <initializer_list>
#include <iostream>
#include <unordered_map>
#include <vector>

#include "geometry_api/junction.h"
#include "geometry_api/lane.h"
#include "geometry_api/road_geometry.h"
#include "geometry_api/segment.h"

namespace maliput {
namespace utility {

namespace api = maliput::geometry_api;


namespace {

template <class T> struct LocalHash;

template <class T>
class IndexMap {
 public:
  IndexMap() {}

  int push_back(const T& thing) {
    auto mi = map_.find(thing);
    if (mi != map_.end()) {
      std::cerr << "found at " << mi->second << "  " << std::endl;
      thing.printt(std::cerr);
      return mi->second;
    }
    int index = vec_.size();
    std::cerr << "not found, now at " << index << "  " << std::endl;
    thing.printt(std::cerr);
    map_[thing] = index;
    vec_.push_back(thing);
    return index;
  }

  const std::vector<T>& vector() const { return vec_; }

 private:
  std::unordered_map<T, int, LocalHash<T>> map_;
  std::vector<T> vec_;
};

struct GeoVertex {
  GeoVertex() {}

  GeoVertex(const api::GeoPosition& v) : v_(v) {}

  void printt(std::ostream& os) const {
    os << "v:" << v_.x_ << " " << v_.y_ << " " << v_.z_ << std::endl;
  }

  api::GeoPosition v_;
};

bool operator==(const GeoVertex& lhs, const GeoVertex& rhs) {
  return ((lhs.v_.x_ == rhs.v_.x_) &&
          (lhs.v_.y_ == rhs.v_.y_) &&
          (lhs.v_.z_ == rhs.v_.z_));
}



struct GeoNormal {
  GeoNormal() {}

  GeoNormal(const api::GeoPosition& v0, const api::GeoPosition& v1)
      : n_({v1.x_ - v0.x_, v1.y_ - v0.y_, v1.z_ - v0.z_}) {}

  void printt(std::ostream& os) const {
    os << "vn: " << n_.x_ << " " << n_.y_ << " " << n_.z_ << std::endl;
  }

  api::GeoPosition n_;
};

bool operator==(const GeoNormal& lhs, const GeoNormal& rhs) {
  return ((lhs.n_.x_ == rhs.n_.x_) &&
          (lhs.n_.y_ == rhs.n_.y_) &&
          (lhs.n_.z_ == rhs.n_.z_));
}




template<> struct LocalHash<GeoVertex> {
  typedef GeoVertex argument_type;
  typedef std::size_t result_type;
  result_type operator()(const argument_type& gv) const {
    const result_type hx(std::hash<double>()(gv.v_.x_));
    const result_type hy(std::hash<double>()(gv.v_.y_));
    const result_type hz(std::hash<double>()(gv.v_.z_));
    return hx ^ (hy << 1) ^ (hz << 2);
  }
};
template<> struct LocalHash<GeoNormal> {
  typedef GeoNormal argument_type;
  typedef std::size_t result_type;
  result_type operator()(const argument_type& gn) const {
    const result_type hx(std::hash<double>()(gn.n_.x_));
    const result_type hy(std::hash<double>()(gn.n_.y_));
    const result_type hz(std::hash<double>()(gn.n_.z_));
    return hx ^ (hy << 1) ^ (hz << 2);
  }
};


struct GeoFace {
  GeoFace() {}

  GeoFace(const std::vector<GeoVertex>& vs,
          const std::vector<GeoNormal>& ns)
      : vs_(vs), ns_(ns) {
    assert(vs.size() == ns.size());
  }

  std::vector<GeoVertex> vs_;
  std::vector<GeoNormal> ns_;
};




struct IndexVertexWithNormal {
  int vertex_index_;
  int normal_index_;
};

struct IndexFace {
  std::vector<IndexVertexWithNormal> vns_;
};


class ObjData {
 public:
  ObjData() {}

  void push_face(const GeoFace& geo_face) {
    IndexFace face;
    for (size_t gi = 0; gi < geo_face.vs_.size(); ++gi) {
      int vi = vertices_.push_back(geo_face.vs_[gi]);
      int ni = normals_.push_back(geo_face.ns_[gi]);
      face.vns_.push_back({vi, ni});
    }
    faces_.push_back(face);
  }


  void dump(std::ostream& os) {

    os << "# Vertices" << std::endl;
    for (const GeoVertex& gv : vertices_.vector()) {
      os << "v " << gv.v_.x_ << " " << gv.v_.y_ << " " << gv.v_.z_ << std::endl;
    }
    os << "# Normals" << std::endl;
    for (const GeoNormal& gn : normals_.vector()) {
      os << "vn "
         << gn.n_.x_ << " " << gn.n_.y_ << " " << gn.n_.z_ << std::endl;
    }
    os << "# Faces" << std::endl;
    for (const IndexFace& f : faces_) {
      os << "f";
      for (const IndexVertexWithNormal& ivwn : f.vns_) {
        os << " "
           << (ivwn.vertex_index_ + 1)
           << "//"
           << (ivwn.normal_index_ + 1);
      }
      os << std::endl;
    }

  }



 private:
  IndexMap<GeoVertex> vertices_;
  IndexMap<GeoNormal> normals_;
  std::vector<IndexFace> faces_;
};


struct SRPos {
  SRPos(const double s, const double r) : s_(s), r_(r) {}

  double s_;
  double r_;
};

struct SRFace {
  SRFace(const std::initializer_list<SRPos> sr) : sr_(sr) {}

  std::vector<SRPos> sr_;
};


void push_face(ObjData* obj, const api::Lane* lane, const SRFace srface) {
  GeoFace geoface;
  for (const SRPos& sr : srface.sr_) {
    api::GeoPosition v0(lane->ToGeoPosition({sr.s_, sr.r_, 0.}));
    api::GeoPosition v1(lane->ToGeoPosition({sr.s_, sr.r_, 1.}));
    geoface.vs_.push_back(GeoVertex(v0));
    geoface.ns_.push_back(GeoNormal(v0, v1));
  }

  obj->push_face(geoface);
}



void cover_lane_with_quads(ObjData* obj, const api::Lane* lane,
                           const double grid_unit) {

  const double s_max = lane->length();
  double s0 = 0.;
  while (s0 < s_max) {
    double s1 = s0 + grid_unit;
    if (s1 > s_max) { s1 = s_max; }

    api::RBounds rb0 = lane->lane_bounds(s0);
    api::RBounds rb1 = lane->lane_bounds(s1);

    // Left side of lane.
    {
      double r00 = 0.;
      double r10 = 0.;
      assert(rb0.r_min_ <= r00);
      assert(rb1.r_min_ <= r10);
      while ((r00 < rb0.r_max_) && (r10 < rb1.r_max_)) {
        double r01 = r00 + grid_unit;
        double r11 = r10 + grid_unit;

        if (r01 > rb0.r_max_) { r01 = rb0.r_max_; }
        if (r11 > rb1.r_max_) { r11 = rb0.r_max_; }

        std::cerr << "{{" << s0 << ", " << r00
                  << "}, {" << s1 << ", " << r10 << "}, {"
                  << s1 << ", " << r11 << "}, {" << s0 << ", " << r01
                  << "}}" << std::endl;
        push_face(obj, lane, {{s0, r00}, {s1, r10}, {s1, r11}, {s0, r01}});

        r00 += grid_unit;
        r10 += grid_unit;
      }
    }
    // Right side of lane.
    {
      double r00 = 0.;
      double r10 = 0.;
      assert(rb0.r_max_ >= r00);
      assert(rb1.r_max_ >= r10);
      while ((r00 > rb0.r_min_) && (r10 > rb1.r_min_)) {
        double r01 = r00 - grid_unit;
        double r11 = r10 - grid_unit;

        if (r01 < rb0.r_min_) { r01 = rb0.r_min_; }
        if (r11 < rb1.r_min_) { r11 = rb0.r_min_; }

        push_face(obj, lane, {{s0, r00}, {s0, r01}, {s1, r11}, {s1, r10}});

        r00 -= grid_unit;
        r10 -= grid_unit;
      }
    }

    s0 += grid_unit;
  }
}


} // namespace


void generate_obj(const api::RoadGeometry* rg,
                  const std::string& filename,
                  const double grid_unit) {

  ObjData obj;

  // Walk the network.
  for (int ji = 0; ji < rg->num_junctions(); ++ji) {
    const api::Junction* junction = rg->junction(ji);
    for (int si = 0; si < junction->num_segments(); ++si) {
      const api::Segment* segment = junction->segment(si);
      for (int li = 0; li < segment->num_lanes(); ++li) {
        const api::Lane* lane = segment->lane(li);
        cover_lane_with_quads(&obj, lane, grid_unit);
      }
    }
  }

  {
    std::cerr << filename << std::endl;
    std::ofstream os(filename);
    obj.dump(os);
  }
}


} // namespace utility
} // namespace maliput
