#include "drake/automotive/maliput/monolane/loader.h"

#include <cmath>
#include <iostream>
#include <string>

#include <gflags/gflags.h>
#include "yaml-cpp/yaml.h"

#include "drake/common/drake_assert.h"
#include "drake/automotive/maliput/monolane/builder.h"
#include "drake/automotive/maliput/utility/generate_obj.h"

namespace api = maliput::geometry_api;
namespace mono = maliput::monolane;

namespace {

api::RBounds rbounds(const YAML::Node& node) {
  DRAKE_DEMAND(node.IsSequence());
  DRAKE_DEMAND(node.size() == 2);
  return {node[0].as<double>(), node[1].as<double>()};
}

double d2r(double degrees) {
  return degrees * M_PI / 180.;
}

mono::XYPoint xypoint(const YAML::Node& node) {
  DRAKE_DEMAND(node.IsSequence());
  DRAKE_DEMAND(node.size() == 3);
  return {
    node[0].as<double>(), node[1].as<double>(), d2r(node[2].as<double>())};
}

mono::ZPoint zpoint(const YAML::Node& node) {
  DRAKE_DEMAND(node.IsSequence());
  DRAKE_DEMAND(node.size() == 4);
  return {
    node[0].as<double>(), node[1].as<double>(),
        d2r(node[2].as<double>()), d2r(node[3].as<double>())};
}

mono::ArcOffset arc_offset(const YAML::Node& node) {
  DRAKE_DEMAND(node.IsSequence());
  DRAKE_DEMAND(node.size() == 2);
  return {node[0].as<double>(), d2r(node[1].as<double>())};
}



const mono::Connection* maybe_make_connection(
    std::string id,
    YAML::Node node,
    const std::map<std::string, mono::XYZPoint>& xyzpoints,
    mono::Builder* builder) {
  DRAKE_DEMAND(node.IsMap());

  // Check if needed symbols are available.
  auto start_symbol = node["start"].as<std::string>();
  auto it_start = xyzpoints.find(start_symbol);
  if (it_start == xyzpoints.end()) { return nullptr; }

  // Discover segment type.
  bool forced_end = false;
  auto it_fe = xyzpoints.end();
  enum SegmentType{ line, arc } segment_type{};
  for (const auto& kv : node) {
    std::string key = kv.first.as<std::string>();

    if (key == "length") {
      segment_type = line;
    } else if (key == "arc") {
      segment_type = arc;
    } else if (key == "forced_end") {
      forced_end = true;
      // More symbol resolution.
      // TODO(rico): fold this and above into a function.
      auto fe_symbol = node["forced_end"].as<std::string>();
      it_fe = xyzpoints.find(fe_symbol);
      if (it_fe == xyzpoints.end()) { return nullptr; }
    }
  }

  // Call appropriate method.
  switch (segment_type) {
    case line: {
      if (forced_end) {
        return builder->Connect(
            id, it_start->second, node["length"].as<double>(),
            zpoint(node["z_end"]), it_fe->second.xy_);
      } else {
        return builder->Connect(
            id, it_start->second, node["length"].as<double>(),
            zpoint(node["z_end"]));
      }
    }
    case arc: {
      if (forced_end) {
        return builder->Connect(id, it_start->second, arc_offset(node["arc"]),
                                zpoint(node["z_end"]), it_fe->second.xy_);
      } else {
        return builder->Connect(id, it_start->second, arc_offset(node["arc"]),
                                zpoint(node["z_end"]));
      }
    }
  }

  return nullptr;
}

mono::XYZPoint xyzpoint(const YAML::Node& node) {
  DRAKE_DEMAND(node.IsMap());
  return {xypoint(node["xypoint"]), zpoint(node["zpoint"])};
}

std::unique_ptr<const api::RoadGeometry> BuildFrom(YAML::Node node) {
  DRAKE_DEMAND(node.IsMap());
  YAML::Node mmb = node["maliput_monolane_builder"];
  DRAKE_DEMAND(mmb.IsMap());

  mono::Builder b(rbounds(mmb["lane_bounds"]),
                  rbounds(mmb["driveable_bounds"]));

  std::cerr << "loading points !\n";
  YAML::Node points = mmb["points"];
  DRAKE_DEMAND(points.IsMap());
  std::map<std::string, mono::XYZPoint> xyzpoints;
  for (const auto& p : points) {
    xyzpoints[std::string("points.") + p.first.as<std::string>()] =
        xyzpoint(p.second);
  }

  std::cerr << "loading raw connections !\n";
  YAML::Node connections = mmb["connections"];
  DRAKE_DEMAND(connections.IsMap());
  std::map<std::string, YAML::Node> raw_connections;
  for (const auto& c : connections) {
    raw_connections[c.first.as<std::string>()] = c.second;
  }

  std::cerr << "building cooked connections !\n";
  std::map<std::string, const mono::Connection*> cooked_connections;
  while (!raw_connections.empty()) {
    std::cerr << "raw count " << raw_connections.size()
              << " cooked count " << cooked_connections.size() << "\n";
    size_t cooked_before_this_pass = cooked_connections.size();
    for (const auto& r : raw_connections) {
      std::string id = r.first;
      const mono::Connection* conn =
          maybe_make_connection(id, r.second, xyzpoints, &b);
      if (!conn) { continue; }
      cooked_connections[id] = conn;
      xyzpoints[std::string("connections.") + id + ".end"] = conn->end();
    }
    DRAKE_DEMAND(cooked_connections.size() > cooked_before_this_pass);
    for (const auto& c : cooked_connections) {
      raw_connections.erase(c.first);
    }
  }

  std::cerr << "building road geometry !\n";
  return b.Build({mmb["id"].Scalar()});
}

}

namespace maliput {
namespace monolane {

std::unique_ptr<const api::RoadGeometry> Load(const std::string& input) {
  return BuildFrom(YAML::Load(input));
}

std::unique_ptr<const api::RoadGeometry> LoadFile(const std::string& filename) {
  return BuildFrom(YAML::LoadFile(filename));
}

}  // namespace monolane
}  // namespace maliput
