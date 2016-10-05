#pragma once

#include "drake/automotive/maliput/geometry_api/road_geometry.h"


namespace maliput {
namespace utility {

namespace api = maliput::geometry_api;

void generate_urdf(const std::string& dirname,
                   const std::string& fileroot,
                   const api::RoadGeometry* rg,
                   const double grid_unit);

}  // namespace utility
}  // namespace maliput
