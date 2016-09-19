#pragma once

#include "drake/automotive/maliput/geometry_api/road_geometry.h"


namespace maliput {
namespace utility {

namespace api = maliput::geometry_api;


void generate_obj(const api::RoadGeometry* rg,
                  const std::string& filename,
                  double grid_unit);


}  // namespace utility
}  // namespace maliput
