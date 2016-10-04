#pragma once

// TODO(jwnimmer-tri) This file provides trajectories in support of demos.
// This data should come from files loaded at runtime, instead.

#include <memory>
#include <tuple>

#include "drake/automotive/curve2.h"
#include "drake/automotive/trajectory_car.h"
#include "drake/automotive/maliput/geometry_api/road_geometry.h"

namespace drake {
namespace automotive {

/**
 * Creates TrajectoryCar constructor demo arguments.  The details of the
 * trajectory are not documented / promised by this API.
 *
 * @param index Selects which pre-programmed trajectory to use.
 * @return tuple of curve, speed, start_time
 */
std::tuple<Curve2<double>, double, double> CreateTrajectoryParams(int index);

/**
 * Creates TrajectoryRoadCar constructor demo arguments.  The details of the
 * trajectory are not documented / promised by this API.
 *
 * @param index Selects which pre-programmed trajectory to use.
 * @return tuple of road, lateral_offset, speed, start_time
 */
std::tuple<const maliput::geometry_api::RoadGeometry*,
           double, double, double> CreateTrajectoryRoadParams(
               const maliput::geometry_api::RoadGeometry& road,
               int index);

}  // namespace automotive
}  // namespace drake
