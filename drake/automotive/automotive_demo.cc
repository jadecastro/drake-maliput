#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

#include <gflags/gflags.h>

#include "drake/automotive/automotive_simulator.h"
#include "drake/automotive/create_trajectory_params.h"
#include "drake/automotive/maliput/monolane/loader.h"
#include "drake/automotive/maliput/utility/generate_obj.h"
#include "drake/common/drake_path.h"
#include "drake/common/text_logging_gflags.h"

DEFINE_string(road_file, "",
              "yaml file defining a maliput monolane road geometry");
// "Ego car" in this instance means "controlled by something smarter than
// this demo code".
DEFINE_int32(num_ego_car, 1, "Number of user-controlled vehicles");
DEFINE_int32(num_ado_car, 1,
             "Number of vehicles controlled by a "
             "(possibly trivial) traffic model");

namespace drake {
namespace automotive {
namespace {

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  logging::HandleSpdlogGflags();

  // TODO(jwnimmer-tri) Allow for multiple simple cars.
  if (FLAGS_num_ego_car > 1) {
    std::cerr << "ERROR: Only one user-controlled car is supported (for now)."
              << std::endl;
    return 1;
  }

  // TODO(liang.fok): Generalize this demo to allow arbitrary models to be
  // specified via command line parameters. This will involve removing some
  // hard-coded assumptions about the model's geometry. For exeample, the call
  // to CreateTrajectoryParams() below expects a "car" to have a particular
  // length and width.
  const std::string kSdfFile =
      GetDrakePath() + "/automotive/models/prius/prius_with_lidar.sdf";
  auto simulator = std::make_unique<AutomotiveSimulator<double>>();

  if (FLAGS_road_file.empty()) {
    // No road description has been specified.  So, we will run in
    // "free-for-all on the xy-plane" mode.

    // User-controlled vehicles are SimpleCars.
    for (int i = 0; i < FLAGS_num_ego_car; ++i) {
      simulator->AddSimpleCarFromSdf(kSdfFile);
    }
    // "Traffic model" is "drive in a figure-8".
    for (int i = 0; i < FLAGS_num_ado_car; ++i) {
      const auto& params = CreateTrajectoryParams(i);
      simulator->AddTrajectoryCarFromSdf(kSdfFile,
                                         std::get<0>(params),
                                         std::get<1>(params),
                                         std::get<2>(params));
    }
  } else {
    // A road description has been specified.  All vehicles will be constrained
    // to drive on the specified road surface.
    std::cerr << "building road from " << FLAGS_road_file << std::endl;
    auto base_road = maliput::monolane::LoadFile(FLAGS_road_file);
    const maliput::utility::InfiniteCircuitRoad* const endless_road =
        simulator->SetRoadGeometry(&base_road);

    // TODO(maddog) Implement user-controlled vehicles.

    // TODO(maddog) Implement traffic models other than "just drive at
    // constant speed".
    for (int i = 0; i < FLAGS_num_ado_car; ++i) {
      const double kConstantSpeed = 10.0;
      const double kLateralOffsetUnit = -2.0;

      const double longitudinal_start =
          endless_road->cycle_length() * (i / 2) / FLAGS_num_ado_car * 2.;
      const double lateral_offset =
          (((i % 2) * 2) - 1) * kLateralOffsetUnit;
      simulator->AddEndlessRoadCar(
          longitudinal_start, lateral_offset, kConstantSpeed);
    }
  }

  simulator->Start();

  const double kTimeStep = 0.02;
  std::chrono::time_point<std::chrono::steady_clock> desired_now =
      std::chrono::steady_clock::now();
  while (true) {
    desired_now += std::chrono::microseconds((int)(kTimeStep * 1e6));
    std::this_thread::sleep_until(desired_now);
    simulator->StepBy(kTimeStep);
  }

  return 0;
}

}  // namespace
}  // namespace automotive
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::automotive::main(argc, argv);
}
