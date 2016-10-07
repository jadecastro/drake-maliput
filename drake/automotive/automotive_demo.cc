#include <iostream>
#include <memory>

#include <gflags/gflags.h>

#include "drake/automotive/automotive_simulator.h"
#include "drake/automotive/create_trajectory_params.h"
#include "drake/automotive/maliput/monolane/loader.h"
#include "drake/automotive/maliput/utility/generate_obj.h"
#include "drake/common/drake_path.h"
#include "drake/common/text_logging_gflags.h"

DEFINE_int32(num_simple_car, 1, "Number of SimpleCar vehicles");
DEFINE_int32(num_trajectory_car, 1, "Number of TrajectoryCar vehicles");
DEFINE_string(map_file, "",
              "yaml input file defining a maliput monolane road geometry");
DEFINE_string(road_yaml_file, "",
              "yaml input file defining a maliput monolane road geometry");
DEFINE_int32(num_circuit_car, 1, "Number of InfiniteCircuitCar vehicles");

namespace drake {
namespace automotive {
namespace {

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  logging::HandleSpdlogGflags();

  // TODO(jwnimmer-tri) Allow for multiple simple cars.
  if (FLAGS_num_simple_car > 1) {
    std::cerr << "ERROR: Only one simple car is supported (for now)."
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
  for (int i = 0; i < FLAGS_num_simple_car; ++i) {
    simulator->AddSimpleCarFromSdf(kSdfFile);
  }
  if (FLAGS_map_file.empty()) {
#if 0
    for (int i = 0; i < FLAGS_num_trajectory_car; ++i) {
      const auto& params = CreateTrajectoryParams(i);
      simulator->AddTrajectoryCarFromSdf(kSdfFile,
                                         std::get<0>(params),
                                         std::get<1>(params),
                                         std::get<2>(params));
    }
#endif
  } else {
    DRAKE_ABORT();
//XXX    for (int i = 0; i < FLAGS_num_trajectory_car; ++i) {
//XXX      auto road = maliput::monolane::LoadFile(FLAGS_map_file);
//XXX      std::string obj_file = std::string("/tmp/") + road->id().id_ + ".obj";
//XXX      maliput::utility::generate_obj(road.get(), obj_file, 1.);
//XXX      // TODO(rico) jam the obj file path into RigidBodyTree for viz.
//XXX
//XXX      const auto& params = CreateTrajectoryRoadParams(*road, i);
//XXX      simulator->AddTrajectoryRoadCar(
//XXX          *std::get<0>(params),
//XXX          std::get<1>(params),
//XXX          std::get<2>(params),
//XXX          std::get<3>(params));
//XXX    }
  }
  if (!FLAGS_road_yaml_file.empty()) {
    std::cerr << "building road from " << FLAGS_road_yaml_file << std::endl;
    auto road = maliput::monolane::LoadFile(FLAGS_road_yaml_file);
    simulator->SetRoadGeometry(&road);

    simulator->AddEndlessRoadCar(0., 0., 1.);
  }
  if (FLAGS_num_circuit_car) {
    // TODO(maddog)  create appropriate cars, etc.
  }

  simulator->Start();

  while (true) {
    simulator->StepBy(0.01);
  }

  return 0;
}

}  // namespace
}  // namespace automotive
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::automotive::main(argc, argv);
}
