#pragma once

#include <memory>
#include <string>

#include "drake/common/drake_export.h"

namespace maliput {

namespace geometry_api {
class RoadGeometry;
}

namespace monolane {

namespace api = maliput::geometry_api;

/// Loader for serialized monolane road networks.
///
/// The serialization is a fairly straightforward mapping of the Builder
/// interface onto YAML. See (TBD) for more detail of the format.

/// Load the input string as a maliput_monolane_builder document.
std::unique_ptr<const api::RoadGeometry> DRAKE_EXPORT
Load(const std::string& input);

/// Load the named file as a maliput_monolane_builder document.
std::unique_ptr<const api::RoadGeometry> DRAKE_EXPORT
LoadFile(const std::string& filename);

}  // namespace monolane
}  // namespace maliput
