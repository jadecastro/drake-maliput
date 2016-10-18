#pragma once

// GENERATED FILE DO NOT EDIT
// See drake/automotive/lcm_vector_gen.py.

#include "drake/automotive/gen/endless_road_oracle_output.h"
#include "drake/drakeAutomotiveLcm_export.h"
#include "drake/systems/lcm/lcm_and_vector_base_translator.h"
#include "drake/lcmt_endless_road_oracle_output_t.hpp"

namespace drake {
namespace automotive {

/**
 * Translates between LCM message objects and VectorBase objects for the
 * EndlessRoadOracleOutput type.
 */
class DRAKEAUTOMOTIVELCM_EXPORT EndlessRoadOracleOutputTranslator
    : public systems::lcm::LcmAndVectorBaseTranslator {
 public:
  EndlessRoadOracleOutputTranslator()
      : LcmAndVectorBaseTranslator(
            EndlessRoadOracleOutputIndices::kNumCoordinates) {}
  std::unique_ptr<systems::BasicVector<double>> AllocateOutputVector()
      const override;
  void Deserialize(const void* lcm_message_bytes, int lcm_message_length,
                   systems::VectorBase<double>* vector_base) const override;
  void Serialize(double time, const systems::VectorBase<double>& vector_base,
                 std::vector<uint8_t>* lcm_message_bytes) const override;
};

}  // namespace automotive
}  // namespace drake
