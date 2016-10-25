#pragma once

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

#include <stdexcept>
#include <string>

#include <Eigen/Core>

#include "drake/common/drake_export.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace automotive {

/// Describes the row indices of a EndlessRoadOracleOutput.
struct DRAKE_EXPORT EndlessRoadOracleOutputIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 2;

  // The index of each individual coordinate.
  static const int kNetDeltaSigma = 0;
  static const int kDeltaSigmaDot = 1;
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class EndlessRoadOracleOutput : public systems::BasicVector<T> {
 public:
  // An abbreviation for our row index constants.
  typedef EndlessRoadOracleOutputIndices K;

  /// Default constructor.  Sets all rows to zero.
  EndlessRoadOracleOutput() : systems::BasicVector<T>(K::kNumCoordinates) {
    this->SetFromVector(VectorX<T>::Zero(K::kNumCoordinates));
  }

  /// @name Getters and Setters
  //@{
  const T net_delta_sigma() const {
    return this->GetAtIndex(K::kNetDeltaSigma);
  }
  void set_net_delta_sigma(const T& net_delta_sigma) {
    this->SetAtIndex(K::kNetDeltaSigma, net_delta_sigma);
  }
  const T delta_sigma_dot() const {
    return this->GetAtIndex(K::kDeltaSigmaDot);
  }
  void set_delta_sigma_dot(const T& delta_sigma_dot) {
    this->SetAtIndex(K::kDeltaSigmaDot, delta_sigma_dot);
  }
  //@}
};

}  // namespace automotive
}  // namespace drake