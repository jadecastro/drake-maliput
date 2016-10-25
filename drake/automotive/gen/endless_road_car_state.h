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

/// Describes the row indices of a EndlessRoadCarState.
struct DRAKE_EXPORT EndlessRoadCarStateIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 4;

  // The index of each individual coordinate.
  static const int kS = 0;
  static const int kR = 1;
  static const int kSigmaDot = 2;
  static const int kRhoDot = 3;
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class EndlessRoadCarState : public systems::BasicVector<T> {
 public:
  // An abbreviation for our row index constants.
  typedef EndlessRoadCarStateIndices K;

  /// Default constructor.  Sets all rows to zero.
  EndlessRoadCarState() : systems::BasicVector<T>(K::kNumCoordinates) {
    this->SetFromVector(VectorX<T>::Zero(K::kNumCoordinates));
  }

  /// @name Getters and Setters
  //@{
  const T s() const { return this->GetAtIndex(K::kS); }
  void set_s(const T& s) { this->SetAtIndex(K::kS, s); }
  const T r() const { return this->GetAtIndex(K::kR); }
  void set_r(const T& r) { this->SetAtIndex(K::kR, r); }
  const T sigma_dot() const { return this->GetAtIndex(K::kSigmaDot); }
  void set_sigma_dot(const T& sigma_dot) {
    this->SetAtIndex(K::kSigmaDot, sigma_dot);
  }
  const T rho_dot() const { return this->GetAtIndex(K::kRhoDot); }
  void set_rho_dot(const T& rho_dot) { this->SetAtIndex(K::kRhoDot, rho_dot); }
  //@}
};

}  // namespace automotive
}  // namespace drake
