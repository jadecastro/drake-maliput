#include "drake/automotive/endless_road_oracle-inl.h"

#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace automotive {

// These instantiations must match the API documentation in
// endless_road_oracle.h.
template class DRAKEAUTOMOTIVE_EXPORT EndlessRoadOracle<double>;
//LATER? template class DRAKEAUTOMOTIVE_EXPORT EndlessRoadOracle<drake::TaylorVarXd>;

}  // namespace automotive
}  // namespace drake
