#include "drake/automotive/endless_road_car-inl.h"

#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace automotive {

// These instantiations must match the API documentation in endless_road_car.h.
template class DRAKEAUTOMOTIVE_EXPORT EndlessRoadCar<double>;
//LATER? template class DRAKEAUTOMOTIVE_EXPORT EndlessRoadCar<drake::TaylorVarXd>;

}  // namespace automotive
}  // namespace drake
