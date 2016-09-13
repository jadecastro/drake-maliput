#if 0
#include "builder.h"



namespace maliput {
namespace monolane {



Builder::Builder(const api::RBounds& lane_bounds,
                 const api::RBounds& driveable_bounds)
    : lane_bounds_(lane_bounds),
      driveable_bounds_(driveable_bounds) {}


const Connection* Builder::Connect(
    const std::string& id,
    const Point& start,
    const double length,
    const ZPoint& z_end) {

  std::unique_ptr<Connection> cnx = make_unique<Connection>(
      id, Type::kLinear
                                                            );



  Connection* cnxp = cnx.get();
  connections_.push_back(cnx);
  return cnxp;
}



} // namespace monolane
} // namespace maliput
#endif
