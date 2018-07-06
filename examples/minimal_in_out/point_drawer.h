#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/common/eigen_types.h"
#include "external/lcmtypes_bot2_core/lcmtypes/bot_core/pointcloud_t.hpp"


namespace drake {
namespace examples {
namespace minimal_in_out {

using systems::LeafSystem;
using systems::Context;
using systems::BasicVector;
using Eigen::Vector3d;

class PointDrawer : public LeafSystem<double> {

 public:
  PointDrawer();

 private:
  void DoOutputCalc(const Context<double>& context, bot_core::pointcloud_t* output) const;

};


} // namespace minimal_in_out
} // namespace examples
} // namespace drake