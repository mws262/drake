#pragma once

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace iiwa_soccer {

using systems::LeafSystem;
using systems::BasicVector;
using systems::Context;
using trajectories::PiecewisePolynomial;

class TrajectoryEvaluatorVector : public LeafSystem<double> {

 public:
  TrajectoryEvaluatorVector();

 private:

  void EvalPolynomial(const Context<double>& context, BasicVector<double>* const output) const;


};

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake}