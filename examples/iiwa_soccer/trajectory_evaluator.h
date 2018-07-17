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
using systems::InputPortDescriptor;

class TrajectoryEvaluator : public LeafSystem<double> {

 public:
  TrajectoryEvaluator(int dimension);

  const InputPortDescriptor<double>& get_input_port_trajectory(){
    return this->get_input_port(0);
  }

 private:
  const int dimension_;

  void EvalPolynomial(const Context<double>& context, BasicVector<double>* const output) const;


};

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake}