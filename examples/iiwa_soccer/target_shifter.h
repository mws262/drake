#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/input_port_descriptor.h"
#include "drake/systems/framework/output_port.h"

namespace drake {
namespace examples {
namespace iiwa_soccer {

using systems::Context;
using systems::BasicVector;
using systems::LeafSystem;
using systems::InputPortDescriptor;
using systems::OutputPort;

class TargetShifter : public LeafSystem<double> {

 public:
  bool use_cartesian = false;
  TargetShifter();

  const InputPortDescriptor<double>& get_input_port_offset() { return this->get_input_port(offset_input_port_id_); }

  const InputPortDescriptor<double>& get_input_port_object_position() { return this->get_input_port(object_pos_input_port_id_); }
  
 private:
  const int offset_input_port_id_;
  const int object_pos_input_port_id_;

  void DoControlCalc(const Context<double>& context, BasicVector<double>* output) const;



};




}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake}