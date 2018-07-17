#pragma once

#include "drake/systems/framework/input_port_descriptor.h"
#include "drake/systems/framework/output_port.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/multibody/kinematics_cache.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/event.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/framework/vector_system.h"
#include "drake/systems/framework/continuous_state.h"

namespace drake {
namespace examples {
namespace iiwa_soccer {

using systems::BasicVector;
using systems::Context;
using systems::InputPortDescriptor;
using systems::OutputPort;
using systems::PublishEvent;
using systems::VectorSystem;
using systems::ContinuousState;

/// "Simple" impedance control. Equivalent to a spring-damper tugging on the end effector.
/// An operational space "stiffness" controller.
/// Inputs:
/// - 0: the estimated current state.
/// - 1: the Cartesian target position.
/// Outputs:
/// - 0: the motor torques.
/// - 1:
class CompliantController : public systems::LeafSystem<double> {

 public:
  /**
   *
   * @param tree
   * @param controlled_frame
   * @param k_p
   * @param k_d
   */
  CompliantController(const RigidBodyTree<double>& tree,
                      const RigidBodyFrame<double>& controlled_frame,
                      const Vector3<double>& k_p,
                      const Vector3<double>& k_d);

  // Constructor with LCM. Will draw target point if publish is on.
  CompliantController(const RigidBodyTree<double>& tree,
                      const RigidBodyFrame<double>& controlled_frame,
                      const Vector3<double>& k_p,
                      const Vector3<double>& k_d,
                      drake::lcm::DrakeLcm& lcm) :
      CompliantController(tree, controlled_frame, k_p, k_d) {

    draw_status_ = true;
    lcm_ = &lcm;
  }


  const InputPortDescriptor<double>& get_input_port_estimated_state() const {
    return this->get_input_port(state_input_port_idx_);
  }

  const InputPortDescriptor<double>& get_input_port_cartesian_target() const {
    return this->get_input_port(setpt_input_port_idx_);
  }

  const OutputPort<double>& get_output_port_control() const {
    return this->get_output_port(torque_output_port_idx_);
  }

  void DoPublish(const Context<double>& context, const std::vector<const PublishEvent<double>*>& events) const;

 private:
  int state_input_port_idx_;
  int setpt_input_port_idx_;
  int torque_output_port_idx_;

  const int state_size_; // angles and angular rates at shoulder and elbow.
  const int target_input_size_ = 6; // x, y, z, xdot, ydot, zdot end effector target.
  const int command_output_size_; // torques to shoulder and elbow.
  const RigidBodyFrame<double> controlled_frame_;
  const RigidBodyTree<double>& tree_;
  const Vector3<double> k_p_;
  const Vector3<double> k_d_;

  // TODO(mws): The three fields below appear to be used to store controller
  // state. This violates the design of Drake systems.
  mutable Vector3<double> x_target;
  mutable Vector3<double> hand_pos;
  mutable BasicVector<double> previous_torques_;
  lcm::DrakeLcm* lcm_;

  // If set to 'true', the controller will output visual debugging / logging
  // info through LCM messages.
  bool draw_status_ = false;

  /**
   *  Calculate what torques to apply to the joints.
   * @param context
   * @param output
   */
  void DoControlCalc(const Context<double>& context, BasicVector<double>* const output) const;

  void GetHandPos(const Context<double>& context, BasicVector<double>* const output) const {
    output->SetFromVector(hand_pos);
  }

  void DoCalcTimeDerivatives(const Context<double>& context, ContinuousState<double>* derivatives) const;

};


}  // namespace acrobot
}  // namespace examples
}  // namespace drake