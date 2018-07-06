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


// TODO(mws): Change the name of this controller or the comment, depending on
// which of us is correct. My understanding of impedance control is that it
// uses the operational-space inertia matrix (at the end effector) and the
// velocity at the end-effector; this appears to be a "stiffness" controller.
/// An operational space "stiffness" controller.
/// Inputs:
/// - 0: the estimated current state.
/// - 1: the Cartesian target position.
/// Outputs:
/// - 0: the motor torques.
/// - 1:
class ImpedanceController : public systems::LeafSystem<double> {

 public:

  // Constructor with LCM. Will draw target point if publish is on.
  ImpedanceController(const RigidBodyTree<double>& tree,
                      const RigidBodyFrame<double>& controlled_frame,
                      const Vector3<double>& k_p,
                      const Vector3<double>& k_d,
                      drake::lcm::DrakeLcm& lcm) :
      ImpedanceController(tree, controlled_frame, k_p, k_d) {

    draw_status_ = true;
    lcm_ = &lcm;
  }

  // Constructor without lcm. Draws nothing.
  ImpedanceController(const RigidBodyTree<double>& tree,
                      const RigidBodyFrame<double>& controlled_frame,
                      const Vector3<double>& k_p,
                      const Vector3<double>& k_d) :
      state_size_(tree.get_num_positions() + tree.get_num_velocities()),
      command_output_size_(tree.get_num_actuators()),
      controlled_frame_(controlled_frame),
      tree_(tree), k_p_(k_p), k_d_(k_d),
      previous_torques_(command_output_size_){

    BasicVector<double> integrator_state(target_input_size_);
    integrator_state.SetZero();

    this->DeclareContinuousState(integrator_state); // For integral control.
    this->DeclareVectorInputPort(BasicVector<double>(state_size_)); // Input 0.
    this->DeclareVectorInputPort(BasicVector<double>(target_input_size_)); // Input 1.
    this->DeclareVectorOutputPort(BasicVector<double>(command_output_size_), &ImpedanceController::DoControlCalc); // Output 0.
    this->DeclareVectorOutputPort(BasicVector<double>(target_input_size_), &ImpedanceController::GetHandPos); // Output 1.
  }

  /**
   * Returns the input port for the estimated state.
   */
  const InputPortDescriptor<double>& get_input_port_estimated_state() const;

  /**
   * Returns the input port for the desired state.
   */
  const InputPortDescriptor<double>& get_input_port_cartesian_target() const;

  /**
   * Returns the output port for computed control.
   */
  const OutputPort<double>& get_output_port_control() const;

  const OutputPort<double>& get_output_port_hand_pos() const;

  void DoPublish(const Context<double>& context, const std::vector<const PublishEvent<double>*>& events) const;

 private:
  const int state_size_; // angles and angular rates at shoulder and elbow.
  const int target_input_size_ = 3; // x, y, z end effector target.
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