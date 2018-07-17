#pragma once

#include "drake/examples/iiwa_soccer/soccer_common.h"
#include "drake/systems/framework/event.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/rigid_body_plant/contact_results.h"
#include "drake/systems/framework/abstract_values.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/multibody/rigid_body_constraint.h"
#include "drake/multibody/ik_options.h"
#include "drake/multibody/rigid_body_ik.h"

namespace drake {
namespace examples {
namespace iiwa_soccer {

using systems::LeafSystem;
using systems::Context;
using systems::DiscreteUpdateEvent;
using systems::DiscreteValues;
using systems::ContactResults;
using systems::AbstractValue;
using trajectories::PiecewisePolynomial;
using Eigen::Vector3d;
using Eigen::Vector2d;
using Eigen::MatrixXd;
using Eigen::VectorXi;

class FSM_System : public LeafSystem<double> {
 public:
  mutable PiecewisePolynomial<double> next_traj;

  enum controller_states {

    IDLE,
    START_IK_TRAJ,

    // States for the simple ball-pushing routine.
    START_SPLINE_TO_ABOVE_BALL, // Plan a spline to resting on top of the ball.
    AWAIT_CONTACT, // Wait for arm to contact ball. When it does, plan a new trajectory to the other side of the ball.
    AWAIT_BREAKING_CONTACT, // Wait for the arm to break contact from the ball. When it does, stop updating the spline, and wait until the arm contacts the ball again.
  };

  /**
   * @param control_togglers: Struct containing pointers to various "switcher-systems" which will be used to trigger replanning and change controllers.
   * @param collision_table: Map of collision_id to the name of the associated body. Used to determine which objects are coming into contact.
   */
  FSM_System(SystemSwitches control_togglers, std::map<unsigned long, std::string> collision_table) : switches_(control_togglers),
                                                                                                      collision_table_(collision_table) {
    arm_state_in_id_ = this->DeclareVectorInputPort(BasicVector<double>(kuka_arm_state_dim_)).get_index();
    ball_state_in_id_ = this->DeclareVectorInputPort(BasicVector<double>(ball_state_dim_)).get_index();
    contact_results_in_id_ = this->DeclareAbstractInputPort().get_index(); // Will be type ContactResults<double>()

    this->DeclarePeriodicDiscreteUpdate(1.0/control_freq_);
    this->DeclareDiscreteState(1); // Stores the controller state as a whole number double corresponding to the enum index in controller_states.
  }

  const InputPortDescriptor<double>& get_input_port_arm_state() const {
    return this->get_input_port(arm_state_in_id_);
  }

  const InputPortDescriptor<double>& get_input_port_ball_state() const {
    return this->get_input_port(ball_state_in_id_);
  }

  const InputPortDescriptor<double>& get_input_port_contact_results() const {
    return this->get_input_port(contact_results_in_id_);
  }


 private:
  int arm_state_in_id_; // Input port ID for the arm's state.
  int ball_state_in_id_; // Input port ID for the ball's state.
  int contact_results_in_id_; // Input port ID for the contact results.

  const int kuka_arm_state_dim_ = 14;  // 7 DoF x 2.
  const int ball_state_dim_ = 13;      // 7 configuration, 6 velocity.
  const double control_freq_ = 100;    // 100Hz update rate of FSM.

  mutable double last_update_time = 0;
  SystemSwitches switches_;

  std::map<unsigned long, std::string> collision_table_;


  mutable int tmp_fun = 1;

  void DoCalcDiscreteVariableUpdates(const Context<double>& context,
                                     const std::vector<const DiscreteUpdateEvent<double>*>& events,
                                     DiscreteValues<double>* discrete_state) const override {

    double time = context.get_time();
    int current_fsm_state = context.get_discrete_state().get_vector().GetAtIndex(0);
    controller_states updated_state = static_cast<controller_states>(current_fsm_state);

    const BasicVector<double>* arm_state = this->EvalVectorInput(context, arm_state_in_id_);
    const BasicVector<double>* ball_state = this->EvalVectorInput(context, ball_state_in_id_);
    const ContactResults<double>* contact_results = this->EvalInputValue<ContactResults<double>>(context, contact_results_in_id_);

    // Do hand kinematics.
    VectorX<double> st_vecx = arm_state->CopyToVector();
    VectorX<double> st_just_pos = st_vecx.topRows(kuka_arm_state_dim_/2);
    KinematicsCache<double> kinematics_cache = switches_.tree->doKinematics(st_just_pos);
    Isometry3<double> hand_world_pose = switches_.tree->CalcFramePoseInWorldFrame(kinematics_cache, *switches_.tree->findFrame("iiwa_link_7").get());

    // Indicate that the manipulator arm is not touching the ball (unless otherwise established).
    bool arm_contacts_ball = false;
    // Check which contacts are active.
    for (int i = 0; i < contact_results->get_num_contacts(); i++) {
      unsigned long id1 = contact_results->get_contact_info(i).get_element_id_1();
      unsigned long id2 = contact_results->get_contact_info(i).get_element_id_2();

      // Make sure the collision ID exists in the collision table we've been given here.
      DRAKE_DEMAND(collision_table_.find(id1) != collision_table_.end() && collision_table_.find(id2) != collision_table_.end());

      const std::string& body1 = collision_table_.at(id1);
      const std::string& body2 = collision_table_.at(id2);

      // Check if the ball is contacting anything besides the ball.
      if (body1 != "world" && body2 != "world") {
        if (body1 == "ball" || body2 == "ball")
          arm_contacts_ball = true;
      }
    }

//    const Vector3d pos_diff = ee_pos->CopyToVector() - ball_state->CopyToVector().topRows(end_effector_pos_dimensions_);
//    const double sq_dist_from_ball = pos_diff.squaredNorm();

    // TODO(mws): Repace all of the magic numbers below with named constants
    // located in a central location (like here).
    switch (current_fsm_state) {
      case IDLE: {

        if (time > last_update_time + 5) {
          updated_state = START_IK_TRAJ;
        }
        break;
      }
      case START_IK_TRAJ: {
        switches_.torque_source_switcher->switch_output("inv_dyn_w_grav_comp");
        double duration = 5 + time;
        last_update_time = time;
        MakeControlledKukaPlan(st_just_pos, ball_state->CopyToVector().topRows(3) + Vector3d(0,0,0.2), time, duration, &next_traj);
        switches_.ik_traj_receiver->set_output(next_traj);
        tmp_fun = -tmp_fun;
        updated_state = IDLE;
        break;
      }


      case START_SPLINE_TO_ABOVE_BALL: {
        switches_.torque_source_switcher->switch_output("imped_w_grav_comp"); // Torque to arm comes from impedance control and gravity compensation added.
        switches_.offset_source_switcher->switch_output("zero_offset"); // Impedance controller tracks center of ball with no offset.
        switches_.setpt_source_switcher->switch_output("spline"); // Impedance controller target point assigned by spline generator.

        switches_.ball_target_offset->set_offset(Vector3d(0.02, 0.01, 0.1));
        switches_.waypoint_target_offset->set_offset(Vector3d(0, 0, 0.5));
        switches_.spline_maker->set_start_pt(hand_world_pose.translation(), context.get_time(), 1, 7);


        updated_state = AWAIT_CONTACT;
        break;
      }
      case AWAIT_CONTACT: {

        if (arm_contacts_ball) {

          const double xdot = ball_state->GetAtIndex(10);
          const double ydot = ball_state->GetAtIndex(11);

          const double x_target_low = -0.3;
          const double x_target_high = 0.3;
          const double y_target_low = -0.3;
          const double y_target_high = 0.3;
          const double xdot_target_scaled = xdot*direction_multiply_target;
          const double ydot_target_scaled = ydot*direction_multiply_target;

          const double x_waypoint_low = -0.3;
          const double x_waypoint_high = 0.3;
          const double y_waypoint_low = -0.3;
          const double y_waypoint_high = 0.3;
          const double xdot_waypoint_scaled = xdot*direction_multiply_target;
          const double ydot_waypoint_scaled = ydot*direction_multiply_target;


          // Set offsets relative to the ball for the spline's target and a waypoint along that spline.
          switches_.ball_target_offset->set_offset(
              Vector3d(ClampWithOffsetFromZero(xdot_target_scaled, x_target_low, x_target_high, direction_target_min),
                       ClampWithOffsetFromZero(ydot_target_scaled, y_target_low, y_target_high, direction_target_min),
                       target_offset_height));
          switches_.waypoint_target_offset->set_offset(
              Vector3d(ClampWithOffsetFromZero(xdot_waypoint_scaled, x_waypoint_low, x_waypoint_high, direction_waypoint_min),
                       ClampWithOffsetFromZero(ydot_waypoint_scaled, y_waypoint_low, y_waypoint_high, direction_waypoint_min),
                       waypoint_offset_height));

          // Trigger new spline. It will forever have the same startpoint although waypoint and target will change. The times will change the speed.
          switches_.spline_maker->set_start_pt(hand_world_pose.translation(), context.get_time(), waypoint_time, target_time);
          updated_state = AWAIT_BREAKING_CONTACT;
        }
        break;
      }
      case AWAIT_BREAKING_CONTACT: {
        const double xdot = ball_state->GetAtIndex(10);
        const double ydot = ball_state->GetAtIndex(11);

        const double x_target_low = -0.3;
        const double x_target_high = 0.3;
        const double y_target_low = -0.3;
        const double y_target_high = 0.3;
        const double xdot_target_scaled = xdot*direction_multiply_target;
        const double ydot_target_scaled = ydot*direction_multiply_target;

        const double x_waypoint_low = -0.3;
        const double x_waypoint_high = 0.3;
        const double y_waypoint_low = -0.3;
        const double y_waypoint_high = 0.3;
        const double xdot_waypoint_scaled = xdot*direction_multiply_target;
        const double ydot_waypoint_scaled = ydot*direction_multiply_target;


        // Set offsets relative to the ball for the spline's target and a waypoint along that spline.
        switches_.ball_target_offset->set_offset(
            Vector3d(ClampWithOffsetFromZero(xdot_target_scaled, x_target_low, x_target_high, direction_target_min),
                     ClampWithOffsetFromZero(ydot_target_scaled, y_target_low, y_target_high, direction_target_min),
                     target_offset_height));
        switches_.waypoint_target_offset->set_offset(
            Vector3d(ClampWithOffsetFromZero(xdot_waypoint_scaled, x_waypoint_low, x_waypoint_high, direction_waypoint_min),
                     ClampWithOffsetFromZero(ydot_waypoint_scaled, y_waypoint_low, y_waypoint_high, direction_waypoint_min),
                     waypoint_offset_height));

        if (!arm_contacts_ball) {
          updated_state = AWAIT_CONTACT;
        }
        break;
      }
      default: {
        throw new std::runtime_error("undefined FSM state");
      }
    }

    auto fsm_state_update_vector = std::make_unique<BasicVector<double>>(1);
    fsm_state_update_vector->SetAtIndex(0, updated_state);
    DiscreteValues<double> fsm_state_update_value(std::move(fsm_state_update_vector));
    discrete_state->SetFrom(fsm_state_update_value);

  }

  /**
   * Clamp a value between a min/max value. Also, add a constant term forcing the value away from zer0.
   *
   * @param value_to_clamp Input to clamp/offset.
   * @param min_allowed Smallest allowed value (after offset).
   * @param max_allowed Largest allowed value (after offset).
   * @param non_zero_offset Offset applied to output, applied before clamping. Applied as signum(value_to_clamp)*non_zero_offset
   * @return Clamped, offset version of value_to_clamp.
   */
  double ClampWithOffsetFromZero(const double& value_to_clamp, const double& min_allowed, const double& max_allowed, const double& non_zero_offset) const {
    double offset = value_to_clamp > 0 ? non_zero_offset : -non_zero_offset;
    return std::fmin(std::fmax(value_to_clamp + offset, min_allowed), max_allowed);
  }


  void MakeControlledKukaPlan(VectorXd& start_config, Vector3d ee_pos_end, double& start_time, double& end_time, PiecewisePolynomial<double>* output) const {

    DRAKE_DEMAND(start_config.size() == 7);

    // Creates a basic pointwise IK trajectory for moving the iiwa arm.
    // It starts in the zero configuration (straight up).
    VectorXd joint_lb = start_config - VectorXd::Constant(7, 0.01);
    VectorXd joint_ub = start_config + VectorXd::Constant(7, 0.01);

    PostureConstraint pc1(switches_.tree, Vector2d(start_time, start_time + 0.009));
    VectorXi joint_idx(7);
    joint_idx << 0, 1, 2, 3, 4, 5, 6;
    pc1.setJointLimits(joint_idx, joint_lb, joint_ub);

    // Defines an end effector constraint.
    Vector3d pos_lb = ee_pos_end - Vector3d::Constant(0.005);
    Vector3d pos_ub = ee_pos_end + Vector3d::Constant(0.005);
    WorldPositionConstraint wpc1(switches_.tree, switches_.tree->FindBodyIndex("iiwa_link_7"),
                                 Vector3d::Zero(), pos_lb, pos_ub,
                                 Vector2d(start_time + 0.01, end_time));


    Vector3d lb;
    Vector3d ub;
    if (tmp_fun > 0) {
      lb = Vector3d(-M_PI, -M_PI, 1.5);
      ub = Vector3d(M_PI, M_PI, 1.52);
    }else {
      lb = Vector3d(-M_PI, -M_PI, -1.52);
      ub = Vector3d(M_PI, M_PI, -1.5);
    }
    const Vector3d euler_ub(ub);
    const Vector3d euler_lb(lb);

    WorldEulerConstraint wqc1(switches_.tree, switches_.tree->FindBodyIndex("iiwa_link_7"), euler_lb, euler_ub, Vector2d(start_time + 0.01, end_time));

//    // After the end effector constraint is released, applies the straight
//    // up configuration again from time 4 to 5.9.
//    PostureConstraint pc2(switches_.tree, Vector2d(4, 5.9));
//    pc2.setJointLimits(joint_idx, joint_lb, joint_ub);
//
//    // Apply the same end effector constraint from time 6 to 9 of the demo.
//    WorldPositionConstraint wpc2(switches_.tree, switches_.tree->FindBodyIndex("iiwa_link_ee"),
//                                 Vector3d::Zero(), pos_lb, pos_ub,
//                                 Vector2d(6, 9));
//
//    // For part of the time wpc2 is active, constrains the second joint while
//    // preserving the end effector constraint.
//    //
//    // Variable `joint_position_start_idx` below is a collection of offsets into
//    // the state vector referring to the positions of the joints to be
//    // constrained.
//    Eigen::VectorXi joint_position_start_idx(1);
//    joint_position_start_idx(0) =
//        switches_.tree->FindChildBodyOfJoint("iiwa_joint_2")->get_position_start_index();
//    PostureConstraint pc3(switches_.tree, Vector2d(6, 8));
//    pc3.setJointLimits(joint_position_start_idx, Vector1d(0.7), Vector1d(0.8));

    double time_diff = end_time - start_time;
    const std::vector<double> kTimes{start_time,
                                     start_time + time_diff/10,
                                     start_time + 2*time_diff/5,
                                     start_time + 3*time_diff/5,
                                     start_time + 4*time_diff/5,
                                     end_time};
    MatrixXd q_seed(switches_.tree->get_num_positions(), kTimes.size());
    MatrixXd q_nom(switches_.tree->get_num_positions(), kTimes.size());
    for (size_t i = 0; i < kTimes.size(); ++i) {
      // Zero configuration is a bad initial guess for IK, to be solved through
      // nonlinear optimization, as the robot configuration is in singularity,
      // and the gradient is zero. So we add 0.1 as the arbitrary pertubation
      // to the zero configuration.
      q_seed.col(i) =
          start_config + 0.1 * Eigen::VectorXd::Ones(switches_.tree->get_num_positions());
      q_nom.col(i) = start_config;
    }

    std::vector<RigidBodyConstraint*> constraint_array;
    constraint_array.push_back(&pc1);
    constraint_array.push_back(&wpc1);
    constraint_array.push_back(&wqc1);
//    constraint_array.push_back(&pc2);
//    constraint_array.push_back(&pc3);
//    constraint_array.push_back(&wpc2);
    IKoptions ikoptions(switches_.tree);
    std::vector<int> info(kTimes.size(), 0);
    MatrixXd q_sol(switches_.tree->get_num_positions(), kTimes.size());
    std::vector<std::string> infeasible_constraint;

    inverseKinPointwise(switches_.tree, kTimes.size(), kTimes.data(), q_seed, q_nom,
                        constraint_array.size(), constraint_array.data(),
                        ikoptions, &q_sol, info.data(), &infeasible_constraint);
//    bool info_good = true;
//    for (size_t i = 0; i < kTimes.size(); ++i) {
//      drake::log()->info("INFO[{}] = {} ", i, info[i]);
//      if (info[i] != 1) {
//        info_good = false;
//      }
//    }
//    printf("\n");
//
//    if (!info_good) {
//      throw std::runtime_error(
//          "inverseKinPointwise failed to compute a valid solution.");
//    }

    std::vector<MatrixXd> knots(kTimes.size());
    for (size_t i = 0; i < kTimes.size(); ++i) {
      // We only use column 0 of the matrix in knots (for joint positions),
      // so we write a vector.
      knots[i] = q_sol.col(i);
    }
    *output = PiecewisePolynomial<double>::FirstOrderHold(kTimes, knots);

  }
};

}  // namespace iiwa_soccer
}  // namespace examples
}  // namespace drake}
