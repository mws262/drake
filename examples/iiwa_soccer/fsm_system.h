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

  enum states {
    IDLE,
    CWISE_FROM_BALL,
    ABOVE_BALL_1,
    CCWISE_FROM_BALL,
    ABOVE_BALL_2,

    // Iffy below
        ABOVE_HOVER,
    TOP_1,
    TOP_1_PUSHING,

    TRAJ1,
    TRAJ_WAIT,

  };

 public:
  FSM_System(switches control_togglers, std::map<unsigned long, std::string> collision_table) : switches_(control_togglers), collision_table_(collision_table) {
    arm_state_in_id = this->DeclareVectorInputPort(BasicVector<double>(14)).get_index();
    ball_state_in_id = this->DeclareVectorInputPort(BasicVector<double>(13)).get_index();
    ee_pos_in_id = this->DeclareVectorInputPort(BasicVector<double>(3)).get_index();
    contact_results_in_id = this->DeclareAbstractInputPort().get_index(); //ContactResults<double>()

    this->DeclarePeriodicDiscreteUpdate(0.01);
    this->DeclareDiscreteState(1);  // This does nothing besides a

  }

 private:
  switches switches_;
  mutable states current_state = states::IDLE;
  int arm_state_in_id;
  int ball_state_in_id;
  int ee_pos_in_id;
  int contact_results_in_id;
  std::map<unsigned long, std::string> collision_table_;
  mutable std::unique_ptr<PiecewisePolynomial<double>> next_traj;

  void DoCalcDiscreteVariableUpdates(
      const Context<double>& context,
      const std::vector<const DiscreteUpdateEvent<double>*>& events,
      DiscreteValues<double>* discrete_state) const override {
    Vector3d ball_adj(0.2,0,0);
    //const BasicVector<double>* arm_state = this->EvalVectorInput(context, arm_state_in_id);
    const BasicVector<double>* ball_state = this->EvalVectorInput(context, ball_state_in_id);
    const BasicVector<double>* ee_pos = this->EvalVectorInput(context, ee_pos_in_id);
    const ContactResults<double>* contact_results = this->EvalInputValue<ContactResults<double>>(context, contact_results_in_id);



    bool arm_contacts_ball = false;
    // Check which contacts are active.
    for (int i = 0; i < contact_results->get_num_contacts(); i++) {
      unsigned long id1 = contact_results->get_contact_info(i).get_element_id_1();
      unsigned long id2 = contact_results->get_contact_info(i).get_element_id_2();

      if (collision_table_.find(id1) != collision_table_.end() && collision_table_.find(id2) != collision_table_.end()) {
        std::string body1 = collision_table_.at(id1);
        std::string body2 = collision_table_.at(id2);

        if (body1 != "world" && body2 != "world") {
          if (body1 == "ball" || body2 == "ball") {
            arm_contacts_ball = true;

          }
        }
      }
    }

    //drake::log()->info(arm_contacts_ball);

    double sq_dist_from-ball = (ee_pos->CopyToVector() - ball_state->CopyToVector()).squaredNorm();

    switch (current_state) {
      case IDLE: // Immediately transition to CWISE.
        switches_.offset_source_switcher->switch_output("ball_offset");
        switches_.setpt_source_switcher->switch_output("follow_obj");
        switches_.offset_adj->set_offset(ball_adj);

        current_state = states::CWISE_FROM_BALL;
        break;
      case CWISE_FROM_BALL:

        // Once close enough go to above ball.
        if (std::fabs(sq_dist_from_ball - 0.2*0.2) < 0.0001) {
          ball_adj[0] = 0;
          ball_adj[2] = 0.4;
          switches_.offset_adj->set_offset(ball_adj);
          current_state = states::ABOVE_BALL_1;
        }
        break;
      case ABOVE_BALL_1:
        // Once above, go to the other side.
        if (std::fabs(zdist*zdist - 0.4*0.4) < 0.001) {
          ball_adj[0] = -0.2;
          ball_adj[2] = 0;
          switches_.offset_adj->set_offset(ball_adj);
          current_state = states::CCWISE_FROM_BALL;
        }
        break;
      case CCWISE_FROM_BALL:
        // Once close enough go to above ball.
        if (std::fabs(sq_dist_from_ball - 0.2*0.2) < 0.0001) {
          ball_adj[0] = 0;
          ball_adj[2] = 0.4;
          switches_.offset_adj->set_offset(ball_adj);
          current_state = states::ABOVE_BALL_2;
        }
        break;

      case ABOVE_BALL_2:
        // Once above, go to the other side.
        if (std::fabs(zdist*zdist - 0.4*0.4) < 0.001) {
          ball_adj[0] = 0.2;
          ball_adj[2] = 0;
          switches_.offset_adj->set_offset(ball_adj);
          current_state = states::CWISE_FROM_BALL;
        }
        break;
        ////////////////////////////

      case ABOVE_HOVER:
        ball_adj[0] = 0.00;
        ball_adj[1] = 0;
        ball_adj[2] = 0.15;

        switches_.offset_source_switcher->switch_output("ball_offset");
        switches_.setpt_source_switcher->switch_output("follow_obj");
        switches_.offset_adj->set_offset(ball_adj);

        if (context.get_time() > 5){
          ball_adj[0] = 0.01;
          ball_adj[1] = 0;
          ball_adj[2] = 0.06;
          switches_.offset_adj->set_offset(ball_adj);
          current_state = TOP_1;
        }

        break;
      case TOP_1:
        if (arm_contacts_ball) {
          ball_adj[0] = -0.01;
          ball_adj[1] = 0;
          ball_adj[2] = 0.00;
          switches_.offset_adj->set_offset(ball_adj);
          current_state = TOP_1_PUSHING;
        }
        break;
      case TOP_1_PUSHING:


        break;
      case TRAJ1:
        switches_.torque_source_switcher->switch_output("inv_dyn_w_grav_comp");
        switches_.offset_source_switcher->switch_output("zero_offset");
        switches_.setpt_source_switcher->switch_output("fixed_input");


        MakeControlledKukaPlan();

        switches_.full_dim_traj_receiver->set_output(next_traj.get());


        current_state = TRAJ_WAIT;

        break;

      case TRAJ_WAIT:
        break;
      default:
        throw new std::runtime_error("undefined FSM state");
    }
  }

  void MakeControlledKukaPlan() const {


    // Creates a basic pointwise IK trajectory for moving the iiwa arm.
    // It starts in the zero configuration (straight up).
    VectorXd zero_conf = switches_.tree->getZeroConfiguration();
    VectorXd joint_lb = zero_conf - VectorXd::Constant(7, 0.01);
    VectorXd joint_ub = zero_conf + VectorXd::Constant(7, 0.01);

    PostureConstraint pc1(switches_.tree, Vector2d(0, 0.5));
    VectorXi joint_idx(7);
    joint_idx << 0, 1, 2, 3, 4, 5, 6;
    pc1.setJointLimits(joint_idx, joint_lb, joint_ub);

    // Defines an end effector constraint and makes it active for the time span
    // from 1 to 3 seconds.
    Vector3d pos_end(0.6, 0, 0.325);
    Vector3d pos_lb = pos_end - Vector3d::Constant(0.005);
    Vector3d pos_ub = pos_end + Vector3d::Constant(0.005);
    WorldPositionConstraint wpc1(switches_.tree, switches_.tree->FindBodyIndex("iiwa_link_ee"),
                                 Vector3d::Zero(), pos_lb, pos_ub,
                                 Vector2d(1, 3));

    // After the end effector constraint is released, applies the straight
    // up configuration again from time 4 to 5.9.
    PostureConstraint pc2(switches_.tree, Vector2d(4, 5.9));
    pc2.setJointLimits(joint_idx, joint_lb, joint_ub);

    // Apply the same end effector constraint from time 6 to 9 of the demo.
    WorldPositionConstraint wpc2(switches_.tree, switches_.tree->FindBodyIndex("iiwa_link_ee"),
                                 Vector3d::Zero(), pos_lb, pos_ub,
                                 Vector2d(6, 9));

    // For part of the time wpc2 is active, constrains the second joint while
    // preserving the end effector constraint.
    //
    // Variable `joint_position_start_idx` below is a collection of offsets into
    // the state vector referring to the positions of the joints to be
    // constrained.
    Eigen::VectorXi joint_position_start_idx(1);
    joint_position_start_idx(0) =
        switches_.tree->FindChildBodyOfJoint("iiwa_joint_2")->get_position_start_index();
    PostureConstraint pc3(switches_.tree, Vector2d(6, 8));
    pc3.setJointLimits(joint_position_start_idx, Vector1d(0.7), Vector1d(0.8));

    const std::vector<double> kTimes{0.0, 2.0, 5.0, 7.0, 9.0};
    MatrixXd q_seed(switches_.tree->get_num_positions(), kTimes.size());
    MatrixXd q_nom(switches_.tree->get_num_positions(), kTimes.size());
    for (size_t i = 0; i < kTimes.size(); ++i) {
      // Zero configuration is a bad initial guess for IK, to be solved through
      // nonlinear optimization, as the robot configuration is in singularity,
      // and the gradient is zero. So we add 0.1 as the arbitrary pertubation
      // to the zero configuration.
      q_seed.col(i) =
          zero_conf + 0.1 * Eigen::VectorXd::Ones(switches_.tree->get_num_positions());
      q_nom.col(i) = zero_conf;
    }

    std::vector<RigidBodyConstraint*> constraint_array;
    constraint_array.push_back(&pc1);
    constraint_array.push_back(&wpc1);
    constraint_array.push_back(&pc2);
    constraint_array.push_back(&pc3);
    constraint_array.push_back(&wpc2);
    IKoptions ikoptions(switches_.tree);
    std::vector<int> info(kTimes.size(), 0);
    MatrixXd q_sol(switches_.tree->get_num_positions(), kTimes.size());
    std::vector<std::string> infeasible_constraint;

    inverseKinPointwise(switches_.tree, kTimes.size(), kTimes.data(), q_seed, q_nom,
                        constraint_array.size(), constraint_array.data(),
                        ikoptions, &q_sol, info.data(), &infeasible_constraint);
    bool info_good = true;
    for (size_t i = 0; i < kTimes.size(); ++i) {
      drake::log()->info("INFO[{}] = {} ", i, info[i]);
      if (info[i] != 1) {
        info_good = false;
      }
    }
    printf("\n");

    if (!info_good) {
      throw std::runtime_error(
          "inverseKinPointwise failed to compute a valid solution.");
    }

    std::vector<MatrixXd> knots(kTimes.size());
    for (size_t i = 0; i < kTimes.size(); ++i) {
      // We only use column 0 of the matrix in knots (for joint positions),
      // so we write a vector.
      knots[i] = q_sol.col(i);
    }
    auto traj_unique_ptr = std::make_unique<PiecewisePolynomial<double>>(PiecewisePolynomial<double>::FirstOrderHold(kTimes, knots));
    next_traj.swap(traj_unique_ptr);
  }



};

}  // namespace iiwa_soccer
}  // namespace examples
}  // namespace drake}
