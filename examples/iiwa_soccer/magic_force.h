#pragma once

#include "drake/systems/framework/vector_system.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/common/trajectories/trajectory.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/solvers/decision_variable.h"
#include "drake/examples/iiwa_soccer/spline_distance_cost.h"
#include "drake/solvers/binding.h"
#include "drake/examples/iiwa_soccer/poly_utilities.h"


namespace drake {
namespace examples {
namespace iiwa_soccer {

using systems::LeafSystem;
using systems::BasicVector;
using systems::RigidBodyPlant;
using trajectories::Trajectory;
using trajectories::PiecewisePolynomial;
using Eigen::Vector3d;
using Eigen::VectorXd;
using systems::ContinuousState;
using systems::AbstractValue;
using systems::ContactResults;
using solvers::VectorDecisionVariable;
using solvers::MathematicalProgram;
using solvers::Binding;
using solvers::VectorXDecisionVariable;



class MagicForce : public LeafSystem<double> {

 public:
  bool repeat = true;
  bool feedback = true;
  double kp = 0.5;

  MagicForce(RigidBodyPlant<double>* plant_ptr,
             RigidBodyTree<double>* tree_ptr,
             PolyWithKnots& poly_data,
             bool use_feedback=false) :
      plant_ptr_(plant_ptr),
      tree_ptr_(tree_ptr),
      poly_data_(poly_data) {
    feedback = use_feedback;
    this->DeclareInputPort(systems::PortDataType::kVectorValued, 13);
    this->DeclareInputPort(systems::PortDataType::kVectorValued, 13);
    this->DeclareAbstractInputPort();
    this->DeclareContinuousState(1);
    accel_multiplier = ball_mass + ball_inertia/(ball_radius*ball_radius);

    path_vel = poly_data_.poly.MakeDerivative(1);
    path_accel = poly_data_.poly.MakeDerivative(2);

    // Path "phase" optimization.
    const std::shared_ptr<SplineDistanceCost>
        cost_fcn = std::make_shared<SplineDistanceCost>(poly_data.poly);
    const VectorXDecisionVariable t = path_location_optim.NewContinuousVariables<1>("t");
    decision_var_t = t;
    Binding<SplineDistanceCost> cost_binding = Binding<SplineDistanceCost>(cost_fcn, t);
    path_location_optim.AddCost(cost_binding);
    path_location_optim.AddBoundingBoxConstraint(poly_data.poly.start_time(), poly_data.poly.end_time(), t);
    path_location_optim.SetInitialGuess(t(0), 0.5*(poly_data.poly.start_time() + poly_data.poly.end_time()));
    path_location_cost = cost_fcn.get();
  }

 private:
  RigidBodyPlant<double>* const plant_ptr_;
  RigidBodyTree<double>* const tree_ptr_;
  PolyWithKnots& poly_data_;

  SplineDistanceCost* path_location_cost;
  mutable MathematicalProgram path_location_optim;
  VectorXDecisionVariable decision_var_t;

  std::unique_ptr<Trajectory<double>> path_vel;
  std::unique_ptr<Trajectory<double>> path_accel;

  const double ball_mass = 0.420;
  const double ball_inertia = 0.00137;
  const double ball_radius = 0.07;
  //const double next_segment_beginning_threshold = 0.005;
  double accel_multiplier;
  mutable bool ballistic_phase = false;
  mutable double time_offset = 0;

  virtual void DoCalcTimeDerivatives(
      const drake::systems::Context<double>& context,
      ContinuousState<double>* derivatives) const {

    // TODO this is only here to keep the compiler from throwing "unused" errors. Fix this properly.
    tree_ptr_->get_num_bodies();

    double time = context.get_time() + time_offset;

    const BasicVector<double>* ball_state = this->EvalVectorInput(context, 0);
    const VectorXd ball_state_vec = ball_state->CopyToVector();

    const BasicVector<double>* manipulator_ball_pos = this->EvalVectorInput(context, 1);
    const VectorXd manipulator_ball_pos_vec = manipulator_ball_pos->CopyToVector();

    Vector3d ball_pos = ball_state_vec.topRows(3);
    Vector3d ball_vel = ball_state_vec.bottomRows(3);
    VectorXd ball_quat = ball_state_vec.segment(3,4);
    Quaternion<double> ball_rot(ball_quat[0], ball_quat[1], ball_quat[2], ball_quat[3]);
    Quaternion<double> inv_ball_rot = ball_rot.conjugate();

    //////////////////

    path_location_cost->SetCurrentState(ball_pos, ball_vel);
//    path_location_optim.Solve();
//    double best_t = path_location_optim.GetSolution(path_location_optim.decision_variable(0));
//    path_location_optim.SetInitialGuess(decision_var_t(0), best_t);


    time = std::fmod(time, poly_data_.poly.end_time());
    //////////////////////
    double fx = 0;
    double fy = 0;

    if (ballistic_phase) {
//      if (time > curr_path_position.start_time() + 0.05) {
//        ballistic_phase = false;
//      }
    }else {
      Vector3d accel  = path_accel->value(time);
      // time = std::fmod(context.get_time(), poly_data_.back().poly.end_time());
      fx = accel_multiplier * accel.col(0)[0];
      fy = accel_multiplier * accel.col(0)[1];

      if (feedback) {
        Eigen::MatrixXd traj_pos = poly_data_.poly.value(time);
        Eigen::MatrixXd traj_vel = path_vel->value(time);

        const double omega = 1;
        const double kp_ = (omega * omega) / accel_multiplier;
        const double kd_ = (2 * omega) / accel_multiplier;

        Vector3d error = traj_pos - ball_state_vec.topRows(3);
        Vector3d error_dt = traj_vel - ball_rot._transformVector(ball_state_vec.bottomRows(3)); // Need to transform because ball velocity is still in its own frame. Grrr.

        fx += kp_ * error[0] + kd_ * error_dt[0];
        fy += kp_ * error[1] + kd_ * error_dt[1];
      }
    }


    // Force directly to rolling ball.
    Vector3d ball_force(fx,fy,0);
    Vector3d ball_force_rot = inv_ball_rot._transformVector(ball_force);

    Eigen::VectorXd wrench(6);
    wrench.setZero();
    wrench.bottomRows(3) = ball_force_rot;
    plant_ptr_->set_wrench(wrench,1);

    // Force to manipulator ball.
    VectorXd manipulator_ball_quat = manipulator_ball_pos_vec.segment(3,4);
    Quaternion<double> manipulator_ball_rot(manipulator_ball_quat[0],
                                manipulator_ball_quat[1],
                                manipulator_ball_quat[2],
                                manipulator_ball_quat[3]);
    Quaternion<double> manipulator_inv_ball_rot = manipulator_ball_rot.conjugate();

    const Vector3d offset(0,0,1);
    const Vector3d grav_comp(0,0,ball_mass*9.81);
    Vector3d manip_f = 10*(offset + ball_state_vec.topRows(3) - manipulator_ball_pos_vec.topRows(3)) + 1*(ball_state_vec.segment(10,3) - manipulator_ball_pos_vec.segment(10,3));
    Vector3d manipulator_ball_force(manip_f);
    manipulator_ball_force.setZero();
    Vector3d manipulator_ball_force_rot = manipulator_inv_ball_rot._transformVector(manipulator_ball_force);

    Eigen::VectorXd manipulator_wrench(6);
    manipulator_wrench.setZero();
    manipulator_wrench.bottomRows(3) = manipulator_ball_force_rot;
    plant_ptr_->set_wrench(manipulator_wrench,2);
  }
};

}
}
}