#pragma once

#include "drake/systems/framework/vector_system.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/common/trajectories/trajectory.h"
#include "drake/common/trajectories/piecewise_polynomial.h"


namespace drake {
namespace examples {
namespace iiwa_soccer {

using systems::VectorSystem;
using systems::BasicVector;
using systems::RigidBodyPlant;
using trajectories::Trajectory;
using trajectories::PiecewisePolynomial;
using Eigen::Vector3d;
using Eigen::VectorXd;


struct PolyWithKnots{
  PiecewisePolynomial<double> poly;

  std::vector<double> breaks;

  std::vector<MatrixX<double>> knots;
};

class MagicForce : public VectorSystem<double> {

 public:
  bool repeat = true;
  bool feedback = false;
  double kp = 0.5;

  MagicForce(RigidBodyPlant<double>* plant_ptr,
             RigidBodyTree<double>* tree_ptr,
             std::vector<PolyWithKnots>& poly_data,
             bool use_feedback=false) :
      VectorSystem<double>(13,0),
      plant_ptr_(plant_ptr),
      tree_ptr_(tree_ptr),
      poly_data_(poly_data) {
    feedback = use_feedback;
    this->DeclareContinuousState(1);
    accel_multiplier = ball_mass + ball_inertia/(ball_radius*ball_radius);
  }

 private:
  RigidBodyPlant<double>* const plant_ptr_;
  RigidBodyTree<double>* const tree_ptr_;
  std::vector<PolyWithKnots>& poly_data_;
  mutable int current_poly = -1;
  mutable PiecewisePolynomial<double> curr_path_position;
  mutable std::unique_ptr<Trajectory<double>> curr_path_vel;
  mutable std::unique_ptr<Trajectory<double>> curr_path_accel;
  const double ball_mass = 0.420;
  const double ball_inertia = 0.00137;
  const double ball_radius = 0.07;
  //const double next_segment_beginning_threshold = 0.005;
  double accel_multiplier;
  mutable bool ballistic_phase = false;
  mutable double time_offset = 0;

  virtual void DoCalcVectorTimeDerivatives(
      const drake::systems::Context<double>& context,
      const Eigen::VectorBlock<const Eigen::VectorXd>& input,
      const Eigen::VectorBlock<const Eigen::VectorXd>& state,
      Eigen::VectorBlock<Eigen::VectorXd>* derivatives) const {

    // TODO this is only here to keep the compiler from throwing "unused" errors. Fix this properly.
    tree_ptr_->get_num_bodies();

    double time = context.get_time() + time_offset;
    double fx = 0;
    double fy = 0;

    if (current_poly < 0. || time > curr_path_position.end_time()) {
      if (current_poly < static_cast<int>(poly_data_.size() - 1)) {
        curr_path_position = poly_data_[++current_poly].poly;
        auto path_dt = curr_path_position.MakeDerivative(1);
        curr_path_vel = std::move(path_dt);
        auto path_ddt = curr_path_position.MakeDerivative(2);
        curr_path_accel = std::move(path_ddt);

        if (time < curr_path_position.start_time()) {
          ballistic_phase = true;
          drake::log()->info("Ballistic phase beginning.");
        }
      }else if (repeat) {
        drake::log()->info("Repeating.");
        time_offset -= curr_path_position.end_time();
        current_poly = 0;
        curr_path_position = poly_data_[current_poly].poly;
        auto path_dt = curr_path_position.MakeDerivative(1);
        curr_path_vel = std::move(path_dt);
        auto path_ddt = curr_path_position.MakeDerivative(2);
        curr_path_accel = std::move(path_ddt);
        time = context.get_time() + time_offset;
      }
    }


    if (ballistic_phase) {

      // Adjust timing around ballistic phase.
//      auto curr_pos = this->EvalVectorInput(context);
//      auto next_segment_beginning_pos = curr_path_position.value(curr_path_position.start_time());
//      auto next_segment_beginning_vel = curr_path_vel->value(curr_path_position.start_time());
//      Vector3d next_poly_vel_start(next_segment_beginning_vel.col(0)[0], next_segment_beginning_vel.col(0)[1], 0); // TODO cross product won't work without doing stuff like this. Figure out what I'm doing wrong.
//      Vector3d error_vec(curr_pos[0] - next_segment_beginning_pos.col(0)[0],
//                         curr_pos[1] - next_segment_beginning_pos.col(0)[1],
//                         0);
//
//      if (std::fabs(error_vec.dot(next_poly_vel_start)) < next_segment_beginning_threshold) {
//        time_offset = time - curr_path_position.start_time();
//        ballistic_phase = false;
//        drake::log()->info("Ballistic phase done.");
//        drake::log()->info(time_offset);
//      }

      if (time >= curr_path_position.start_time()) {
        ballistic_phase = false;
      }
    }else {
      auto path_accel = curr_path_accel->value(time);
      if (time < curr_path_accel->end_time()) { // Should we execute over and over and over?

       // time = std::fmod(context.get_time(), poly_data_.back().poly.end_time());
        fx = accel_multiplier * path_accel.col(0)[0];
        fy = accel_multiplier * path_accel.col(0)[1];
      }

      if (feedback) {
        Eigen::MatrixXd traj_pos = curr_path_position.value(time);
        Eigen::MatrixXd traj_vel = curr_path_vel->value(time);

        Vector3d up_vec(0, 0, 1);
        Vector3d vel_vec(traj_vel(0), traj_vel(1), traj_vel(2)); // TODO do this correctly

        auto traj_normal = vel_vec.cross(up_vec);
        traj_normal.normalize();

        auto curr_pos = this->EvalVectorInput(context);
        auto error = curr_pos.topRows(3) - traj_pos;

        auto error_perp = error.dot(traj_normal)*traj_normal;

        fx += kp*(-error_perp(0));
        fy += kp*(-error_perp(1));
      }
    }

//    if (path_accel.col(0)[0] * path_accel.col(0)[0] + path_accel.col(0)[1] * path_accel.col(0)[1] < 1.5*1.5){
//      fx = 0;
//      fy = 0;
//    }


    VectorXd ball_vec = this->EvalVectorInput(context);
    //Vector3d ball_position = ball_vec.topRows(3);
    VectorXd ball_quat = ball_vec.segment(3,4);
    Quaternion<double> ball_rot(ball_quat[0], ball_quat[1], ball_quat[2], ball_quat[3]);
    Quaternion<double> inv_ball_rot = ball_rot.conjugate();

    Vector3d ball_force(fx,fy,1);
    Vector3d ball_force_rot = inv_ball_rot._transformVector(ball_force);

    Eigen::VectorXd wrench(6);
    wrench.setZero();
    wrench.bottomRows(3) = ball_force_rot;
    plant_ptr_->set_wrench(wrench);

    (*derivatives)(0) = 0;
  }
};

}
}
}