#pragma once

#include "drake/systems/framework/vector_system.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/common/trajectories/trajectory.h"
#include "drake/common/trajectories/piecewise_polynomial.h"


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

struct PolyWithKnots{
  PiecewisePolynomial<double> poly;

  std::vector<double> breaks;

  std::vector<MatrixX<double>> knots;
};

class MagicForce : public LeafSystem<double> {

 public:
  bool repeat = true;
  bool feedback = false;
  double kp = 0.5;

  MagicForce(RigidBodyPlant<double>* plant_ptr,
             RigidBodyTree<double>* tree_ptr,
             std::vector<PolyWithKnots>& poly_data,
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

  virtual void DoCalcTimeDerivatives(
      const drake::systems::Context<double>& context,
      ContinuousState<double>* derivatives) const {

    // TODO this is only here to keep the compiler from throwing "unused" errors. Fix this properly.
    tree_ptr_->get_num_bodies();

    double time = context.get_time() + time_offset;

    const BasicVector<double>* ball_pos = this->EvalVectorInput(context, 0);
    const VectorXd ball_pos_vec = ball_pos->CopyToVector();

    const BasicVector<double>* manipulator_ball_pos = this->EvalVectorInput(context, 1);
    const VectorXd manipulator_ball_pos_vec = manipulator_ball_pos->CopyToVector();

    const ContactResults<double>* contact_results = this->EvalInputValue<ContactResults<double>>(context, 2);
    VectorX<double> c_force = contact_results->get_generalized_contact_force();
    c_force.topRows(1);

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

        VectorXd traj_normal = vel_vec.cross(up_vec);
        traj_normal.normalize();

        VectorXd error = ball_pos_vec.topRows(3) - traj_pos;

        VectorXd error_perp = error.dot(traj_normal)*traj_normal;

        fx += kp*(-error_perp(0));
        fy += kp*(-error_perp(1));
      }
    }


    // Force directly to rolling ball.

    VectorXd ball_quat = ball_pos_vec.segment(3,4);
    Quaternion<double> ball_rot(ball_quat[0], ball_quat[1], ball_quat[2], ball_quat[3]);
    Quaternion<double> inv_ball_rot = ball_rot.conjugate();

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
    Vector3d manip_f = 10*(offset + ball_pos_vec.topRows(3) - manipulator_ball_pos_vec.topRows(3)) + 1*(ball_pos_vec.segment(10,3) - manipulator_ball_pos_vec.segment(10,3));
    Vector3d manipulator_ball_force(manip_f);
//    manipulator_ball_force.setZero();
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