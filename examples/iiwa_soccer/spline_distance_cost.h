#pragma once

#include "drake/common/eigen_types.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/function.h"
#include "drake/solvers/cost.h"

namespace drake {
namespace examples {
namespace iiwa_soccer {

using trajectories::PiecewisePolynomial;
using solvers::detail::VecIn;
using solvers::detail::VecOut;
using trajectories::Trajectory;
using Eigen::Vector3d;
using solvers::Cost;

class SplineDistanceCost : public Cost {

 public:
  const double pos_error_weight = 1;
  const double vel_error_weight = 0.1;
  SplineDistanceCost(PiecewisePolynomial<double>& poly) : Cost(1), poly_(poly) {

    std::unique_ptr<Trajectory<double>> poly_dt = poly_.MakeDerivative(1);
    std::unique_ptr<Trajectory<double>> poly_ddt = poly_.MakeDerivative(2);
    poly_dt_ptr_ = std::move(poly_dt);
    poly_ddt_ptr_ = std::move(poly_ddt);
  }

  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd& y) const {
    y(0) = ScalarEval(x(0));
  }

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                          AutoDiffVecXd& y) const {


    y(0).value() = ScalarEval(x(0).value());
    auto h = ScalarDiffEval(x(0).value());

    y[0].derivatives() = Vector1d(h);

  }

  void SetCurrentState(Vector3d& current_location, Vector3d& current_velocity) {
    curr_loc_ = current_location;
    curr_vel_ = current_velocity;
  }


  // Convenience method.
  double ScalarEval(double t) const {
    Vector3d poly_loc = poly_.value(t);
    Vector3d poly_vel = poly_dt_ptr_->value(t);

    Vector3d dist_error = curr_loc_ - poly_loc;
    double dist_error_sq = dist_error.squaredNorm();

    Vector3d vel_error = curr_vel_ - poly_vel;
    double vel_error_sq = vel_error.squaredNorm();

    return pos_error_weight * dist_error_sq + vel_error_weight * vel_error_sq;
  }

  double ScalarDiffEval(double t) const {
    Vector3d poly_loc = poly_.value(t);
    Vector3d poly_vel = poly_dt_ptr_->value(t);
    Vector3d poly_accel = poly_ddt_ptr_->value(t);

    Vector3d dist_error = curr_loc_ - poly_loc;
    Vector3d vel_error = curr_vel_ - poly_vel;

    return 2 * pos_error_weight * dist_error.dot(-poly_vel) + 2 * vel_error_weight * vel_error.dot(-poly_accel);
  }

 private:
  PiecewisePolynomial<double> poly_;
  std::unique_ptr<Trajectory<double>> poly_dt_ptr_;
  std::unique_ptr<Trajectory<double>> poly_ddt_ptr_;

  Vector3d curr_loc_;
  Vector3d curr_vel_;
};

}
}
}
