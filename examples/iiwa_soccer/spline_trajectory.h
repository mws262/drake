#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/framework/event.h"
#include "drake/lcm/drake_lcm.h"


namespace drake {
namespace examples {
namespace iiwa_soccer {

using systems::LeafSystem;
using systems::Context;
using trajectories::PiecewisePolynomial;
using systems::PublishEvent;


class SplineTrajectories : public LeafSystem<double> {
 public:
  bool debug_draw = false;
  SplineTrajectories(double start_time, double duration, lcm::DrakeLcm& lcm,  bool lock_start_pos=true);

  void DoPublish(const Context<double>& context, const std::vector<const PublishEvent<double>*>& events) const;

 private:
  void CalcTrajectory(const Context<double>& context, PiecewisePolynomial<double>* const output) const;
  void PublishFrames(std::vector<Eigen::Isometry3d> poses, std::vector<std::string> name) const;

  const double start_time_;
  const double end_time_;

  const bool lock_start_pos_;
  mutable systems::BasicVector<double> current_origin_;
  mutable lcm::DrakeLcm* lcm_;
  mutable PiecewisePolynomial<double> current_trajectory_;
  mutable std::vector<double> breaks_; // The "phase" or "time" along the polynomial.
  mutable std::vector<MatrixX<double>> knots_; // Points we want to go through.

};

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake}
