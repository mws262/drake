#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/framework/event.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/common/eigen_types.h"


namespace drake {
namespace examples {
namespace iiwa_soccer {

using Eigen::Vector3d;
using systems::Context;
using systems::InputPortDescriptor;
using systems::LeafSystem;
using systems::OutputPort;
using systems::PublishEvent;
using trajectories::PiecewisePolynomial;
using trajectories::Trajectory;

class SplineTrajectories : public LeafSystem<double> {
 public:
  bool debug_draw = false;

  /** Construct without vis. **/
  SplineTrajectories();

  /** Construct with vis. **/
  SplineTrajectories(lcm::DrakeLcm& lcm);

  /**
   * Specify that spline generation should be based on a new set of parameters.
   * @param new_start_pt: Beginning point of spline. Remains constant until set again by this method.
   * @param beginning_time: Time at the beginning of the spline.
   * @param time_to_waypt: How long after the beginning should we be at the waypoint?
   * @param time_to_destination: How long after the beginning should we be at the destination.
   */
  void set_start_pt(const Vector3d& new_start_pt,
                    const double& beginning_time,
                    const double& time_to_waypt,
                    const double& time_to_destination);

  const InputPortDescriptor<double>& get_input_port_waypoint() {
    return this->get_input_port(waypoint_in_port_idx_);
  }

  const InputPortDescriptor<double>& get_input_port_target() {
    return this->get_input_port(target_in_port_idx_);
  }

  const OutputPort<double>& get_output_port_trajectory() {
    return this->get_output_port(traj_out_port_idx_);
  }

 private:
  const int traj_out_port_idx_;
  const int waypoint_in_port_idx_;
  const int target_in_port_idx_;

  double start_time_ = -1;
  double waypoint_time_ = -1;
  double end_time_ = -1;

  mutable lcm::DrakeLcm* lcm_; // For drawing spline trajectories using frames, if enabled.
  mutable PiecewisePolynomial<double> current_trajectory_;
  mutable std::vector<double> breaks_; // The "phase" or "time" along the polynomial.
  mutable std::vector<MatrixX<double>> knots_; // Points we want to go through.
  mutable Vector3d start_pt_; // A safe trajectory starting point until a new one is assigned.

  void CalcTrajectory(const Context<double>& context, PiecewisePolynomial<double>* const output) const;
  void PublishFrames(std::vector<Eigen::Isometry3d> poses, std::vector<std::string> name) const;
  void DoPublish(const Context<double>& context, const std::vector<const PublishEvent<double>*>& events) const;

};

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake}
