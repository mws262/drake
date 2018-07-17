#include "drake/examples/iiwa_soccer/spline_trajectory.h"
#include "drake/multibody/rigid_body_plant/frame_visualizer.h"

namespace drake {
namespace examples {
namespace iiwa_soccer {

using systems::BasicVector;
using systems::Context;
using trajectories::PiecewisePolynomial;
using systems::PublishEvent;
using Eigen::Vector3d;
using trajectories::Trajectory;

SplineTrajectories::SplineTrajectories() :
    traj_out_port_idx_(this->DeclareAbstractOutputPort(&SplineTrajectories::CalcTrajectory).get_index()),
    waypoint_in_port_idx_(this->DeclareVectorInputPort(BasicVector<double>(3)).get_index()),
    target_in_port_idx_(this->DeclareVectorInputPort(BasicVector<double>(3)).get_index()) {}

SplineTrajectories::SplineTrajectories(lcm::DrakeLcm& lcm) : SplineTrajectories() {
  debug_draw = true;
  lcm_ = &lcm;
}


void SplineTrajectories::set_start_pt(const Vector3d& new_start_pt,
                                      const double& beginning_time,
                                      const double& time_to_waypt,
                                      const double& time_to_destination) {
  start_pt_ << new_start_pt;
  start_time_ = beginning_time;
  waypoint_time_ = beginning_time + time_to_waypt;
  end_time_ = beginning_time + time_to_destination;
}

void SplineTrajectories::CalcTrajectory(const Context<double>& context, PiecewisePolynomial<double>* output) const {
  if (start_time_ < 0) { // Return an empty trajectory if set_start_pt has not yet been called.
    current_trajectory_ = PiecewisePolynomial<double>();
  }else {
    const systems::BasicVector<double> *waypt = this->EvalVectorInput(context, 0); // Control point along spline.
    const systems::BasicVector<double> *destination = this->EvalVectorInput(context, 1); // Spline terminal point.

    std::vector<double> breaks; // The "phase" or "time" along the polynomial.
    std::vector<MatrixX<double>> knots; // Points we want to go through.

    breaks.push_back(start_time_);
    breaks.push_back(waypoint_time_);
    breaks.push_back(end_time_);

    knots.push_back(start_pt_);
    knots.push_back(waypt->CopyToVector());
    knots.push_back(destination->CopyToVector());

    breaks_ = breaks;
    knots_ = knots;
    current_trajectory_ = PiecewisePolynomial<double>::Cubic(breaks_, knots_);
  }
  *output = current_trajectory_;
}


void SplineTrajectories::DoPublish(const Context<double>& context, const std::vector<const PublishEvent<double>*>& events) const {
  if (debug_draw && (0.05 - std::fmod(context.get_time(), 0.05)) < 0.001) { // Only update this 20 hz to prevent terrible vis lag.
    if (!current_trajectory_.empty() &&
        current_trajectory_.start_time() < context.get_time() &&
        current_trajectory_.end_time() > context.get_time() &&
        current_trajectory_.get_number_of_segments() > 0) {

      std::vector<Eigen::Isometry3d> poses_to_draw;
      std::vector<std::string> pose_names;

      for (double i = current_trajectory_.start_time(); i < current_trajectory_.end_time(); i += (current_trajectory_.end_time() - current_trajectory_.start_time())/20) {
        auto val = current_trajectory_.value(i);

        Eigen::Vector3d vec = {val.col(0)[0], val.col(0)[1], val.col(0)[2]};
        Eigen::Isometry3d traj_pos;
        traj_pos.setIdentity();
        traj_pos.translate(vec);
        poses_to_draw.push_back(traj_pos);
        pose_names.push_back(std::to_string(i));
      }

      PublishFrames(poses_to_draw, pose_names);
    }
  }
}


void SplineTrajectories::PublishFrames(std::vector<Eigen::Isometry3d> poses, std::vector<std::string> name) const {
  drake::lcmt_viewer_draw frame_msg{};
  frame_msg.timestamp = 0;
  int32_t vsize = poses.size();
  frame_msg.num_links = vsize;
  frame_msg.link_name.resize(vsize);
  frame_msg.robot_num.resize(vsize, 0);

  for (size_t i = 0; i < poses.size(); i++) {
    Eigen::Isometry3f pose = poses[i].cast<float>();
    // Create a frame publisher
    Eigen::Vector3f goal_pos = pose.translation();
    Eigen::Quaternion<float> goal_quat =
        Eigen::Quaternion<float>(pose.linear());
    frame_msg.link_name[i] = name[i];
    frame_msg.position.push_back({goal_pos(0), goal_pos(1), goal_pos(2)});
    frame_msg.quaternion.push_back(
        {goal_quat.w(), goal_quat.x(), goal_quat.y(), goal_quat.z()});
  }

  const int num_bytes = frame_msg.getEncodedSize();
  const size_t size_bytes = static_cast<size_t>(num_bytes);
  std::vector<uint8_t> bytes(size_bytes);
  frame_msg.encode(bytes.data(), 0, num_bytes);
  lcm_->Publish("DRAKE_DRAW_FRAMES", bytes.data(), num_bytes, {});
}

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake}