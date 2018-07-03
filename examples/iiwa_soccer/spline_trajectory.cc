#include "drake/examples/iiwa_soccer/spline_trajectory.h"
#include "drake/multibody/rigid_body_plant/frame_visualizer.h"


namespace drake {
namespace examples {
namespace iiwa_soccer {

using systems::BasicVector;
using systems::Context;
using trajectories::PiecewisePolynomial;
using systems::PublishEvent;


SplineTrajectories::SplineTrajectories(double start_time, double duration, lcm::DrakeLcm& lcm, bool lock_start_pos) : start_time_(start_time),
                                                                                                                      end_time_(start_time + duration),
                                                                                                                      lock_start_pos_(lock_start_pos),
                                                                                                                      current_origin_(3) {
  this->DeclareVectorInputPort(BasicVector<double>(3)); // Origin.
  this->DeclareVectorInputPort(BasicVector<double>(3)); // Destination.

  this->DeclareAbstractOutputPort(&SplineTrajectories::CalcTrajectory);
  lcm_ = &lcm;
  current_origin_.SetZero();

}

void SplineTrajectories::CalcTrajectory(const Context<double>& context, PiecewisePolynomial<double>* output) const {
  if (!lock_start_pos_ || (current_origin_[0] == 0 &&
  current_origin_[1] == 0 && current_origin_[2] == 0) ) {
    current_origin_.SetFromVector(this->EvalVectorInput(context, 0)->CopyToVector());
  }

  const systems::BasicVector<double>* destination = this->EvalVectorInput(context, 1);


  std::vector<double> breaks; // The "phase" or "time" along the polynomial.
  std::vector<MatrixX<double>> knots; // Points we want to go through.

  breaks.push_back(start_time_);
  breaks.push_back((start_time_ + end_time_)/2);//context.get_time() + 15);
  breaks.push_back(end_time_);//context.get_time() + 30);

  MatrixX<double> pt1 = MatrixX<double>::Zero(3,1);
  pt1(0,0) = current_origin_.GetAtIndex(0);
  pt1(1,0) = current_origin_.GetAtIndex(1);
  pt1(2,0) = current_origin_.GetAtIndex(2);
  knots.push_back(pt1);

  MatrixX<double> pt2 = MatrixX<double>::Zero(3,1);
  pt2(0,0) = (destination->GetAtIndex(0) - current_origin_.GetAtIndex(0))*2/3; // Go 2/3 the distance in the first half the time.
  pt2(1,0) = (destination->GetAtIndex(1) - current_origin_.GetAtIndex(1))*2/3;
  pt2(2,0) = std::max(current_origin_.GetAtIndex(2) + (destination->GetAtIndex(2) - current_origin_.GetAtIndex(2))*1/3 + 0.2, 0.2);
  knots.push_back(pt2);

  MatrixX<double> pt3 = MatrixX<double>::Zero(3,1);
  pt3(0,0) = destination->GetAtIndex(0);
  pt3(1,0) = destination->GetAtIndex(1);
  pt3(2,0) = destination->GetAtIndex(2);
  knots.push_back(pt3);

  breaks_ = breaks;
  knots_ = knots;
  current_trajectory_= PiecewisePolynomial<double>::Cubic(breaks_, knots_);
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