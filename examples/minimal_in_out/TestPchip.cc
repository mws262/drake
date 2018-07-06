
#include <vector>
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/eigen_types.h"
#include "drake/common/eigen_stl_types.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/multibody/rigid_body_plant/frame_visualizer.h"
#include "drake/lcmt_viewer_geometry_data.hpp"


namespace drake {
namespace examples {
namespace minimal_in_out {


using trajectories::PiecewisePolynomial;
using Eigen::MatrixXd;
using Eigen::VectorXd;


void PublishFrames(std::vector<Eigen::Isometry3d> poses, std::vector<std::string> name, drake::lcm::DrakeLcm& dlcm) {
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
  dlcm.Publish("DRAKE_DRAW_FRAMES", bytes.data(), num_bytes, {});

  drake::log()->info("Completed Frame Publishing");

//  drake::lcmt_viewer_geometry_data shape;
//  shape.type = shape.SPHERE;
//  shape.position[0] = 1;//static_cast<float>(poses.back().translation()[0]);
//  shape.position[1] = 1;//static_cast<float>(poses.back().translation()[1]);
//  shape.position[2] = 1;//static_cast<float>(poses.back().translation()[2]);
//  shape.string_data = "test_shape";
//  shape.quaternion[0] = 1;
//  shape.quaternion[1] = 1;
//  shape.quaternion[2] = 1;
//  shape.quaternion[3] = 1;
//  shape.color[0] = 1;
//  shape.color[1] = 1;
//  shape.color[2] = 1;
//  shape.color[3] = 1;
//  shape.num_float_data = 0;
//
//  const int num_bytes2 = shape.getEncodedSize();
//  const size_t size_bytes2 = static_cast<size_t>(num_bytes2);
//  std::vector<uint8_t> bytes2(size_bytes2);
//  shape.encode(bytes2.data(), 0, num_bytes2);
//  dlcm.Publish("DRAKE_DRAW_SHAPE", bytes2.data(), num_bytes2, {});
}

int Run() {

  std::vector<double> breaks; // The "phase" or "time" along the polynomial.
  std::vector<MatrixX<double>> knots; // Points we want to go through.


  breaks.push_back(-5);
  breaks.push_back(0);
  breaks.push_back(5);

  MatrixX<double> pt1 = MatrixX<double>::Zero(3,1);
  pt1(0,0) = 0; pt1(1,0) = 0; pt1(2,0) = 0;
  knots.push_back(pt1);

  MatrixX<double> pt2 = MatrixX<double>::Zero(3,1);
  pt2(0,0) = 1; pt2(1,0) = 2; pt2(2,0) = 5;
  knots.push_back(pt2);

  MatrixX<double> pt3 = MatrixX<double>::Zero(3,1);
  pt3(0,0) = 1; pt3(1,0) = 2; pt3(2,0) = 1;
  knots.push_back(pt3);

//  MatrixX<double> pt4 = MatrixX<double>::Zero(3,1);
//  pt4(1,0) = 1; pt4(1,0) = 2; pt4(2,0) = 1;
//  knots.push_back(pt4);

  auto interp_poly = PiecewisePolynomial<double>::Cubic(breaks, knots);


  std::vector<Eigen::Isometry3d> poses_to_draw;
  std::vector<std::string> pose_names;

  for (double i = -5; i < 5; i+= 0.1){
    auto val = interp_poly.value(i);

    Eigen::Vector3d vec = {val.col(0)[0], val.col(0)[1], val.col(0)[2]};
    Eigen::Isometry3d traj_pos;
    traj_pos.setIdentity();
    traj_pos.translate(vec);
    poses_to_draw.push_back(traj_pos);
    pose_names.push_back(std::to_string(i));
  }

  drake::lcm::DrakeLcm lcm; // Communications to robot or visualizer.

  PublishFrames(poses_to_draw, pose_names, lcm);

  return 0;
}


} // namespace minimal_in_out
} // namespace examples
} // namespace drake


int main(int argc, char **argv) {
  return drake::examples::minimal_in_out::Run();
}

