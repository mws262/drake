/// @file
///
/// Kuka iiwa plays soccer. Wow!
///

#include <memory>
#include <vector>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/common/text_logging.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_lcm.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/manipulation/util/sim_diagram_builder.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/frame_visualizer.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/controllers/inverse_dynamics.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/matrix_gain.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/primitives/saturation.h"
#include "drake/examples/iiwa_soccer/spline_trajectory.h"
#include "drake/examples/iiwa_soccer/trajectory_evaluator_vector.h"
#include "drake/systems/primitives/pass_through.h"
#include "drake/examples/iiwa_soccer/target_shifter.h"
#include "drake/systems/primitives/zero_order_hold.h"
#include "drake/examples/iiwa_soccer/impedance_controller.h"
#include "drake/examples/iiwa_soccer/time_based_source_switcher.h"
#include "drake/manipulation/util/world_sim_tree_builder.h"


DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(), "Number of seconds to simulate.");
DEFINE_double(target_realtime_rate, 1.0, "Playback speed x realtime");

DEFINE_double(ball_x, 0.3, "Ball position x");
DEFINE_double(ball_y, 0.3, "Ball position y");
DEFINE_double(ball_z, 0.2, "Ball position z");

DEFINE_double(kp, 60, "Cartesian kp for impedance control. Gets used for all xyz directions.");
DEFINE_double(kd, 30, "Cartesian kp for impedance control. Gets used for all xyz directions.");

namespace drake {
namespace examples {
namespace iiwa_soccer {
namespace {

using kuka_iiwa_arm::kIiwaArmNumJoints;
using kuka_iiwa_arm::kIiwaLcmStatusPeriod;
using kuka_iiwa_arm::IiwaStatusSender;
using kuka_iiwa_arm::IiwaCommandReceiver;
using manipulation::util::SimDiagramBuilder;
using parsers::urdf::AddModelInstanceFromUrdfFileToWorld;
using systems::ConstantVectorSource;
using systems::Context;
using systems::Diagram;
using systems::DiagramBuilder;
using systems::FrameVisualizer;
using systems::RigidBodyPlant;
using systems::Simulator;
using systems::controllers::InverseDynamics;
using trajectories::PiecewisePolynomial;
using manipulation::util::WorldSimTreeBuilder;
using manipulation::util::ModelInstanceInfo;



const char* armModelPath = "drake/examples/iiwa_soccer/models/iiwa14_spheres_collision.urdf";
const char* ballModelPath = "drake/examples/iiwa_soccer/models/soccer_ball.urdf";

const Eigen::Vector3d robot_base_location(0, 0, 0);

/**
 * Send a frame to be drawn over LCM.
 *
 * @param poses
 * @param name
 * @param dlcm
 */
static void PublishFrames(std::vector<Eigen::Isometry3d> poses, std::vector<std::string> name, drake::lcm::DrakeLcm& dlcm) {
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
    frame_msg.quaternion.push_back({goal_quat.w(), goal_quat.x(), goal_quat.y(), goal_quat.z()});
  }

  const int num_bytes = frame_msg.getEncodedSize();
  const size_t size_bytes = static_cast<size_t>(num_bytes);
  std::vector<uint8_t> bytes(size_bytes);
  frame_msg.encode(bytes.data(), 0, num_bytes);
  dlcm.Publish("DRAKE_DRAW_FRAMES", bytes.data(), num_bytes, {});

  drake::log()->info("Completed Frame Publishing");
}



int DoMainControlled() {
  /******** Setup the world. ********/
  drake::lcm::DrakeLcm lcm; // Communications to robot or visualizer.

  SimDiagramBuilder<double> builder; // Wraps normal DiagramBuilder. Has additional utility stuff for controlled systems.

  auto tree = std::make_unique<RigidBodyTree<double>>();

  // Import arm to tree.
  auto kuka_id_table = AddModelInstanceFromUrdfFileToWorld(FindResourceOrThrow(armModelPath),
                                                           multibody::joints::kFixed,
                                                           tree.get());

  // Import ball to tree.
  auto ball_id_table = AddModelInstanceFromUrdfFileToWorld(FindResourceOrThrow(ballModelPath),
                                                     multibody::joints::kQuaternion,
                                                     tree.get());

  // Make ground part of tree.
  multibody::AddFlatTerrainToWorld(tree.get(), 100., 10.);

  auto ball_id = ball_id_table.at("soccer_ball");
  auto kuka_id = kuka_id_table.at("iiwa14");

  RigidBodyPlant<double>* plant_ptr = builder.AddPlant(std::move(tree));


  /******** Add systems. ********/

  // Creates and adds LCM publisher for visualization.
  auto visualizer = builder.AddVisualizer(&lcm);//<systems::DrakeVisualizer>(plant_ptr->get_rigid_body_tree(), &lcm);
  visualizer->set_publish_period(kIiwaLcmStatusPeriod);

  systems::DiagramBuilder<double>* base_builder = builder.get_mutable_builder(); // Get the basic DiagramBuilder inside the SimDiagramBuilder.

  
  // Instance of the tree with ONLY the arm. Makes picking relevant info easier.
  RigidBodyTree<double> tree_for_control;
  AddModelInstanceFromUrdfFileToWorld(FindResourceOrThrow(armModelPath),
                                      multibody::joints::kFixed,
                                      &tree_for_control);

  // Gains in cartesian-land.
  Vector3<double> k_p;
  k_p << FLAGS_kp, FLAGS_kp, FLAGS_kp;
  Vector3<double> k_d;
  k_d << FLAGS_kd, FLAGS_kd, FLAGS_kd;

  // Target tracking controller.
  auto imped_controller = base_builder->AddSystem<CompliantController>(tree_for_control,
                                                                       *tree_for_control.findFrame("iiwa_link_ee_kuka").get(),
                                                                       k_p,
                                                                       k_d,
                                                                       lcm); // Add an CompliantController. Give it the tree to help with kinematics.
  // Gravity compensation controller.
  auto inverse_dyn = base_builder->AddSystem<InverseDynamics<double>>(tree_for_control, true);

  // Add torques from various controllers.
  auto adder = base_builder->AddSystem<systems::Adder<double>>(2, 7);

  // Source switcher for end effector target.
  auto source_switcher = base_builder->AddSystem<TimeBasedSourceSwitcher<double>>(3);

  // Spline trajectory
  double reach_ball_duration = 5;
  auto spline_maker = base_builder->AddSystem<SplineTrajectories>(0, reach_ball_duration, lcm);
  auto spline_evaluator = base_builder->AddSystem<TrajectoryEvaluatorVector>();
  spline_maker->debug_draw = true;

//  // Spline trajectory
//  auto spline_maker2 = base_builder->AddSystem<SplineTrajectories>(10, 20, lcm);
//  auto spline_evaluator2 = base_builder->AddSystem<TrajectoryEvaluator>();

  // Ball position extractor.
  Eigen::MatrixXd ball_filt_mat(3,13); // Cuts out all the states which are not the ball's. This seems... elaborate.
  ball_filt_mat.setZero();
  ball_filt_mat.col(0)[0] = 1;
  ball_filt_mat.col(1)[1] = 1;
  ball_filt_mat.col(2)[2] = 1;
  auto matrix_ball_state_filter = base_builder->AddSystem<systems::MatrixGain>(ball_filt_mat);

  // Actuator saturation:
  double lim = 100;
  VectorX<double> min_lim(7);
  min_lim << -lim, -lim, -lim, -lim, -lim, -lim, -lim;
  VectorX<double> max_lim(7);
  max_lim << lim, lim, lim, lim, lim, lim, lim;
  auto saturation = base_builder->AddSystem<systems::Saturation>(min_lim, max_lim);

  // Constant source 2.
  auto src_vals2 = Eigen::Vector3d();
  src_vals2 << 1, 1, 1;
  auto const_src2 = base_builder->AddSystem<systems::ConstantVectorSource<double>> (src_vals2);
  const_src2->set_name("constant_source2");

  // Constant source 1.
  auto src_vals1 = Eigen::Vector3d();
  src_vals1 << 0.0, 0.0, 1;
  //double src_dur1 = 20;
  auto const_src1 = base_builder->AddSystem<systems::ConstantVectorSource<double>> (src_vals1);
  const_src1->set_name("constant_source1");

  // Target shifter 1.
  auto target_shifter1 = base_builder->AddSystem<TargetShifter>();
  // Ball offset 1.
  auto ball_offset1_val = Eigen::Vector3d();
  ball_offset1_val << 0.1, 0, 0;
  auto ball_offset1 = base_builder->AddSystem<systems::ConstantVectorSource<double>> (ball_offset1_val);
  ball_offset1->set_name("ball_offset1");


  /******** FULL IIWA INFO TO LCM **********/

  auto iiwa_status_pub = base_builder->AddSystem(systems::lcm::LcmPublisherSystem::Make<lcmt_iiwa_status>("IIWA_STATUS", &lcm));
  iiwa_status_pub->set_name("iiwa_status_publisher");
  iiwa_status_pub->set_publish_period(kIiwaLcmStatusPeriod);
  auto iiwa_status_sender = base_builder->AddSystem<IiwaStatusSender>(7);
  iiwa_status_sender->set_name("iiwa_status_sender");
  auto iiwa_command_receiver = base_builder->AddSystem<IiwaCommandReceiver>(7);
  iiwa_command_receiver->set_name("iwwa_command_receiver");
  auto iiwa_command_sub = base_builder->AddSystem(systems::lcm::LcmSubscriberSystem::Make<lcmt_iiwa_command>("IIWA_COMMAND", &lcm));
  iiwa_command_sub->set_name("iiwa_command_subscriber");

  base_builder->Connect(iiwa_command_sub->get_output_port(), iiwa_command_receiver->get_input_port(0));
  base_builder->Connect(plant_ptr->model_instance_state_output_port(kuka_id), iiwa_status_sender->get_state_input_port());
  base_builder->Connect(iiwa_command_receiver->get_output_port(0), iiwa_status_sender->get_command_input_port());
  base_builder->Connect(adder->get_output_port(), iiwa_status_sender->get_commanded_torque_input_port());
  base_builder->Connect(iiwa_status_sender->get_output_port(0), iiwa_status_pub->get_input_port());


  /********* FEEDBACK FROM SYSTEM **********/

  // Big state -> just ball xyz
  base_builder->Connect(plant_ptr->model_instance_state_output_port(ball_id), matrix_ball_state_filter->get_input_port());
  // Arm state -> Impedance contrller
  base_builder->Connect(plant_ptr->model_instance_state_output_port(kuka_id), imped_controller->get_input_port_estimated_state());
  // Arm state -> Gravity compensation
  base_builder->Connect(plant_ptr->model_instance_state_output_port(kuka_id), inverse_dyn->get_input_port_estimated_state());



  /******** LOW-LEVEL CONTROLLERS **********/

  // Target tracking controller torque output -> Adder
  base_builder->Connect(imped_controller->get_output_port_control(), adder->get_input_port(1));
  // Gravity compensation controller -> Adder
  base_builder->Connect(inverse_dyn->get_output_port_torque(), adder->get_input_port(0));
  // Total torque sum to saturation input
  base_builder->Connect(adder->get_output_port(), saturation->get_input_port());
  // Saturation output -> robot actuators
  base_builder->Connect(saturation->get_output_port(), plant_ptr->get_input_port(0));


  /******** SPLINE TRAJECTORY -> SOURCE SWITCHER **********/

  // Start position to spline maker.
  base_builder->Connect(const_src1->get_output_port(), spline_maker->get_input_port(0));
  // Offset value to the coordinate offsetter.
  base_builder->Connect(ball_offset1->get_output_port(), target_shifter1->get_input_port(0));

  // Add offset to ball position.
  auto offset_adder = base_builder->AddSystem<systems::Adder<double>>(2, 3);
  base_builder->Connect(ball_offset1->get_output_port(), offset_adder->get_input_port(0));
  base_builder->Connect(matrix_ball_state_filter->get_output_port(), offset_adder->get_input_port(1));


  // Ball position to spline maker.
  base_builder->Connect(offset_adder->get_output_port(), spline_maker->get_input_port(1));



  // Spline maker to spline evaluator.
  base_builder->Connect(spline_maker->get_output_port(0), spline_evaluator->get_input_port(0));


  /******** TRACKING COMMANDS TO SOURCE SWITCHER *********/

  // Ball position to tracker
  // double ball_track_duration = 10;
  // source_switcher->add_output_with_duration(matrix_ball_state_filter->get_output_port(), ball_track_duration, *builder);

  // Time-varying splinetrajectory to switcher.
  source_switcher->add_output_with_duration(spline_evaluator->get_output_port(0), reach_ball_duration, *base_builder);

  // Constant target to switcher.
//  double src_dur2 = 5;
//  source_switcher->add_output_with_duration(const_src2->get_output_port(), src_dur2, *builder);

  // Switcher's choice to tracking controller
  base_builder->Connect(source_switcher->get_output_port(0), imped_controller->get_input_port_cartesian_target());


  auto diagram = builder.Build();

  // to draw frames ./bazel-bin/tools/drake_visualizer --script ./multibody/rigid_body_plant/visualization/show_frames.py

  ///////////////////////////
  std::vector<Eigen::Isometry3d> poses_to_draw;
  Eigen::Isometry3d target_pose;
  target_pose.setIdentity();
  Eigen::Vector3d tarPos;

  std::vector<std::string> pose_names;
  pose_names.push_back("ee_target");

  PublishFrames(poses_to_draw, pose_names, lcm);

  ////////////////////////////////

  Simulator<double> simulator(*diagram);
  simulator.set_publish_at_initialization(false);
  simulator.set_publish_every_time_step(true);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);

  Context<double>& context = simulator.get_mutable_context();
  plant_ptr->set_position(&context, 7, FLAGS_ball_x); // Moving the bowling ball away from the origin.
  plant_ptr->set_position(&context, 8, FLAGS_ball_y);
  plant_ptr->set_position(&context, 9, FLAGS_ball_z);

  simulator.reset_integrator<systems::RungeKutta2Integrator<double>>(*diagram, 0.001, &context);
  simulator.Initialize();
  simulator.StepTo(FLAGS_simulation_sec);

  return 0;
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  return drake::examples::iiwa_soccer::DoMainControlled();
}
