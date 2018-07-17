#include <gflags/gflags.h>
#include "drake/systems/framework/diagram.h"
#include "drake/examples/iiwa_soccer/source_switcher.h"
#include "drake/systems/primitives/pass_through.h"
#include "drake/systems/framework/input_port_descriptor.h"
#include "drake/systems/primitives/adder.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/common/find_resource.h"
#include "drake/examples/iiwa_soccer/compliant_controller.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/controllers/inverse_dynamics.h"
#include "drake/manipulation/util/sim_diagram_builder.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/matrix_gain.h"
#include "drake/examples/iiwa_soccer/spline_trajectory.h"
#include "drake/examples/iiwa_soccer/trajectory_evaluator.h"
#include "drake/examples/iiwa_soccer/soccer_common.h"
#include "drake/examples/iiwa_soccer/fsm_system.h"
#include "drake/examples/iiwa_soccer/target_shifter.h"
#include "drake/examples/iiwa_soccer/nonconstant_vector_source.h"
#include "drake/examples/iiwa_soccer/nonconstant_abstract_source.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/systems/primitives/multiplexer.h"

namespace drake {
namespace examples {
namespace iiwa_soccer {

using systems::Diagram;
using systems::DiagramBuilder;
using systems::PassThrough;
using systems::System;
using systems::InputPortDescriptor;
using systems::Adder;
using parsers::urdf::AddModelInstanceFromUrdfFileToWorld;
using systems::controllers::InverseDynamicsController;
using systems::controllers::InverseDynamics;
using manipulation::util::SimDiagramBuilder;
using systems::RigidBodyPlant;
using systems::Simulator;
using systems::ConstantVectorSource;
using systems::DrakeVisualizer;
using systems::MatrixGain;
using systems::Multiplexer;

DEFINE_double(direction_multiply_target, 0, "");
DEFINE_double(direction_multiply_waypoint, 10, "");
DEFINE_double(direction_target_min, 0.05, "");
DEFINE_double(direction_waypoint_min, 0.01,"");
DEFINE_double(waypoint_time, 1, "");
DEFINE_double(target_time, 4, "");
DEFINE_double(waypoint_offset_height, 0.15, "");
DEFINE_double(target_offset_height, 0.09, "");

DEFINE_double(Kp, 100, "Proportional gain.");
DEFINE_double(Kd, 20, "Derivative gain.");
DEFINE_double(Ki, 0, "Integral gain.");

// Set of systems to be passed to the FSM for turning controllers on/off.
SystemSwitches control_switches;

const char* armModelPath = "drake/examples/iiwa_soccer/models/iiwa14_spheres_collision.urdf";
const char* ballModelPath = "drake/examples/iiwa_soccer/models/soccer_ball.urdf";

RigidBodyTree<double> tree_for_control;
drake::lcm::DrakeLcm lcm;

/**
 * Make a diagram containing controllers and switching systems between them.
 * Any necessary switching interfaces will be in control_switches. Diagram
 * will take in states and put out torques.
 *
 * @return
 */
std::unique_ptr<Diagram<double>> make_junction_box() {

  DiagramBuilder<double> builder;

  /**** State comes into diagram. ****/
  const PassThrough<double>* arm_state_in = builder.AddSystem<PassThrough<double>>(arm_num_states);
  builder.ExportInput(arm_state_in->get_input_port());
  const PassThrough<double>* ball_state_in = builder.AddSystem<PassThrough<double>>(ball_num_states);
  builder.ExportInput(ball_state_in->get_input_port());

  // Ball position extractor.
  Eigen::MatrixXd ball_filt_mat(3, ball_num_states); // Cuts out all the states which are not the ball's position.
  ball_filt_mat.setZero();
  ball_filt_mat.col(0)[0] = 1;
  ball_filt_mat.col(1)[1] = 1;
  ball_filt_mat.col(2)[2] = 1;

  const MatrixGain<double>* ball_position = builder.AddSystem<MatrixGain>(ball_filt_mat);
  builder.Connect(ball_state_in->get_output_port(), ball_position->get_input_port());

  Eigen::MatrixXd ball_filt_pos_vel_mat(6, ball_num_states); // Cuts out all the states which are not the ball's position.
  ball_filt_pos_vel_mat.setZero();
  ball_filt_pos_vel_mat.col(0)[0] = 1; // Position
  ball_filt_pos_vel_mat.col(1)[1] = 1;
  ball_filt_pos_vel_mat.col(2)[2] = 1;

  ball_filt_pos_vel_mat.col(10)[3] = 1; // Velocity
  ball_filt_pos_vel_mat.col(11)[4] = 1;
  ball_filt_pos_vel_mat.col(12)[5] = 1;

  const MatrixGain<double>* ball_position_velocity = builder.AddSystem<MatrixGain>(ball_filt_pos_vel_mat);
  builder.Connect(ball_state_in->get_output_port(), ball_position_velocity->get_input_port());



  /**** Switch who sends torque to motors. ****/
  SourceSwitcher<double>* SWITCH_torque = builder.AddSystem<SourceSwitcher<double>>(arm_num_joints);
  const InputPortDescriptor<double>& SWITCH_torque_adder_imped_w_grav = SWITCH_torque->add_selectable_output("imped_w_grav_comp");
  const InputPortDescriptor<double>& SWITCH_torque_imped = SWITCH_torque->add_selectable_output("imped_ctrl_alone");
  const InputPortDescriptor<double>& SWITCH_torque_grav = SWITCH_torque->add_selectable_output("gravity_comp_alone");
  const InputPortDescriptor<double>& SWITCH_torque_invdyn_traj = SWITCH_torque->add_selectable_output("inv_dyn_w_grav_comp");

  const Adder<double>* torque_adder_imped = builder.AddSystem<Adder<double>>(2, arm_num_joints); // Two inputs, compliant controller torque and gravity compensation torque. 7 dimensions, one for each joint.
  const Adder<double>* torque_adder_invdyn = builder.AddSystem<Adder<double>>(2, arm_num_joints);

  SWITCH_torque->switch_output("imped_w_grav_comp");


  // Instance of the tree with ONLY the arm. Used by controllers.
  AddModelInstanceFromUrdfFileToWorld(FindResourceOrThrow(armModelPath), multibody::joints::kFixed, &tree_for_control);

  // Joint gains.
  VectorXd joint_kp(7);
  VectorXd joint_kd(7);
  VectorXd joint_ki(7);
  joint_kp.setConstant(FLAGS_Kp);
  joint_kd.setConstant(FLAGS_Kd);
  joint_ki.setConstant(FLAGS_Ki);


  // Gravity compensation controller.
  const InverseDynamics<double>* grav_comp = builder.AddSystem<InverseDynamics<double>>(tree_for_control, true);

  // Target tracking controller.
  Vector3d k_p;  // Gains in cartesian-land.
  k_p.setConstant(FLAGS_Kp);
  Vector3d k_d;
  k_d.setConstant(FLAGS_Kd);
  const CompliantController* imped_controller = builder.AddSystem<CompliantController>(tree_for_control,
                                                                 *tree_for_control.findFrame("iiwa_link_7").get(),
                                                                 k_p,
                                                                 k_d,
                                                                 lcm);
  // Inverse dynamics controller for full joint trajectory tracking.
  const InverseDynamicsController<double>* inv_dyn_controller = builder.AddSystem<InverseDynamicsController<double>>(
      tree_for_control.Clone(), joint_kp, joint_ki, joint_kd, false);

  // Grav comp torque to switch.
  builder.Connect(grav_comp->get_output_port_torque(), SWITCH_torque_grav);

  // Imped controller torque to switch.
  builder.Connect(imped_controller->get_output_port_control(), SWITCH_torque_imped);

  // Torques to adder, adder to switch.
  builder.Connect(grav_comp->get_output_port_torque(), torque_adder_imped->get_input_port(0));
  builder.Connect(imped_controller->get_output_port_control(), torque_adder_imped->get_input_port(1));
  builder.Connect(torque_adder_imped->get_output_port(), SWITCH_torque_adder_imped_w_grav);

  builder.Connect(arm_state_in->get_output_port(), grav_comp->get_input_port_estimated_state());
  builder.Connect(arm_state_in->get_output_port(), imped_controller->get_input_port_estimated_state());
  builder.Connect(arm_state_in->get_output_port(), inv_dyn_controller->get_input_port_estimated_state());

  builder.Connect(inv_dyn_controller->get_output_port_control(), SWITCH_torque_invdyn_traj);

  // Torque output port to motors.
  builder.ExportOutput(SWITCH_torque->get_output_port(0));


  /**** Target offset switching. ****/
  SourceSwitcher<double>* SWITCH_offset = builder.AddSystem<SourceSwitcher>(3);
  const InputPortDescriptor<double>& SWITCH_offset_zero = SWITCH_offset->add_selectable_output("zero_offset");
  const InputPortDescriptor<double>& SWITCH_offset_ball = SWITCH_offset->add_selectable_output("ball_offset");

  // Constant Vector3d of zeroes.
  const ConstantVectorSource<double>* zeros_src3 = builder.AddSystem<ConstantVectorSource>(Vector3d::Zero());

  // No offset.
  TargetShifter* zero_offset = builder.AddSystem<TargetShifter>();
  builder.Connect(zeros_src3->get_output_port(), zero_offset->get_input_port(0));
  builder.Connect(ball_position->get_output_port(), zero_offset->get_input_port(1));
  builder.Connect(zero_offset->get_output_port(0), SWITCH_offset_zero);


  // Offset from ball.
  NonconstantVectorSource* ball_offset_src = builder.AddSystem<NonconstantVectorSource>(3);

  TargetShifter* ball_offset_shifter = builder.AddSystem<TargetShifter>();
  builder.Connect(ball_offset_src->get_output_port(0), ball_offset_shifter->get_input_port_offset());
  builder.Connect(ball_position->get_output_port(), ball_offset_shifter->get_input_port_object_position());
  builder.Connect(ball_offset_shifter->get_output_port(0), SWITCH_offset_ball);

  Adder<double>* ball_offset_adder = builder.AddSystem<Adder<double>>(2,3);
  builder.Connect(ball_offset_shifter->get_output_port(0), ball_offset_adder->get_input_port(0));
  builder.Connect(ball_position->get_output_port(), ball_offset_adder->get_input_port(1));


  /**** Trajectory generation. ****/
  // Spline trajectory
  SplineTrajectories* spline_maker = builder.AddSystem<SplineTrajectories>(lcm);
  TrajectoryEvaluator* spline_evaluator = builder.AddSystem<TrajectoryEvaluator>(6); // 6 is cartesian position and velocity.

  // Waypoint position to spline maker.

  NonconstantVectorSource* waypoint_offset_src = builder.AddSystem<NonconstantVectorSource>(3);
  TargetShifter* waypoint_offset = builder.AddSystem<TargetShifter>();
  Adder<double>* waypoint_offset_adder = builder.AddSystem<Adder<double>>(2,3);
  builder.Connect(waypoint_offset_src->get_output_port(0), waypoint_offset->get_input_port_offset());
  builder.Connect(ball_position->get_output_port(), waypoint_offset->get_input_port_object_position());
  builder.Connect(waypoint_offset->get_output_port(0), waypoint_offset_adder->get_input_port(0));
  builder.Connect(ball_position->get_output_port(), waypoint_offset_adder->get_input_port(1));

  builder.Connect(waypoint_offset_adder->get_output_port(), spline_maker->get_input_port_waypoint());

  // Ball position to spline maker.
  builder.Connect(ball_offset_adder->get_output_port(), spline_maker->get_input_port_target());

  // Spline maker to spline evaluator.
  builder.Connect(spline_maker->get_output_port_trajectory(), spline_evaluator->get_input_port_trajectory());

  /* For IK */
  NonconstantAbstractSource<PiecewisePolynomial<double>>* ik_traj_receiver =
      builder.AddSystem<NonconstantAbstractSource<PiecewisePolynomial<double>>>();

  TrajectoryEvaluator* ik_traj_evaluator = builder.AddSystem<TrajectoryEvaluator>(arm_num_states);

  // Connect ik trajectory evaluator to the inverse dynamics controller.
  builder.Connect(ik_traj_receiver->get_output_port(0), ik_traj_evaluator->get_input_port(0));
  builder.Connect(ik_traj_evaluator->get_output_port(0), inv_dyn_controller->get_input_port_desired_state());



  /**** Switch who sends setpoints to the impedance controller. ****/
  SourceSwitcher<double>* SWITCH_setpt = builder.AddSystem<SourceSwitcher>(6);
  const InputPortDescriptor<double>& SWITCH_setpt_fixed = SWITCH_setpt->add_selectable_output("fixed_input");
  const InputPortDescriptor<double>& SWITCH_setpt_follow = SWITCH_setpt->add_selectable_output("follow_obj");
  const InputPortDescriptor<double>& SWITCH_setpt_spline = SWITCH_setpt->add_selectable_output("spline");

  SWITCH_setpt->switch_output("fixed_input");

  // Dummy target
  VectorXd offset(6);
  offset << 1, 1, 1, 0, 0, 0;
  const ConstantVectorSource<double>* const_src = builder.AddSystem<ConstantVectorSource<double>>(offset);

  builder.Connect(const_src->get_output_port(), SWITCH_setpt_fixed);
  builder.Connect(ball_position_velocity->get_output_port(), SWITCH_setpt_follow);
  builder.Connect(spline_evaluator->get_output_port(0), SWITCH_setpt_spline);



  // Add targets from the selected offsetter and the selected setpoint setter.
  Adder<double>* offset_adder = builder.AddSystem<Adder<double>>(2, 6);
  builder.Connect(SWITCH_setpt->get_output_port(0), offset_adder->get_input_port(0));

  // Concatenate position and velocity offsets
  std::vector<int> multiplex_input_sizes;
  multiplex_input_sizes.push_back(3);
  multiplex_input_sizes.push_back(3);

  const Multiplexer<double>* multiplex_offset = builder.AddSystem<Multiplexer<double>>(multiplex_input_sizes);
  builder.Connect(SWITCH_offset->get_output_port(0), multiplex_offset->get_input_port(0));
  builder.Connect(zeros_src3->get_output_port(), multiplex_offset->get_input_port(1)); // Zero offset for velocity for now.

  builder.Connect(multiplex_offset->get_output_port(0), offset_adder->get_input_port(1)); // Add the selected offset to the impedance controller target.


  builder.Connect(offset_adder->get_output_port(), imped_controller->get_input_port_cartesian_target());

  control_switches = {SWITCH_torque, SWITCH_setpt, SWITCH_offset,
                      ball_offset_src, waypoint_offset_src, spline_maker, ik_traj_receiver, &tree_for_control};


  // Don't want this for all.
  waypoint_offset->use_cartesian = true;
  ball_offset_shifter->use_cartesian = true;


  return builder.Build();
}


void DoMain() {
  direction_multiply_target = FLAGS_direction_multiply_target;
  direction_multiply_waypoint = FLAGS_direction_multiply_waypoint;
  waypoint_time = FLAGS_waypoint_time;
  target_time = FLAGS_target_time;
  target_offset_height = FLAGS_target_offset_height;
  waypoint_offset_height = FLAGS_waypoint_offset_height;
  direction_target_min = FLAGS_direction_target_min;
  direction_waypoint_min = FLAGS_direction_target_min;

  /******** Setup the world. ********/
  SimDiagramBuilder<double> builder; // Wraps normal DiagramBuilder. Has additional utility stuff for controlled systems.

  auto tree = std::make_unique<RigidBodyTree<double>>();

  // Import arm to tree.
  parsers::ModelInstanceIdTable kuka_id_table = AddModelInstanceFromUrdfFileToWorld(FindResourceOrThrow(armModelPath),
                                                                                    multibody::joints::kFixed,
                                                                                    tree.get());
  // Import ball to tree.
  parsers::ModelInstanceIdTable ball_id_table = AddModelInstanceFromUrdfFileToWorld(FindResourceOrThrow(ballModelPath),
                                                                                    multibody::joints::kQuaternion,
                                                                                    tree.get());
  // Make ground part of tree.
  multibody::AddFlatTerrainToWorld(tree.get(), 100., 10.);

  const int ball_id = ball_id_table.at("soccer_ball");
  const int kuka_id = kuka_id_table.at("iiwa14");

  RigidBodyPlant<double>* plant_ptr = builder.AddPlant(std::move(tree));

  // Map collision id numbers to bodies.
  std::map<unsigned long, std::string> collision_to_body;

  for (int i = 0; i < plant_ptr->get_num_bodies(); i++ ){
    auto col_ids = plant_ptr->get_rigid_body_tree().get_body(i).get_collision_element_ids();
    for (unsigned long j = 0; j < col_ids.size(); j++) {
      collision_to_body.insert(std::make_pair(col_ids[j], plant_ptr->get_rigid_body_tree().get_body(i).get_name()));
    }
  }

  // Creates and adds LCM publisher for visualization.
  DrakeVisualizer* visualizer = builder.AddVisualizer(&lcm);
  visualizer->set_publish_period(0.005);

  DiagramBuilder<double>* base_builder = builder.get_mutable_builder();

  std::unique_ptr<Diagram<double>> switch_box_diagram = make_junction_box();

  Diagram<double>* switch_box = base_builder->AddSystem(std::move(switch_box_diagram));

  // +3 magic number is necessary due to the port ordering in RigidBodyPlant.
  base_builder->Connect(plant_ptr->get_output_port(kuka_id + 3), switch_box->get_input_port(0));
  base_builder->Connect(plant_ptr->get_output_port(ball_id + 3), switch_box->get_input_port(1));
  base_builder->Connect(switch_box->get_output_port(0), plant_ptr->get_input_port(0));


  FSM_System* fsm = base_builder->AddSystem<FSM_System>(control_switches, collision_to_body);
  base_builder->Connect(plant_ptr->get_output_port(kuka_id + 3), fsm->get_input_port_arm_state());
  base_builder->Connect(plant_ptr->get_output_port(ball_id + 3), fsm->get_input_port_ball_state());
  base_builder->Connect(plant_ptr->contact_results_output_port(), fsm->get_input_port_contact_results());


  std::unique_ptr<Diagram<double>> diagram = builder.Build();

  Simulator<double> simulator(*diagram);
  simulator.set_publish_at_initialization(false);
  simulator.set_publish_every_time_step(true);
  simulator.set_target_realtime_rate(1.0);

  Context<double>& context = simulator.get_mutable_context();
  plant_ptr->set_position(&context, 7, 0.3); // Moving the bowling ball away from the origin.
  plant_ptr->set_position(&context, 8, 0.3);
  plant_ptr->set_position(&context, 9, 0.11);

  context.get_mutable_discrete_state_vector()[0] = FSM_System::controller_states::START_IK_TRAJ;//START_SPLINE_TO_ABOVE_BALL; // Set controller FSM's initial state.

  simulator.reset_integrator<systems::RungeKutta2Integrator<double>>(*diagram, 0.002, &context);
  simulator.Initialize();
  simulator.StepTo(INT64_MAX);
}


}  // namespace iiwa_soccer
}  // namespace examples
}  // namespace drake}

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  drake::examples::iiwa_soccer::DoMain();
  return 0;
}