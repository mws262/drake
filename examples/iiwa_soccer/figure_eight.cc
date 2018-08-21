#include <gflags/gflags.h>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/trajectory.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/rigid_body_plant/frame_visualizer.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/common/find_resource.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/examples/iiwa_soccer/magic_force.h"
#include "drake/examples/iiwa_soccer/arrows_to_lcm.h"
#include "drake/examples/iiwa_soccer/nonconstant_abstract_source.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/common/text_logging.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/solvers/binding.h"
#include "drake/examples/iiwa_soccer/spline_distance_cost.h"
#include "drake/examples/iiwa_soccer/poly_utilities.h"

namespace drake {
namespace examples {
namespace iiwa_soccer {

using Eigen::VectorXd;
using Eigen::MatrixXd;
using trajectories::PiecewisePolynomial;
using systems::DrakeVisualizer;
using systems::DiagramBuilder;
using systems::Diagram;
using systems::Simulator;
using systems::Value;
using Eigen::Vector3d;
using parsers::urdf::AddModelInstanceFromUrdfFileToWorld;
using systems::RigidBodyPlant;
using systems::Context;
using trajectories::Trajectory;
using drake::systems::lcm::LcmPublisherSystem;
using systems::ConstantVectorSource;
using solvers::Binding;
using solvers::VectorXDecisionVariable;

DEFINE_double(trashvar1, 1, "");
DEFINE_double(trashvar2, 1, "");

DEFINE_double(cutsizethresh, 0.3, "won't cut if size is below this length (in seconds)");
DEFINE_double(min_around_cut, 0.05, "");
DEFINE_double(cutthresh, 1, "");
DEFINE_bool(feedback, true, "use feedback controls?");

const char* ballModelPath = "drake/examples/iiwa_soccer/models/soccer_ball.urdf";
const char* armModelPath = "drake/examples/iiwa_soccer/models/iiwa14_spheres_collision.urdf";


PolyWithKnots MakeTRIPoly() {

  std::vector<double> breaks;
  std::vector<MatrixX<double>> knots;

  double time_scaling = 15;
  int num_pts = 21;

  MatrixXd cursive_mat(num_pts,3);
  cursive_mat << 0,0,0,
      0.6, 1.5, 0.0, // Letter t
      0.5, 2.0, 0.0,
      0.45, 1.4, 0.0,

      1.0, 0.0, 0.0, // Letter r
      1.5, 1.0, 0.0,
      1.8, 0.8, 0.0,
      1.55, 0.5, 0.0,

      2.0, 0.0, 0.0, // Letter i
      2.53, 0.5, 0.0,
      2.5, 1.0, 0.0,
      2.47, 0.5, 0.0,
      3.0, 0.0, 0.0,

      3.5, 0.3, 0, // Off to side
      4, 0.8, 0,

      2.5, 1.5, 0, // Make dot for i
      2.45, 1.45, 0,

      2.5, 1.5, 0,

      1.5, 1.6, 0,
      0.5, 1.5, 0, // Cross the t
      -1, 1.2, 0
      ;


  VectorXd times(num_pts);

  cursive_mat.transposeInPlace();

  for (int i = 0; i < cursive_mat.cols(); i++) {
    knots.push_back(cursive_mat.col(i));
    breaks.push_back(time_scaling*(static_cast<double>(i)/static_cast<double>(num_pts)));
  }

  PiecewisePolynomial<double> original_poly = PiecewisePolynomial<double>::Cubic(breaks, knots, true);//, start_dt, end_dt);

  PolyWithKnots figure8dat = {original_poly, breaks, knots};
  return figure8dat;
}



void CutUntil(PolyWithKnots& original_poly, double thresh) {
  int max_cuts = 12;
  int idx = 0;do{
    SplitPolyInPlace(original_poly, 0, thresh);
    idx++;
  } while(idx < max_cuts);
}

void SetupWorld() {
  double time_scaling = 5;
  double half_height = 2;
  double half_width = 1;

  Vector3d poly_offset(0, 0, 0);
  PolyWithKnots poly_data = MakeFigure8Poly(poly_offset, time_scaling, half_height, half_width);
  MakeTRIPoly();
  CutUntil(poly_data, FLAGS_cutthresh);

  // Initial conditions based on first piecewise polynomial
  std::unique_ptr<Trajectory<double>> path_poly_dt1 = poly_data.poly.MakeDerivative(1);
  std::unique_ptr<Trajectory<double>> path_poly_ddt1 = poly_data.poly.MakeDerivative(2);

  double x0 = poly_data.poly.value(0).col(0)[0];
  double y0 = poly_data.poly.value(0).col(0)[1];
  double vx0 = path_poly_dt1->value(0).col(0)[0];
  double vy0 = path_poly_dt1->value(0).col(0)[1];



  DiagramBuilder<double> builder; // Wraps normal DiagramBuilder. Has additional utility stuff for controlled systems.

  auto tree = std::make_unique<RigidBodyTree<double>>();

  // Import ball to tree.
  parsers::ModelInstanceIdTable ball_id_table = AddModelInstanceFromUrdfFileToWorld(FindResourceOrThrow(ballModelPath),
                                                                                    multibody::joints::kQuaternion, tree.get());

  // Ball for manipulation.
  AddModelInstanceFromUrdfFileToWorld(FindResourceOrThrow(ballModelPath),
                                      multibody::joints::kQuaternion, tree.get());
  // Make ground part of tree.
  multibody::AddFlatTerrainToWorld(tree.get(), 100., 10.);

  // add arm
//  AddModelInstanceFromUrdfFileToWorld(FindResourceOrThrow(armModelPath),
//                                      multibody::joints::kFixed, tree.get());

  //const int ball_id = ball_id_table.at("soccer_ball");

  lcm::DrakeLcm lcm;

  RigidBodyTree<double>* tree_ptr = tree.get();

  auto plant = std::make_unique<RigidBodyPlant<double>>(std::move(tree));
  RigidBodyPlant<double>* plant_ptr = builder.AddSystem<RigidBodyPlant<double>>(std::move(plant));
  plant_ptr->set_name("plant");


//  VectorXd const_t(7);
//  const_t.setZero();
//  ConstantVectorSource<double>* const_torque = builder.AddSystem<ConstantVectorSource<double>>(const_t);
//
//  builder.Connect(const_torque->get_output_port(), plant_ptr->get_input_port(0));


  DrakeVisualizer* visualizer = builder.AddSystem<DrakeVisualizer>(plant_ptr->get_rigid_body_tree(), &lcm);
  visualizer->set_name("main_vis");

  visualizer->set_publish_period(0.005);

  MagicForce* fext = builder.AddSystem<MagicForce>(plant_ptr, tree_ptr, poly_data, FLAGS_feedback);
  fext->repeat = true;

  builder.Connect(plant_ptr->get_output_port(2), fext->get_input_port(0)); // Manipulated ball.
  builder.Connect(plant_ptr->get_output_port(3), fext->get_input_port(1)); // Manipulator ball.
  builder.Connect(plant_ptr->contact_results_output_port(), fext->get_input_port(2));
  builder.Connect(plant_ptr->get_output_port(0), visualizer->get_input_port(0));


  // Draw path markers
  std::vector<Eigen::Isometry3d> poses_to_draw;
  std::vector<std::string> pose_names;

//  for (PolyWithKnots poly_dat : poly_data) {
  for (double i = poly_data.poly.start_time(); i < poly_data.poly.end_time(); i += 0.01) {
    VectorXd val = poly_data.poly.value(i);

    Eigen::Vector3d vec = {val.col(0)[0], val.col(0)[1], val.col(0)[2]};
    Eigen::Isometry3d traj_pos;
    traj_pos.setIdentity();
    traj_pos.translate(vec);
    poses_to_draw.push_back(traj_pos);
    pose_names.push_back(std::to_string(i));
  }
//  }

  PublishFrames(poses_to_draw, pose_names, lcm);



  // Draw acceleration marker arrows.
  LcmPublisherSystem* arrow_publisher = builder.AddSystem(
      LcmPublisherSystem::Make<lcmt_arbitrary_arrow_collection>(
          "MARKER_ARROWS", &lcm));
  arrow_publisher->set_publish_period(5);

  ArrowsToLcm* arrow_drawer = builder.AddSystem<ArrowsToLcm>();

  NonconstantAbstractSource<std::vector<VectorXd>>* arrow_vec_src =
      builder.AddSystem<NonconstantAbstractSource<std::vector<VectorXd>>>();
  std::vector<VectorXd> arrow_vecs;

  double arrow_scaling = 0.02;
//  double arrow_scaling_vel = 0.05;
  double red_thresh = std::sqrt(FLAGS_cutthresh);
  //for (PolyWithKnots poly_dat : poly_data) {
  std::unique_ptr<Trajectory<double>> poly_ddt = poly_data.poly.MakeDerivative(2);
  std::unique_ptr<Trajectory<double>> poly_dt = poly_data.poly.MakeDerivative(1);
  for (double i = poly_data.poly.start_time(); i < poly_data.poly.end_time(); i += 0.005) {

    // Accel/force.
    VectorXd arrow_loc = poly_data.poly.value(i);

    Vector3d arrow_vec = poly_ddt->value(i);
    double mag = arrow_vec.norm();
    VectorXd arrow_complete(9);
    arrow_complete << arrow_loc(0), arrow_loc(1), arrow_loc(2),
        arrow_scaling * arrow_vec(0), arrow_scaling * arrow_vec(1), arrow_scaling * arrow_vec(2),
        mag > red_thresh, mag <= red_thresh, 0;
    arrow_vecs.push_back(arrow_complete);

//    // Vel
//    Vector3d arrow_vec_vel = poly_dt->value(i);
//    double vel_mag = arrow_vec_vel.norm();
//
//    VectorXd arrow_vel(9);
//    arrow_vel << arrow_loc(0), arrow_loc(1), arrow_loc(2),
//        0, 0, arrow_scaling_vel*vel_mag,
//        0, 0, 1;
//    arrow_vecs.push_back(arrow_vel);

  }
  //}

///////////////////////
  arrow_vec_src->set_output(arrow_vecs);
  builder.Connect(arrow_vec_src->get_output_port(0), arrow_drawer->get_input_port(0));
  builder.Connect(arrow_drawer->get_output_port(0), arrow_publisher->get_input_port());

  std::unique_ptr<Diagram<double>> diagram = builder.Build();

  Simulator<double> simulator(*diagram);
  simulator.set_target_realtime_rate(1.0);

  Context<double>& context = simulator.get_mutable_context();
  plant_ptr->set_position(&context, 0, x0); // Moving the bowling ball away from the origin.
  plant_ptr->set_position(&context, 1, y0);
  plant_ptr->set_position(&context, 2, 0.071);
  plant_ptr->set_velocity(&context, 0, -vy0/0.07);
  plant_ptr->set_velocity(&context, 1, vx0/0.07);
  plant_ptr->set_velocity(&context, 3, vx0);
  plant_ptr->set_velocity(&context, 4, vy0);

  plant_ptr->set_position(&context, 7, 2);
  plant_ptr->set_position(&context, 8, 2);
  plant_ptr->set_position(&context, 9, 0.071);
//  plant_ptr->set_velocity(&context, 6, -vy0/0.07);
//  plant_ptr->set_velocity(&context, 7, vx0/0.07);
//  plant_ptr->set_velocity(&context, 9, vx0);
//  plant_ptr->set_velocity(&context, 10, vy0);

  drake::log()->info(vx0);
  drake::log()->info(vy0);

  simulator.reset_integrator<systems::RungeKutta2Integrator<double>>(*diagram, 0.002, &context);
  simulator.Initialize();
  simulator.StepTo(INT64_MAX);



}

}
}
}

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  drake::examples::iiwa_soccer::SetupWorld();
  return 0;
}