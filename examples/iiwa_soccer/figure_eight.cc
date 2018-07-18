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


DEFINE_double(cutsizethresh, 0.3, "won't cut if size is below this length (in seconds)");
DEFINE_double(min_around_cut, 0.05, "");
DEFINE_double(cutthresh, 10, "");
DEFINE_bool(feedback, false, "use feedback controls?");

const char* ballModelPath = "drake/examples/iiwa_soccer/models/soccer_ball.urdf";
const char* armModelPath = "drake/examples/iiwa_soccer/models/iiwa14_spheres_collision.urdf";


void PublishFrames(std::vector<Eigen::Isometry3d>& poses, std::vector<std::string>& name, lcm::DrakeLcm& lcm) {
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
  lcm.Publish("DRAKE_DRAW_FRAMES", bytes.data(), num_bytes, {});
}


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
PolyWithKnots MakeFigure8Poly(Vector3d& offset) {
  double time_scaling = 5;
  double num_pts = 8;


  double height = 0; // height of path parallel above the ground
  double lobe_length = 1; // center to top of 8.
  double lobe_width = 0.5; // center of lobe to horizontal edge max.

  std::vector<double> breaks;
  std::vector<MatrixX<double>> knots;
  VectorXd pt1(3);
  pt1 << 0, 0, height;

  VectorXd pt2(3);
  pt2 << lobe_width, lobe_length/2, height;

  VectorXd pt3(3);
  pt3 << 0, lobe_length, height;

  VectorXd pt4(3);
  pt4 << -lobe_width, lobe_length/2, height;

  VectorXd pt5(3);
  pt5 << 0, 0, height;

  VectorXd pt6(3);
  pt6 << lobe_width, -lobe_length/2, height;

  VectorXd pt7(3);
  pt7 << 0, -lobe_length, height;

  VectorXd pt8(3);
  pt8 << -lobe_width, -lobe_length/2, height;


  knots.push_back(pt1 + offset);
  knots.push_back(pt2 + offset);
  knots.push_back(pt3 + offset);
  knots.push_back(pt4 + offset);
  knots.push_back(pt5 + offset);
  knots.push_back(pt6 + offset);
  knots.push_back(pt7 + offset);
  knots.push_back(pt8 + offset);
  knots.push_back(pt1 + offset);

  breaks.push_back(time_scaling*(0/num_pts));
  breaks.push_back(time_scaling*(1/num_pts));
  breaks.push_back(time_scaling*(2/num_pts));
  breaks.push_back(time_scaling*(3/num_pts));
  breaks.push_back(time_scaling*(4/num_pts));
  breaks.push_back(time_scaling*(5/num_pts));
  breaks.push_back(time_scaling*(6/num_pts));
  breaks.push_back(time_scaling*(7/num_pts));
  breaks.push_back(time_scaling*(8/num_pts));

//  Vector3d start_dt(0,0,0);
//  Vector3d end_dt(0,0,0);
  PiecewisePolynomial<double> original_poly = PiecewisePolynomial<double>::Cubic(breaks, knots, true);//, start_dt, end_dt);

  PolyWithKnots figure8dat = {original_poly, breaks, knots};
  return figure8dat;
}


std::vector<PolyWithKnots> SplitPoly(PolyWithKnots& poly_data, double threshold) {

  if (poly_data.poly.get_number_of_segments() < 2) {
    std::vector<PolyWithKnots> poly_dat;
    poly_dat.push_back(poly_data);
    return poly_dat;
  }

  //////////////////

  std::unique_ptr<Trajectory<double>> path_poly_dt = poly_data.poly.MakeDerivative(1);
  std::unique_ptr<Trajectory<double>> path_poly_ddt = poly_data.poly.MakeDerivative(2);

  Vector3d start_dt = path_poly_dt->value(path_poly_dt->start_time());
  Vector3d end_dt = path_poly_dt->value(path_poly_dt->end_time());


  double cut_start_t = 0;
  double cut_end_t = 0;
  Vector3d cut_start_pt;
  Vector3d cut_end_pt;
  double cut_size = 0;

  bool success = false;
  // Decide where to make the cut. Must have some minimum area before the cut begins.
  for (double i = FLAGS_min_around_cut; i < path_poly_ddt->end_time(); i += 0.001){
    Vector3d accel = path_poly_ddt->value(i);
    if (accel.squaredNorm() < threshold) {
      cut_size = 0;
      cut_start_pt = poly_data.poly.value(i);
      cut_start_t = i;
      do{
        i += 0.001;
        cut_size += 0.001;
        accel = path_poly_ddt->value(i);
      }while (accel.squaredNorm() < threshold && i < path_poly_ddt->end_time() - FLAGS_min_around_cut);
      cut_end_pt = poly_data.poly.value(i);
      cut_end_t = i;
      if (cut_size > FLAGS_cutsizethresh){
        success = true;
        break;
      }
    }
  }
  if (!success) {
    std::vector<PolyWithKnots> negative_result;
    negative_result.push_back(poly_data);
    return negative_result;
  }

  // Direction of straight line between the two sides of the cut.
  Vector3d break_dir = cut_end_pt - cut_start_pt;
  double cut_vel = break_dir.norm()/(cut_end_t - cut_start_t); // Assuming constant velocity over the "ballistic" part.
  break_dir.normalize();

  std::vector<double> new_breaks1;
  std::vector<MatrixX<double>> new_knots1;

  // Add all knots and breaks before the cut
  double t1 = poly_data.poly.start_time();
  int knot_idx = 0;
  while (t1 < cut_start_t) {
    new_breaks1.push_back(poly_data.breaks[knot_idx]);
    new_knots1.push_back(poly_data.knots[knot_idx]);
    t1 = poly_data.breaks[++knot_idx];
  }
  // Add new knot at the cut.
  new_knots1.push_back(cut_start_pt);
  new_breaks1.push_back(cut_start_t);
  Vector3d cut_dt = cut_vel * break_dir;

  PiecewisePolynomial<double> poly_section1 = PiecewisePolynomial<double>::Cubic(new_breaks1, new_knots1, start_dt, cut_dt);
  PolyWithKnots poly_data_1 = {poly_section1, new_breaks1, new_knots1};

//  // We just cut off the end of the piecewise poly, so there is nothing after the cut any more.
//  if (knot_idx + 1 == static_cast<int>(poly_data.breaks.size())) {
//    std::vector<PolyWithKnots> poly_dat;
//    poly_dat.push_back(poly_data_1);
//    return poly_dat;
//  }


  // Get to the knots which are past the cut.
  while (t1 < cut_end_t) {
    t1 = poly_data.breaks[++knot_idx];
  }


  std::vector<double> new_breaks2;
  std::vector<MatrixX<double>> new_knots2;

  // Add new knot at the cut.
  new_knots2.push_back(cut_end_pt);
  new_breaks2.push_back(cut_end_t);

  double t2 = cut_end_t;
  // knot_idx++;
  while (t2 <= path_poly_dt->end_time()) { // Add all knots and breaks after the cut.
    new_breaks2.push_back(poly_data.breaks[knot_idx]);
    new_knots2.push_back(poly_data.knots[knot_idx]);
    if (knot_idx >= static_cast<int>(poly_data.knots.size() - 1)) break;
    t2 = poly_data.breaks[++knot_idx];
  }

  PiecewisePolynomial<double> poly_section2 = PiecewisePolynomial<double>::Cubic(new_breaks2, new_knots2, cut_dt, end_dt);
  PolyWithKnots poly_data_2 = {poly_section2, new_breaks2, new_knots2};

  std::vector<PolyWithKnots> both_poly_dat;
  both_poly_dat.push_back(poly_data_1);
  both_poly_dat.push_back(poly_data_2);


  return both_poly_dat;
}

std::vector<PolyWithKnots> CutUntil(PolyWithKnots original_poly, double thresh) {
  std::vector<PolyWithKnots> poly_data1 = SplitPoly(original_poly, thresh);

  std::vector<PolyWithKnots> results;

  if (poly_data1.size() > 1){
    for (PolyWithKnots poly_dat : poly_data1) {
      drake::log()->info("cutting again");
      std::vector<PolyWithKnots> cut = CutUntil(poly_dat, thresh);
      for (PolyWithKnots cut_res : cut) {
        results.push_back(cut_res);
      }
    }
    return results;
  }else{
    return poly_data1;
  }
}

void SetupWorld() {
  Vector3d poly_offset(0, 0, 0);
  PolyWithKnots figure8poly = MakeFigure8Poly(poly_offset);
  MakeTRIPoly();
  std::vector<PolyWithKnots> poly_data = CutUntil(figure8poly, FLAGS_cutthresh);

  // Initial conditions based on first piecewise polynomial
  std::unique_ptr<Trajectory<double>> path_poly_dt1 = poly_data[0].poly.MakeDerivative(1);
  std::unique_ptr<Trajectory<double>> path_poly_ddt1 = poly_data[0].poly.MakeDerivative(2);

  double x0 = poly_data[0].poly.value(0).col(0)[0];
  double y0 = poly_data[0].poly.value(0).col(0)[1];
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

  builder.Connect(plant_ptr->get_output_port(0), visualizer->get_input_port(0));


  // Draw path markers
  std::vector<Eigen::Isometry3d> poses_to_draw;
  std::vector<std::string> pose_names;

  for (PolyWithKnots poly_dat : poly_data) {
    for (double i = poly_dat.poly.start_time(); i < poly_dat.poly.end_time(); i += 0.01) {
      VectorXd val = poly_dat.poly.value(i);

      Eigen::Vector3d vec = {val.col(0)[0], val.col(0)[1], val.col(0)[2]};
      Eigen::Isometry3d traj_pos;
      traj_pos.setIdentity();
      traj_pos.translate(vec);
      poses_to_draw.push_back(traj_pos);
      pose_names.push_back(std::to_string(i));
    }
  }

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

  double arrow_scaling = 0.05;
  double arrow_scaling_vel = 0.05;
  double red_thresh = std::sqrt(FLAGS_cutthresh);
  for (PolyWithKnots poly_dat : poly_data) {
    std::unique_ptr<Trajectory<double>> poly_ddt = poly_dat.poly.MakeDerivative(2);
    std::unique_ptr<Trajectory<double>> poly_dt = poly_dat.poly.MakeDerivative(1);
    for (double i = poly_dat.poly.start_time(); i < poly_dat.poly.end_time(); i += 0.05) {

      // Accel/force.
      VectorXd arrow_loc = poly_dat.poly.value(i);

      Vector3d arrow_vec = poly_ddt->value(i);
      double mag = arrow_vec.norm();
      VectorXd arrow_complete(9);
      arrow_complete << arrow_loc(0), arrow_loc(1), arrow_loc(2),
          arrow_scaling * arrow_vec(0), arrow_scaling * arrow_vec(1), arrow_scaling * arrow_vec(2),
          mag > red_thresh, mag <= red_thresh, 0;
      arrow_vecs.push_back(arrow_complete);

      // Vel
      Vector3d arrow_vec_vel = poly_dt->value(i);
      double vel_mag = arrow_vec_vel.norm();

      VectorXd arrow_vel(9);
      arrow_vel << arrow_loc(0), arrow_loc(1), arrow_loc(2),
          0, 0, arrow_scaling_vel*vel_mag,
          0, 0, 1;
      arrow_vecs.push_back(arrow_vel);

    }
  }

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
  plant_ptr->set_position(&context, 9, 4);

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