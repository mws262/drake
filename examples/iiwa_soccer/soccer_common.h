#pragma once

#include "drake/examples/iiwa_soccer/source_switcher.h"
#include "drake/examples/iiwa_soccer/nonconstant_abstract_source.h"
#include "drake/examples/iiwa_soccer/spline_trajectory.h"
#include "drake/examples/iiwa_soccer/nonconstant_vector_source.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/common/trajectories/piecewise_polynomial.h"

namespace drake {
namespace examples {
namespace iiwa_soccer {

// System constants.
const int arm_num_joints = 7;
const int arm_num_states = 14;
const int ball_num_states = 13;

// Knob twiddlers.
double direction_multiply_target = 0;
double direction_multiply_waypoint = 10;
double waypoint_time = 2;
double target_time = 4;
double target_offset_height = 0.0;
double waypoint_offset_height = 0.1;
double direction_target_min = 1e-5;
double direction_waypoint_min = 1e-4;

// Systems which switch between controllers, trigger replanning, etc.
struct SystemSwitches {
  SourceSwitcher<double> *torque_source_switcher; // Call switch_output to determine who sends torque to motors.
  SourceSwitcher<double> *setpt_source_switcher; // Switch who sends a cartesian setpoint to the compliant controller.
  SourceSwitcher<double> *offset_source_switcher; // Determine who sends an offset to the setpt.

  NonconstantVectorSource *ball_target_offset; // Adjustable target vector.
  NonconstantVectorSource *waypoint_target_offset; // Adjustable vector for waypoint of a spline.

  SplineTrajectories *spline_maker; // Makes splines for the end effector to follow. Can be triggered to replan.

  NonconstantAbstractSource<PiecewisePolynomial<double>>* ik_traj_receiver; // Can receive full joint trajectories, i.e. from IK.

  //NonconstantAbstractSource<trajectories::PiecewisePolynomial<double>> *full_dim_traj_receiver;
  RigidBodyTree<double> *tree;
  };

}
}
}