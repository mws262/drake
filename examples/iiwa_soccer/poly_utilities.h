#pragma once

#include "drake/common/eigen_types.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/trajectory.h"
#include "drake/common/text_logging.h"
#include "drake/common/drake_assert.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/rigid_body_plant/frame_visualizer.h"

namespace drake {
namespace examples {
namespace iiwa_soccer {

using trajectories::Trajectory;
using trajectories::PiecewisePolynomial;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;

struct PolyWithKnots{
  PiecewisePolynomial<double> poly;

  std::vector<double> breaks;

  std::vector<MatrixX<double>> knots;
};

const double cutsizethresh = 0.3;
const double cut_step = 0.00001; // Interval to march through the poly to find potential cut areas.
const double time_merging_thresh = 0.001;

/**
 * Makes a new `PolyWithKnots` that is trimmed down, with new boundary conditions AND every linear interior section
 * is required to stay linear.
 *
 * @param poly_data Original `PolyWithKnots`.
 * @param start_seg_inclusive Will trim to this segment index (starting at 0). This index WILL remain in the trimmed version.
 * @param end_seg_inclusive Will trim to this segment index and still include it in the trimmed output.
 * @param start_t New time for the first point.
 * @param end_t New time for the last point.
 * @param start_p New point for the beginning of the first segment.
 * @param end_p New point for the end of the last segment.
 * @param start_d Derivative at `start_seg_inclusive` beginning.
 * @param end_d Derivative at `end_seg_inclusive` ending.
 * @return
 */
PolyWithKnots LinearSegmentPreservingCubicRemesh(const PolyWithKnots& poly_data,
                                                 const int start_seg_inclusive,
                                                 const int end_seg_inclusive,
                                                 const double start_t,
                                                 const double end_t,
                                                 const Vector3d& start_p,
                                                 const Vector3d& end_p,
                                                 const Vector3d& start_d,
                                                 const Vector3d& end_d,
                                                 const bool use_existing_start,
                                                 const bool use_existing_end) {

  // Input sanity checks.
  if (start_seg_inclusive < 0) throw std::runtime_error("BAD_FIRST_IDX_LESS_THAN_0: Start segment index cannot be less than 0.");
  if (end_seg_inclusive < start_seg_inclusive) throw std::runtime_error("BAD_SEGMENT_IDX_ORDER: Start segment index is greater than the end index.");
  if (static_cast<int>(poly_data.breaks.size()) < end_seg_inclusive + 2) throw std::runtime_error("BAD_END_IDX_TOO_BIG: Trying to trim outside the bounds of the given piecewise polynomial");

  if (use_existing_start && poly_data.breaks[start_seg_inclusive + 1] <= start_t) throw std::runtime_error("BAD_START_TIME_TOO_BIG_AS_REPLACEMENT: Bad start time. use_existing_start is true, so the start time may be greater than the first start_seg_inclusive break, but it may not be greater than the one after it.");
  if (!use_existing_start && poly_data.breaks[start_seg_inclusive] <= start_t) throw std::runtime_error("BAD_START_TIME_TOO_BIG_AS_INSERTION: Bad start time. use_existing_start is false, so the start time must be less than the first start_seg_inclusive_break.");

  if (use_existing_end && poly_data.breaks[end_seg_inclusive] >= end_t) throw std::runtime_error("BAD_END_TIME_TOO_SMALL_AS_REPLACEMENT: Bad end time. use_existing_end is true, so we only have to be greater than the next-to-last included break since we are replacing the last.");
  if (!use_existing_end && poly_data.breaks[end_seg_inclusive + 1] >= end_t) throw std::runtime_error("BAD_END_TIME_TOO_BIG_AS_INSERTION: Bad end time. use_existing_end is false, so the end time must be greater than the last element of the included breaks.");

  PolyWithKnots remeshed_poly_holder;

  std::vector<int> first_order_idxs;
  // Find where any first order sections are (if any).
  for (int i = start_seg_inclusive; i <= end_seg_inclusive; i++){
    // It's a line if all three polynomials (i.e. x = f(t), y = f(t), z = f(t)) have degree 1.
    auto& poly_mat = poly_data.poly.getPolynomialMatrix(i);
    if (poly_mat(0).GetDegree() == 1 && poly_mat(1).GetDegree() == 1 && poly_mat(2).GetDegree() == 1) {
      if (use_existing_start) {
        first_order_idxs.push_back(i - start_seg_inclusive);
      } else {
        first_order_idxs.push_back(i - start_seg_inclusive + 1); // If we are going to add a knot at the beginning, then this is offset by 1.
      }
    }
  }

  // Get knots and breaks associated with this subsection.
  std::vector<double>::const_iterator scn_break_start = poly_data.breaks.begin() + start_seg_inclusive;
  std::vector<double>::const_iterator scn_break_end = poly_data.breaks.begin() + end_seg_inclusive + 2; // +2 because length is +1 over index, and knots are +1 over segments.
  std::vector<double> scn_breaks(scn_break_start, scn_break_end);

  std::vector<MatrixX<double>>::const_iterator scn_knots_start = poly_data.knots.begin() + start_seg_inclusive;
  std::vector<MatrixX<double>>::const_iterator scn_knots_end = poly_data.knots.begin() + end_seg_inclusive + 2;
  std::vector<MatrixX<double>> scn_knots(scn_knots_start, scn_knots_end);



  if (first_order_idxs.empty()){ // No linear sections. Carry on!
    if (use_existing_start) {
      scn_breaks.front() = start_t; // Change start break to new.
      scn_knots.front() = start_p; // Change start knot to new.

    }else { // Put new knot in.
      scn_breaks.insert(scn_breaks.begin(), start_t);
      scn_knots.insert(scn_knots.begin(), start_p);
    }

    if (use_existing_end) {
      scn_breaks.back() = end_t; // Change end break to new.
      scn_knots.back() = end_p; // Change end knot to new.

    }else { // Put new knot in.
      scn_breaks.push_back(end_t);
      scn_knots.push_back(end_p);
    }

    PiecewisePolynomial<double> remeshed_poly = PiecewisePolynomial<double>::Cubic(scn_breaks, scn_knots, start_d, end_d);
    remeshed_poly_holder.poly = remeshed_poly;



  }else {

    const int first_linear_idx = first_order_idxs.front(); // We only want to alter sections which are before/after the linear sections, not any which are in between linear sections.
    const int last_linear_idx = first_order_idxs.back();

    if (use_existing_start) {
      if (first_linear_idx != 0) { // Protect the linear section if it's first and we aren't adding a point afterwards.
        scn_breaks.front() = start_t; // Change start break to new.
        scn_knots.front() = start_p; // Change start knot to new.
      }
    }else { // Put new knot in.
      scn_breaks.insert(scn_breaks.begin(), start_t);
      scn_knots.insert(scn_knots.begin(), start_p);
    }

    if (use_existing_end) {
      if (last_linear_idx != static_cast<int>(scn_breaks.size() - 1)) { // Protect the linear section if it's last
        scn_breaks.back() = end_t; // Change end break to new.
        scn_knots.back() = end_p; // Change end knot to new.
      }
    }else { // Put new knot in.
      if (scn_breaks.back() > end_t) throw std::runtime_error("Trying to insert a lower time than the previous at the end of a set of breaks.");
      scn_breaks.push_back(end_t);
      scn_knots.push_back(end_p);
    }


//    if (use_existing_start && first_linear_idx == 0) {
//      drake::log()->info("Cubic spline trimmed with linear first section. Specified positions and velocities for the first segment are ignored.");
//    }
//    if (use_existing_end && last_linear_idx == end_seg_inclusive) {
//      drake::log()->info("Cubic spline trimmed with linear last section. Specified positions and velocities for the last segment are ignored.");
//    }

    std::unique_ptr<Trajectory<double>> poly_dt = poly_data.poly.MakeDerivative(1);

    if (scn_breaks.size() == 2 && use_existing_start && use_existing_end && first_linear_idx == 0) { // Special case -- only two points to slice and it's a linear section already!
      PiecewisePolynomial<double> single_section = poly_data.poly.slice(start_seg_inclusive, 1);
      remeshed_poly_holder.poly = single_section;

    } else if (use_existing_start && first_linear_idx == 0) { // Linear section is right there at the beginning of the selected region. No need for a pp before middle region.

      // Middle polynomial is from the first to last linear segments. We want to completely preserve these and their boundary conditions.
      PiecewisePolynomial<double> mid_section = poly_data.poly.slice(start_seg_inclusive + first_linear_idx - (use_existing_start ? 0 : 1),
                                                                     last_linear_idx - first_linear_idx + 1); // +1 because the last linear index includes the segment going from it TO the next knot.



      std::vector<double> sub_breaks_last(scn_breaks.begin() + last_linear_idx + 1, scn_breaks.end());
      std::vector<MatrixX<double>> sub_knots_last(scn_knots.begin() + last_linear_idx + 1, scn_knots.end());

      PiecewisePolynomial<double> last_poly = PiecewisePolynomial<double>::Cubic(sub_breaks_last,
                                                                                 sub_knots_last,
                                                                                 poly_dt->value(sub_breaks_last.front()),
                                                                                 end_d);
      mid_section.ConcatenateInTime(last_poly);
      remeshed_poly_holder.poly = mid_section;
    }else if (use_existing_end && last_linear_idx >= static_cast<int>(scn_breaks.size()) - 2) { // 2 means we don't want to leave a SINGLE node after mid.

      // New pp at the beginning. Takes provided start derivative. Maintains end derivative demanded by the next linear section.
      std::vector<double> sub_breaks_first(scn_breaks.begin(), scn_breaks.begin() + first_linear_idx + 1);
      std::vector<MatrixX<double>> sub_knots_first(scn_knots.begin(), scn_knots.begin() + first_linear_idx + 1);

      PiecewisePolynomial<double> first_poly = PiecewisePolynomial<double>::Cubic(sub_breaks_first,
                                                                                  sub_knots_first,
                                                                                  start_d,
                                                                                  poly_dt->value(sub_breaks_first.back()));

      // Middle polynomial is from the first to last linear segments. We want to completely preserve these and their boundary conditions.
      PiecewisePolynomial<double> mid_section = poly_data.poly.slice(start_seg_inclusive + first_linear_idx - (use_existing_start ? 0 : 1),
                                                                     last_linear_idx - first_linear_idx + 1); // +1 because the last linear index includes the segment going from it TO the next knot.

      first_poly.ConcatenateInTime(mid_section);
      remeshed_poly_holder.poly = first_poly;
    }else {
      // New pp at the beginning. Takes provided start derivative. Maintains end derivative demanded by the next linear section.
      std::vector<double> sub_breaks_first(scn_breaks.begin(), scn_breaks.begin() + first_linear_idx + 1);
      std::vector<MatrixX<double>> sub_knots_first(scn_knots.begin(), scn_knots.begin() + first_linear_idx + 1);

      PiecewisePolynomial<double> first_poly = PiecewisePolynomial<double>::Cubic(sub_breaks_first,
                                                                                  sub_knots_first,
                                                                                  start_d,
                                                                                  poly_dt->value(sub_breaks_first.back()));

      // Middle polynomial is from the first to last linear segments. We want to completely preserve these and their boundary conditions.
      PiecewisePolynomial<double> mid_section = poly_data.poly.slice(start_seg_inclusive + first_linear_idx - (use_existing_start ? 0 : 1),
                                                                     last_linear_idx - first_linear_idx + 1); // +1 because the last linear index includes the segment going from it TO the next knot.


      std::vector<double> sub_breaks_last(scn_breaks.begin() + last_linear_idx + 1, scn_breaks.end());
      std::vector<MatrixX<double>> sub_knots_last(scn_knots.begin() + last_linear_idx + 1, scn_knots.end());

      PiecewisePolynomial<double> last_poly = PiecewisePolynomial<double>::Cubic(sub_breaks_last,
                                                                                 sub_knots_last,
                                                                                 poly_dt->value(sub_breaks_last.front()),
                                                                                 end_d);
      first_poly.ConcatenateInTime(mid_section);
      first_poly.ConcatenateInTime(last_poly);
      remeshed_poly_holder.poly = first_poly;
    }





  }

  // Knots and breaks don't actually change. Only various derivative conditions do.
  remeshed_poly_holder.breaks = scn_breaks;
  remeshed_poly_holder.knots = scn_knots;

  return remeshed_poly_holder;
}

double SplitPolyInPlace(PolyWithKnots& poly_data, double start_looking_time, double threshold) {

  /** Existing piecewise polynomial evaluations/derivatives/etc. **/
  std::unique_ptr<Trajectory<double>> path_poly_dt = poly_data.poly.MakeDerivative(1);
  std::unique_ptr<Trajectory<double>> path_poly_ddt = poly_data.poly.MakeDerivative(2);
  double start_time = path_poly_dt->start_time();
  DRAKE_DEMAND(start_time <= start_looking_time);
  double end_time = path_poly_dt->end_time();
  DRAKE_DEMAND(end_time >= start_looking_time);


  Vector3d start_pt = poly_data.poly.value(start_time);
  Vector3d end_pt = poly_data.poly.value(end_time);

  Vector3d start_dt = path_poly_dt->value(start_time);
  //Vector3d end_dt = path_poly_dt->value(end_time);

  /** Identify a cut location. It must be long enough of a cut, with all points in it having below a certain
   *  acceleration. So far, we do NOT check for cuts which wrap from before end of trajectory back around to after the
   *  beginning.
   */

  double cut_start_t = 0; // Time (or phase) along the trajectory where the cut will start.
  double cut_end_t = 0; // Time (or phase) along the trajectory where the cut will end.

  Vector3d cut_start_pt; // Point in space along the existing pp where the beginning of the cut will be made.
  Vector3d cut_end_pt; // Point in space along the existing pp where the end of the cut will be made.
  double cut_size = 0; // Track the length (in time) of the current cut. It must pass a threshold to be a candidate.
  bool success = false; // Track whether a good cut candidate has been found.

  // Decide where to make the cut.
  for (double i = start_looking_time; i < end_time; i += cut_step) {

    Vector3d accel = path_poly_ddt->value(i);

    if (accel.squaredNorm() < threshold && accel.squaredNorm() > 0.0001) { // We want to find low-accel, but NOT already linear sections.
      cut_size = 0;
      cut_start_pt = poly_data.poly.value(i);
      cut_start_t = i;
      do { // Check the length of this potential cut before deciding whether to make it or not.
        i += cut_step;
        cut_size += cut_step;
        accel = path_poly_ddt->value(i);
      } while (accel.squaredNorm() < threshold && i < end_time  && accel.squaredNorm() > 0.0001);
      cut_end_pt = poly_data.poly.value(i);
      cut_end_t = i;
      if (cut_size > cutsizethresh) {
        success = true;
        break;
      }
    }
  }
  if (!success) {
    drake::log()->info("Polynomial trimming found no places to cut. Returning.");
    return end_time;
  }

  if (cut_start_t < start_looking_time)
    throw std::runtime_error("We demanded cuts after " + std::to_string(start_looking_time) +
        " but got a cut starting at " + std::to_string(cut_start_t));

  /**
   * Several cases at this point.
   * 1. We've decided to make a cut where both start and end lie between nodes (not necessarily the same nodes though).
   *    In this case, add two nodes, remake the pp before, during, and after the cut. Match end of first pp and last pp
   *    derivatives.
   *
   * 2. Start pt lies very near the start point of pp. In this case, make NO pp before the cut, add linear section at
   *    the beginning, make pp after the cut. Problem -- end gradient might not match beginning gradient any longer.
   *    Solution -- make the final pp section match this derivative.
   *
   * 3. End pt lies very near the end point of pp. Make pp before cut, make linear pp, don't make pp after the cut.
   *    Problem -- end gradient might not match beginning gradient any longer.
   *    Solution -- make the first pp section match this derivative.
   *
   * 4. Start pt, end pt, or both lie very near interior knots of the PP. Use this/these as the actual point of the cut.
   *    otherwise same as 1.
   *
   * 5. Cut "wraps" around the phase variable, i.e. `cut_start_t` > `cut_end_t`. This case is not allowed and not
   *    handled even though it is a reasonable thing to consider in the future.
   *
   * 2'. Same as 2, but end point lies near an interior knot as in case 4.
   * 3'. Same as 3, but start point lies near an interior knot as in case 4.
   *
   */

  /** Identify which case we have. **/

  // Begin cut point matches start pt.
  bool start_matches_first_knot = std::fabs(cut_start_t - start_time) < time_merging_thresh;

  // End cut point matches end pt.
  bool end_matches_last_knot = std::fabs(cut_end_t - end_time) < time_merging_thresh;

  bool start_matches_interior = false;
  double start_match_interior_break; // Only assigned if start_matches_interior becomes true.
  bool end_matches_interior = false;
  double end_match_interior_break; // Only assigned if end_matches_interior becomes true.

  if (start_matches_first_knot && end_matches_last_knot) {
    throw std::runtime_error("Beginning and end of cut match beginning and end of piecewise polynomial. I can't think of a reason we would want this.");
  }

  // Find interior matches between cut points and existing knots.
  for (double single_break : poly_data.breaks){
    if (single_break != start_time && std::fabs(cut_start_t - single_break) < time_merging_thresh) {
      if (start_matches_first_knot) throw std::runtime_error("First point of cut matches start of piecewise polynomial AND another point too.");
      if (start_matches_interior) throw std::runtime_error("First point of cut matches multiple existing knots. Something is degenerate-ish.");
      start_matches_interior = true;
      start_match_interior_break = single_break;
    }
    if (single_break != end_time && std::fabs(cut_end_t - single_break) < time_merging_thresh) {
      if (end_matches_last_knot) throw std::runtime_error("Last point of cut matches end of piecewise polynomial AND another point too.");
      if (end_matches_interior) throw std::runtime_error("Last point of cut matches multiple existing knots. Something is degenerate-ish.");
      end_matches_interior = true;
      end_match_interior_break = single_break;
    }
  }


  // Fill these up based on which case we have. Merge them back at the end.
  PiecewisePolynomial<double> poly_before_cut;
  std::vector<double> breaks_before_cut;
  std::vector<MatrixX<double>> knots_before_cut;

  PiecewisePolynomial<double> poly_linear_cut;
  VectorXd linear_breaks(2);
  MatrixXd linear_knots(3, 2);

  PiecewisePolynomial<double> poly_after_cut;
  std::vector<double> breaks_after_cut;
  std::vector<MatrixX<double>> knots_after_cut;



  /** Cut is at the beginning of the PP **/
  if (start_matches_first_knot){
    drake::log()->info("Trimming from start.");

    cut_start_t = start_time; // Beginning of cut replaced with beginning of the old PP.
    cut_start_pt = start_pt;

    // Subcase -- end of cut matches another interior knot.
    if (end_matches_interior){
      cut_end_t = end_match_interior_break; // End of cut replaced with near match.
      cut_end_pt = poly_data.poly.value(end_match_interior_break);
    }

    // Make the linear section.
    linear_breaks << cut_start_t, cut_end_t;
    linear_knots.col(0) = cut_start_pt;
    linear_knots.col(1) = cut_end_pt;
    poly_linear_cut = PiecewisePolynomial<double>::FirstOrderHold(linear_breaks, linear_knots);

    // Find the velocity at start/end of cut.
    Vector3d cut_dt = (cut_end_pt - cut_start_pt) / (cut_end_t - cut_start_t);

    /// poly_before_cut stays empty.

    // Poly after cut must match the derivative of the linear section on both sides even though the first and last
    // points of this pp are not collocated.

    PolyWithKnots poly_holder_after_cut = LinearSegmentPreservingCubicRemesh(poly_data,
                                                                             1 + (start_matches_interior ? 0 : 1), // Start segment is the second since first is replaced by linear section.
                                                                             poly_data.poly.get_number_of_segments() - 1,
                                                                             cut_end_t,
                                                                             end_time,
                                                                             cut_end_pt,
                                                                             end_pt,
                                                                             cut_dt,
                                                                             cut_dt,
                                                                             end_matches_interior,
                                                                             true);

    poly_after_cut = poly_holder_after_cut.poly;
    breaks_after_cut = poly_holder_after_cut.breaks;
    knots_after_cut = poly_holder_after_cut.knots;

//    poly_after_cut = PiecewisePolynomial<double>::Cubic(breaks_after_cut, knots_after_cut, cut_dt, cut_dt);

    DRAKE_DEMAND(std::fabs(poly_after_cut.start_time() - poly_linear_cut.end_time()) < 1e-9);
  }else if (end_matches_last_knot){ /** Start of cut is at end of PP **/
    drake::log()->info("Trimming at end.");
    cut_end_t = end_time; // End of cut is replaced with end of the old PP
    cut_end_pt = end_pt;

    // Subcase -- beginning of cut matches another interior point.
    if (start_matches_interior) {
      cut_start_t = start_match_interior_break; // Start of cut replaced with a near match.
      cut_start_pt = poly_data.poly.value(start_match_interior_break);
    }

    Vector3d cut_dt;
    cut_dt.setZero();

    // Another weird case: We already have a linear section at the beginning. This linear section at the end MUST match dts.
    auto& poly_mat = poly_data.poly.getPolynomialMatrix(0);
    if (poly_mat(0).GetDegree() == 1 && poly_mat(1).GetDegree() == 1 && poly_mat(2).GetDegree() == 1) {
      auto old_poly_dt = poly_data.poly.MakeDerivative(1);
      double cut_delta = cut_end_t - cut_start_t;
      cut_dt = start_dt; // We're stuck with this dt.
      cut_start_pt = cut_end_pt - cut_dt*cut_delta;
      cut_start_t = cut_end_t - cut_delta;
    }else {
      // Find the velocity at start/end of cut.
      cut_dt = (cut_end_pt - cut_start_pt) / (cut_end_t - cut_start_t);
    }
    // Make the linear section.
    linear_breaks << cut_start_t, cut_end_t;
    linear_knots.col(0) = cut_start_pt;
    linear_knots.col(1) = cut_end_pt;
    poly_linear_cut = PiecewisePolynomial<double>::FirstOrderHold(linear_breaks, linear_knots);


    PolyWithKnots poly_holder_before_cut = LinearSegmentPreservingCubicRemesh(poly_data,
                                                                              0,
                                                                              poly_data.poly.get_number_of_segments() - 2, // Only the last segment is replaced.
                                                                              start_time,
                                                                              cut_start_t,
                                                                              start_pt,
                                                                              cut_start_pt,
                                                                              cut_dt,
                                                                              cut_dt,
                                                                              true,
                                                                              start_matches_interior);

    poly_before_cut = poly_holder_before_cut.poly;
    breaks_before_cut = poly_holder_before_cut.breaks;
    knots_before_cut = poly_holder_before_cut.knots;


    auto dt_test = poly_holder_before_cut.poly.MakeDerivative(1);
    dt_test->value(0);
    /// poly_after_cut stays empty.

    DRAKE_DEMAND(std::fabs(poly_before_cut.end_time() - poly_linear_cut.start_time()) < 1e-9);
  } else { /** Beginning and end of cut lie on the interior of the PP. They may still be on existing interior knots. **/
    drake::log()->info("Trimming from interior.");
    // Replace cut start/end points with nearby existing matches if necessary.
    if (start_matches_interior) {
      cut_start_t = start_match_interior_break; // Start of cut replaced with a near match.
      cut_start_pt = poly_data.poly.value(start_match_interior_break);
    }

    if (end_matches_interior) {
      cut_end_t = end_match_interior_break; // End of cut replaced with near match.
      cut_end_pt = poly_data.poly.value(end_match_interior_break);
    }

    // Find the velocity at start/end of cut.
    Vector3d cut_dt = (cut_end_pt - cut_start_pt) / (cut_end_t - cut_start_t);

    // Find breaks and knots before cut.
    int cut_idx = 0;
    if (start_matches_interior) {
      while (poly_data.breaks[cut_idx] < cut_start_t) cut_idx++;
      if (poly_data.breaks[cut_idx] == cut_start_t) cut_idx--;
    }else {
      while (poly_data.breaks[cut_idx + 2] < cut_start_t) cut_idx++;
    }

//    while (cut_idx < static_cast<double>(poly_data.breaks.size()) - 1) cut_idx++;
//    for (int i = 0; i < static_cast<double>(poly_data.breaks.size()); i++) {
//      if (poly_data.breaks[i] < cut_start_t - time_merging_thresh) {
//        cut_idx = i - 1;
//      }

    // Find breaks and knots after cut.
    int after_cut_idx = 0;
    if (end_matches_interior) {
      while (poly_data.breaks[after_cut_idx + 1] <= cut_end_t) after_cut_idx++;
    }else {
      while (poly_data.breaks[after_cut_idx] < cut_end_t) after_cut_idx++;
    }



    PolyWithKnots poly_holder_before_cut = LinearSegmentPreservingCubicRemesh(poly_data,
                                                                              0,
                                                                              cut_idx,
                                                                              start_time,
                                                                              cut_start_t,
                                                                              start_pt,
                                                                              cut_start_pt,
                                                                              start_dt,
                                                                              cut_dt,
                                                                              true,
                                                                              start_matches_interior);

    // Make the linear section.
    linear_breaks << cut_start_t, cut_end_t;
    linear_knots.col(0) = cut_start_pt;
    linear_knots.col(1) = cut_end_pt;
    poly_linear_cut = PiecewisePolynomial<double>::FirstOrderHold(linear_breaks, linear_knots);


    // Stupid special case: We want to cut and add a point. Normally, we'd grab the segment after the area we
    // want to add a point. In this case, there is no segment after this one, since it's in the final segment.
    // Solution: move the cut index back, but also don't add another point, just change the value of the one before.
    if (!end_matches_interior && after_cut_idx == static_cast<int>(poly_data.breaks.size()) - 1) {
      after_cut_idx -= 1;
      end_matches_interior = true;
    }


    PolyWithKnots poly_holder_after_cut = LinearSegmentPreservingCubicRemesh(poly_data,
                                                                             after_cut_idx,
                                                                             poly_data.poly.get_number_of_segments() - 1,
                                                                             cut_end_t,
                                                                             end_time,
                                                                             cut_end_pt,
                                                                             end_pt,
                                                                             cut_dt,
                                                                             start_dt,
                                                                             end_matches_interior,
                                                                             true);

    poly_before_cut = poly_holder_before_cut.poly;
    poly_after_cut = poly_holder_after_cut.poly;

    breaks_before_cut = poly_holder_before_cut.breaks;
    breaks_after_cut = poly_holder_after_cut.breaks;

    knots_before_cut = poly_holder_before_cut.knots;
    knots_after_cut = poly_holder_after_cut.knots;


    DRAKE_DEMAND(std::fabs(poly_after_cut.start_time() - poly_linear_cut.end_time()) < 1e-9);
    DRAKE_DEMAND(std::fabs(poly_before_cut.end_time() - poly_linear_cut.start_time()) < 1e-9);
  }

  if (poly_linear_cut.empty()) throw std::runtime_error("After trying to trim a piecewise polynomial, the resulting connecting line didn't exist.");


  PiecewisePolynomial<double>* resulting_poly;
  // Concatenate all the polynomials. They're beginning/end times should match perfectly. Ignore empty ones since concat doesn't like these.
  if (poly_before_cut.empty()){
    resulting_poly = &poly_linear_cut;
  }else{
    resulting_poly = &poly_before_cut;
    resulting_poly->ConcatenateInTime(poly_linear_cut);
  }

  if (!poly_after_cut.empty()) resulting_poly->ConcatenateInTime(poly_after_cut); // Concatenate it with the previous polynomial.


  // Collect all the knots and breaks we've used so far.
  std::vector<double> total_breaks;
  std::vector<MatrixX<double>> total_knots;

  // Only need to put in knots from linear section if they are on either boundary of the beginning/end.
  if (start_matches_first_knot) total_breaks.push_back(linear_breaks[0]);
  total_breaks.insert(std::end(total_breaks), std::begin(breaks_before_cut), std::end(breaks_before_cut)); // Put all of the first poly's breaks in.
  total_breaks.insert(std::end(total_breaks), std::begin(breaks_after_cut), std::end(breaks_after_cut));
  if (end_matches_last_knot) total_breaks.push_back(linear_breaks[1]);

  if (start_matches_first_knot) total_knots.push_back(linear_knots.col(0));
  total_knots.insert(std::end(total_knots), std::begin(knots_before_cut), std::end(knots_before_cut));
  total_knots.insert(std::end(total_knots), std::begin(knots_after_cut), std::end(knots_after_cut));
  if (end_matches_last_knot) total_knots.push_back(linear_knots.col(1));


  poly_data.poly = *resulting_poly; // Now concatenated to include all 3 sections.
  poly_data.breaks = total_breaks;
  poly_data.knots = total_knots;
  return cut_end_t;
}

PolyWithKnots MakeFigure8Poly(Vector3d& offset, double time_scaling, double half_height, double half_width) {
  const double num_pts = 8;
  const double height = 0; // height of path parallel above the ground

  std::vector<double> breaks;
  std::vector<MatrixX<double>> knots;
  VectorXd pt1(3);
  pt1 << 0, 0, height;

  VectorXd pt2(3);
  pt2 << half_width, half_height/2, height;

  VectorXd pt3(3);
  pt3 << 0, half_height, height;

  VectorXd pt4(3);
  pt4 << -half_width, half_height/2, height;

  VectorXd pt5(3);
  pt5 << 0, 0, height;

  VectorXd pt6(3);
  pt6 << half_width, -half_height/2, height;

  VectorXd pt7(3);
  pt7 << 0, -half_height, height;

  VectorXd pt8(3);
  pt8 << -half_width, -half_height/2, height;


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

  PiecewisePolynomial<double> original_poly = PiecewisePolynomial<double>::Cubic(breaks, knots, true);

  PolyWithKnots figure8dat = {original_poly, breaks, knots};
  return figure8dat;
}

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

}
}
}