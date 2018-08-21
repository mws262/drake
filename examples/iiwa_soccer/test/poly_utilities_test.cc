#include <memory>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>
#include "drake/common/eigen_types.h"
#include "drake/examples/iiwa_soccer/poly_utilities.h"
#include "drake/common/test_utilities/expect_throws_message.h"


using std::make_unique;
using Eigen::Vector3d;
using Eigen::Vector2d;

namespace drake {
namespace examples {
namespace iiwa_soccer{

class PolyUtilitiesTest : public ::testing::Test {
 protected:
  Vector3d center_offset;
  const double time_scaling = 7;
  const double half_height = 3;
  const double half_width = 1;
  PolyWithKnots poly_data;

  void SetUp() override {
    center_offset << 1, 0.5, 1.2;
    poly_data = MakeFigure8Poly(center_offset, time_scaling, half_height, half_width);

  }
};

/** Make sure the first and last derivatives still match. Only use on something which is SUPPOSED to be periodic-ish. **/
void MatchBeginningEndDt(PiecewisePolynomial<double>& poly) {
  auto poly_dt = poly.MakeDerivative(1);
  Vector3d start_dt = poly_dt->value(poly_dt->start_time());
  Vector3d end_dt = poly_dt->value(poly_dt->end_time());
  EXPECT_NEAR(start_dt[0], end_dt[0], 1e-12);
  EXPECT_NEAR(start_dt[1], end_dt[1], 1e-12);
  EXPECT_NEAR(start_dt[2], end_dt[2], 1e-12);
}

// Test the figure-8 spline maker.
TEST_F(PolyUtilitiesTest, make_figure8_test) {

  VectorXd beginning_loc = poly_data.poly.value(0);
  VectorXd ending_loc = poly_data.poly.value(time_scaling);

  // Should have 1 more knot and break than the number of segments ("fencepost problem").
  EXPECT_EQ(poly_data.poly.get_number_of_segments() + 1, static_cast<int>(poly_data.breaks.size()));
  EXPECT_EQ(poly_data.poly.get_number_of_segments() + 1, static_cast<int>(poly_data.knots.size()));

  // Should always be a spline in 3d.
  EXPECT_EQ(beginning_loc.size(), 3);
  EXPECT_EQ(ending_loc.size(), 3);

  // 8 should begin and end at the same place.
  EXPECT_NEAR(beginning_loc[0], ending_loc[0], 1e-10);
  EXPECT_NEAR(beginning_loc[1], ending_loc[1], 1e-10);
  EXPECT_NEAR(beginning_loc[2], ending_loc[2], 1e-10);

  // Beginning (and end) should match the given center offset.
  EXPECT_NEAR(beginning_loc[0], center_offset[0], 1e-10);
  EXPECT_NEAR(beginning_loc[1], center_offset[1], 1e-10);
  EXPECT_NEAR(beginning_loc[2], center_offset[2], 1e-10);

  for (size_t i = 0; i < poly_data.breaks.size(); i++) {
    // Check knot dimensions.
    EXPECT_EQ(poly_data.knots[i].rows(), 3);
    EXPECT_EQ(poly_data.knots[i].cols(), 1);

    // Breaks should all lie within the time bounds of the piecewise polynomial.
    EXPECT_TRUE(poly_data.breaks[i] >= poly_data.poly.start_time());
    EXPECT_TRUE(poly_data.breaks[i] <= poly_data.poly.end_time());

    // Knot location as determined by the polynomial evaluated at the breaks should equal known knots.
    VectorXd knot_loc = poly_data.poly.value(poly_data.breaks[i]);
    EXPECT_NEAR(knot_loc[0], poly_data.knots[i].col(0)[0], 1e-10);
    EXPECT_NEAR(knot_loc[1], poly_data.knots[i].col(0)[1], 1e-10);
    EXPECT_NEAR(knot_loc[2], poly_data.knots[i].col(0)[2], 1e-10);
  }
}

// Remesh with only two points and identical other conditions. Should remain identical.
TEST_F(PolyUtilitiesTest, linear_segment_remesh_two_pts_identity) {
  Vector3d start_dt(7,1,2);
  Vector3d end_dt(-1,3,-4);
  Vector3d s1(0,0,0);
  Vector3d s2(5,6,7);
  double t1 = 0;
  double t2 = 4;

  PolyWithKnots cubic_poly_holder1;
  cubic_poly_holder1.knots.push_back(s1);
  cubic_poly_holder1.knots.push_back(s2);
  cubic_poly_holder1.breaks.push_back(t1);
  cubic_poly_holder1.breaks.push_back(t2);

  cubic_poly_holder1.poly = PiecewisePolynomial<double>::Cubic(cubic_poly_holder1.breaks, cubic_poly_holder1.knots, start_dt, end_dt);

  // Should return the same thing unchanged.
  PolyWithKnots remeshed_poly_holder = LinearSegmentPreservingCubicRemesh(cubic_poly_holder1,
                                                                          0,
                                                                          0,
                                                                          t1,
                                                                          t2,
                                                                          s1,
                                                                          s2,
                                                                          start_dt,
                                                                          end_dt,
                                                                          true,
                                                                          true);

  // Number of knots and breaks should still match.
  EXPECT_EQ(cubic_poly_holder1.breaks.size(), remeshed_poly_holder.breaks.size());
  EXPECT_EQ(cubic_poly_holder1.knots.size(), remeshed_poly_holder.knots.size());
  EXPECT_EQ(cubic_poly_holder1.knots.size(), 2);


  // Start and end times should match.
  EXPECT_NEAR(cubic_poly_holder1.poly.start_time(), remeshed_poly_holder.poly.start_time(), 1e-12);
  EXPECT_NEAR(cubic_poly_holder1.poly.end_time(), remeshed_poly_holder.poly.end_time(), 1e-12);

  // Breaks and knots should match.
  for (double i = 0; i < cubic_poly_holder1.breaks.size(); i++) {
    EXPECT_NEAR(cubic_poly_holder1.breaks[i], remeshed_poly_holder.breaks[i], 1e-12);

    EXPECT_NEAR(cubic_poly_holder1.knots[i](0), remeshed_poly_holder.knots[i](0), 1e-12);
    EXPECT_NEAR(cubic_poly_holder1.knots[i](1), remeshed_poly_holder.knots[i](1), 1e-12);
    EXPECT_NEAR(cubic_poly_holder1.knots[i](2), remeshed_poly_holder.knots[i](2), 1e-12);
  }

  // Check points densely along the spline.
  for (double i = t1; i < t2; i += 0.01) {
    Vector3d old_pos = cubic_poly_holder1.poly.value(i);
    Vector3d new_pos = remeshed_poly_holder.poly.value(i);
    EXPECT_NEAR(new_pos[0], old_pos[0], 1e-12);
    EXPECT_NEAR(new_pos[1], old_pos[1], 1e-12);
    EXPECT_NEAR(new_pos[2], old_pos[2], 1e-12);
  }
}

// Remesh with only two points but different output conditions.
TEST_F(PolyUtilitiesTest, linear_segment_remesh_two_pts_different) {
  Vector3d start_dt(7,1,2);
  Vector3d end_dt(-1,3,-4);
  Vector3d s1(0,0,0);
  Vector3d s2(5,6,7);
  double t1 = 0;
  double t2 = 4;

  PolyWithKnots cubic_poly_holder1;
  cubic_poly_holder1.knots.push_back(s1);
  cubic_poly_holder1.knots.push_back(s2);
  cubic_poly_holder1.breaks.push_back(t1);
  cubic_poly_holder1.breaks.push_back(t2);

  cubic_poly_holder1.poly = PiecewisePolynomial<double>::Cubic(cubic_poly_holder1.breaks, cubic_poly_holder1.knots, start_dt, end_dt);


  // New conditions.
  Vector3d start_dt_new(4,10,-6);
  Vector3d end_dt_new(-4,8,3);
  Vector3d s1_new(6,3,0);
  Vector3d s2_new(-7,0,3);
  double t1_new = 3;
  double t2_new = 8.8;

  PolyWithKnots remeshed_poly_holder = LinearSegmentPreservingCubicRemesh(cubic_poly_holder1,
                                                                          0,
                                                                          0,
                                                                          t1_new,
                                                                          t2_new,
                                                                          s1_new,
                                                                          s2_new,
                                                                          start_dt_new,
                                                                          end_dt_new,
                                                                          true,
                                                                          true);

  // Number of knots and breaks should still match.
  EXPECT_EQ(cubic_poly_holder1.breaks.size(), remeshed_poly_holder.breaks.size());
  EXPECT_EQ(cubic_poly_holder1.knots.size(), remeshed_poly_holder.knots.size());
  EXPECT_EQ(cubic_poly_holder1.knots.size(), 2);

  // Start and end times are the new ones.
  EXPECT_NEAR(t1_new, remeshed_poly_holder.poly.start_time(), 1e-12);
  EXPECT_NEAR(t2_new, remeshed_poly_holder.poly.end_time(), 1e-12);

  // Breaks and knots are new ones.
  EXPECT_NEAR(t1_new, remeshed_poly_holder.breaks[0], 1e-12);
  EXPECT_NEAR(t2_new, remeshed_poly_holder.breaks[1], 1e-12);

  EXPECT_NEAR(s1_new[0], remeshed_poly_holder.knots[0](0), 1e-12);
  EXPECT_NEAR(s1_new[1], remeshed_poly_holder.knots[0](1), 1e-12);
  EXPECT_NEAR(s1_new[2], remeshed_poly_holder.knots[0](2), 1e-12);

  EXPECT_NEAR(s2_new[0], remeshed_poly_holder.knots[1](0), 1e-12);
  EXPECT_NEAR(s2_new[1], remeshed_poly_holder.knots[1](1), 1e-12);
  EXPECT_NEAR(s2_new[2], remeshed_poly_holder.knots[1](2), 1e-12);


}

// Test the linear-section-preserving capabilities without points added at the beginning/end.
TEST_F(PolyUtilitiesTest, linear_segment_remesh_no_added_pts) {

  // Make a cubic spline, followed by a linear piecewise polynomialic spline.
  Vector3d start_dt(7,1,2);
  Vector3d cut_dt(1,1,1);
  Vector3d end_dt(-1,3,-4);
  Vector3d s1(0,0,0);
  Vector3d s2(5,6,7);
  Vector3d s3(1,2,1);

  double t1, t2, t3, t4, t5;
  t1 = 0;
  t2 = 1;
  t3 = 2;
  t4 = 3;
  t5 = 6;

  PolyWithKnots cubic_poly_holder1;
  cubic_poly_holder1.knots.push_back(s1);
  cubic_poly_holder1.knots.push_back(s2);
  cubic_poly_holder1.knots.push_back(s3);
  cubic_poly_holder1.breaks.push_back(t1);
  cubic_poly_holder1.breaks.push_back(t2);
  cubic_poly_holder1.breaks.push_back(t3);

  cubic_poly_holder1.poly = PiecewisePolynomial<double>::Cubic(cubic_poly_holder1.breaks, cubic_poly_holder1.knots, start_dt, cut_dt);

  Vector3d l1(1,2,1);
  Vector3d l2(2,3,2);
  Vector2d lin_breaks(t3,t4);

  MatrixXd lin_knots(3,2);
  lin_knots.col(0) = l1;
  lin_knots.col(1) = l2;

  PiecewisePolynomial<double> lin_poly = PiecewisePolynomial<double>::FirstOrderHold(lin_breaks, lin_knots);


  Vector3d s4(2,3,2);
  Vector3d s5(9,12,5);
  PolyWithKnots cubic_poly_holder2;
  cubic_poly_holder2.knots.push_back(s4);
  cubic_poly_holder2.knots.push_back(s5);
  cubic_poly_holder2.breaks.push_back(t4);
  cubic_poly_holder2.breaks.push_back(t5);

  cubic_poly_holder2.poly = PiecewisePolynomial<double>::Cubic(cubic_poly_holder2.breaks, cubic_poly_holder2.knots, cut_dt, end_dt);


  PolyWithKnots combined_poly_holder;
  cubic_poly_holder1.poly.ConcatenateInTime(lin_poly);
  cubic_poly_holder1.poly.ConcatenateInTime(cubic_poly_holder2.poly);
  combined_poly_holder.poly = cubic_poly_holder1.poly;

  combined_poly_holder.knots.push_back(s1);
  combined_poly_holder.knots.push_back(s2);
  combined_poly_holder.knots.push_back(s3);
  combined_poly_holder.knots.push_back(s4);
  combined_poly_holder.knots.push_back(s5);
  combined_poly_holder.breaks.push_back(t1);
  combined_poly_holder.breaks.push_back(t2);
  combined_poly_holder.breaks.push_back(t3);
  combined_poly_holder.breaks.push_back(t4);
  combined_poly_holder.breaks.push_back(t5);

  Vector3d new_start_d(4,2,7);
  Vector3d new_end_d(1,-4,3);
  const int remeshed_start_seg = 0;
  const int remeshed_end_seg = 3;
  const double remeshed_start_t = t1;
  const double remeshed_end_t = t5;
  Vector3d remeshed_start_p = s1;
  Vector3d remeshed_end_p = s5;

  PolyWithKnots remeshed_poly_holder = LinearSegmentPreservingCubicRemesh(combined_poly_holder,
                                                                          remeshed_start_seg,
                                                                          remeshed_end_seg,
                                                                          remeshed_start_t,
                                                                          remeshed_end_t,
                                                                          remeshed_start_p,
                                                                          remeshed_end_p,
                                                                          new_start_d,
                                                                          new_end_d,
                                                                          true,
                                                                          true);

  // Number of knots preserved.
  EXPECT_EQ(static_cast<int>(remeshed_poly_holder.breaks.size()), 5);
  EXPECT_EQ(static_cast<int>(remeshed_poly_holder.knots.size()), 5);

  // Knots match knots received when evaluating the new piecewise polynomial.
  for (int i = 0; i < combined_poly_holder.poly.get_number_of_segments(); i++) {
    Vector3d eval_knot = remeshed_poly_holder.poly.value(remeshed_poly_holder.breaks[i]);
    // Old knots
    EXPECT_NEAR(eval_knot[0], combined_poly_holder.knots[i](0), 1e-14);
    EXPECT_NEAR(eval_knot[1], combined_poly_holder.knots[i](1), 1e-14);
    EXPECT_NEAR(eval_knot[2], combined_poly_holder.knots[i](2), 1e-14);

    // New knots
    EXPECT_NEAR(eval_knot[0], remeshed_poly_holder.knots[i](0), 1e-14);
    EXPECT_NEAR(eval_knot[1], remeshed_poly_holder.knots[i](1), 1e-14);
    EXPECT_NEAR(eval_knot[2], remeshed_poly_holder.knots[i](2), 1e-14);

  }

  // Start/end times preserved.
  const double remeshed_start_time = remeshed_poly_holder.poly.start_time();
  const double remeshed_end_time = remeshed_poly_holder.poly.end_time();

  EXPECT_EQ(remeshed_start_time, remeshed_start_t);
  EXPECT_EQ(remeshed_end_time, remeshed_end_t);

  // Check that new derivative conditions are met.
  std::unique_ptr<Trajectory<double>> remeshed_poly_dt = remeshed_poly_holder.poly.MakeDerivative(1);
  const Vector3d start_dt_error_remeshed = remeshed_poly_dt->value(remeshed_start_time) - new_start_d;
  const Vector3d end_dt_error_remeshed = remeshed_poly_dt->value(remeshed_end_time) - new_end_d;

  EXPECT_NEAR(start_dt_error_remeshed[0], 0, 1e-14);
  EXPECT_NEAR(start_dt_error_remeshed[1], 0, 1e-14);
  EXPECT_NEAR(start_dt_error_remeshed[2], 0, 1e-14);

  EXPECT_NEAR(end_dt_error_remeshed[0], 0, 1e-14);
  EXPECT_NEAR(end_dt_error_remeshed[1], 0, 1e-14);
  EXPECT_NEAR(end_dt_error_remeshed[2], 0, 1e-14);

  // Check that the linear section stayed linear in all 3 dimensions.
  EXPECT_EQ(remeshed_poly_holder.poly.getPolynomialMatrix(2)(0).GetDegree(), 1);
  EXPECT_EQ(remeshed_poly_holder.poly.getPolynomialMatrix(2)(1).GetDegree(), 1);
  EXPECT_EQ(remeshed_poly_holder.poly.getPolynomialMatrix(2)(2).GetDegree(), 1);

  /** Move boundaries and resize. **/
  // Also resize. Should now have 1 poly section before, 1 linear section, 1 poly section after.
  const int remeshed_resized_start_seg = 1;
  const int remeshed_resized_end_seg = 3;
  const double remeshed_resized_start_t = 0.5; // Change timings.
  const double remeshed_resized_end_t = 7;
  Vector3d remeshed_resized_start_p(4,3,9); // Change start positions too.
  Vector3d remeshed_resized_end_p(-3,8,-1);

  PolyWithKnots remeshed_resized_poly_holder = LinearSegmentPreservingCubicRemesh(combined_poly_holder,
                                                                                  remeshed_resized_start_seg,
                                                                                  remeshed_resized_end_seg,
                                                                                  remeshed_resized_start_t,
                                                                                  remeshed_resized_end_t,
                                                                                  remeshed_resized_start_p,
                                                                                  remeshed_resized_end_p,
                                                                                  new_start_d,
                                                                                  new_end_d,
                                                                                  true,
                                                                                  true);

  EXPECT_EQ(static_cast<int>(remeshed_resized_poly_holder.breaks.size()), 4);
  EXPECT_EQ(static_cast<int>(remeshed_resized_poly_holder.knots.size()), 4);

  // Knots in struct match knots received when evaluating the piecewise polynomial.
  for (int i = 0; i < remeshed_resized_poly_holder.poly.get_number_of_segments(); i++) {
    Vector3d eval_knot = remeshed_resized_poly_holder.poly.value(remeshed_resized_poly_holder.breaks[i]);
    // New structure is consistent.
    EXPECT_NEAR(eval_knot[0], remeshed_resized_poly_holder.knots[i](0), 1e-14);
    EXPECT_NEAR(eval_knot[1], remeshed_resized_poly_holder.knots[i](1), 1e-14);
    EXPECT_NEAR(eval_knot[2], remeshed_resized_poly_holder.knots[i](2), 1e-14);
  }

  // Knots match old EXCEPT for at boundaries.
  for (int i = 1; i < remeshed_resized_poly_holder.poly.get_number_of_segments() - 1; i++) {
    Vector3d eval_knot = remeshed_resized_poly_holder.poly.value(remeshed_resized_poly_holder.breaks[i]);
    EXPECT_NEAR(eval_knot[0], combined_poly_holder.knots[i + 1](0), 1e-14);
    EXPECT_NEAR(eval_knot[1], combined_poly_holder.knots[i + 1](1), 1e-14);
    EXPECT_NEAR(eval_knot[2], combined_poly_holder.knots[i + 1](2), 1e-14);
  }
  // Start/end times correct.
  const double remeshed_resized_start_time = remeshed_resized_poly_holder.poly.start_time();
  const double remeshed_resized_end_time = remeshed_resized_poly_holder.poly.end_time();

  EXPECT_EQ(remeshed_resized_start_time, remeshed_resized_start_t);
  EXPECT_EQ(remeshed_resized_end_time, remeshed_resized_end_t);

  // Check that new derivative conditions are met.
  std::unique_ptr<Trajectory<double>> remeshed_resized_poly_dt = remeshed_resized_poly_holder.poly.MakeDerivative(1);
  const Vector3d start_dt_error_remeshed_resized = remeshed_resized_poly_dt->value(remeshed_resized_start_time) - new_start_d;
  const Vector3d end_dt_error_remeshed_resized = remeshed_resized_poly_dt->value(remeshed_resized_end_time) - new_end_d;

  EXPECT_NEAR(start_dt_error_remeshed_resized[0], 0, 1e-14);
  EXPECT_NEAR(start_dt_error_remeshed_resized[1], 0, 1e-14);
  EXPECT_NEAR(start_dt_error_remeshed_resized[2], 0, 1e-14);

  EXPECT_NEAR(end_dt_error_remeshed_resized[0], 0, 1e-14);
  EXPECT_NEAR(end_dt_error_remeshed_resized[1], 0, 1e-14);
  EXPECT_NEAR(end_dt_error_remeshed_resized[2], 0, 1e-14);

  // Check that the linear section stayed linear in all 3 dimensions.
  EXPECT_EQ(remeshed_resized_poly_holder.poly.getPolynomialMatrix(1)(0).GetDegree(), 1);
  EXPECT_EQ(remeshed_resized_poly_holder.poly.getPolynomialMatrix(1)(1).GetDegree(), 1);
  EXPECT_EQ(remeshed_resized_poly_holder.poly.getPolynomialMatrix(1)(2).GetDegree(), 1);

  // Make sure the old poly didn't change (should be const, but I'm paranoid).
  std::unique_ptr<Trajectory<double>> old_poly_dt = combined_poly_holder.poly.MakeDerivative(1);
  const Vector3d start_dt_error_original = old_poly_dt->value(t1) - start_dt;
  const Vector3d end_dt_error_original = old_poly_dt->value(t5) - end_dt;

  EXPECT_NEAR(start_dt_error_original[0], 0, 1e-14);
  EXPECT_NEAR(start_dt_error_original[1], 0, 1e-14);
  EXPECT_NEAR(start_dt_error_original[2], 0, 1e-14);

  EXPECT_NEAR(end_dt_error_original[0], 0, 1e-14);
  EXPECT_NEAR(end_dt_error_original[1], 0, 1e-14);
  EXPECT_NEAR(end_dt_error_original[2], 0, 1e-14);

}

// Test the linear-section-preserving capabilities with points added at the beginning/end (general case).
TEST_F(PolyUtilitiesTest, linear_segment_remesh_multiple_linear) {

  // Make a cubic spline, followed by a linear piecewise polynomial, followed by another cubic spline.
  Vector3d start_dt(3,-1,2);
  Vector3d end_dt(-1,3,-4);
  Vector3d s1(0,0,0);
  Vector3d s2(5,6,-7);
  Vector3d s3(-1,2,1); // linear 1 start
  Vector3d s4(3, 4, 5);
  Vector3d s5(9,-12,5);
  Vector3d s6(5, 5, 8); // linear2 start
  Vector3d s7(-4, 2, 7);
  Vector3d s8(-1, -2, 6);

  double t1, t2, t3, t4, t5, t6, t7, t8;
  t1 = 0.5;
  t2 = 1;
  t3 = 2;
  t4 = 3;
  t5 = 6;
  t6 = 8;
  t7 = 9;
  t8 = 10;

  PolyWithKnots cubic_poly_holder1;
  cubic_poly_holder1.knots.push_back(s1);
  cubic_poly_holder1.knots.push_back(s2);
  cubic_poly_holder1.knots.push_back(s3);
  cubic_poly_holder1.breaks.push_back(t1);
  cubic_poly_holder1.breaks.push_back(t2);
  cubic_poly_holder1.breaks.push_back(t3);

  Vector3d slope_l1((s4 - s3)/(t4 - t3));
  cubic_poly_holder1.poly = PiecewisePolynomial<double>::Cubic(cubic_poly_holder1.breaks, cubic_poly_holder1.knots, start_dt, slope_l1);

  Vector2d lin_breaks1(t3,t4);

  MatrixXd lin_knots1(3,2);
  lin_knots1.col(0) = s3;
  lin_knots1.col(1) = s4;

  PiecewisePolynomial<double> lin_poly1 = PiecewisePolynomial<double>::FirstOrderHold(lin_breaks1, lin_knots1);

  PolyWithKnots cubic_poly_holder2;
  cubic_poly_holder2.knots.push_back(s4);
  cubic_poly_holder2.knots.push_back(s5);
  cubic_poly_holder2.breaks.push_back(t4);
  cubic_poly_holder2.breaks.push_back(t5);

  Vector3d slope_l2((s6 - s5)/(t6 - t5));
  cubic_poly_holder2.poly = PiecewisePolynomial<double>::Cubic(cubic_poly_holder2.breaks, cubic_poly_holder2.knots, slope_l1, slope_l2);

  Vector2d lin_breaks2(t5,t6);
  MatrixXd lin_knots2(3,2);
  lin_knots2.col(0) = s5;
  lin_knots2.col(1) = s6;

  PiecewisePolynomial<double> lin_poly2 = PiecewisePolynomial<double>::FirstOrderHold(lin_breaks2, lin_knots2);

  PolyWithKnots cubic_poly_holder3;
  cubic_poly_holder3.knots.push_back(s7);
  cubic_poly_holder3.knots.push_back(s8);
  cubic_poly_holder3.breaks.push_back(t7);
  cubic_poly_holder3.breaks.push_back(t8);

  cubic_poly_holder3.poly = PiecewisePolynomial<double>::Cubic(cubic_poly_holder3.breaks, cubic_poly_holder3.knots, slope_l2, end_dt);


  PolyWithKnots combined_poly_holder;
  cubic_poly_holder1.poly.ConcatenateInTime(lin_poly1);
  cubic_poly_holder1.poly.ConcatenateInTime(cubic_poly_holder2.poly);
  cubic_poly_holder1.poly.ConcatenateInTime(lin_poly2);
  combined_poly_holder.poly = cubic_poly_holder1.poly;

  combined_poly_holder.knots.push_back(s1);
  combined_poly_holder.knots.push_back(s2);
  combined_poly_holder.knots.push_back(s3);
  combined_poly_holder.knots.push_back(s4);
  combined_poly_holder.knots.push_back(s5);
  combined_poly_holder.knots.push_back(s6);
  combined_poly_holder.breaks.push_back(t1);
  combined_poly_holder.breaks.push_back(t2);
  combined_poly_holder.breaks.push_back(t3);
  combined_poly_holder.breaks.push_back(t4);
  combined_poly_holder.breaks.push_back(t5);
  combined_poly_holder.breaks.push_back(t6);

  auto combined_poly_dt = combined_poly_holder.poly.MakeDerivative(1);

  const Vector3d start_seg1(1,1,1);
  const Vector3d end_seg1(-9, 4, 1);
  const Vector3d start_segd1(1,1,1);
  const Vector3d end_segd1(-9, 4, 1);
  const double start_t1 = 0.2;
  const double end_t1 = 10;

  PolyWithKnots added_end_poly_holder = LinearSegmentPreservingCubicRemesh(combined_poly_holder,
                                                                           0,
                                                                           combined_poly_holder.knots.size() - 2,
                                                                           start_t1,
                                                                           end_t1,
                                                                           start_seg1,
                                                                           end_seg1,
                                                                           start_segd1,
                                                                           end_segd1,
                                                                           true,
                                                                           false);

  // Add nodes on both sides.
  PolyWithKnots added_ends_poly_holder = LinearSegmentPreservingCubicRemesh(combined_poly_holder,
                                                                            0,
                                                                            combined_poly_holder.knots.size() - 2,
                                                                            start_t1,
                                                                            end_t1,
                                                                            start_seg1,
                                                                            end_seg1,
                                                                            start_segd1,
                                                                            end_segd1,
                                                                            false,
                                                                            false);
  
  std::vector<PolyWithKnots> polys_to_test;
  polys_to_test.push_back(added_end_poly_holder);
  polys_to_test.push_back(added_ends_poly_holder);

  int front_offset = 0;
  for (PolyWithKnots poly_holder : polys_to_test) {

    // We should have added extra knot(s)
    EXPECT_EQ(poly_holder.breaks.size(), combined_poly_holder.breaks.size() + 1 + front_offset);

    // Consistent dimensions
    EXPECT_EQ(poly_holder.breaks.size(), poly_holder.knots.size());
    EXPECT_EQ(poly_holder.breaks.size(), poly_holder.poly.get_number_of_segments() + 1);

    // End positions met.
    Vector3d eval_at_beginning = poly_holder.poly.value(start_t1);
    Vector3d eval_at_end = poly_holder.poly.value(end_t1);
    EXPECT_NEAR(eval_at_beginning[0], start_seg1[0], 1e-14);
    EXPECT_NEAR(eval_at_beginning[1], start_seg1[1], 1e-14);
    EXPECT_NEAR(eval_at_beginning[2], start_seg1[2], 1e-14);

    EXPECT_NEAR(eval_at_end[0], end_seg1[0], 1e-14);
    EXPECT_NEAR(eval_at_end[1], end_seg1[1], 1e-14);
    EXPECT_NEAR(eval_at_end[2], end_seg1[2], 1e-14);

    // End derivatives met.
    auto added_end_poly_dt = poly_holder.poly.MakeDerivative(1);
    Vector3d dt_eval_at_beginning = added_end_poly_dt->value(start_t1);
    Vector3d dt_eval_at_end = added_end_poly_dt->value(end_t1);

    EXPECT_NEAR(dt_eval_at_beginning[0], start_segd1[0], 1e-13);
    EXPECT_NEAR(dt_eval_at_beginning[1], start_segd1[1], 1e-13);
    EXPECT_NEAR(dt_eval_at_beginning[2], start_segd1[2], 1e-13);

    EXPECT_NEAR(dt_eval_at_end[0], end_segd1[0], 1e-13);
    EXPECT_NEAR(dt_eval_at_end[1], end_segd1[1], 1e-13);
    EXPECT_NEAR(dt_eval_at_end[2], end_segd1[2], 1e-13);

    // Make sure linear sections are still linear!
    // Check that the linear section stayed linear in all 3 dimensions.
    EXPECT_EQ(poly_holder.poly.getPolynomialMatrix(2 + front_offset)(0).GetDegree(), 1);
    EXPECT_EQ(poly_holder.poly.getPolynomialMatrix(2 + front_offset)(1).GetDegree(), 1);
    EXPECT_EQ(poly_holder.poly.getPolynomialMatrix(2 + front_offset)(2).GetDegree(), 1);

    EXPECT_EQ(poly_holder.poly.getPolynomialMatrix(4 + front_offset)(0).GetDegree(), 1);
    EXPECT_EQ(poly_holder.poly.getPolynomialMatrix(4 + front_offset)(1).GetDegree(), 1);
    EXPECT_EQ(poly_holder.poly.getPolynomialMatrix(4 + front_offset)(2).GetDegree(), 1);

    Vector3d dt_eval_at_linear1 = added_end_poly_dt->value(poly_holder.breaks[2 + front_offset]);
    Vector3d dt_eval_at_linear2 = added_end_poly_dt->value(poly_holder.breaks[4 + front_offset]);

    EXPECT_NEAR(dt_eval_at_linear1[0], slope_l1[0], 1e-13);
    EXPECT_NEAR(dt_eval_at_linear1[1], slope_l1[1], 1e-13);
    EXPECT_NEAR(dt_eval_at_linear1[2], slope_l1[2], 1e-13);

    EXPECT_NEAR(dt_eval_at_linear2[0], slope_l2[0], 1e-13);
    EXPECT_NEAR(dt_eval_at_linear2[1], slope_l2[1], 1e-13);
    EXPECT_NEAR(dt_eval_at_linear2[2], slope_l2[2], 1e-13);
    front_offset++;
  }


}

// Test the piecewise poly slicer.
TEST_F(PolyUtilitiesTest, slice_figure8) {

  double threshold = 15; // Threshold below which something is cut.
  int cut_attempts = 7;

  for (int i = 0; i < cut_attempts; i++) {
    SplitPolyInPlace(poly_data, 0, threshold);
    MatchBeginningEndDt(poly_data.poly);
  }

  // First and last dts must match.
  auto poly_dt = poly_data.poly.MakeDerivative(1);
  // Super-hacky way of checking if derivatives are continuous. I can't think of another way.
  Vector3d prev_dt = poly_dt->value(poly_dt->start_time());
  Vector3d curr_dt = poly_dt->value(poly_dt->start_time());
  for (double i = poly_dt->start_time(); i < poly_dt->end_time(); i += 0.01) {
    curr_dt = poly_dt->value(i);

    EXPECT_NEAR(curr_dt[0], prev_dt[0], 2);
    EXPECT_NEAR(curr_dt[1], prev_dt[1], 2);
    EXPECT_NEAR(curr_dt[2], prev_dt[2], 2);
    prev_dt = curr_dt;
  }


  // Draw path markers
  lcm::DrakeLcm lcm;
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
  PublishFrames(poses_to_draw, pose_names, lcm);

}

/** Test exceptions. **/
TEST_F(PolyUtilitiesTest, test_exceptions) {


  // Make a cubic spline, followed by a linear piecewise polynomial, followed by another cubic spline.
  Vector3d start_dt(3,-1,2);
  Vector3d end_dt(-1,3,-4);
  Vector3d s1(0,0,0);
  Vector3d s2(5,6,-7);
  Vector3d s3(-1,2,1); // linear 1 start
  Vector3d s4(3, 4, 5);
  Vector3d s5(9,-12,5);
  Vector3d s6(5, 5, 8); // linear2 start
  Vector3d s7(-4, 2, 7);
  Vector3d s8(-1, -2, 6);

  double t1, t2, t3, t4, t5, t6, t7, t8;
  t1 = 0.5;
  t2 = 1;
  t3 = 2;
  t4 = 3;
  t5 = 6;
  t6 = 8;
  t7 = 9;
  t8 = 10;

  PolyWithKnots cubic_poly_holder1;
  cubic_poly_holder1.knots.push_back(s1);
  cubic_poly_holder1.knots.push_back(s2);
  cubic_poly_holder1.knots.push_back(s3);
  cubic_poly_holder1.breaks.push_back(t1);
  cubic_poly_holder1.breaks.push_back(t2);
  cubic_poly_holder1.breaks.push_back(t3);

  Vector3d slope_l1((s4 - s3)/(t4 - t3));
  cubic_poly_holder1.poly = PiecewisePolynomial<double>::Cubic(cubic_poly_holder1.breaks, cubic_poly_holder1.knots, start_dt, slope_l1);

  Vector2d lin_breaks1(t3,t4);

  MatrixXd lin_knots1(3,2);
  lin_knots1.col(0) = s3;
  lin_knots1.col(1) = s4;

  PiecewisePolynomial<double> lin_poly1 = PiecewisePolynomial<double>::FirstOrderHold(lin_breaks1, lin_knots1);

  PolyWithKnots cubic_poly_holder2;
  cubic_poly_holder2.knots.push_back(s4);
  cubic_poly_holder2.knots.push_back(s5);
  cubic_poly_holder2.breaks.push_back(t4);
  cubic_poly_holder2.breaks.push_back(t5);

  Vector3d slope_l2((s6 - s5)/(t6 - t5));
  cubic_poly_holder2.poly = PiecewisePolynomial<double>::Cubic(cubic_poly_holder2.breaks, cubic_poly_holder2.knots, slope_l1, slope_l2);

  Vector2d lin_breaks2(t5,t6);
  MatrixXd lin_knots2(3,2);
  lin_knots2.col(0) = s5;
  lin_knots2.col(1) = s6;

  PiecewisePolynomial<double> lin_poly2 = PiecewisePolynomial<double>::FirstOrderHold(lin_breaks2, lin_knots2);

  PolyWithKnots cubic_poly_holder3;
  cubic_poly_holder3.knots.push_back(s7);
  cubic_poly_holder3.knots.push_back(s8);
  cubic_poly_holder3.breaks.push_back(t7);
  cubic_poly_holder3.breaks.push_back(t8);

  cubic_poly_holder3.poly = PiecewisePolynomial<double>::Cubic(cubic_poly_holder3.breaks, cubic_poly_holder3.knots, slope_l2, end_dt);


  PolyWithKnots combined_poly_holder;
  cubic_poly_holder1.poly.ConcatenateInTime(lin_poly1);
  cubic_poly_holder1.poly.ConcatenateInTime(cubic_poly_holder2.poly);
  cubic_poly_holder1.poly.ConcatenateInTime(lin_poly2);
  combined_poly_holder.poly = cubic_poly_holder1.poly;

  combined_poly_holder.knots.push_back(s1);
  combined_poly_holder.knots.push_back(s2);
  combined_poly_holder.knots.push_back(s3);
  combined_poly_holder.knots.push_back(s4);
  combined_poly_holder.knots.push_back(s5);
  combined_poly_holder.knots.push_back(s6);
  combined_poly_holder.breaks.push_back(t1);
  combined_poly_holder.breaks.push_back(t2);
  combined_poly_holder.breaks.push_back(t3);
  combined_poly_holder.breaks.push_back(t4);
  combined_poly_holder.breaks.push_back(t5);
  combined_poly_holder.breaks.push_back(t6);

  auto combined_poly_dt = combined_poly_holder.poly.MakeDerivative(1);

  Vector3d p1(4, -1, 4);
  Vector3d p2(-1, -5, -3);

  Vector3d d1(4, -1, 4);
  Vector3d d2(-1, -5, -3);

  int max_cut_idx = combined_poly_holder.poly.get_number_of_segments() - 1;

  // This should be ok!
  EXPECT_NO_THROW(LinearSegmentPreservingCubicRemesh(combined_poly_holder,
                                                     0,
                                                     max_cut_idx,
                                                     combined_poly_holder.poly.start_time(),
                                                     combined_poly_holder.poly.end_time(),
                                                     p1,
                                                     p2,
                                                     d1,
                                                     d2,
                                                     true,
                                                     true));

  // Bad start segment (>0)
  DRAKE_EXPECT_THROWS_MESSAGE(LinearSegmentPreservingCubicRemesh(combined_poly_holder,
                                                      -1,
                                                      max_cut_idx + 1,
                                                      combined_poly_holder.poly.start_time(),
                                                      combined_poly_holder.poly.end_time(),
                                                      p1,
                                                      p2,
                                                      d1,
                                                      d2,
                                                      true,
                                                      true), std::runtime_error, "BAD_FIRST_IDX_LESS_THAN_0.*");
  // Bad sequencing (start seg > end seg
  DRAKE_EXPECT_THROWS_MESSAGE(LinearSegmentPreservingCubicRemesh(combined_poly_holder,
                                                      3,
                                                      2,
                                                      combined_poly_holder.poly.start_time(),
                                                      combined_poly_holder.poly.end_time(),
                                                      p1,
                                                      p2,
                                                      d1,
                                                      d2,
                                                      true,
                                                      true), std::runtime_error, "BAD_SEGMENT_IDX_ORDER.*");

  // Bad segment indices (i.e. no segment index 4).
  DRAKE_EXPECT_THROWS_MESSAGE(LinearSegmentPreservingCubicRemesh(combined_poly_holder,
                                                      0,
                                                      max_cut_idx + 1,
                                                      combined_poly_holder.poly.start_time(),
                                                      combined_poly_holder.poly.end_time(),
                                                      p1,
                                                      p2,
                                                      d1,
                                                      d2,
                                                      true,
                                                      true), std::runtime_error, "BAD_END_IDX_TOO_BIG.*");

  // Bad start time (i.e. greater than next point).
  DRAKE_EXPECT_THROWS_MESSAGE(LinearSegmentPreservingCubicRemesh(combined_poly_holder,
                                                      2,
                                                      max_cut_idx - 1,
                                                      combined_poly_holder.breaks[3],
                                                      combined_poly_holder.breaks[max_cut_idx],
                                                      p1,
                                                      p2,
                                                      d1,
                                                      d2,
                                                      true,
                                                      true), std::runtime_error, "BAD_START_TIME_TOO_BIG_AS_REPLACEMENT.*");

  // OK start time (i.e. greater than first point but not second with replacing the first pt.).
  EXPECT_NO_THROW(LinearSegmentPreservingCubicRemesh(combined_poly_holder,
                                                     2,
                                                     max_cut_idx - 1,
                                                     (combined_poly_holder.breaks[2] + combined_poly_holder.breaks[3])/2,
                                                     combined_poly_holder.breaks[max_cut_idx],
                                                     p1,
                                                     p2,
                                                     d1,
                                                     d2,
                                                     true,
                                                     true));

  // Bad start time (i.e. greater than first point and trying to insert new point).
  DRAKE_EXPECT_THROWS_MESSAGE(LinearSegmentPreservingCubicRemesh(combined_poly_holder,
                                                      2,
                                                      max_cut_idx - 1,
                                                      (combined_poly_holder.breaks[2] + combined_poly_holder.breaks[3])/2,
                                                      combined_poly_holder.breaks[max_cut_idx],
                                                      p1,
                                                      p2,
                                                      d1,
                                                      d2,
                                                      false,
                                                      true), std::runtime_error, "BAD_START_TIME_TOO_BIG_AS_INSERTION.*");

  // OK start time (less than first break).
  EXPECT_NO_THROW(LinearSegmentPreservingCubicRemesh(combined_poly_holder,
                                                     2,
                                                     max_cut_idx - 1,
                                                     combined_poly_holder.breaks[2] - 0.2,
                                                     combined_poly_holder.breaks[max_cut_idx],
                                                     p1,
                                                     p2,
                                                     d1,
                                                     d2,
                                                     false,
                                                     true));

  // Bad end time (i.e. less than next-to-last point).
  DRAKE_EXPECT_THROWS_MESSAGE(LinearSegmentPreservingCubicRemesh(combined_poly_holder,
                                                      2,
                                                      max_cut_idx - 1,
                                                      combined_poly_holder.breaks[2],
                                                      combined_poly_holder.breaks[max_cut_idx - 1] - 0.2,
                                                      p1,
                                                      p2,
                                                      d1,
                                                      d2,
                                                      true,
                                                      true), std::runtime_error, "BAD_END_TIME_TOO_SMALL_AS_REPLACEMENT.*");

  // OK end time (i.e. inside last segment, replacing last).
  EXPECT_NO_THROW(LinearSegmentPreservingCubicRemesh(combined_poly_holder,
                                                     2,
                                                     max_cut_idx - 1,
                                                     combined_poly_holder.breaks[2],
                                                     combined_poly_holder.breaks[max_cut_idx - 1] + 0.2,
                                                     p1,
                                                     p2,
                                                     d1,
                                                     d2,
                                                     true,
                                                     true));

  // Bad end time (i.e. inside last segment, trying to add a pt).
  DRAKE_EXPECT_THROWS_MESSAGE(LinearSegmentPreservingCubicRemesh(combined_poly_holder,
                                                      2,
                                                      max_cut_idx - 1,
                                                      combined_poly_holder.breaks[2],
                                                      combined_poly_holder.breaks[max_cut_idx - 1] + 0.2,
                                                      p1,
                                                      p2,
                                                      d1,
                                                      d2,
                                                      true,
                                                      false), std::runtime_error, "BAD_END_TIME_TOO_BIG_AS_INSERTION.*");

  // OK end time (i.e. beyond last segment, added).
  EXPECT_NO_THROW(LinearSegmentPreservingCubicRemesh(combined_poly_holder,
                                                     2,
                                                     max_cut_idx - 1,
                                                     combined_poly_holder.breaks[2],
                                                     combined_poly_holder.breaks[max_cut_idx] + 0.5,
                                                     p1,
                                                     p2,
                                                     d1,
                                                     d2,
                                                     true,
                                                     false));
}

}
}
}
