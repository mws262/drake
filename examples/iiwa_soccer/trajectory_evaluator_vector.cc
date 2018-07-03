#include "drake/examples/iiwa_soccer/trajectory_evaluator_vector.h"

namespace drake {
namespace examples {
namespace iiwa_soccer {


using trajectories::PiecewisePolynomial;

TrajectoryEvaluatorVector::TrajectoryEvaluatorVector() {
  this->DeclareAbstractInputPort();
  this->DeclareVectorOutputPort(systems::BasicVector<double>(3), &TrajectoryEvaluatorVector::EvalPolynomial);
}

void TrajectoryEvaluatorVector::EvalPolynomial(const Context<double>& context, BasicVector<double>* const output) const {

  const PiecewisePolynomial<double> input_poly = this->EvalAbstractInput(context, 0)->GetValue<PiecewisePolynomial<double>>();

  double time = context.get_time();

  if (input_poly.empty() || input_poly.start_time() > time) { // We're too early or have no trajectory. Go to "home" position.
    BasicVector<double> worst_case_vec(3);
    worst_case_vec[0] = 0;
    worst_case_vec[1] = 0;
    worst_case_vec[2] = 1;

    output->SetFrom(worst_case_vec);

  } else if (input_poly.end_time() < time) { // We're too late. End time has passed. Just track the last point on the trajectory.
    MatrixX<double> val_at_end = input_poly.value(input_poly.end_time());
    output->SetFrom(BasicVector<double>(val_at_end));
  } else { // We're in progress. Track current time along trajectory.
    MatrixX<double> val_at_time = input_poly.value(time);
    output->SetFrom(BasicVector<double>(val_at_time));
  }

}



}  // namespace iiwa_soccer
}  // namespace examples
}  // namespace drake}