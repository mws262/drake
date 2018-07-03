#include "drake/examples/iiwa_soccer/trajectory_evaluator.h"

namespace drake {
namespace examples {
namespace iiwa_soccer {


using trajectories::PiecewisePolynomial;

TrajectoryEvaluator::TrajectoryEvaluator(int dimension) : dimension_(dimension) {
  this->DeclareAbstractInputPort();
  this->DeclareVectorOutputPort(systems::BasicVector<double>(dimension_), &TrajectoryEvaluator::EvalPolynomial);
}

void TrajectoryEvaluator::EvalPolynomial(const Context<double>& context, BasicVector<double>* const output) const {

  const PiecewisePolynomial<double>* input_poly = this->EvalInputValue<PiecewisePolynomial<double>>(context, 0);

  double time = context.get_time();

  if (input_poly == nullptr || input_poly->empty() || input_poly->start_time() > time) { // We're too early or have no trajectory. Go to "home" position.
    BasicVector<double> worst_case_vec(dimension_);
    worst_case_vec.SetZero();

    output->SetFrom(worst_case_vec);

  } else if (input_poly->end_time() < time) { // We're too late. End time has passed. Just track the last point on the trajectory.
    MatrixX<double> val_at_end = input_poly->value(input_poly->end_time());
    output->SetFrom(BasicVector<double>(val_at_end));
  } else { // We're in progress. Track current time along trajectory.
    MatrixX<double> val_at_time = input_poly->value(time);
    output->SetFrom(BasicVector<double>(val_at_time));
  }

}



}  // namespace iiwa_soccer
}  // namespace examples
}  // namespace drake}