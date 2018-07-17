#include "drake/examples/iiwa_soccer/trajectory_evaluator.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace examples {
namespace iiwa_soccer {


using trajectories::PiecewisePolynomial;
using Eigen::VectorXd;

TrajectoryEvaluator::TrajectoryEvaluator(int dimension) : dimension_(dimension) {

  DRAKE_DEMAND(dimension % 2 == 0); // Dimension should be even. Every coordinate should have a velocity.

  this->DeclareAbstractInputPort();
  this->DeclareVectorOutputPort(systems::BasicVector<double>(dimension_), &TrajectoryEvaluator::EvalPolynomial);
}

void TrajectoryEvaluator::EvalPolynomial(const Context<double>& context, BasicVector<double>* const output) const {

  const PiecewisePolynomial<double>* input_poly = this->EvalInputValue<PiecewisePolynomial<double>>(context, 0);

  const double time = context.get_time();

  VectorXd output_vec(dimension_);
  if (input_poly == nullptr || input_poly->empty() || input_poly->start_time() > time) { // We're too early or have no trajectory. Go to "home" position.
    output_vec.setZero(); // TODO: Make the robot go somewhere safer than starting position probably is.
  }else if (input_poly->end_time() < time) { // We're too late. End time has passed. Just track the last point on the trajectory.
    output_vec.head(dimension_/2) = input_poly->value(input_poly->end_time());
    output_vec.segment(dimension_/2, dimension_/2) = input_poly->MakeDerivative().get()->value(input_poly->end_time()); // TODO: Maybe make the end derivative 0.
  }else {
    output_vec.head(dimension_/2) = input_poly->value(time);
    output_vec.segment(dimension_/2, dimension_/2) = input_poly->MakeDerivative().get()->value(time);
  }
  output->SetFromVector(output_vec);
}

}  // namespace iiwa_soccer
}  // namespace examples
}  // namespace drake}