#include <memory>
#include <iostream>

#include "drake/common/default_scalars.h"
#include "drake/examples/minimal_in_out/minimal_system.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/analysis/simulator.h"

namespace drake {
namespace examples {
namespace minimal_in_out {

using systems::LeafSystem;
using systems::Context;
using systems::BasicVector;
using systems::VectorBase;
using systems::ContinuousState;
using systems::Simulator;

template <typename T>
MinimalSystem<T>::MinimalSystem() {
  this->DeclareVectorInputPort(BasicVector<T>(input_size));
  this->DeclareVectorOutputPort(systems::BasicVector<T>(input_size), &MinimalSystem<T>::DoOutputCalc);
}

template <typename T>
void MinimalSystem<T>::DoOutputCalc(const Context<T>& context, BasicVector<T>* const output) const {

  // Get the input vector, print it, and pass it onward.
  const systems::BasicVector<T>* input_vector = this->EvalVectorInput(context, 0);
//  std::cout << input_vector->GetAtIndex(0);
  drake::log()->info("In DoOutputCalc.");

  // Write system output.
  output->set_value(input_vector->CopyToVector() * 10);
}

/**
 * Overriding default publish behavior so we get a message to screen.
 *
 * @tparam T
 * @param context
 * @param events
 */
template <typename T>
void MinimalSystem<T>::DoPublish(const Context<T>& context, const std::vector<const PublishEvent<T>*>& events) const{
  drake::log()->info("In minimal_system's publish.");
}

} // namespace minimal_in_out
} // namespace examples
} // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::examples::minimal_in_out::MinimalSystem)