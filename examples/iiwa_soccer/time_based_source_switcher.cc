#include "drake/examples/iiwa_soccer/time_based_source_switcher.h"

namespace drake {
namespace examples {
namespace iiwa_soccer {

using systems::LeafSystem;
using systems::OutputPort;
using systems::InputPortDescriptor;
using systems::BasicVector;
using systems::Context;
using systems::Value;
using systems::PublishEvent;

template <typename T>
TimeBasedSourceSwitcher<T>::TimeBasedSourceSwitcher(const int output_vector_size) : output_vector_size_(output_vector_size) {
this->DeclareVectorOutputPort(systems::BasicVector<T>(output_vector_size),
    &TimeBasedSourceSwitcher<T>::DoOutputCalc);
}

template <typename T>
void TimeBasedSourceSwitcher<T>::add_output_with_duration(const OutputPort<T> &queued_output,
                                                          double &active_duration,
                                                          systems::DiagramBuilder<T> &builder) {

  DRAKE_DEMAND(queued_output.size() == output_vector_size_);

  this->DeclareVectorInputPort(BasicVector<T>(output_vector_size_)); // Make a new input port.
  builder.Connect(queued_output, this->get_input_port(this->get_num_input_ports() - 1)); // Connect the supplied output to our new input port.
  active_durations_.push(&active_duration); // Add the time to the queue we're going to run through.
}


template<typename T>
void TimeBasedSourceSwitcher<T>::DoOutputCalc(const Context<T>& context, BasicVector<T>* output) const {

  // Check if a new output should be activated.
  auto current_time = context.get_time();

  if (next_output_switch_time_ == nullptr){ // Case 1: First time being called. Need to load up the first output.
    next_output_switch_time_ = active_durations_.front();
    active_durations_.pop();
    current_input_port_++;
  } else if (!active_durations_.empty() && *next_output_switch_time_ < current_time) { // Current output duration has elapsed. Switch to next unless we're already on the last one.
    PopNextInput();
  } // Otherwise, all is well, keep passing on the current output.

  output->SetFromVector(this->EvalVectorInput(context, current_input_port_)->get_value());
}

template<typename T>
void TimeBasedSourceSwitcher<T>::PopNextInput() const {
  *next_output_switch_time_ += *active_durations_.front();
  active_durations_.pop();
  current_input_port_++;
}

template<typename T>
void TimeBasedSourceSwitcher<T>::DoPublish(const Context<T>& context, const std::vector<const PublishEvent<T>*>& events) const{
  //drake::log()->info("In minimal_system's publish.");
}

} // namespace minimal_in_out
} // namespace drake
} // namespace systems

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::examples::iiwa_soccer::TimeBasedSourceSwitcher)