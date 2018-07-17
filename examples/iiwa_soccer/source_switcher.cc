#include "drake/examples/iiwa_soccer/source_switcher.h"

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
using std::pair;
using std::string;

template <typename T>
SourceSwitcher<T>::SourceSwitcher(const int& output_vector_size) : output_vector_size_(output_vector_size) {
this->DeclareVectorOutputPort(systems::BasicVector<T>(output_vector_size),
    &SourceSwitcher<T>::DoOutputCalc);
}

template <typename T>
const InputPortDescriptor<T>& SourceSwitcher<T>::add_selectable_output(const string& output_name) {
  int port_being_added = this->get_num_input_ports();

  // Add this output to the string name keyed map.
  output_map_.insert(pair<string,int>(output_name, port_being_added));
  current_output_ = port_being_added; // Make the most recently added input the active one.

  return this->DeclareVectorInputPort(BasicVector<T>(output_vector_size_));
}

template <typename T>
void SourceSwitcher<T>::switch_output(const string& output_name) {
  current_output_ = output_map_.at(output_name);
}

template <typename T>
void SourceSwitcher<T>::switch_output(const int& output_num) {
  DRAKE_DEMAND(output_num < this->get_num_output_ports() && output_num >= 0);

  current_output_ = output_num;
}

template <typename T>
const InputPortDescriptor<T>& SourceSwitcher<T>::get_output_port_by_name(const string& output_name) {
  return this->get_input_port(output_map_.at(output_name));
}

template<typename T>
void SourceSwitcher<T>::DoOutputCalc(const Context<T>& context, BasicVector<T>* output) const {
  DRAKE_DEMAND(this->get_num_input_ports() > 0);
  output->SetFromVector(this->EvalVectorInput(context, current_output_)->get_value());
}


template<typename T>
void SourceSwitcher<T>::DoPublish(const Context<T>& context, const std::vector<const PublishEvent<T>*>& events) const{
  //drake::log()->info("In minimal_system's publish.");
}

} // namespace minimal_in_out
} // namespace drake
} // namespace systems

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::examples::iiwa_soccer::SourceSwitcher)