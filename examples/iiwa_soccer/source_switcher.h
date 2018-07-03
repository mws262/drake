#pragma once

#include <queue>
#include <memory>
#include <iostream>

#include "drake/common/default_scalars.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/value.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/event.h"
#include "drake/systems/framework/input_port_descriptor.h"

namespace drake {
namespace examples {
namespace iiwa_soccer {

using systems::LeafSystem;
using systems::OutputPort;
using systems::BasicVector;
using systems::Context;
using systems::AbstractValue;
using systems::PublishEvent;
using std::map;
using std::string;
using systems::InputPortDescriptor;

template<typename T>
class SourceSwitcher : public LeafSystem<T> {
 public:
  /**
   * Make a new source switcher. The single output can be commanded to swap source systems.
   * All added systems must have the same output vector size.
   *
   * @param output_vector_size
   */
  SourceSwitcher(const int output_vector_size);

  /**
   * Add a new output that this switcher can be toggled to. Switcher will switch to this
   * output after adding it.
   *
   * @param added_output Added output.
   * @param output_name Name of added output. Can be used to switch to this added output.
   * @param builder
   */
  const InputPortDescriptor<T>& add_selectable_output(string output_name);

  void switch_output(string output_name);

  void switch_output(int output_num);



//  /**
//   * Get the sole output of this switcher.
//   */
//  const OutputPort<T>& get_output_port();

 private:
  const int output_vector_size_;
  map<string, int> output_map_;

  int current_output_ = -1;
  void DoOutputCalc(const Context<T>& context, BasicVector<T>* output) const;
  void DoPublish(const Context<T>& context, const std::vector<const PublishEvent<T>*>& events) const;
};

} // namespace minimal_in_out
} // namespace drake
} // namespace systems