#pragma once

#include <queue>
#include <memory>
#include <iostream>

#include "drake/common/default_scalars.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/value.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/event.h"

namespace drake {
namespace examples {
namespace iiwa_soccer {

using systems::LeafSystem;
using systems::OutputPort;
using systems::BasicVector;
using systems::Context;
using systems::AbstractValue;
using systems::PublishEvent;

/**
 * Single-output-port system with multiple sources which are toggled
 * based on time. The output-port type may not change.
 *
 * @tparam T
 */
template<typename T>
class TimeBasedSourceSwitcher : public LeafSystem<T> {
 public:
  TimeBasedSourceSwitcher(const int output_vector_size);

  void add_output_with_duration(const OutputPort<T> &queued_output,
                                double &active_duration,
                                systems::DiagramBuilder<T> &builder);

 private:
  const int output_vector_size_;

  mutable std::queue<double *> active_durations_;
  mutable double *next_output_switch_time_ = nullptr;
  mutable int current_input_port_ = -1; // So we get errors if the advancement to 0 doesn't happen.

  void DoOutputCalc(const Context<T>& context, BasicVector<T>* output) const;
  void PopNextInput() const;
  void DoPublish(const Context<T>& context, const std::vector<const PublishEvent<T>*>& events) const;
};

} // namespace minimal_in_out
} // namespace drake
} // namespace systems