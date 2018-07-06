#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/event.h"

namespace drake {
namespace examples {
namespace minimal_in_out {

using systems::LeafSystem;
using systems::Context;
using systems::BasicVector;
using systems::InputPortDescriptor;
using systems::PublishEvent;


template <typename T>
class MinimalSystem : public LeafSystem<T> {
 public:
  //DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MinimalSystem)

  MinimalSystem();

//  std::unique_ptr<Context<T>> CreateContext() const;

  const InputPortDescriptor<T>& get_input_port() const {
    return LeafSystem<T>::get_input_port(0);
  }

  void DoOutputCalc(const Context<T>& context, BasicVector<T>* const output) const;
  const int input_size = 1;

  void DoPublish(const Context<T>& context, const std::vector<const PublishEvent<T>*>& events) const;
 private:

//  const int output_size = 1;

};

} // namespace minimal_in_out
} // namespace examples
} // namespace drake

