#pragma once

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace iiwa_soccer {

using systems::LeafSystem;
using systems::Context;
using systems::BasicVector;

/**
 * Behaves like a "constant_value_source" for abstract types, but set_output can be called to
 * change the output value at any time. Will remain constant in between set_output calls.
 *
 * @tparam V Type of parameter which is broadcast by this system.
 */
template <typename V>
class NonconstantAbstractSource : public LeafSystem<double> {

 public:
  NonconstantAbstractSource<V>() {
    this->DeclareAbstractOutputPort(&NonconstantAbstractSource::DoCalcOutput);
  }

  void set_output(V& new_output) {
    current_output = &new_output;
  }

 private:
  V* current_output = nullptr;

  void DoCalcOutput(const Context<double>& context, V* output) const {
    if (current_output != nullptr) {
      *output = *current_output;
    }
  }
};

}
}
}