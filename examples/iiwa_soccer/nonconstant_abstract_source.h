#pragma once

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace iiwa_soccer {

using systems::LeafSystem;
using systems::Context;
using systems::BasicVector;

template <typename V>
class NonconstantAbstractSource : public LeafSystem<double> {

 public:
  NonconstantAbstractSource<V>() {
    this->DeclareAbstractOutputPort(&NonconstantAbstractSource::DoCalcOutput);
  }

  void set_output(V* new_output) {
    current_output = new_output;
  }

 private:
  V* current_output = nullptr;

  void DoCalcOutput(const Context<double>& context, V* output) const {
    output = current_output;
  }

};

}
}
}