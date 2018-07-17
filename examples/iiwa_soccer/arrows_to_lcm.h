#pragma once

#include <memory>
#include "drake/lcmt_arbitrary_arrow_info.hpp"
#include "drake/lcmt_arbitrary_arrow_collection.hpp"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace examples {
namespace iiwa_soccer {

using systems::LeafSystem;
using systems::Context;

class ArrowsToLcm : public LeafSystem<double> {
 public:
  ArrowsToLcm();

 private:
  void CalcLcmOutput(const Context<double> &context, lcmt_arbitrary_arrow_collection *output) const;

};

}
}
}
