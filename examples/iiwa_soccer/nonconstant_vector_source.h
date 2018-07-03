#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/context.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace examples {
namespace iiwa_soccer {

using systems::LeafSystem;
using systems::Context;
using Eigen::VectorXd;
using systems::BasicVector;


class NonconstantVectorSource : public LeafSystem<double> {
 public:
  using LeafSystem<double>::DeclareVectorOutputPort;

  explicit NonconstantVectorSource(int vector_size) : output_vector_(3) {
    output_vector_.SetZero();
    DeclareVectorOutputPort(BasicVector<double>(vector_size), &NonconstantVectorSource::DoCalcOutput);
  }

  void DoCalcOutput(const Context<double>& context, BasicVector<double>* output) const {
    output->SetFrom(output_vector_);

  }

  void set_offset(const VectorXd& new_vec) {
    output_vector_.set_value(new_vec);
  }

 private:
  BasicVector<double> output_vector_;


};

}
}
}