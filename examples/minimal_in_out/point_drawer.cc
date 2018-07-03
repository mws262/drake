#include "drake/examples/minimal_in_out/point_drawer.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/multibody/rigid_body_tree.h"



namespace drake {
namespace examples {
namespace minimal_in_out {

using systems::BasicVector;
using Eigen::Vector3d;

PointDrawer::PointDrawer() {
  this->DeclareVectorInputPort(BasicVector<double>(3));
  this->DeclareAbstractOutputPort(&PointDrawer::DoOutputCalc);
}

void PointDrawer::DoOutputCalc(const Context<double> &context, bot_core::pointcloud_t* output) const {

  // Get the input vector, print it, and pass it onward.
//  const systems::BasicVector<double>* input_vector = this->EvalVectorInput(context, 0);

  bot_core::pointcloud_t& message = *output;

  message.frame_id = std::string(RigidBodyTreeConstants::kWorldName);
  message.n_points = 1;
  message.points.clear();

//  int i = 0;
//  while ( i < input_vector->size()) {

    message.points.push_back(std::vector<float>{3,3,3});
//        static_cast<float>(input_vector->GetAtIndex(i++)),
//        static_cast<float>(input_vector->GetAtIndex(i++)),
//        static_cast<float>(input_vector->GetAtIndex(i++))});
//  }
  message.n_channels = 0;
}



} // namespace minimal_in_out
} // namespace examples
} // namespace drake}