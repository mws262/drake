#include "drake/examples/iiwa_soccer/arrows_to_lcm.h"

#include <memory>

#include "drake/common/eigen_types.h"
#include "drake/lcmt_arbitrary_arrow_collection.hpp"
#include "drake/lcmt_arbitrary_arrow_info.hpp"

namespace drake {
namespace examples {
namespace iiwa_soccer {

using Eigen::VectorXd;
using systems::Value;

ArrowsToLcm::ArrowsToLcm() {
  set_name("ArrowsToLcmSystem");
  DeclareAbstractInputPort(Value<std::vector<VectorXd>>());
  DeclareAbstractOutputPort(&ArrowsToLcm::CalcLcmOutput);
}

void ArrowsToLcm::CalcLcmOutput(
    const Context<double> &context, lcmt_arbitrary_arrow_collection *output) const {
  // Get input / output.
  const auto &arrows_to_draw = EvalAbstractInput(context, 0)->template GetValue<std::vector<VectorXd>>();
  auto &msg = *output;

  msg.timestamp = static_cast<int64_t>(context.get_time() * 1e6);
  msg.num_arrows = arrows_to_draw.size();
  msg.arrow_info.resize(msg.num_arrows);

  for (ulong i = 0; i < arrows_to_draw.size(); ++i) {
    lcmt_arbitrary_arrow_info &info_msg = msg.arrow_info[i];
    info_msg.timestamp = static_cast<int64_t>(context.get_time() * 1e6);

    info_msg.arrow_origin[0] = arrows_to_draw[i](0);
    info_msg.arrow_origin[1] = arrows_to_draw[i](1);
    info_msg.arrow_origin[2] = arrows_to_draw[i](2);

    info_msg.arrow_vector[0] = arrows_to_draw[i](3);
    info_msg.arrow_vector[1] = arrows_to_draw[i](4);
    info_msg.arrow_vector[2] = arrows_to_draw[i](5);

    info_msg.rgb[0] = arrows_to_draw[i](6);
    info_msg.rgb[1] = arrows_to_draw[i](7);
    info_msg.rgb[2] = arrows_to_draw[i](8);

  }
}

}
}
}
