#include "drake/examples/minimal_in_out/time_based_source_switcher.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/pass_through.h"
#include "drake/systems/framework/output_port.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/primitives/signal_logger.h"


namespace drake {
namespace examples {
namespace minimal_in_out {

int DoStuff() {

  systems::DiagramBuilder<double> builder;

  // Adds a constant source for desired state.
  Eigen::VectorXd const_val = Eigen::VectorXd::Zero(1);
  const_val(0) = 5.;

  auto const_src1 = builder.AddSystem <systems::ConstantVectorSource<double>> (const_val);
  const_src1->set_name("constant_source1");

  auto const_src2 = builder.AddSystem <systems::ConstantVectorSource<double>> (2*const_val);
  const_src2->set_name("constant_source2");

  auto const_src3 = builder.AddSystem <systems::ConstantVectorSource<double>> (4*const_val);
  const_src3->set_name("constant_source3");

  auto const_src4 = builder.AddSystem <systems::ConstantVectorSource<double>> (8*const_val);
  const_src4->set_name("constant_source4");

  auto switcher = builder.AddSystem<TimeBasedSourceSwitcher<double>>(1);
  double duration = 2;
  switcher->add_output_with_duration(const_src1->get_output_port(), duration, builder);
  switcher->add_output_with_duration(const_src2->get_output_port(), duration, builder);
  switcher->add_output_with_duration(const_src3->get_output_port(), duration, builder);
  switcher->add_output_with_duration(const_src4->get_output_port(), duration, builder);

  //auto dummy_sys1 = builder.AddSystem<systems::PassThrough<double>>(1);

  auto signal_logger_sys = builder.AddSystem<systems::SignalLogger>(1);
  signal_logger_sys->set_name("logger");

  //TODO single output so replace method.
  builder.Connect(switcher->get_output_port(0), signal_logger_sys->get_input_port());

  auto diagram = builder.Build();

  systems::Simulator<double> sim(*diagram);
  sim.set_target_realtime_rate(1);
  sim.set_publish_at_initialization(false);
  sim.set_publish_every_time_step(true);
  sim.Initialize();
  sim.StepTo(15);

  return 0;
}

} // namespace minimal_in_out
} // namespace drake
} // namespace systems


int main(int argc, char **argv) {
  return drake::examples::minimal_in_out::DoStuff();
}
