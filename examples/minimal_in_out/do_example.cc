#include "drake/examples/minimal_in_out/minimal_system.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/random_source.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/primitives/signal_logger.h"
#include "drake/systems/primitives/sine.h"

namespace drake {
namespace examples {
namespace minimal_in_out {

using systems::Simulator;
using systems::SignalLogger;


int DoMain() {
//  systems::DiagramBuilder<double> builder;
//
//  auto system = builder.AddSystem<systems::ConstantVectorSource<double>>(2.0);
//  auto logger = LogOutput(system->get_output_port(), &builder);
//
//  builder.AddSystem<systems::Sine>(1,10,0,1);
//  auto diagram = builder.Build();
//
//// Construct Simulator
//  systems::Simulator<double> simulator(*diagram);
//  simulator.set_publish_every_time_step(true);
//  //simulator.get_mutable_integrator()->set_fixed_step_mode(true);
//  simulator.get_mutable_integrator()->set_maximum_step_size(0.1);
//
//// Run simulation
//  simulator.StepTo(1);
//
//  std::cout << logger->sample_times() << std::endl;
//  std::cout << simulator.get_num_publishes();

  systems::DiagramBuilder<double> builder;

  // Adds a constant source for desired state.
  Eigen::VectorXd const_val = Eigen::VectorXd::Zero(1);
  const_val(0) = 5.;

  auto const_src = builder.AddSystem<systems::ConstantVectorSource<double>>(const_val);
  const_src->set_name("constant_source");

//  auto const_src = builder.AddSystem<systems::internal::RandomSource>(1, 0.1);
//  const_src->set_name("rand_source");


  auto min_sys = builder.AddSystem<MinimalSystem<double>>();
  min_sys->set_name("minimal_system");

  auto signal_logger_sys = builder.AddSystem<SignalLogger>(1);
  signal_logger_sys->set_name("logger");

  builder.Connect(const_src->get_output_port(), min_sys->get_input_port());
  builder.Connect(min_sys->get_output_port(0), signal_logger_sys->get_input_port()); // min_sys must be connected to something for its output to be calculated.
  auto diagram = builder.Build();

  // Haven't figured this out yet:
//  auto context = min_sys->CreateDefaultContext();
//  auto output = min_sys->AllocateOutput(*context);
//  min_sys->CalcOutput(*context, output.get());


  Simulator<double> sim(*diagram);


  sim.set_target_realtime_rate(1.0);
  sim.set_publish_at_initialization(true);
  sim.set_publish_every_time_step(true);
  sim.Initialize();
  // Run simulation.

//  sim.reset_integrator<systems::RungeKutta2Integrator<double>>(
//      *diagram, 1e-3, &sim.get_mutable_context());

  drake::log()->info("About to start simulation.");

  sim.StepTo(10);

  return 0;
} // main

} // namespace minimal_in_out
} // namespace examples
} // namespace drake

int main(int argc, char **argv) {
  return drake::examples::minimal_in_out::DoMain();
}