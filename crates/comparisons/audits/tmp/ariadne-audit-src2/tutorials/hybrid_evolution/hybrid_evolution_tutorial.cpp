/***************************************************************************
 *            hybrid_evolution_tutorial.cpp
 *
 *  Copyright  2020  Luca Geretti
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <ariadne.hpp>

using namespace Ariadne;

//! [get_tank]
HybridAutomaton get_tank()
{
    // Declare the system constants
    RealConstant alpha("alpha",0.02_dec);
    RealConstant beta("beta",0.3_dec);

    // Declare the variables for the dynamics
    RealVariable aperture("aperture");
    RealVariable height("height");

    // Create the tank automaton
    HybridAutomaton automaton;

    // The water level is always given by the same dynamic.
    // The inflow is controlled by the valve aperture, the outflow depends on the
    // pressure, which is proportional to the water height.
    automaton.new_mode({dot(height)=beta*aperture-alpha*height});

    return automaton;
}
//! [get_tank]

//! [get_valve]
HybridAutomaton get_valve()
{
    // Declare some constants. Note that system parameters should be given as variables.
    RealConstant T("T",4);

    // Declare the shared system variables
    RealVariable aperture("aperture");

    // Declare the events we use
    DiscreteEvent stop_opening("stop_opening");
    DiscreteEvent stop_closing("stop_closing");
    DiscreteEvent can_open("can_open");
    DiscreteEvent can_close("can_close");

    // Declare the variable for the automaton name
    StringVariable valve("valve");

    // Create the valve automaton
    HybridAutomaton automaton(valve.name());

    // Declare the values the valve variable can have
    DiscreteLocation opening(valve|"opening");
    DiscreteLocation closed(valve|"closed");
    DiscreteLocation opened(valve|"opened");
    DiscreteLocation closing(valve|"closing");

    // Define the algebraic equations for the opened/closed locations.
    automaton.new_mode(opened,{let(aperture)=1});
    automaton.new_mode(closed,{let(aperture)=0});
    // Define the differential equations for the opening/closing locations.
    automaton.new_mode(opening,{dot(aperture)=+1/T});
    automaton.new_mode(closing,{dot(aperture)=-1/T});

    // Define the transitions: source location, event and target location;
    // then a mix of reset, guard and event kind can be present; if the event kind
    // is not specified, then also the guard can't be specified: this implicitly
    // means that the event is an input event for this automaton.
    automaton.new_transition(closed,can_open,opening,{next(aperture)=aperture});
    automaton.new_transition(opening,stop_opening,opened,aperture>=1,EventKind::URGENT);
    automaton.new_transition(opened,can_close,closing,{next(aperture)=aperture});
    automaton.new_transition(closing,stop_closing,closed,aperture<=0,EventKind::URGENT);
    automaton.new_transition(opening,can_close,closing,{next(aperture)=aperture});
    automaton.new_transition(closing,can_open,opening,{next(aperture)=aperture});

    return automaton;
}
//! [get_valve]

//! [get_controller]
HybridAutomaton get_controller()
{
    // Declare some constants
    RealConstant hmin("hmin",5.75_dec);
    RealConstant hmax("hmax",7.75_dec);
    RealConstant delta("delta",0.02_dec);

    // Declare the shared system variables
    RealVariable height("height");

    // Declare the events we use
    DiscreteEvent can_open("can_open");
    DiscreteEvent can_close("can_close");
    DiscreteEvent must_open("must_open");
    DiscreteEvent must_close("must_close");

    // Declare the variable for the automaton name
    StringVariable controller("controller");

    // Create the controller automaton
    HybridAutomaton automaton(controller.name());

    // Declare the locations for the controller
    DiscreteLocation rising(controller|"rising");
    DiscreteLocation falling(controller|"falling");

    // Instantiate modes for each location with no dynamics
    automaton.new_mode(rising);
    automaton.new_mode(falling);

    // Specify the invariants valid in each mode. Note that every invariant
    // must have an action label. This is used internally, for example, to
    // check non-blockingness of urgent actions.
    automaton.new_invariant(falling,height>=hmin-delta,must_open);
    automaton.new_invariant(rising,height<=hmax+delta,must_close);

    // Specify the transitions, starting from the source location, according to an event, to a target location;
    // Following those arguments you specify a guard and whether the event is permissive or urgent.
    automaton.new_transition(falling,can_open,rising,height<=hmin+delta,EventKind::PERMISSIVE);
    automaton.new_transition(rising,can_close,falling,height>=hmax-delta,EventKind::PERMISSIVE);

    return automaton;
}
//! [get_controller]

//! [simulate_evolution]
Void simulate_evolution(CompositeHybridAutomaton const& system, HybridBoundedConstraintSet const& initial_set, HybridTime const& final_time)
{
    // Re-introduce the shared system variables required for plotting
    RealVariable aperture("aperture");
    RealVariable height("height");
    TimeVariable time;

    // Create a simulator object
    HybridSimulator simulator(system);
    simulator.configuration().set_step_size(0.01);

    CONCLOG_PRINTLN_VAR(simulator.configuration());

    // Compute a simulation trajectory
    CONCLOG_PRINTLN("Computing simulation trajectory...");
    auto orbit = simulator.orbit(initial_set,final_time);

    // Plot the simulation trajectory using three different projections
    CONCLOG_PRINTLN("Plotting simulation trajectory...");
    plot("simulation_t-height",Axes2d(0<=time<=30,5<=height<=9),orbit);
    plot("simulation_t-aperture",Axes2d(0<=time<=30,-0.1<=aperture<=1.1),orbit);
    plot("simulation_height-aperture",Axes2d(5<=height<=9,-0.1<=aperture<=1.1),orbit);
    CONCLOG_PRINTLN("Done computing and plotting simulation trajectory!");
}
//! [simulate_evolution]

//! [create_evolver]
GeneralHybridEvolver create_evolver(CompositeHybridAutomaton const& system)
{
    // Create a GeneralHybridEvolver object
    GeneralHybridEvolver evolver(system);

    // Set the evolver configuration
    evolver.configuration().set_maximum_enclosure_radius(3.0);
    evolver.configuration().set_maximum_step_size(0.25);

    CONCLOG_PRINTLN_VAR(evolver.configuration());

    return evolver;
}
//! [create_evolver]

//! [compute_evolution]
Void compute_evolution(const GeneralHybridEvolver& evolver, HybridBoundedConstraintSet const& initial_set, HybridTime const& final_time)
{
    // Re-introduce the shared system variables required for plotting
    RealVariable aperture("aperture");
    RealVariable height("height");
    TimeVariable time;

    // Compute the orbit using upper semantics
    CONCLOG_PRINTLN("Computing evolution flow tube...");
    auto orbit = evolver.orbit(initial_set,final_time,Semantics::UPPER);

    // Plot the flow tube using three different projections
    CONCLOG_PRINTLN("Plotting evolution flow tube...");
    plot("finite_evolution_t-height",Axes2d(0<=time<=30,5<=height<=9),orbit);
    plot("finite_evolution_t-aperture",Axes2d(0<=time<=30,-0.1<=aperture<=1.1),orbit);
    plot("finite_evolution_height-aperture",Axes2d(5<=height<=9,-0.1<=aperture<=1.1),orbit);
    CONCLOG_PRINTLN("Done computing and plotting evolution flow tube!");
}
//! [compute_evolution]

//! [create_analyser]
HybridReachabilityAnalyser create_analyser(GeneralHybridEvolver const& evolver)
{
    // Create a ReachabilityAnalyser object
    HybridReachabilityAnalyser analyser(evolver);

    //  Set the analyser configuration
    analyser.configuration().set_maximum_grid_fineness(6);
    analyser.configuration().set_lock_to_grid_time(5);

    CONCLOG_PRINTLN_VAR(analyser.configuration());

    return analyser;
}
//! [create_analyser]

//! [compute_reachability]
Void compute_reachability(HybridReachabilityAnalyser const& analyser, HybridBoundedConstraintSet const& initial_set, HybridTime const& final_time)
{
    // Re-introduce the shared system variables required for plotting
    RealVariable aperture("aperture");
    RealVariable height("height");

    // Compute over-approximation to infinite-time reachable set using upper semantics.
    CONCLOG_PRINTLN("Computing upper reach set...");
    auto upper_reach = analyser.upper_reach(initial_set,final_time);
    CONCLOG_PRINTLN("Plotting upper reach set...");
    plot("upper_reach",Axes2d(5<=height<=9,-0.1<=aperture<=1.1),upper_reach);
    CONCLOG_PRINTLN("Done computing and plotting upper reach set!");

    // Compute over-approximation to infinite-time reachable set using upper semantics.
    CONCLOG_PRINTLN("Computing outer chain reach set...");
    auto outer_chain_reach = analyser.outer_chain_reach(initial_set);
    CONCLOG_PRINTLN("Plotting outer chain reach set...");
    plot("outer_chain_reach",Axes2d(5<=height<=9,-0.1<=aperture<=1.1),outer_chain_reach);
    CONCLOG_PRINTLN("Done computing and plotting outer chain reach set!");
}
//! [compute_reachability]

//! [get_system]
CompositeHybridAutomaton get_system()
{
    // Create the composed automaton
    CompositeHybridAutomaton system("watertank",{get_tank(),get_valve(),get_controller()});

    // Print the system description on the command line
    CONCLOG_PRINTLN_VAR(system);

    return system;
}
//! [get_system]

//! [get_initial_set]
HybridBoundedConstraintSet get_initial_set()
{
    // Re-introduce variables to be used for the initial set
    RealVariable height("height");
    StringVariable valve("valve");
    StringVariable controller("controller");
    String opened("opened");
    String rising("rising");

    // Define the initial set, by supplying the location as a list of locations for each composed automata, and
    // the continuous set as a list of variable assignments for each variable controlled on that location
    // (the assignment can be either a singleton value using the == symbol or an interval using the <= symbols)
    HybridBoundedConstraintSet initial_set({valve|opened,controller|rising},{6.9_dec<=height<=7});

    // Print the initial set on the command line
    CONCLOG_PRINTLN_VAR(initial_set);

    return initial_set;
}
//! [get_initial_set]

//! [get_final_time]
HybridTime get_final_time()
{
    // Define the final time: continuous time and maximum number of transitions
    HybridTime final_time(30.0_dec,5);
    CONCLOG_PRINTLN_VAR(final_time);

    return final_time;
}
//! [get_final_time]

//! [main]
Int main(Int argc, const char* argv[])
{
    // Acquire arguments from the command line, use "-h" to see options
    if (not CommandLineInterface::instance().acquire(argc,argv)) return -1;

    // Get the system
    auto system = get_system();

    // Get the initial set
    auto initial_set = get_initial_set();

    // Get the final time
    auto final_time = get_final_time();

    // Compute an approximate simulation of the system evolution
    simulate_evolution(system,initial_set,final_time);

    // Create an evolver object
    auto evolver = create_evolver(system);

    // Compute the system evolution
    compute_evolution(evolver,initial_set,final_time);

    // Create an analyser object
    auto analyser = create_analyser(evolver);

    // Compute the system reachability
    compute_reachability(analyser,initial_set,final_time);
}
//! [main]
