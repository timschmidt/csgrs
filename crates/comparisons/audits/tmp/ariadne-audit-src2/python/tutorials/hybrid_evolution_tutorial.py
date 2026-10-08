#!/usr/bin/python3

##############################################################################
#            hybrid_evolution_tutorial.py
#
#  Copyright  2020-21  Luca Geretti, Pieter Collins
##############################################################################

# Ariadne is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# Ariadne is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with Ariadne. If not, see <https://www.gnu.org/licenses/>.

# Import all classes in the ariadne module
from pyariadne import *

#! [get_tank]
def get_tank():
    # Declare the system constants
    alpha = RealConstant("alpha",dec_(0.02))
    beta = RealConstant("beta",dec_(0.3))

    # Declare the variables for the dynamics
    aperture = RealVariable("aperture")
    height = RealVariable("height")

    # Create the tank automaton
    automaton = HybridAutomaton()

    # The water level is always given by the same dynamic.
    # The inflow is controlled by the valve aperture, the outflow depends on the
    # pressure, which is proportional to the water height.
    automaton.new_mode({dot(height):beta*aperture-alpha*height})

    return automaton

#! [/get_tank]


#! [get_valve]
def get_valve():
    # Declare some constants. Note that system parameters should be given as variables.
    T = RealConstant("T",4)

    # Declare the shared system variables
    aperture = RealVariable("aperture")

    # Declare the events we use
    stop_opening = DiscreteEvent("stop_opening")
    stop_closing = DiscreteEvent("stop_closing")
    can_open = DiscreteEvent("can_open")
    can_close = DiscreteEvent("can_close")

    # Declare the variable for the automaton name
    valve = StringVariable("valve")

    # Create the valve automaton
    automaton = HybridAutomaton(valve.name())

    # Declare the values the valve variable can have
    opening = DiscreteLocation({valve:"opening"})
    closed = DiscreteLocation({valve:"closed"})
    opened = DiscreteLocation({valve:"opened"})
    closing = DiscreteLocation({valve:"closing"})

    # Define the algebraic equations for the opened/closed locations.
    automaton.new_mode(opened,{let(aperture):1})
    automaton.new_mode(closed,{let(aperture):0})
    # Define the differential equations for the opening/closing locations.
    automaton.new_mode(opening,{dot(aperture):+1/T})
    automaton.new_mode(closing,{dot(aperture):-1/T})

    # Define the transitions: source location, event and target location
    # then a mix of reset, guard and event kind can be present; if the event kind
    # is not specified, then also the guard can't be specified: this implicitly
    # means that the event is an input event for this automaton.
    automaton.new_transition(closed,can_open,opening,{next(aperture):aperture})
    automaton.new_transition(opening,stop_opening,opened,aperture>=1,URGENT)
    automaton.new_transition(opened,can_close,closing,{next(aperture):aperture})
    automaton.new_transition(closing,stop_closing,closed,aperture<=0,URGENT)
    automaton.new_transition(opening,can_close,closing,{next(aperture):aperture})
    automaton.new_transition(closing,can_open,opening,{next(aperture):aperture})

    return automaton

#! [/get_valve]


#! [get_controller]
def get_controller():
    # Declare some constants
    hmin = RealConstant("hmin",dec_(5.75))
    hmax = RealConstant("hmax",dec_(7.75))
    delta = RealConstant("delta",dec_(0.02))

    # Declare the shared system variables
    height = RealVariable("height")

    # Declare the events we use
    can_open = DiscreteEvent("can_open")
    can_close = DiscreteEvent("can_close")
    must_open = DiscreteEvent("must_open")
    must_close = DiscreteEvent("must_close")

    # Declare the variable for the automaton name
    controller = StringVariable("controller")

    # Create the controller automaton
    automaton = HybridAutomaton(controller.name())

    # Declare the locations for the controller
    rising = DiscreteLocation({controller:"rising"})
    falling = DiscreteLocation({controller:"falling"})

    # Instantiate modes for each location with no dynamics
    automaton.new_mode(rising)
    automaton.new_mode(falling)

    # Specify the invariants valid in each mode. Note that every invariant
    # must have an action label. This is used internally, for example, to
    # check non-blockingness of urgent actions.
    automaton.new_invariant(falling,height>=hmin-delta,must_open)
    automaton.new_invariant(rising,height<=hmax+delta,must_close)

    # Specify the transitions, starting from the source location, according to an event, to a target location
    # Following those arguments you specify a guard and whether the event is permissive or urgent.
    automaton.new_transition(falling,can_open,rising,height<=hmin+delta,PERMISSIVE)
    automaton.new_transition(rising,can_close,falling,height>=hmax-delta,PERMISSIVE)

    return automaton

#! [/get_controller]


#! [simulate_evolution]
def simulate_evolution(system,initial_set,final_time):
    # Re-introduce the shared system variables required for the initial set
    aperture = RealVariable("aperture")
    height = RealVariable("height")

    # Create a simulator object
    simulator = HybridSimulator(system)
    simulator.configuration().set_step_size(0.01)

    print("simulator.configuration() = ",simulator.configuration())

    # Compute a simulation trajectory
    print("Computing simulation trajectory...")
    orbit = simulator.orbit(initial_set,HybridTerminationCriterion(final_time))

    # Plot the simulation trajectory using all different projections
    print("Plotting simulation trajectory..")
    plot("simulation_t-height",Axes2d(0,TimeVariable(),30,5,height,9),orbit)
    plot("simulation_t-aperture",Axes2d(0,TimeVariable(),30, -0.1,aperture,1.1),orbit)
    plot("simulation_height-aperture",Axes2d(5,height,9, -0.1,aperture,1.1),orbit)
    print("Done computing and plotting simulation trajectory..")

#! [/simulate_evolution]


#! [create_evolver]
def create_evolver(system):
    # Create a GeneralHybridEvolver object
    evolver = GeneralHybridEvolver(system)

    # Set the evolver configuration
    evolver.configuration().set_maximum_enclosure_radius(3.0)
    evolver.configuration().set_maximum_step_size(0.25)

    print("evolver.configuration() =",evolver.configuration())

    return evolver

#! [/create_evolver]


#! [compute_evolution]
def compute_evolution(evolver,initial_set,final_time):
    # Re-introduce the shared system variables required for plotting
    aperture = RealVariable("aperture")
    height = RealVariable("height")
    time = TimeVariable()

    # Compute the evolution flow tube using upper semantics
    print("Computing evolution flow tube...")
    orbit = evolver.orbit(initial_set,HybridTerminationCriterion(final_time),Semantics.UPPER)


    tmax = Approximation[FloatDP](final_time.continuous_time().get(dp)).get_d()

    # Plot the flow tube using two different projections
    print("Plotting evolution flow tube...")
    plot("finite_evolution_t-height",Axes2d(0,time,tmax, 5,height,9),orbit)
    plot("finite_evolution_t-aperture",Axes2d(0,time,tmax, -0.1,aperture,1.1),orbit)
    plot("finite_evolution_height-aperture",Axes2d(5,height,9, -0.1,aperture,1.1),orbit)
    print("Done computing and plotting evolution flow tube!\n")

#! [/compute_evolution]


#! [create_analyser]
def create_analyser(evolver):
    # Create a ReachabilityAnalyser object
    analyser = HybridReachabilityAnalyser(evolver)

    #  Set the analyser configuration
    analyser.configuration().set_maximum_grid_fineness(6)
    analyser.configuration().set_lock_to_grid_time(5)

    print("analyser.configuration() =",analyser.configuration())

    return analyser

#! [/create_analyser]


#! [compute_reachability]
def compute_reachability(analyser,initial_set,final_time):
    # Re-introduce the shared system variables required for plotting
    aperture = RealVariable("aperture")
    height = RealVariable("height")

    # Compute over-approximation to finite-time reachable set using upper semantics.
    print("Computing upper reach set...")
    upper_reach = analyser.upper_reach(initial_set,final_time)
    print("Plotting upper reach set...")
    plot("upper_reach",Axes2d(5,height,9, -0.1,aperture,1.1),upper_reach)
    print("Done computing and plotting upper reach set!\n")

    # Compute over-approximation to infinite-time reachable set using upper semantics.
    print("Computing outer chain reach set...")
    outer_chain_reach = analyser.outer_chain_reach(initial_set)
    print("Plotting outer chain reach set...")
    plot("outer_chain_reach",Axes2d(5,height,9, -0.1,aperture,1.1),outer_chain_reach)
    print("Done computing and plotting outer chain reach set!\n")

#! [/compute_reachability]


#! [get_system]
def get_system():
    # Create the composed automaton
    system = CompositeHybridAutomaton("watertank",[get_tank(),get_valve(),get_controller()])

    # Print the system description on the command line
    print("system =",system)

    return system

#! [/get_system]


#! [get_initial_set]
def get_initial_set():
    # Re-introduce variables to be used for the initial set
    height = RealVariable("height")
    valve = StringVariable("valve")
    controller = StringVariable("controller")
    opened = String("opened")
    rising = String("rising")

    # Define the initial set, by supplying the location as a list of locations for each composed automata, and
    # the continuous set as a list of variable assignments for each variable controlled on that location
    # (the assignment can be either a singleton value using the == symbol or an interval using the <= symbols)
    initial_set = HybridBoundedConstraintSet({valve:opened,controller:rising},{height:(dec_(6.9),7)})

    # Print the initial set on the command line
    print("initial_set =",initial_set)

    return initial_set

#! [/get_initial_set]


#! [get_final_time]
def get_final_time():
    # Define the final time: continuous time and maximum number of transitions
    final_time = HybridTime(dec_(60.0),5)
    print("final_time =",final_time)

    return final_time

#! [/get_final_time]


#! [main]
if __name__ == '__main__':

    from sys import argv
    if not CommandLineInterface.instance().acquire(argv):
        exit()

    # Get the system
    system = get_system()

    # Get the initial set
    initial_set = get_initial_set()

    # Get the final time
    final_time = get_final_time()

    # Compute an approximate simulation of the system evolution
    simulate_evolution(system,initial_set,final_time)

    # Create an evolver object
    evolver = create_evolver(system)

    # Compute the system evolution
    compute_evolution(evolver,initial_set,final_time)

    # Create an analyser object
    analyser = create_analyser(evolver)

    # Compute the system reachability
    compute_reachability(analyser,initial_set,final_time)

#! [/main]
