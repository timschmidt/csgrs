#!/usr/bin/python3

##############################################################################
#            watertank.py
#
#  Copyright  2020-24  Luca Geretti, Pieter Collins
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

from pyariadne import *

def get_tank():
    alpha = RealConstant("alpha",dec_(0.02))
    beta = RealConstant("beta",dec_(0.3))

    aperture = RealVariable("aperture")
    height = RealVariable("height")

    automaton = HybridAutomaton()

    automaton.new_mode({dot(height):beta*aperture-alpha*height})

    return automaton

def get_valve():
    # Declare some constants. Note that system parameters should be given as variables.
    T = RealConstant("T",4)

    # Declare the shared system variables
    aperture = RealVariable("aperture")

    stop_opening = DiscreteEvent("stop_opening")
    stop_closing = DiscreteEvent("stop_closing")
    can_open = DiscreteEvent("can_open")
    can_close = DiscreteEvent("can_close")

    valve = StringVariable("valve")

    automaton = HybridAutomaton(valve.name())

    opening = DiscreteLocation({valve:"opening"})
    closed = DiscreteLocation({valve:"closed"})
    opened = DiscreteLocation({valve:"opened"})
    closing = DiscreteLocation({valve:"closing"})

    automaton.new_mode(opened,{let(aperture):1})
    automaton.new_mode(closed,{let(aperture):0})
    automaton.new_mode(opening,{dot(aperture):+1/T})
    automaton.new_mode(closing,{dot(aperture):-1/T})

    automaton.new_transition(closed,can_open,opening,{next(aperture):aperture})
    automaton.new_transition(opening,stop_opening,opened,aperture>=1,URGENT)
    automaton.new_transition(opened,can_close,closing,{next(aperture):aperture})
    automaton.new_transition(closing,stop_closing,closed,aperture<=0,URGENT)
    automaton.new_transition(opening,can_close,closing,{next(aperture):aperture})
    automaton.new_transition(closing,can_open,opening,{next(aperture):aperture})

    return automaton


def get_controller():
    hmin = RealConstant("hmin",dec_(5.75))
    hmax = RealConstant("hmax",dec_(7.75))
    delta = RealConstant("delta",dec_(0.02))

    height = RealVariable("height")

    can_open = DiscreteEvent("can_open")
    can_close = DiscreteEvent("can_close")
    must_open = DiscreteEvent("must_open")
    must_close = DiscreteEvent("must_close")

    controller = StringVariable("controller")

    automaton = HybridAutomaton(controller.name())

    rising = DiscreteLocation({controller:"rising"})
    falling = DiscreteLocation({controller:"falling"})

    automaton.new_mode(rising)
    automaton.new_mode(falling)

    automaton.new_invariant(falling,height>=hmin-delta,must_open)
    automaton.new_invariant(rising,height<=hmax+delta,must_close)

    automaton.new_transition(falling,can_open,rising,height<=hmin+delta,PERMISSIVE)
    automaton.new_transition(rising,can_close,falling,height>=hmax-delta,PERMISSIVE)

    return automaton


def simulate_evolution(system,initial_set,final_time):
    aperture = RealVariable("aperture")
    height = RealVariable("height")

    simulator = HybridSimulator(system)
    simulator.configuration().set_step_size(0.01)

    print("simulator.configuration() = ",simulator.configuration())

    print("Computing simulation trajectory...")
    orbit = simulator.orbit(initial_set,HybridTerminationCriterion(final_time))

    print("Plotting simulation trajectory..")
    plot("simulation_t-height",Axes2d(0,TimeVariable(),30,5,height,9),orbit)
    plot("simulation_t-aperture",Axes2d(0,TimeVariable(),30, -0.1,aperture,1.1),orbit)
    plot("simulation_height-aperture",Axes2d(5,height,9, -0.1,aperture,1.1),orbit)
    print("Done computing and plotting simulation trajectory..")


def create_evolver(system):
    evolver = GeneralHybridEvolver(system)

    # Set the evolver configuration
    evolver.configuration().set_maximum_enclosure_radius(3.0)
    evolver.configuration().set_maximum_step_size(0.25)

    print("evolver.configuration() =",evolver.configuration())

    return evolver


def compute_evolution(evolver,initial_set,final_time):
    aperture = RealVariable("aperture")
    height = RealVariable("height")
    time = TimeVariable()

    print("Computing evolution flow tube...")
    orbit = evolver.orbit(initial_set,HybridTerminationCriterion(final_time),Semantics.UPPER)

    print("Plotting evolution flow tube...")
    tmax = Approximation[FloatDP](final_time.continuous_time().get(dp)).get_d()
    plot("finite_evolution_t-height",Axes2d(0,time,tmax, 5,height,9),orbit)
    plot("finite_evolution_t-aperture",Axes2d(0,time,tmax, -0.1,aperture,1.1),orbit)
    plot("finite_evolution_height-aperture",Axes2d(5,height,9, -0.1,aperture,1.1),orbit)
    print("Done computing and plotting evolution flow tube!\n")


def create_analyser(evolver):
    analyser = HybridReachabilityAnalyser(evolver)

    analyser.configuration().set_maximum_grid_fineness(6)
    analyser.configuration().set_lock_to_grid_time(5)

    print("analyser.configuration() =",analyser.configuration())

    return analyser


def compute_reachability(analyser,initial_set,final_time):
    aperture = RealVariable("aperture")
    height = RealVariable("height")

    print("Computing upper reach set...")
    upper_reach = analyser.upper_reach(initial_set,final_time)
    print("Plotting upper reach set...")
    plot("upper_reach",Axes2d(5,height,9, -0.1,aperture,1.1),upper_reach)
    print("Done computing and plotting upper reach set!\n")

    print("Computing outer chain reach set...")
    outer_chain_reach = analyser.outer_chain_reach(initial_set)
    print("Plotting outer chain reach set...")
    plot("outer_chain_reach",Axes2d(5,height,9, -0.1,aperture,1.1),outer_chain_reach)
    print("Done computing and plotting outer chain reach set!\n")


def get_system():
    system = CompositeHybridAutomaton("watertank",[get_tank(),get_valve(),get_controller()])
    print("system =",system)
    return system


def get_initial_set():
    height = RealVariable("height")
    valve = StringVariable("valve")
    controller = StringVariable("controller")
    opened = String("opened")
    rising = String("rising")

    initial_set = HybridBoundedConstraintSet({valve:opened,controller:rising},[(dec_(6.9)<=height)&(height<=7)])

    print("initial_set =",initial_set)

    return initial_set


def get_final_time():
    final_time = HybridTime(dec_(60.0),5)
    print("final_time =",final_time)

    return final_time


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
