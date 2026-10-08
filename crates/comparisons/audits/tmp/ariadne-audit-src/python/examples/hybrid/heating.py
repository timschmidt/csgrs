#!/usr/bin/python3

##############################################################################
#            heating.py
#
#  Copyright  2008-24  Pieter Collins
##############################################################################

# Ariadne is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# Ariadne is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with Ariadne. If not, see <https:#www.gnu.org/licenses/>.

from pyariadne import *

if __name__=='__main__':
    # Create the system
    # Set the system dynamic parameters
    P=RealConstant("P",Decimal(4.0))
    K=RealConstant("K",Decimal(1.0))
    Tav=RealConstant("Tav",Decimal(16.0))
    Tamp=RealConstant("Tamp",Decimal(8.0))

    # Set the system control parameters
    Tmax=RealConstant("Tmax",Decimal(23.0))
    Tmin=RealConstant("Tmin",Decimal(14.0))
    Toff=RealConstant("Toff",Decimal(21.0))
    Ton=RealConstant("Ton",Decimal(15.0))
    Ton_upper=RealConstant("Ton_upper",Decimal(15.25))
    Ton_lower=RealConstant("Ton_lower",Decimal(14.75))

    # Create the discrete states
    heating=StringVariable("heating")
    on=StringConstant("on")
    off=StringConstant("off")

    # Create the discrete events
    must_switch_on=DiscreteEvent("must_switch_on")
    switch_on=DiscreteEvent("switch_on")
    switch_off=DiscreteEvent("switch_off")
    midnight=DiscreteEvent("midnight")

    # Declare the system variables.
    T=RealVariable("T")
    C=RealVariable("C")
    t=TimeVariable()

    # Create the heater subsystem
    heater=HybridAutomaton()
    heater.new_mode( {heating:on}, {dot(T):P+K*(Tav-Tamp*cos(2*pi*C)-T)} )
    heater.new_mode( {heating:off}, {dot(T):K*(Tav-Tamp*cos(2*pi*C)-T)} )
    heater.new_invariant( {heating:off}, T>=Ton_lower, must_switch_on )
    heater.new_transition( {heating:off}, switch_on, {heating:on}, {next(T):T}, T<=Ton_upper, EventKind.PERMISSIVE )
    # Comment out above two lines and uncomment the line below to make the switch_on transition urgent
    #heater.new_transition( {heating:off}, switch_on, {heating:on}, {next(T):T}, T<=Ton_upper, EventKind.URGENT )
    heater.new_transition( {heating:on}, switch_off, {heating:off}, {next(T):T}, T>=Toff, EventKind.URGENT )

    # Create the clock subsystem
    clock=HybridAutomaton()
    clock.new_mode( {dot(C):1} )
    clock.new_transition( {},midnight, {}, {next(C):0}, C>=1, EventKind.URGENT )

    heating_system=CompositeHybridAutomaton([clock,heater])
    print(heating_system)

    # Create the analyser classes

    series_integrator=GradedTaylorSeriesIntegrator(1e-3)
    series_integrator.set_maximum_spacial_order(6)
    series_integrator.set_maximum_temporal_order(12)
    picard_integrator=TaylorPicardIntegrator(1e-5)
    solver=IntervalNewtonSolver(1e-12,8)

    # Create a GeneralHybridEvolver object
    evolver=GeneralHybridEvolver(heating_system)
#    evolver.set_solver(solver)

    # Set the evolution parameters
    evolver.configuration().set_maximum_enclosure_radius(0.25)
    evolver.configuration().set_maximum_step_size(7.0/16)
    print(evolver.configuration())

    evolver.configuration().set_enable_reconditioning(true)
    evolver.configuration().set_enable_subdivisions(true)


    # Set colours for drawing
    guard_colour=Colour(0.5,0.5,0.5)
    midnight_guard_colour=Colour(0.75,0.75,0.75)
    picard_orbit_colour=Colour(0.0,1.0,1.0)
    series_orbit_colour=Colour(0.0,0.0,1.0)
    chain_reach_on_colour=Colour(0.75,0.0,0.75)
    chain_reach_off_colour=Colour(0.0,0.0,1.0)

    # Set the initial set.
    r=1/two**10
    Ti=Dyadic("16.25")
    Tinitmin=Real(Ti+r)
    Tinitmax=Real(Ti+3*r)
    Cinitmin=Real(0+r)
    Cinitmax=Real(0+3*r)
    # Tinit=16.0
    initial_set=HybridBoundedConstraintSet({heating:off}, RealVariablesBox({T:(Tinitmin,Tinitmax),C:(Cinitmin,Cinitmax)}) )
    print(initial_set)
    # Compute the initial set as a validated enclosure.
#    initial_enclosure = evolver.enclosure(initial_set)
#    print(initial_enclosure)
    initial_enclosure=initial_set

    evolution_time=HybridTime(Dyadic("2.75"),127)
    evolution_time=HybridTime(Dyadic("0.75"),127)
    termination_criterion=HybridTerminationCriterion(evolution_time)
    print(evolution_time)

    print("Compute orbit using series integrator.")
    evolver.set_integrator(series_integrator)
    series_orbit = evolver.orbit(initial_enclosure,termination_criterion,Semantics.UPPER)

    print("Computed",len(series_orbit.reach()),"reach enclosures and",len(series_orbit.final()),"final enclosures.")

    tmax=evolution_time.continuous_time()
    dTmin=Tmin.value()
    dTmax=Tmax.value()
    guard=HybridVariablesBox({heating:off},RealVariablesBox({T:(Ton_lower.value(),Ton_upper.value()),C:(0,1),t:(0,tmax)}))
    midnight_guard=HybridVariablesBox({heating:off},RealVariablesBox({T:(dTmin,dTmax),C:(0,1),t:(1,2)}))
    print("Plotting time trace of orbit... ",end='')
#    plot("heating-orbit-time.png",Axes2d(0,t,tmax, dTmin,T,dTmax), midnight_guard_colour, midnight_guard, guard_colour, guard, series_orbit_colour, series_orbit)
    print("done.")

    evolution_termination=HybridTerminationCriterion(Dyadic("2.75"),127,{midnight})
    print(evolution_termination)

    print("Computing event-terminated orbit using series integrator...")
    evolver.set_integrator(series_integrator)
    series_orbit = evolver.orbit(initial_enclosure,evolution_termination,Semantics.UPPER)
    print("done.")

    print("Computed",len(series_orbit.reach()),"reach enclosures and",len(series_orbit.final()),"final enclosures.")

    print("Plotting time trace of orbit... ")
#    plot("heating-orbit-termination.png",Axes2d(0.0<=t<=1.25,dTmin<=T<=dTmax), midnight_guard_colour, midnight_guard, guard_colour, guard, series_orbit_colour, series_orbit)
    print("done.")

    analyser=HybridReachabilityAnalyser(evolver)
    analyser.configuration().set_lock_to_grid_time(1+1.0/1024)
    analyser.configuration().set_lock_to_grid_steps(1)
    analyser.configuration().set_scaling(T,8.0)
    analyser.configuration().set_scaling(C,1.0)
    analyser.configuration().set_maximum_grid_fineness(5)
    print(analyser.configuration())

    print("Compute chain-reachable set.")
    chain_reach_set = analyser.outer_chain_reach(initial_set)
    print("Plot chain-reachable set.")
    chain_reach_set_off=chain_reach_set
    chain_reach_set_off[heating|on].clear()
    chain_reach_set_on=chain_reach_set
    chain_reach_set_on[heating|off].clear()
    plot("heating-chainreach.png",Axes2d(0.0<=C<=1.0,dTmin<=T<=dTmax), guard_colour, guard, chain_reach_off_colour, chain_reach_set_off, chain_reach_on_colour, chain_reach_set_on)
    plot("heating-chainreach-off.png",Axes2d(0.0<=C<=1.0,dTmin<=T<=dTmax), guard_colour, guard, chain_reach_off_colour, chain_reach_set_off)
    plot("heating-chainreach-on.png",Axes2d(0.0<=C<=1.0,dTmin<=T<=dTmax), guard_colour, guard, chain_reach_on_colour, chain_reach_set_on)

