#!/usr/bin/python3

##############################################################################
#            bouncingball.py
#
#  Copyright  2008-24  Davide Bresolin, Pieter Collins
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

if __name__=='__main__':

    # Set the system parameters
    a=Real(Decimal(0.5))  # Coefficient of restitution
    g=Real(Decimal(9.8))

    # Set the position and velocity functions.
    x=RealVariable("x")
    v=RealVariable("v")

    # Create an automaton object
    ball=HybridAutomaton()

    freefall=DiscreteLocation()
    bounce=DiscreteEvent("bounce")

    # Build the automaton
    ball.new_mode(freefall,{dot(x):v,dot(v):-g})
    ball.new_transition(freefall,bounce,freefall,x<=0,{next(x):x,next(v):-a*v},EventKind.IMPACT)
    # Finished building the automaton

    print("Ball:",ball)
    # Compute the system evolution

    # Create a GeneralHybridEvolver object and
    # set the evolution parameters
    evolver=GeneralHybridEvolver(ball)
    evolver.configuration().set_maximum_enclosure_radius(2.0)
    evolver.configuration().set_maximum_step_size(1.0/32)
    print(evolver)

    e=Decimal(0.0625)
    initial_set=HybridBoundedConstraintSet(freefall,RealVariablesBox({x:(2-e,2+e),v:(-e,+e)}))
    initial_set=HybridBoundedConstraintSet(freefall,{x:(2-e,2+e),v:(-e,+e)})
    evolution_time=HybridTime(Decimal(1.5),4)
    termination_criterion=HybridTerminationCriterion(evolution_time)

    print("Computing evolution... ",end='')
    orbit = evolver.orbit(initial_set,termination_criterion,Semantics.LOWER)
    print("done.")

    t = TimeVariable()
    plot("bouncingball-xv",Axes2d(-0.1,x,2.1, -10.1,v,10.1), orbit)
    plot("bouncingball-tx",Axes2d(0.0,t,1.5,-0.1,x,2.1), orbit)
