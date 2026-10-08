#!/usr/bin/python3

##############################################################################
#            attractor.py
#
#  Copyright  2009-21  Luca Geretti
##############################################################################

# This file is part of Ariadne.

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

x = RealVariable("x")
y = RealVariable("y")

system = VectorField([dot(x)<<2*x-x*y,dot(y)<<2*x*x-y])
initial_set = RealExpressionBoundedConstraintSet([(dec_(0.9)<=x)&(x<=1),(dec_(-2.2)<=y)&(y<=-2)],[sqr(x)+sqr(y+2)<=1])
safe_set = RealExpressionBoundedConstraintSet([(-1<=x)&(x<=4),(-4<=y)&(y<=6)],[sqr(x-2)+sqr(y-1)<=22])
print(system)
print(initial_set)
print(safe_set)

initial_constraint_set = initial_set.euclidean_set(system.state_space())
safe_constraint_set = safe_set.euclidean_set(system.state_space())
print(initial_constraint_set)
print(safe_constraint_set)

evolution_time = Real(dec_(50.0))

simulator = VectorFieldSimulator(system)
simulator.configuration().set_step_size(0.1)
print("Simulating...")
simulator_orbit = simulator.orbit(initial_set,evolution_time)

g = LabelledFigure(Axes2d(-2,x,5, -4,y,6))
g.draw(simulator_orbit)
g.write("attractor_simulation")

integrator = TaylorPicardIntegrator(0.01)
print("Evolving...")
evolver = VectorFieldEvolver(system,integrator)
evolver.configuration().set_maximum_step_size(0.1)
evolver_orbit = evolver.orbit(initial_set,evolution_time,Semantics.UPPER)
g.clear()
g.draw(evolver_orbit)
g.write("attractor_evolution")

analyser = ContinuousReachabilityAnalyser(evolver)
analyser.configuration().set_transient_time(dec_(0.75))
analyser.configuration().set_lock_to_grid_time(dec_(0.75))
analyser.configuration().set_maximum_grid_extent(5)

print("Computing safety...")
safety = analyser.verify_safety(initial_constraint_set,safe_constraint_set)
print("safety.is_safe =", safety.is_safe)
g.clear()
g.properties().set_fill_colour(lightgrey)
g.draw(safety.safe_set)
g.properties().set_fill_colour(orange)
g.draw(safety.chain_reach_set)
g.write("attractor_chain_reach")