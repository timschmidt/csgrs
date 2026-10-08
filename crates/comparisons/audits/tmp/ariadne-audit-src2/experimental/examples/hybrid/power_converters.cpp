/***************************************************************************
 *            power_converters.cpp
 *
 *  Copyright  2017-20  Luca Geretti, Pieter Collins
 *
 ****************************************************************************/

/*
 *  This file is part of Ariadne.
 *
 *  Ariadne is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Ariadne is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with Ariadne.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "power_converters.hpp"
#include <cstring>

namespace Ariadne {

using std::cout;

void verify_buck() {
    auto problem = make_buck_problem();

    auto initial_set = problem.initial_set;
    auto system = problem.system;
    HybridConstraintSet safe_set = problem.safe_set;

    cout << "initial_set=" << initial_set << "\n";
    cout << "system=" << system << "\n";
    cout << "safe_set=" << safe_set << "\n";

    cout << "state_space=" << system.state_space() << "\n";
    DiscreteLocation initial_location = initial_set.location();
    cout << "initial_location=" << initial_location << "\n";
    RealSpace initial_space = system.state_space()[initial_location];
    cout << "initial_space=" << initial_space << "\n";
    BoundedConstraintSet initial_constraint_set = initial_set.euclidean_set(initial_location,initial_space);
    cout << "initial_constraint_set=" << initial_constraint_set << "\n";

    HybridBoxesSet bounding_set({swtch|open,swtch|closed},{-Vref<=V<=Vref+Verr,-1<=I<=5,0<=tau<=T});
    HybridBoundedConstraintSet bounded_safe_set=intersection(bounding_set,safe_set);
    cout << "bounded_safe_set=" << bounded_safe_set << "\n";


    MaximumError max_err=0.01;
    TaylorSeriesIntegrator integrator(max_err);
    integrator.set_maximum_step_size(8);
    integrator.set_maximum_step_size(1);
    auto& factory=integrator.function_factory();

    GeneralHybridEvolver evolver(system);
    evolver.set_integrator(integrator);
    evolver.verbosity=1;

    HybridReachabilityAnalyser analyser(system,evolver);
    analyser.configuration().set_transient_time(4.5);
    analyser.configuration().set_lock_to_grid_time(1.0);
    analyser.configuration().set_maximum_grid_depth(3);
    analyser.verbosity=1;


/*    cout << "\nComputing orbit...\n";
    Integer n=20; Real tmax=n*T;
    HybridTime evolution_time(tmax,2*n);
    auto orbit=evolver.orbit(initial_set,evolution_time,Semantics::UPPER);

    cout << "\nDrawing orbit...\n";
    plot("buck",{0<=V<=1,-1<=I<=1},Colour(.5,.0,.5),orbit.reach());
    plot("buck-V",{0<=t<=tmax,0<=V<=1},Colour(.5,.0,.5),orbit.reach());
    plot("buck-I",{0<=t<=tmax,0<=I<=1},Colour(.5,.0,.5),orbit.reach());
*/

    cout << "\nComputing safety...\n";
    auto safety = analyser.verify_safety(initial_set,bounded_safe_set);

    cout << "\nDrawing safety...\n";
    HybridFigure g; g.set_axes({0<=V<=Vref+10*Verr,0<=I<=5});
    g.clear(); g.set_fill_colour(1.0,0.75,0.75); g.draw(safety.safe_set); g.draw(bounded_safe_set); g.set_fill_colour(0.5,0,0.5); g.draw(safety.chain_reach_set); g.set_fill_colour(0.5,0,0.0); g.write("buck-safety");

}


void test() {

    verify_buck();
}

} // namespace Ariadne

int main() {
    Ariadne::test();
}
