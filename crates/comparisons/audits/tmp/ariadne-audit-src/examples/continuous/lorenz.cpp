/***************************************************************************
 *            lorenz.cpp
 *
 *  Copyright  2017-23  Luca Geretti, Pieter Collins
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

#include "utility/stopwatch.hpp"
#include "ariadne_main.hpp"

void ariadne_main()
{
    CONCLOG_PRINTLN("Lorenz system");

    RealConstant rho("rho",28), sigma("sigma",10), beta("beta",8/3_q);
    RealVariable x("x"), y("y"), z("z");

    VectorField dynamics({dot(x)=sigma*(y-x), dot(y)=x*(rho-z)-y, dot(z)=x*y-beta*z});

    VectorFieldSimulator simulator(dynamics);
    simulator.configuration().set_step_size(0.0078125);

    StepMaximumError max_err=1e-8;
    TaylorPicardIntegrator integrator(max_err);

    VectorFieldEvolver evolver(dynamics,integrator);
    evolver.configuration().set_maximum_enclosure_radius(1.0);
    evolver.configuration().set_maximum_step_size(0.25);
    evolver.configuration().set_maximum_spacial_error(1e-6);
    CONCLOG_PRINTLN(evolver.configuration());

    Real x0 = 1.0_dec;
    Real y0 = 1.0_dec;
    Real z0 = 1.0_dec;
    Real eps = 0.0001_dec;

    RealExpressionBoundedConstraintSet initial_set({x0-eps<=x<=x0+eps,y0-eps<=y<=y0+eps,z0-eps<=z<=z0+eps});

    CONCLOG_PRINTLN("Initial set: " << initial_set);
    Real evolution_time = 100;
    Stopwatch<Milliseconds> sw;
    CONCLOG_PRINTLN("Computing simulation...");
    auto simulation = simulator.orbit(initial_set,evolution_time);
    sw.click();
    CONCLOG_PRINTLN_AT(1,"Done in " << sw.elapsed_seconds() << " seconds.");

    CONCLOG_PRINTLN("Plotting...");;
    Axes2d axes({-32<=x<=32,-32<=y<=32});
    LabelledFigure fig=LabelledFigure(axes);
    fig.draw(simulation);
    fig.write("lorenz_simulation");

    sw.restart();
    CONCLOG_PRINTLN("Computing evolution... ");
    evolution_time = 6;
    auto evolution = evolver.orbit(initial_set,evolution_time,Semantics::UPPER);
    sw.click();
    CONCLOG_PRINTLN_AT(1,"Done in " << sw.elapsed_seconds() << " seconds.");

    CONCLOG_PRINTLN("Plotting...");
    fig.clear();
    fig.draw(evolution);
    fig.write("lorenz_evolution");
}
