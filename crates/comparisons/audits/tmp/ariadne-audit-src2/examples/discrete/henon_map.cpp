/***************************************************************************
 *            henon_map.cpp
 *
 *  Copyright  2006-20  Pieter Collins
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

#include "ariadne_main.hpp"

void ariadne_main()
{
    // The Henon map \f$(x,y)\mapsto(a-x^2+by,x)
    Real a=Decimal(1.3), b=Decimal(0.3);
    RealVariable x("x"), y("y");
    IteratedMap henon({next(x)=a-x*x+b*y,next(y)=x});
    CONCLOG_PRINTLN_VAR(henon);

    // Compute a fixed point
    IntervalNewtonSolver solver(maximum_error=1e-2, maximum_number_of_steps=16);
    ExactBoxType search_box({{0,1},{0,1}});
    Point<FloatDPBounds> fixed_point = Point(solver.fixed_point(henon.update_function(),search_box));
    CONCLOG_PRINTLN_VAR(fixed_point);
    LabelledSet<Point<FloatDPBounds>> labelled_fixed_point(henon.state_space(),fixed_point);

    // Set up the evaluators
    IteratedMapEvolver evolver(henon);
    typedef IteratedMapEvolver::EnclosureType EnclosureType;
    ReachabilityAnalyser<IteratedMap> analyser(evolver);
    analyser.configuration().set_bounding_domain(ExactBoxType({{-4,4},{-4,4}}));
    analyser.configuration().set_maximum_grid_fineness(5);

    // Set-up initial set and time for evolution
    RealExpressionBoundedConstraintSet initial_set={0.5_dec<=x<=0.6_dec,0.95_dec<=y<=1.05_dec};

    // Set up the evolution parameters and grid
    Integer evolve_time(6);

    // Compute the reachable sets
    Orbit<EnclosureType> orbit = evolver.orbit(initial_set,evolve_time,Semantics::UPPER);

    Axes2d figure_axes(-4<=x<=2,-3<=y<=3);
    LabelledFigure fig(figure_axes);
    fig << fill_colour(magenta) << orbit.reach();
    fig << fill_colour(red) << orbit.final();
    fig << line_width(4) << fill_colour(green) << labelled_fixed_point << line_width(1);
    fig.write("henon_map-reach");

    // Compute the chain-reach set
    LabelledStorage chain_reach_set = analyser.outer_chain_reach(initial_set.euclidean_set(henon.state_space()));

    fig.clear();
    fig << chain_reach_set;
    fig.write("henon_map-chain_reach");
}
