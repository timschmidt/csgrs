/***************************************************************************
 *            watertank-proportional.cpp
 *
 *  Copyright  2017  Luca Geretti
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

#include "ariadne_main.hpp"
#include "tank.hpp"
#include "valve-proportional-urgent.hpp"

void ariadne_main()
{
    // Declare the shared system variables
    RealVariable aperture("aperture");
    RealVariable height("height");

    StringVariable valve("valve");
    StringConstant opened("opened");
    StringConstant modulated("modulated");
    StringConstant closed("closed");

    // Get the automata and compose them
    HybridAutomaton tank_automaton = getTank();
    HybridAutomaton valve_automaton = getValve();
    CompositeHybridAutomaton watertank_system({tank_automaton,valve_automaton});

    // Print the system description on the command line
    CONCLOG_PRINTLN_VAR(watertank_system)

    // Compute the system evolution

    // Create a GeneralHybridEvolver object
    GeneralHybridEvolver evolver(watertank_system);
    // Set the evolution parameters
    evolver.configuration().set_maximum_enclosure_radius(3.05); // The maximum size of an evolved set before early termination
    evolver.configuration().set_maximum_step_size(0.25); // The maximum value that can be used as a time step for integration

    // Declare the type to be used for the system evolution
    typedef GeneralHybridEvolver::OrbitType OrbitType;

    CONCLOG_PRINTLN("Computing evolution...")

    // Define the initial set, by supplying the location as a list of locations for each composed automata, and
    // the continuous set as a list of variable assignments for each variable controlled on that location
    // (the assignment can be either a singleton value using the == symbol or an interval using the <= symbols)
    HybridSet initial_set({valve|opened},{height==0});
    // Define the evolution time: continuous time and maximum number of transitions
    HybridTime evolution_time(80.0_x,5);
    // Compute the orbit using upper semantics
    OrbitType orbit = evolver.orbit(initial_set,evolution_time,Semantics::UPPER);
    CONCLOG_PRINTLN("done.")

    // Plot the trajectory using two different projections
    CONCLOG_PRINTLN("Plotting trajectory... ")
    Axes2d time_height_axes(0<=TimeVariable()<=80,-0.1<=height<=9.1);
    plot("watertank_proportional_t-height",time_height_axes, Colour(0.0,0.5,1.0), orbit);
    Axes2d height_aperture_axes(-0.1,height,9.1, -0.1,aperture,1.3);
    plot("watertank_proportional_height-aperture",height_aperture_axes, Colour(0.0,0.5,1.0), orbit);
    CONCLOG_PRINTLN("done.")
}
