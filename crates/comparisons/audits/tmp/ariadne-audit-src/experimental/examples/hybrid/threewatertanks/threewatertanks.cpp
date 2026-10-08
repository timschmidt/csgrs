/***************************************************************************
 *            twowatertanks.cpp
 *
 *  Copyright  2017	Luca Geretti
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

#include <cstdarg>
#include "ariadne.hpp"
#include "upper_tank1.hpp"
#include "upper_tank2.hpp"
#include "lower_tank3.hpp"
#include "controller1.hpp"
#include "controller2.hpp"
#include "controller3.hpp"
#include "valve1.hpp"
#include "valve2.hpp"
#include "valve3.hpp"

using namespace Ariadne;
using std::cout; using std::endl;

Int main(Int argc, const char* argv[])
{
    if (not CommandLineInterface::instance().acquire(argc,argv)) return -1;

    // Declare the shared system variables
    RealVariable aperture1("aperture1");
    RealVariable aperture2("aperture2");
    RealVariable aperture3("aperture3");
    RealVariable height1("height1");
    RealVariable height2("height2");
    RealVariable height3("height3");

    StringVariable valve1("valve1");
    StringVariable valve2("valve2");
    StringVariable valve3("valve3");
    StringVariable controller1("controller1");
    StringVariable controller2("controller2");
    StringVariable controller3("controller3");

    StringConstant opened1("opened1");
    StringConstant opened2("opened2");
    StringConstant opened3("opened3");

    StringConstant rising1("rising1");
    StringConstant rising2("rising2");
    StringConstant rising3("rising3");

    HybridAutomaton tank1_automaton = getTank1();
    HybridAutomaton tank2_automaton = getTank2();
    HybridAutomaton tank3_automaton = getTank3();
    HybridAutomaton valve1_automaton = getValve1();
    HybridAutomaton valve2_automaton = getValve2();
    HybridAutomaton valve3_automaton = getValve3();
    HybridAutomaton controller1_automaton = getController1();
    HybridAutomaton controller2_automaton = getController2();
    HybridAutomaton controller3_automaton = getController3();
    CompositeHybridAutomaton threewatertanks_system({tank1_automaton,tank2_automaton,tank3_automaton,
                                                     valve1_automaton,valve2_automaton,valve3_automaton,
                                                     controller1_automaton,controller2_automaton,controller3_automaton});

    CONCLOG_PRINTLN(threewatertanks_system);

    // Create a GeneralHybridEvolver object
    GeneralHybridEvolver evolver(threewatertanks_system);

    // Set the evolution parameters
    evolver.configuration().set_maximum_enclosure_radius(3.05);
    evolver.configuration().set_maximum_step_size(0.1);

    // Declare the type to be used for the system evolution
    typedef GeneralHybridEvolver::OrbitType OrbitType;

    CONCLOG_PRINTLN("Computing evolution... ");
    HybridSet initial_set({valve1|opened1,valve2|opened2,valve3|opened3,controller1|rising1,controller2|rising2,controller3|rising3},
                          {height1==7,height2==7,height3==7});
    HybridTime evolution_time(35.0_x,15);
    OrbitType orbit = evolver.orbit(initial_set,evolution_time,Semantics::UPPER);

    CONCLOG_PRINTLN("Plotting trajectory... ");
    Axes2d time_height1_axes(0<=TimeVariable()<=evolution_time.continuous_time(),-0.1<=height1<=9.1);
    plot("watertank-t-height1",time_height1_axes, Colour(0.0,0.5,1.0), orbit);
    Axes2d time_height2_axes(0<=TimeVariable()<=evolution_time.continuous_time(),-0.1<=height2<=9.1);
    plot("watertank-t-height2",time_height2_axes, Colour(0.0,0.5,1.0), orbit);
    Axes2d time_height3_axes(0<=TimeVariable()<=evolution_time.continuous_time(),-0.1<=height3<=12.1);
    plot("watertank-t-height3",time_height3_axes, Colour(0.0,0.5,1.0), orbit);
    Axes2d height1_aperture1_axes(-0.1,height1,9.1, -0.1,aperture1,1.1);
    plot("watertank-height1-aperture1",height1_aperture1_axes, Colour(0.0,0.5,1.0), orbit);
    Axes2d height2_aperture2_axes(-0.1,height2,9.1, -0.1,aperture2,1.1);
    plot("watertank-height2-aperture1",height2_aperture2_axes, Colour(0.0,0.5,1.0), orbit);
    Axes2d height1_height2_axes(-0.1,height1,9.1, -0.1,height2,9.1);
    plot("watertank-height1-height2",height1_height2_axes, Colour(0.0,0.5,1.0), orbit);
    Axes2d aperture1_aperture2_axes(-0.1,aperture1,1.1, -0.1,aperture2,1.1);
    plot("watertank-aperture1-aperture2",aperture1_aperture2_axes, Colour(0.0,0.5,1.0), orbit);
}
