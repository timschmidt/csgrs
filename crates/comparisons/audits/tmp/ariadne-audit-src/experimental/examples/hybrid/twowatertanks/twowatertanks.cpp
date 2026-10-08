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

#include "ariadne.hpp"
#include "tank1.hpp"
#include "tank2.hpp"
#include "valve-urgent.hpp"

using namespace Ariadne;
using std::cout; using std::endl;

Int main(Int argc, const char* argv[])
{
    if (not CommandLineInterface::instance().acquire(argc,argv)) return -1;

    // Declare the shared system variables
    RealVariable aperture1("aperture1");
    RealVariable aperture2("aperture2");
    RealVariable height1("height1");
    RealVariable height2("height2");

    StringVariable valve("valve");
    StringConstant fully1("fully1");
    StringConstant towards1("towards1");
    StringConstant fully2("fully2");
    StringConstant towards2("towards2");

    HybridAutomaton tank1_automaton = getTank1();
    HybridAutomaton tank2_automaton = getTank2();
    HybridAutomaton valve_automaton = getValve();
    CompositeHybridAutomaton twowatertanks_system({tank1_automaton,tank2_automaton,valve_automaton});

    cout << twowatertanks_system << endl;

    // Create a GeneralHybridEvolver object
    GeneralHybridEvolver evolver(twowatertanks_system);

    // Set the evolution parameters
    evolver.configuration().set_maximum_enclosure_radius(3.05);
    evolver.configuration().set_maximum_step_size(1.0);

    // Declare the type to be used for the system evolution
    typedef GeneralHybridEvolver::OrbitType OrbitType;

    std::cout << "Computing evolution... " << std::flush;
    HybridSet initial_set({valve|fully1},{height1==2,height2==4});
    HybridTime evolution_time(200.0_x,16);
    OrbitType orbit = evolver.orbit(initial_set,evolution_time,Semantics::UPPER);
    std::cout << "done." << std::endl;

    std::cout << "Plotting trajectory... "<<std::flush;
    Axes2d time_height1_axes(0<=TimeVariable()<=evolution_time.continuous_time(),-0.1<=height1<=9.1);
    plot("watertank-t-height1",time_height1_axes, Colour(0.0,0.5,1.0), orbit);
    Axes2d time_height2_axes(0<=TimeVariable()<=evolution_time.continuous_time(),-0.1<=height2<=9.1);
    plot("watertank-t-height2",time_height2_axes, Colour(0.0,0.5,1.0), orbit);
    Axes2d height1_aperture1_axes(-0.1,height1,9.1, -0.1,aperture1,1.1);
    plot("watertank-height1-aperture1",height1_aperture1_axes, Colour(0.0,0.5,1.0), orbit);
    Axes2d height2_aperture2_axes(-0.1,height2,9.1, -0.1,aperture2,1.1);
    plot("watertank-height2-aperture1",height2_aperture2_axes, Colour(0.0,0.5,1.0), orbit);
    Axes2d height1_height2_axes(-0.1,height1,9.1, -0.1,height2,9.1);
    plot("watertank-height1-height2",height1_height2_axes, Colour(0.0,0.5,1.0), orbit);
    Axes2d aperture1_aperture2_axes(-0.1,aperture1,1.1, -0.1,aperture2,1.1);
    plot("watertank-aperture1-aperture2",aperture1_aperture2_axes, Colour(0.0,0.5,1.0), orbit);
    std::cout << "done." << std::endl;
}
