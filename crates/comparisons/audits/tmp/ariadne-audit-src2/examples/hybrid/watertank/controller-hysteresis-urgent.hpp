/***************************************************************************
 *            controller-urgent.hpp
 *
 *  Copyright  2018 Luca Geretti
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

using namespace Ariadne;

HybridAutomaton getController()
{
    // Declare some constants. Note that system parameters should be given as variables.
    RealConstant hmin("hmin",5.75_decimal);
    RealConstant hmax("hmax",7.75_decimal);

    // Declare the shared system variables
    RealVariable height("height");

    // Declare the events we use
    DiscreteEvent e_can_open("can_open");
    DiscreteEvent e_can_close("can_close");

    // Declare the variable for the automaton name
    StringVariable controller("controller");

    // Create the controller automaton
    HybridAutomaton automaton(controller.name());

    // Declare the locations for the controller
    DiscreteLocation rising(controller|"rising");
    DiscreteLocation falling(controller|"falling");

    // Instantiate modes for each location; since no dynamics is present, we create an empty list
    automaton.new_mode(rising,List<RealAssignment>());
    automaton.new_mode(falling,List<RealAssignment>());

    // Specify the transitions, starting from the source location, according to an event, to a target location;
    // Following those arguments you specify a guard and whether the event is permissive or urgent.
    automaton.new_transition(falling,e_can_open,rising,height<=hmin,EventKind::URGENT);
    automaton.new_transition(rising,e_can_close,falling,height>=hmax,EventKind::URGENT);

    return automaton;
}
