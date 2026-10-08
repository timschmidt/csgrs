/***************************************************************************
 *            valve.hpp
 *
 *  Copyright  2017 Luca Geretti
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

using namespace Ariadne;

inline HybridAutomaton getValve1()
{

    // Declare some constants. Note that system parameters should be given as variables.
    RealConstant T("T",4.0_decimal);

    // Declare the shared system variables
    RealVariable aperture("aperture1");

    // Declare the events we use
    DiscreteEvent e_idle("idle1");
    DiscreteEvent e_can_open("can_open1");
    DiscreteEvent e_can_close("can_close1");

    StringVariable valve("valve1");
    HybridAutomaton automaton(valve.name());

    // Declare the values the valve can variable can have
    DiscreteLocation closed(valve|"closed1");
    DiscreteLocation opened(valve|"opened1");
    DiscreteLocation opening(valve|"opening1");
    DiscreteLocation closing(valve|"closing1");

    // Since aperture is a known constant when the valve is open or closed,
    // specify aperture by an algebraic equation.
    automaton.new_mode(closed,{let(aperture)=0});
    automaton.new_mode(opened,{let(aperture)=1});
    automaton.new_mode(opening,{dot(aperture)=+1/T});
    automaton.new_mode(closing,{dot(aperture)=-1/T});

    automaton.new_transition(opening,e_idle,opened,aperture>=1.0_dec,EventKind::URGENT);
    automaton.new_transition(closing,e_idle,closed,aperture<=0.0_dec,EventKind::URGENT);
    automaton.new_transition(closed,e_can_open,opening,{next(aperture)=aperture});
    automaton.new_transition(opened,e_can_close,closing,{next(aperture)=aperture});
    automaton.new_transition(opening,e_can_close,closing,{next(aperture)=aperture});
    automaton.new_transition(closing,e_can_open,opening,{next(aperture)=aperture});

    return automaton;
}
