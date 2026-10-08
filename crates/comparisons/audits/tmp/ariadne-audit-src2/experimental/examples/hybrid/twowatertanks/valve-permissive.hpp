/***************************************************************************
 *            valve-permissive.hpp
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

inline HybridAutomaton getValve()
{

    // Declare some constants. Note that system parameters should be given as variables.
    RealConstant T("T",4.0_decimal);
    RealConstant hmin("hmin",5.5_decimal);
    RealConstant delta("delta",0.05_decimal);

    // Declare the shared system variables
    RealVariable aperture1("aperture1");
    RealVariable aperture2("aperture2");
    RealVariable height1("height1");
    RealVariable height2("height2");

    // Declare the events we use
    DiscreteEvent start_towards1("start_towards1");
    DiscreteEvent start_towards2("start_towards2");
    DiscreteEvent finished_towards1("finished_towards1");
    DiscreteEvent finished_towards2("finished_towards2");
    DiscreteEvent must_start_towards1("must_start_towards1");
    DiscreteEvent must_start_towards2("must_start_towards2");

    StringVariable valve("valve");
    HybridAutomaton automaton(valve.name());

    // Declare the values the valve can variable can have
    DiscreteLocation fully1(valve|"fully1");
    DiscreteLocation towards1(valve|"towards1");
    DiscreteLocation fully2(valve|"fully2");
    DiscreteLocation towards2(valve|"towards2");

    // Since aperture is a known constant when the valve is open or closed,
    // specify aperture by an algebraic equation.
    automaton.new_mode(fully1,{let(aperture1)=+1.0_decimal,let(aperture2)=0.0_decimal});
    automaton.new_mode(fully2,{let(aperture1)=0.0_decimal,let(aperture2)=+1.0_decimal});
    // Specify the differential equation for how the valve opens/closes.
    automaton.new_mode(towards1,{dot(aperture1)=+1/T,dot(aperture2)=-1/T});
    automaton.new_mode(towards2,{dot(aperture1)=-1/T,dot(aperture2)=+1/T});

    // Specify the invariants valid in each mode. Note that every invariant
    // must have an action label. This is used internally, for example, to
    // check non-blockingness of urgent actions.
    automaton.new_invariant(fully1,height2<=hmin-delta,must_start_towards2);
    automaton.new_invariant(fully2,height1<=hmin-delta,must_start_towards1);

    automaton.new_transition(fully1,start_towards2,towards2,{next(aperture1)=aperture1,next(aperture2)=aperture2},height2<=hmin+delta,EventKind::PERMISSIVE);
    automaton.new_transition(fully2,start_towards1,towards1,{next(aperture1)=aperture1,next(aperture2)=aperture2},height1<=hmin+delta,EventKind::PERMISSIVE);
    automaton.new_transition(towards1,finished_towards1,fully1,aperture1>=1,EventKind::URGENT);
    automaton.new_transition(towards2,finished_towards2,fully2,aperture2>=1,EventKind::URGENT);

    return automaton;
}
