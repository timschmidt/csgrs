/*****************************************************************************************************
 *            skin-exposure.hpp
 *
 *  Copyright  2016  Luca Geretti
 *
 * Provides the behavior of a measure of power over the skin point when the laser is close.
 *
 *****************************************************************************************************/

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

#include "ariadne.hpp"

#ifndef SKIN_EXPOSURE_H_
#define SKIN_EXPOSURE_H_

namespace Ariadne {

HybridAutomaton getSkinExposure()
{
    /// Parameters
	RealConstant velocity("velocity",0.092_dec);
	RealConstant L("L",0.00025_dec);
	RealConstant x0("x0",0.0023_dec);

    /// Build the Hybrid System

    /// Create a HybridAutomaton object
	AtomicHybridAutomaton automaton("skin-exposure");

    /// Create the discrete states
	AtomicDiscreteLocation close("close");
	AtomicDiscreteLocation far("far");

    RealVariable x("x");
    RealVariable vx("vx");
    RealVariable p("p");

    // Events
    DiscreteEvent comes("comes");
    DiscreteEvent leaves("leaves");

	RealExpression sqr_distance = sqr(x-x0);

	automaton.new_mode(far, {dot(p)=0});
	automaton.new_mode(close, {dot(p)=-vx*pi/L/L * (x-x0) * sin(Ariadne::pi/L/L * sqr_distance)});

	automaton.new_transition(far,comes,close,{next(p)=0},sqr_distance<=L*L);
	automaton.new_transition(close,leaves,far,{next(p)=0},sqr_distance>=L*L);

	return automaton;
}

}

#endif
