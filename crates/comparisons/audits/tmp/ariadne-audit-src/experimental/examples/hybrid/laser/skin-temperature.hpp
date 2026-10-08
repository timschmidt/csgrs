/*****************************************************************************************************
 *            skin_temperature.hpp
 *
 *  Copyright  2016  Luca Geretti
 *
 * Provides the behavior of the skin temperature under laser cutting.
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

#ifndef SKIN_TEMPERATURE_H_
#define SKIN_TEMPERATURE_H_

namespace Ariadne {

HybridAutomaton getSkinTemperature()
{
    /// Parameters
	RealConstant lambda("lambda",6825.5643_dec);
	RealConstant mu("mu",6.03796e5_dec);
	RealConstant T0("T0",37.0_dec);
	RealConstant Tevap("Tevap",100.0_dec);

    /// Build the Hybrid System

    /// Create a HybridAutomaton object
	AtomicHybridAutomaton automaton("skin-T");

    /// Create the discrete states
	AtomicDiscreteLocation varying("varying");
	AtomicDiscreteLocation evaporating("evaporating");

    RealVariable p("p");
    RealVariable T("T");

    // Events
    DiscreteEvent start_evaporating("start_evaporating");
    DiscreteEvent stop_evaporating("stop_evaporating");

    automaton.new_mode(varying,{dot(T)=mu*p - lambda*(T-T0)});
	automaton.new_mode(evaporating,{dot(T)=0});

	automaton.new_transition(varying,start_evaporating,evaporating,{next(T)=Tevap},T>=Tevap);
	automaton.new_transition(evaporating,stop_evaporating,varying);

	return automaton;
}

}

#endif
