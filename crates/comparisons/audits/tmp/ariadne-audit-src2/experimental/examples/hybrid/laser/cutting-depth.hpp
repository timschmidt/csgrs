/*****************************************************************************************************
 *            cutting_depth.hpp
 *
 *  Copyright  2016  Luca Geretti
 *
 * Provides the behavior of the cutting depth under a laser.
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

#ifndef CUTTING_DEPTH_H
#define CUTTING_DEPTH_H

namespace Ariadne {

HybridAutomaton getCuttingDepth()
{
    /// Parameters
	RealConstant kcut("kcut",1.78833087e-8_dec);
	RealConstant lambda("lambda",6825.5643_dec);
	RealConstant mu("mu",6.03796e5_dec);
	RealConstant T0("T0",37.0_dec);
	RealConstant Tevap("Teval",100.0_dec);
	RealConstant z_thr("z_thr",30e-6_dec);

    /// Build the Hybrid System

    /// Create a HybridAutomaton object
	AtomicHybridAutomaton automaton("depth");

    /// Create the discrete states
	AtomicDiscreteLocation ablating("ablating");
	AtomicDiscreteLocation idle("idle");
	AtomicDiscreteLocation carbonization("carbonization");

    RealVariable p("p");
    RealVariable z("z"); // The cutting depth
    RealVariable zi("zi"); // The depth for each pass of the laser

    // Events
    DiscreteEvent start_evaporating("start_evaporating");
    DiscreteEvent stop_evaporating("stop_evaporating");
    DiscreteEvent start_carbonization("start_carbonization");
    DiscreteEvent must_stop("must_stop");

    RealExpression z_der = kcut*(mu*p - lambda*(Tevap-T0));

	automaton.new_mode(ablating,{dot(z)=z_der,dot(zi)=z_der});
	automaton.new_mode(carbonization,{dot(z)=0,dot(zi)=0});
	automaton.new_mode(idle,{dot(z)=0,dot(zi)=-lambda*zi});

	/// Invariants
	ContinuousPredicate always_false = Real(1) < 0;
	automaton.new_invariant(carbonization,always_false,must_stop);

	/// Transitions
	automaton.new_transition(idle,start_evaporating,ablating);
	automaton.new_transition(ablating,stop_evaporating,idle,z_der<=0);
	automaton.new_transition(ablating,start_carbonization,carbonization,zi>=z_thr);

	return automaton;

}

}

#endif
