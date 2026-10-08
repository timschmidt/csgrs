/*****************************************************************************************************
 *            laser-trajectory.hpp
 *
 *  Copyright  2016  Luca Geretti
 *
 * Provides the trajectory of the laser.
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

#ifndef LASER_TRAJECTORY_H_
#define LASER_TRAJECTORY_H_

namespace Ariadne {

HybridAutomaton getLaserTrajectory()
{
    /// Build the Hybrid System

    /// Create a HybridAutomaton object
	AtomicHybridAutomaton automaton("trajectory");

    // Parameters
    RealConstant velocity("velocity",0.092_dec); // Velocity in modulus
    RealConstant width("width",0.0046_dec); // Width of the cut

    /// Modes

    AtomicDiscreteLocation scanning("scanning");

    // Variables

    RealVariable x("x"); // X position
    RealVariable vx("vx"); // X velocity

    DiscreteEvent switch_left("switch_left");
    DiscreteEvent switch_right("switch_right");

	automaton.new_mode(scanning, {dot(x)=vx,dot(vx)=0});

	automaton.new_guard(scanning,switch_right,x<=0,EventKind::IMPACT);
	automaton.new_update(scanning,switch_right,scanning,{next(x)=0,next(vx)=velocity});

	automaton.new_guard(scanning,switch_left,x>=width,EventKind::IMPACT);
	automaton.new_update(scanning,switch_left,scanning,{next(x)=width,next(vx)=-velocity});

	return automaton;
}

}

#endif
