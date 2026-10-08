/***************************************************************************
 *            bouncingball.cpp
 *
 *  Copyright  2008-20  Davide Bresolin
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

void ariadne_main()
{
    typedef GeneralHybridEvolver GeneralHybridEvolverType;

    /// Set the system parameters
    Real a = 0.5_dec;  // Coefficient of restitution
    Real g = 9.8_dec;

    /// Set the position and velocity functions.
    RealVariable x("x");
    RealVariable v("v");

    /// Create an automaton object
    HybridAutomaton ball;

    DiscreteLocation freefall;
    DiscreteEvent bounce("bounce");

    /// Build the automaton
    ball.new_mode(freefall,{dot(x)=v,dot(v)=-g});
    ball.new_guard(freefall,bounce,x<=0,EventKind::IMPACT);
    ball.new_update(freefall,bounce,freefall,{next(x)=x,next(v)=-a*v});
    /// Finished building the automaton

    CONCLOG_PRINTLN("Ball = " << ball)
    /// Compute the system evolution

    /// Create a GeneralHybridEvolver object
    GeneralHybridEvolverType evolver(ball);

    /// Set the evolution parameters
    evolver.configuration().set_maximum_enclosure_radius(2.0);
    evolver.configuration().set_maximum_step_size(1.0/32);
    CONCLOG_PRINTLN_VAR(evolver.configuration())

    Real e =0.0625_dec;
    HybridSet initial_set(freefall,{2-e<=x<=2+e,-e<=v<=e});
    HybridTime evolution_time(1.5_dec,4);

    CONCLOG_PRINTLN("Computing evolution... ")
    auto orbit = evolver.orbit(initial_set,evolution_time,Semantics::LOWER);
    CONCLOG_PRINTLN("done.")

    plot("bouncingball-xv",Axes2d(-0.1<=x<=2.1, -10.1<=v<=10.1), orbit);
    plot("bouncingball-tx",Axes2d(0.0<=TimeVariable()<=1.5,-0.1<=x<=2.1), orbit);
}
