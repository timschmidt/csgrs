/***************************************************************************
 *            laub-loomis-nonoise.cpp
 *
 *  Copyright  2018  Luca Geretti
 *
 ****************************************************************************/

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

#include "ariadne_main.hpp"
#include "utility/stopwatch.hpp"

using namespace Ariadne;

void ariadne_main()
{
    RealVariable x1("x1"),x2("x2"),x3("x3"),x4("x4"),x5("x5"),x6("x6"),x7("x7");

    VectorField dynamics({dot(x1)=1.4_dec*x3-0.9_dec*x1,
                          dot(x2)=2.5_dec*x5-1.5_dec*x2,
                          dot(x3)=0.6_dec*x7-0.8_dec*x2*x3,
                          dot(x4)=2.0_dec-1.3_dec*x3*x4,
                          dot(x5)=0.7_dec*x1-x4*x5,
                          dot(x6)=0.3_dec*x1-3.1_dec*x6,
                          dot(x7)=1.8_dec*x6-1.5_dec*x2*x7
                         });

    StepMaximumError max_err=1e-6;
    TaylorPicardIntegrator integrator(max_err);

    VectorFieldEvolver evolver(dynamics,integrator);
    evolver.configuration().set_maximum_enclosure_radius(1.0);
    evolver.configuration().set_maximum_step_size(0.02);
    evolver.configuration().set_maximum_spacial_error(1e-6);
    CONCLOG_PRINTLN(evolver.configuration())

    Real x1_0(1.2_dec);
    Real x2_0(1.05_dec);
    Real x3_0(1.5_dec);
    Real x4_0(2.4_dec);
    Real x5_0(1.0_dec);
    Real x6_0(0.1_dec);
    Real x7_0(0.45_dec);
    Real eps = 1/10_q;

    RealExpressionBoundedConstraintSet initial_set({{x1_0-eps<=x1<=x1_0+eps},{x2_0-eps<=x2<=x2_0+eps},{x3_0-eps<=x3<=x3_0+eps},{x4_0-eps<=x4<=x4_0+eps},{x5_0-eps<=x5<=x5_0+eps},{x6_0-eps<=x6<=x6_0+eps},{x7_0-eps<=x7<=x7_0+eps}});

    CONCLOG_PRINTLN_VAR(initial_set);
    Real evolution_time(10);

    Stopwatch<Milliseconds> sw;
    CONCLOG_PRINTLN("Computing orbit...")
    auto orbit = evolver.orbit(initial_set,evolution_time,Semantics::UPPER);
    sw.click();
    CONCLOG_PRINTLN("Done in " << sw.elapsed_seconds() << " seconds.");

    CONCLOG_PRINTLN("Plotting...")
    TimeVariable t;
    /*
    for (auto const& v : dynamics.state_space().variables()) {
        CONCLOG_PRINTLN_AT(1,"Plotting t-" << v.name())
        LabelledFigure fig(Axes2d{{0<=t<=evolution_time},{0<=v<=5}});
        fig << orbit;
        char filename[64];
        snprintf(filename,64,"laub-loomis_t-%s",v.name().c_str());
        fig.write(filename);
    }*/
    CONCLOG_PRINTLN_AT(1,"Plotting x5-x7")
    LabelledFigure fig(Axes2d{{0<=x5<=1.2_dec},{0.1_dec<=x7<=0.6_dec}});
    fig << orbit;
    fig.write("laub-loomis_x5-x7");
    sw.click();
    CONCLOG_PRINTLN("Done in " << sw.elapsed_seconds() << " seconds.");
}
