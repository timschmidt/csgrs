/***************************************************************************
 *            SUTR21.hpp
 *
 *  Copyright  2021  Luca Geretti
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

#include "arch.hpp"

using namespace Ariadne;

void SUTR21() {

    ArchBenchmark benchmark("SUTR21");

    CONCLOG_PRINTLN("SUTRA benchmark (SUTR21):");

    RealVariable SA("SA"), SI("SI"), A("A"), I("I"), RI("RI"), RA("RA"), D("D"), AplusI("A+I");
    RealConstant beta("beta",0.25_dec);
    RealConstant gamma("gamma",0.02_dec);
    RealConstant eta("eta",0.02_dec);

    VectorField dynamics({dot(SA) = -beta*SA*(A+I),
                          dot(SI) = -beta*SI*(A+I),
                          dot(A) = beta*SA*(A+I)-gamma*A,
                          dot(I) = beta*SI*(A+I)-gamma*I,
                          dot(RI) = gamma*A,
                          dot(RA) = gamma*I,
                          dot(D) = eta*I
                         },{let(AplusI) = A+I});

    StepMaximumError max_err = 8e-6;
    TaylorPicardIntegrator integrator(max_err);

    VectorFieldEvolver evolver(dynamics, integrator);
    evolver.configuration().set_maximum_enclosure_radius(0.09);
    evolver.configuration().set_maximum_step_size(10.0);
    evolver.configuration().set_maximum_spacial_error(1e-3);

    RealExpressionBoundedConstraintSet initial_set({0.5_dec<=SA<=0.7_dec,
                                                    0.2_dec<=SI<=0.3_dec,
                                                    0.1_dec<=A<=0.14_dec,
                                                    0.01_dec<=I<=0.05_dec,
                                                    RA==0,RI==0,D==0});

    Real evolution_time = 200;

    Stopwatch<Milliseconds> sw;

    CONCLOG_PRINTLN_AT(2,"Computing orbit...");
    CONCLOG_RUN_AT(2, auto orbit = evolver.orbit(initial_set, evolution_time, Semantics::UPPER));

    sw.click();
    CONCLOG_PRINTLN_AT(2,"Done in " << sw.elapsed_seconds() << " seconds.");
/*
    CONCLOG_PRINTLN_AT(2,"Checking properties...");
    Nat ce = 0;
    for (auto set : orbit.reach()) {
        auto bb = set.bounding_box();
        if (possibly(bb[x4] >= 4.5_dec)) {
            CONCLOG_PRINTLN_AT(3,"Set with value " << bb[x4] << " does not respect the specification.");
            ++ce;
        }
    }

    auto x4_width = orbit.final().bounding_box()[x4].width();

    sw.click();
    if (ce>0) CONCLOG_PRINTLN_AT(2,"Number of counterexamples: " << ce);
    CONCLOG_PRINTLN_AT(2,"Width of final x4: " << x4_width);
    CONCLOG_PRINTLN_AT(2,"Done in " << sw.elapsed_seconds() << " seconds.");

    auto instance = benchmark.create_instance("W001");
    if (ce==0)
        instance.set_verified(1).set_execution_time(sw.elapsed_seconds()).add_loss(x4_width.get_d());
    instance.write();
*/
    CONCLOG_PRINTLN("Plotting...");
    {
        LabelledFigure fig(Axes2d({0<=TimeVariable()<=evolution_time,0<=SA<=1}));
        fig.draw(orbit);
        std::stringstream ss;
        ss << benchmark.name() << "_t_SA";
        fig.write(ss.str().c_str());
    }
    {
        LabelledFigure fig(Axes2d({0<=TimeVariable()<=evolution_time,0<=SI<=1}));
        fig.draw(orbit);
        std::stringstream ss;
        ss << benchmark.name() << "_t_SI";
        fig.write(ss.str().c_str());
    }
    {
        LabelledFigure fig(Axes2d({0<=TimeVariable()<=evolution_time,0<=A<=1}));
        fig.draw(orbit);
        std::stringstream ss;
        ss << benchmark.name() << "_t_A";
        fig.write(ss.str().c_str());
    }
    {
        LabelledFigure fig(Axes2d({0<=TimeVariable()<=evolution_time,0<=I<=1}));
        fig.draw(orbit);
        std::stringstream ss;
        ss << benchmark.name() << "_t_I";
        fig.write(ss.str().c_str());
    }
    {
        LabelledFigure fig(Axes2d({0<=TimeVariable()<=evolution_time,0<=RA<=1}));
        fig.draw(orbit);
        std::stringstream ss;
        ss << benchmark.name() << "_t_RA";
        fig.write(ss.str().c_str());
    }
    {
        LabelledFigure fig(Axes2d({0<=TimeVariable()<=evolution_time,0<=RI<=1}));
        fig.draw(orbit);
        std::stringstream ss;
        ss << benchmark.name() << "_t_RI";
        fig.write(ss.str().c_str());
    }
    {
        LabelledFigure fig(Axes2d({0<=TimeVariable()<=evolution_time,0<=D<=1}));
        fig.draw(orbit);
        std::stringstream ss;
        ss << benchmark.name() << "_t_D";
        fig.write(ss.str().c_str());
    }
    {
        LabelledFigure fig(Axes2d({0<=TimeVariable()<=evolution_time,0<=AplusI<=1}));
        fig.draw(orbit);
        std::stringstream ss;
        ss << benchmark.name() << "_t_AplusI";
        fig.write(ss.str().c_str());
    }

    CONCLOG_PRINTLN("File " << benchmark.name() << ".png written.");
}
