/***************************************************************************
 *            LALO20.hpp
 *
 *  Copyright  2020  Luca Geretti
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

void LALO20() {

    ArchBenchmark benchmark("LALO20");

    RealVariable x1("x1"), x2("x2"), x3("x3"), x4("x4"), x5("x5"), x6("x6"), x7("x7"), t("t");

    VectorField dynamics({dot(x1) = 1.4_dec * x3 - 0.9_dec * x1,
                                 dot(x2) = 2.5_dec * x5 - 1.5_dec * x2,
                                 dot(x3) = 0.6_dec * x7 - 0.8_dec * x2 * x3,
                                 dot(x4) = 2 - 1.3_dec * x3 * x4,
                                 dot(x5) = 0.7_dec * x1 - x4 * x5,
                                 dot(x6) = 0.3_dec * x1 - 3.1_dec * x6,
                                 dot(x7) = 1.8_dec * x6 - 1.5_dec * x2 * x7
                         });

    Real x1_0(1.2_dec);
    Real x2_0(1.05_dec);
    Real x3_0(1.5_dec);
    Real x4_0(2.4_dec);
    Real x5_0(1.0_dec);
    Real x6_0(0.1_dec);
    Real x7_0(0.45_dec);

    CONCLOG_PRINTLN("Laub-Loomis benchmark (LALO20):");

    ListSet<LabelledEnclosure> reach1, reach2, reach3;

    {
        CONCLOG_PRINTLN_AT(1,"Running for W=0.01...");

        StepMaximumError max_err = 1e-7;
        TaylorPicardIntegrator integrator(max_err);

        VectorFieldEvolver evolver(dynamics, integrator);
        evolver.configuration().set_maximum_enclosure_radius(0.09);
        evolver.configuration().set_maximum_step_size(0.2);
        evolver.configuration().set_maximum_spacial_error(1e-4);

        Real eps = 1 / 100_q;

        RealVariablesBox initial_set({x1_0 - eps <= x1 <= x1_0 + eps,
                                      x2_0 - eps <= x2 <= x2_0 + eps,
                                      x3_0 - eps <= x3 <= x3_0 + eps,
                                      x4_0 - eps <= x4 <= x4_0 + eps,
                                      x5_0 - eps <= x5 <= x5_0 + eps,
                                      x6_0 - eps <= x6 <= x6_0 + eps,
                                      x7_0 - eps <= x7 <= x7_0 + eps
                                     });

        Real evolution_time(20);

        Stopwatch<Milliseconds> sw;

        CONCLOG_PRINTLN_AT(2,"Computing orbit...");
        CONCLOG_RUN_AT(2, auto orbit = evolver.orbit(initial_set, evolution_time, Semantics::UPPER));

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

        reach1.adjoin(orbit.reach());
    }

    {
        CONCLOG_PRINTLN_AT(1,"Running for W=0.05...");

        StepMaximumError max_err = 1e-7;
        TaylorPicardIntegrator integrator(max_err);

        VectorFieldEvolver evolver(dynamics, integrator);
        evolver.configuration().set_maximum_enclosure_radius(1.0);
        evolver.configuration().set_maximum_step_size(0.2);
        evolver.configuration().set_maximum_spacial_error(1e-4);

        Real eps = 1 / 20_q;

        RealVariablesBox initial_set({x1_0 - eps <= x1 <= x1_0 + eps,
                                      x2_0 - eps <= x2 <= x2_0 + eps,
                                      x3_0 - eps <= x3 <= x3_0 + eps,
                                      x4_0 - eps <= x4 <= x4_0 + eps,
                                      x5_0 - eps <= x5 <= x5_0 + eps,
                                      x6_0 - eps <= x6 <= x6_0 + eps,
                                      x7_0 - eps <= x7 <= x7_0 + eps
                                     });

        Real evolution_time(20);

        Stopwatch<Milliseconds> sw;

        CONCLOG_PRINTLN_AT(2,"Computing orbit...");
        CONCLOG_RUN_AT(2, auto orbit = evolver.orbit(initial_set, evolution_time, Semantics::UPPER));

        CONCLOG_PRINTLN_AT(2,"Checking properties...");
        Nat ce = 0;
        for (auto set : orbit.reach()) {
            auto bb = set.bounding_box();
            if (possibly(bb[x4] >= 4.5_dec)) {
                CONCLOG_PRINTLN_AT(3,"set with value " << bb[x4] << " does not respect the specification.");
                ++ce;
            }
        }

        auto x4_width = orbit.final().bounding_box()[x4].width();

        sw.click();
        if (ce>0) CONCLOG_PRINTLN_AT(2,"Number of counterexamples: " << ce);
        CONCLOG_PRINTLN_AT(2,"Width of final x4: " << x4_width);
        CONCLOG_PRINTLN_AT(2,"Done in " << sw.elapsed_seconds() << " seconds.");

        auto instance = benchmark.create_instance("W005");
        if (ce==0)
            instance.set_verified(1).set_execution_time(sw.elapsed_seconds()).add_loss(x4_width.get_d());
        instance.write();

        reach2.adjoin(orbit.reach());
    }

    {
        CONCLOG_PRINTLN_AT(1,"Running for W=0.1...");

        StepMaximumError max_err = 1e-7;
        TaylorPicardIntegrator integrator(max_err);

        VectorFieldEvolver evolver(dynamics, integrator);
        evolver.configuration().set_maximum_enclosure_radius(1.0);
        evolver.configuration().set_maximum_step_size(0.2);
        evolver.configuration().set_maximum_spacial_error(1e-4);

        Real eps = 1 / 10_q;

        RealVariablesBox initial_set({x1_0 - eps <= x1 <= x1_0 + eps,
                                      x2_0 - eps <= x2 <= x2_0 + eps,
                                      x3_0 - eps <= x3 <= x3_0 + eps,
                                      x4_0 - eps <= x4 <= x4_0 + eps,
                                      x5_0 - eps <= x5 <= x5_0 + eps,
                                      x6_0 - eps <= x6 <= x6_0 + eps,
                                      x7_0 - eps <= x7 <= x7_0 + eps
                                     });

        Real evolution_time(20);

        Stopwatch<Milliseconds> sw;

        CONCLOG_PRINTLN_AT(2,"Computing orbit...");
        CONCLOG_RUN_AT(2, auto orbit = evolver.orbit(initial_set, evolution_time, Semantics::UPPER));

        CONCLOG_PRINTLN_AT(2,"Checking properties...");
        Nat ce = 0;
        for (auto set : orbit.reach()) {
            auto bb = set.bounding_box();
            if (possibly(bb[x4] >= 5)) {
                CONCLOG_PRINTLN_AT(3,"set with value " << bb[x4] << " does not respect the specification.");
                ++ce;
            }
        }

        auto final_bounds = orbit.final().bounding_box();
        auto x4_width = final_bounds[x4].width();

        sw.click();
        if (ce>0) CONCLOG_PRINTLN_AT(2,"Number of counterexamples: " << ce);
        CONCLOG_PRINTLN_AT(2,"Width of final x4: " << x4_width);
        CONCLOG_PRINTLN_AT(2,"Done in " << sw.elapsed_seconds() << " seconds.");

        auto instance = benchmark.create_instance("W01");
        if (ce==0)
            instance.set_verified(1).set_execution_time(sw.elapsed_seconds()).add_loss(x4_width.get_d());
        instance.write();

        reach3.adjoin(orbit.reach());
    }

    CONCLOG_PRINTLN("Plotting...");
    LabelledFigure fig(Axes2d({0<=TimeVariable()<=20,1.5<=x4<=5}));
    fig << line_colour(0.0,0.0,0.0);
    fig << line_style(false);
    fig << fill_colour(1.0,0.75,0.5);
    fig.draw(reach3);
    fig << fill_colour(0.6,0.6,0.6);
    fig.draw(reach2);
    fig << fill_colour(1.0,1.0,1.0);
    fig.draw(reach1);
    CONCLOG_RUN_MUTED(fig.write(benchmark.name().c_str()))
    CONCLOG_PRINTLN("File " << benchmark.name() << ".png written.");
}
