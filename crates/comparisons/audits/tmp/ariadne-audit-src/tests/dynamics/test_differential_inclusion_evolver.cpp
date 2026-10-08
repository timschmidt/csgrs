/***************************************************************************
 *            test_differential_inclusion_evolver.cpp
 *
 *  Copyright  2008-21  Luca Geretti, Pieter Collins, Sanja Zivanovic
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

#include "dynamics/differential_inclusion_evolver.hpp"
#include "dynamics/orbit.hpp"
#include "algebra/sweeper.hpp"
#include "solvers/integrator.hpp"
#include "geometry/box.hpp"
#include "function/function.hpp"
#include "function/formula.hpp"
#include "function/taylor_model.hpp"
#include "algebra/algebra.hpp"
#include "geometry/function_set.hpp"
#include "io/figure.hpp"
#include "io/command_line_interface.hpp"
#include "symbolic/expression_set.hpp"

#include "../test.hpp"

using namespace Ariadne;

class TestDifferentialInclusionEvolver {

    void run_single_test(String name, DifferentialInclusion const &ivf, RealVariablesBox const &initial, Real evolution_time,
                         ExactDouble step, List<InputApproximation> approximations,
                        IntegratorInterface const &integrator, Reconditioner const &reconditioner, bool draw) const {
        auto evolver = DifferentialInclusionEvolver(ivf, integrator, reconditioner);
        evolver.configuration().set_approximations(approximations);
        evolver.configuration().set_maximum_step_size(step);
        ARIADNE_TEST_PRINT(evolver.configuration());

        evolver.orbit(initial, evolution_time);
    }

    void run_each_approximation(String name, DifferentialInclusion const &ivf, RealVariablesBox const &initial,
                                Real evolution_time, ExactDouble step, List<InputApproximation> approximations,
                                IntegratorInterface const &integrator, Reconditioner const &reconditioner, bool draw) const {
        for (auto appro: approximations) {
            List<InputApproximation> singleapproximation = {appro};
            run_single_test(name, ivf, initial, evolution_time, step, singleapproximation, integrator, reconditioner, draw);
        }
    }

    void run_test(String name, const DottedRealAssignments &dynamics, const RealVariablesBox &inputs,
                  const RealVariablesBox &initial, Real evolution_time, ExactDouble step) const {

        DifferentialInclusion ivf(dynamics, inputs);

        SizeType period_of_parameter_reduction = 3;
        ExactDouble ratio_of_parameters_to_keep = 3.0_x;
        ExactDouble sw_threshold = 1e-8_pr;
        ThresholdSweeperDP sweeper(DoublePrecision(), sw_threshold);

        List<InputApproximation> approximations = {ZeroApproximation(), ConstantApproximation(), AffineApproximation(),
                                                   SinusoidalApproximation(), PiecewiseApproximation()};

        TaylorPicardIntegrator integrator(
                step_maximum_error = 1e-6_pr,
                sweeper,
                lipschitz_tolerance = 0.5_x,
                minimum_temporal_order = 4,
                maximum_temporal_order = 12);

        LohnerReconditioner reconditioner(initial.variables().size(), inputs.variables().size(),
                                          period_of_parameter_reduction, ratio_of_parameters_to_keep);

        run_each_approximation(name, ivf, initial, evolution_time, step, approximations, integrator, reconditioner, false);
    }

public:

    void test_higgins_selkov() const {
        RealVariable S("S"), P("P"), v0("v0"), k1("k1"), k2("k2");
        DottedRealAssignments dynamics = {dot(S) = v0 - S * k1 * pow(P, 2), dot(P) = S * k1 * pow(P, 2) - k2 * P};
        RealVariablesBox inputs = {0.9998_dec <= v0 <= 1.0002_dec, 0.9998_dec <= k1 <= 1.0002_dec,
                                   0.99981_dec <= k2 <= 1.00021_dec};

        Real e = 1 / 100_q;
        RealVariablesBox initial = {{2 - e <= S <= 2 + e},
                                    {1 - e <= P <= 1 + e}};

        Real evolution_time = 1 / 20_q;
        ExactDouble step = 1.0_x / pow(two, 6);

        this->run_test("higgins-selkov", dynamics, inputs, initial, evolution_time, step);
    }


    void test_jet_engine() const {
        RealVariable x("x"), y("y"), u1("u1"), u2("u2");
        DottedRealAssignments dynamics = {dot(x) = -y - 1.5_dec * pow(x, 2) - 0.5_dec * pow(x, 3) - 0.5_dec + u1, dot(
                y) = 3 * x - y + u2};
        RealVariablesBox inputs = {-5 / 1000_q <= u1 <= 5 / 1000_q, -5 / 1000_q <= u2 <= 5 / 1000_q};

        Real e1 = 5 / 100_q;
        Real e2 = 7 / 100_q;
        RealVariablesBox initial = {{1 - e1 <= x <= 1 + e1},
                                    {1 - e2 <= y <= 1 + e2}};

        Real evolution_time = 1 / 20_q;
        ExactDouble step = 1.0_x / pow(two, 6);

        this->run_test("jet-engine", dynamics, inputs, initial, evolution_time, step);
    }

    void test_rossler() const {
        RealVariable x("x"), y("y"), z("z"), u("u");
        DottedRealAssignments dynamics = {dot(x) = -y - z, dot(y) = x + y * 0.1_dec, dot(z) = z * (x - 6) + u};
        RealVariablesBox inputs = {0.099_dec <= u <= 0.101_dec};

        Real e = 1 / 1024_q;
        RealVariablesBox initial = {{-9 - e <= x <= -9 + e},
                                    {-e <= y <= e},
                                    {0.01_dec - e <= z <= 0.01_dec + e}};

        Real evolution_time = 1 / 100_q;
        ExactDouble step = 1.0_x / pow(two, 8);

        this->run_test("rossler", dynamics, inputs, initial, evolution_time, step);
    }

    void test_recondition() const {

        RealVariable S("S"), P("P"), v0("v0"), k1("k1"), k2("k2");
        DottedRealAssignments dynamics = {dot(S) = v0 - S * k1 * pow(P, 2), dot(P) = S * k1 * pow(P, 2) - k2 * P};
        RealVariablesBox inputs = {0.9998_dec <= v0 <= 1.0002_dec, 0.9998_dec <= k1 <= 1.0002_dec,
                                   0.99981_dec <= k2 <= 1.00021_dec};

        Real e = 1 / 100_q;
        RealVariablesBox initial = {{2 - e <= S <= 2 + e},
                                    {1 - e <= P <= 1 + e}};

        Real evolution_time = 3 / 20_q;
        ExactDouble step = 1.0_x / pow(two, 6);

        DifferentialInclusion ivf(dynamics, inputs);

        SizeType period_of_parameter_reduction = 3;
        ExactDouble ratio_of_parameters_to_keep = 1.25_x;
        ExactDouble sw_threshold = 1e-10_pr;
        ThresholdSweeperDP sweeper(DoublePrecision(), sw_threshold);

        TaylorPicardIntegrator integrator(
                step_maximum_error = 1e-6_pr,
                sweeper,
                lipschitz_tolerance = 0.5_x,
                minimum_temporal_order = 4,
                maximum_temporal_order = 12);

        LohnerReconditioner reconditioner(initial.variables().size(), inputs.variables().size(),
                                          period_of_parameter_reduction, ratio_of_parameters_to_keep);

        run_each_approximation("recondition", ivf, initial, evolution_time, step, {AffineApproximation()}, integrator, reconditioner, false);
    }

    void test() const {
        ARIADNE_TEST_CALL(test_higgins_selkov());
        ARIADNE_TEST_CALL(test_jet_engine());
        ARIADNE_TEST_CALL(test_rossler());
        ARIADNE_TEST_CALL(test_recondition());
    }
};

int main(int argc, const char* argv[]) {
    if (not CommandLineInterface::instance().acquire(argc,argv)) return -1;
    TestDifferentialInclusionEvolver().test();
    return ARIADNE_TEST_FAILURES;
}
