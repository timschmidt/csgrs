#include <iostream>

#include "config.hpp"
#include "algebra/matrix.hpp"
#include "algebra/vector.hpp"
#include "function/function.hpp"
#include "solvers/linear_programming.hpp"
#include "solvers/inclusion_integrator.hpp"
#include "solvers/nonlinear_programming.hpp"
#include "solvers/runge_kutta_integrator.hpp"

using namespace Ariadne;

int main(int argc, char**) {
    const auto ex = EffectiveScalarMultivariateFunction::coordinate(1, 0);
    const EffectiveVectorMultivariateFunction effective_f = {ex};
    const ApproximateVectorMultivariateFunction approximate_f = effective_f;
    const FloatDPApproximationVector x0({1.0_x}, dp);
    const FloatDPApproximation h(1.0_x, dp);

    if (argc > 1) {
        RungeKutta4Integrator rk_zero(0.0);
        std::cout << "entering_zero_step_evolve" << std::endl;
        (void)rk_zero.evolve(
            approximate_f, x0, FloatDPApproximation(1.0_x, dp));
        return 0;
    }

    RungeKutta4Integrator rk_one(1.0);
    std::cout << "rk4_step_xprime_x_h1="
              << rk_one.step(approximate_f, x0, h) << std::endl;

    RungeKutta4Integrator rk_quarter(0.25);
    const auto quarter_path = rk_quarter.evolve(
        approximate_f, x0, FloatDPApproximation(1.0_x, dp));
    RungeKutta4Integrator rk_half(0.5);
    const auto half_path_after_quarter = rk_half.evolve(
        approximate_f, x0, FloatDPApproximation(1.0_x, dp));
    std::cout << "rk4_quarter_path=" << quarter_path << std::endl;
    std::cout << "rk4_half_path_after_quarter=" << half_path_after_quarter << std::endl;

    SimplexSolver<FloatDP> simplex;
    const Vector<FloatDP> xl({0.0_x, 0.0_x}, dp);
    const Vector<FloatDP> xu({1.0_x, 1.0_x}, dp);
    const Matrix<FloatDP> A({{1.0_x, 1.0_x}}, dp);
    const Vector<FloatDP> b({0.5_x}, dp);
    Array<Slackness> vt = {Slackness::BASIS, Slackness::UPPER};
    std::cout << "simplex_bad_basis_certificate="
              << simplex.verify_feasibility(xl, xu, A, b, vt) << std::endl;
    std::cout << "simplex_hotstarted_feasible="
              << simplex.hotstarted_feasible(xl, xu, A, b, vt) << std::endl;

    NonlinearInfeasibleInteriorPointOptimiser optimiser;
    const ValidatedVectorMultivariateFunction validated_identity = effective_f;
    const ExactBoxType D = {{-2.0_x, 2.0_x}};
    const ExactBoxType C = {{0.0_x, 1.0_x}};
    const FloatDPApproximationVector near_point({1.05}, dp);
    std::cout << "almost_feasible_1.05_eps_0.1="
              << optimiser.almost_feasible_point(
                     D, validated_identity, C, near_point,
                     FloatDPApproximation(0.1, dp))
              << std::endl;

    const FloatDPBoundsVector interior_box({{0.25_x, 0.5_x}}, dp);
    std::cout << "inequality_only_contains_feasible="
              << optimiser.contains_feasible_point(
                     ExactBoxType{{0.0_x, 1.0_x}}, validated_identity, C,
                     interior_box)
              << std::endl;

    const ValidatedVectorMultivariateFunction validated_zero =
        EffectiveVectorMultivariateFunction::zeros(1, 1);
    try {
        std::cout << "singular_equality_contains_feasible="
                  << optimiser.contains_feasible_point(
                         ExactBoxType{{0.0_x, 1.0_x}}, validated_zero,
                         ExactBoxType{{0.0_x, 0.0_x}}, interior_box)
                  << std::endl;
    } catch (const std::exception& error) {
        std::cout << "singular_equality_exception=" << error.what()
                  << std::endl;
    }

    try {
        std::cout << "dexp_zero=" << dexp(FloatDPUpperBound(0, dp))
                  << std::endl;
    } catch (const std::exception& error) {
        std::cout << "dexp_zero_exception=" << error.what() << std::endl;
    }
    try {
        std::cout << "psi0_zero=" << psi0(FloatDP(0, dp))
                  << std::endl;
    } catch (const std::exception& error) {
        std::cout << "psi0_zero_exception=" << error.what() << std::endl;
    }
    try {
        std::cout << "psi1_zero=" << psi1(FloatDP(0, dp))
                  << std::endl;
    } catch (const std::exception& error) {
        std::cout << "psi1_zero_exception=" << error.what() << std::endl;
    }

    NonlinearInteriorPointOptimiser deprecated;
    FloatDPApproximationVector point({0.25}, dp);
    FloatDPApproximation violation(7.0, dp);
    FloatDPApproximationVector slack({3.0, 4.0}, dp);
    deprecated.compute_tz(D, approximate_f, C, point, violation, slack);
    std::cout << "deprecated_compute_tz_after=" << point << ',' << violation
              << ',' << slack << std::endl;
}
