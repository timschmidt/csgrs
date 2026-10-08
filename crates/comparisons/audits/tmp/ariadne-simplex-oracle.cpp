#include <iostream>

#include "config.hpp"
#include "algebra/matrix.hpp"
#include "algebra/vector.hpp"
#include "solvers/linear_programming.hpp"

using namespace Ariadne;

int main() {
    SimplexSolver<FloatDP> solver;
    const Vector<FloatDP> lower({0.0_x, 0.0_x}, dp);
    const Vector<FloatDP> upper({1.0_x, 1.0_x}, dp);
    std::size_t cases = 0;
    std::size_t wrong_true = 0;
    std::size_t wrong_false = 0;
    std::size_t indeterminate_count = 0;
    std::size_t exceptions = 0;
    for (int a0 = -4; a0 <= 4; ++a0) {
        for (int a1 = -4; a1 <= 4; ++a1) {
            if (a0 == 0 && a1 == 0) {
                continue;
            }
            const int exact_min = std::min(0, a0) + std::min(0, a1);
            const int exact_max = std::max(0, a0) + std::max(0, a1);
            for (int rhs = -9; rhs <= 9; ++rhs) {
                ++cases;
                Matrix<FloatDP> A(1, 2, FloatDP(0, dp));
                A[0][0] = FloatDP(a0, dp);
                A[0][1] = FloatDP(a1, dp);
                const Vector<FloatDP> target(1, FloatDP(rhs, dp));
                const bool expected = rhs >= exact_min && rhs <= exact_max;
                try {
                    const ValidatedKleenean result =
                        solver.feasible(lower, upper, A, target);
                    if (definitely(result) && !expected) {
                        ++wrong_true;
                        std::cout << "wrong_true a=" << a0 << ',' << a1
                                  << " rhs=" << rhs << std::endl;
                    } else if (definitely(!result) && expected) {
                        ++wrong_false;
                        std::cout << "wrong_false a=" << a0 << ',' << a1
                                  << " rhs=" << rhs << std::endl;
                    } else if (!is_determinate(result)) {
                        ++indeterminate_count;
                    }
                } catch (const std::exception& error) {
                    ++exceptions;
                    std::cout << "exception a=" << a0 << ',' << a1
                              << " rhs=" << rhs << " what=" << error.what()
                              << std::endl;
                }
            }
        }
    }
    std::cout << "cases=" << cases << " wrong_true=" << wrong_true
              << " wrong_false=" << wrong_false
              << " indeterminate=" << indeterminate_count
              << " exceptions=" << exceptions << std::endl;
    return wrong_true == 0 && wrong_false == 0 ? 0 : 1;
}
