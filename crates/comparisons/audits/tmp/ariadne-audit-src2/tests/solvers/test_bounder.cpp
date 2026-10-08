/***************************************************************************
 *            test_bounder.cpp
 *
 *  Copyright  2018-20  Luca Geretti
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

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#include "config.hpp"

#include "solvers/bounder.hpp"
#include "function/formula.hpp"
#include "function/taylor_model.hpp"

#include "../test.hpp"

using namespace Ariadne;
using namespace std;

class TestBounder
{
  private:
    std::unique_ptr<BounderInterface> bounder_ptr;
    EffectiveScalarMultivariateFunction x, y;
  public:
    TestBounder(const BounderInterface& b)
        : bounder_ptr(b.clone())
    {
        x=EffectiveScalarMultivariateFunction::coordinate(2,0);
        y=EffectiveScalarMultivariateFunction::coordinate(2,1);
    }

    Int test() {
        ARIADNE_TEST_CALL(test_print_name());
        ARIADNE_TEST_CALL(test_D_mismatch());
        ARIADNE_TEST_CALL(test_DA_mismatch());
        ARIADNE_TEST_CALL(test_suggested_step_acceptable());
        ARIADNE_TEST_CALL(test_suggested_step_not_acceptable());
        ARIADNE_TEST_CALL(test_unbounded());
        ARIADNE_TEST_CALL(test_time_invariant_with_parameter());
        ARIADNE_TEST_CALL(test_time_variant_without_parameter());
        ARIADNE_TEST_CALL(test_time_variant_with_parameter());
        return 0;
    }

    Void test_print_name() {
        ARIADNE_TEST_PRINT(*bounder_ptr);
    }

    Void test_D_mismatch() {
        EffectiveVectorMultivariateFunction f={x,-y};
        ExactBoxType D={ExactIntervalType(-0.25_x,0.25_x)};
        StepSizeType hsug(0.25_x);

        ARIADNE_TEST_FAIL(bounder_ptr->compute(f,D,suggest(hsug)));
    }

    Void test_DA_mismatch() {
        EffectiveVectorMultivariateFunction f={x,-y};
        ExactBoxType D={ExactIntervalType(-0.25_x,0.25_x),ExactIntervalType(-0.25_x,0.25_x)};
        ExactBoxType A={ExactIntervalType(-0.25_x,0.25_x)};
        StepSizeType hsug(0.25_x);

        ARIADNE_TEST_FAIL(bounder_ptr->compute(f,D,A,suggest(hsug)));
    }

    Void test_suggested_step_acceptable() {
        EffectiveVectorMultivariateFunction f={x,-y};
        ExactBoxType dom={ExactIntervalType(-0.25_x,0.25_x),ExactIntervalType(-0.25_x,0.25_x)};
        StepSizeType hsug(0.25_x);

        StepSizeType h;
        UpperBoxType B;
        std::tie(h,B) = bounder_ptr->compute(f,dom,suggest(hsug));

        ARIADNE_TEST_PRINT(h);
        ARIADNE_TEST_PRINT(B);
        ARIADNE_TEST_EQUAL(h,hsug);
        ARIADNE_TEST_ASSERT(definitely(is_bounded(B)));
    }

    Void test_suggested_step_not_acceptable() {
        EffectiveVectorMultivariateFunction f={x,-y};
        ExactBoxType dom={ExactIntervalType(-0.25_x,0.25_x),ExactIntervalType(-0.25_x,0.25_x)};
        StepSizeType hsug(1.0_x);

        StepSizeType h;
        UpperBoxType B;
        std::tie(h,B) = bounder_ptr->compute(f,dom,suggest(hsug));

        ARIADNE_TEST_PRINT(h);
        ARIADNE_TEST_PRINT(B);
        ARIADNE_TEST_COMPARE(h,<,hsug);
        ARIADNE_TEST_ASSERT(definitely(is_bounded(B)));
    }

    Void test_unbounded() {
        EffectiveVectorMultivariateFunction f={pow(x,3),-pow(y,5)};
        ExactBoxType dom={ExactIntervalType(-20.0_x,20.0_x),ExactIntervalType(-20.0_x,20.0_x)};
        StepSizeType hsug(10.0_x);

        ARIADNE_TEST_THROWS(bounder_ptr->compute(f,dom,suggest(hsug)),BoundingNotFoundException);
    }

    Void test_time_invariant_with_parameter() {
        EffectiveScalarMultivariateFunction x0=EffectiveScalarMultivariateFunction::coordinate(3,0);
        EffectiveScalarMultivariateFunction x1=EffectiveScalarMultivariateFunction::coordinate(3,1);
        EffectiveScalarMultivariateFunction a0=EffectiveScalarMultivariateFunction::coordinate(3,2);

        EffectiveVectorMultivariateFunction f={x0+a0,-x1};
        ExactBoxType D={ExactIntervalType(-0.25_x,0.25_x),ExactIntervalType(-0.25_x,0.25_x)};
        ExactBoxType A={ExactIntervalType(-0.25_x,0.25_x)};
        StepSizeType td(1.0_x);
        StepSizeType hsug(1.0_x);

        StepSizeType h;
        UpperBoxType B;

        std::tie(h,B) = bounder_ptr->compute(f,D,A,suggest(hsug));

        ARIADNE_TEST_COMPARE(h,<=,hsug);
        ARIADNE_TEST_ASSERT(definitely(is_bounded(B)));
    }

    Void test_time_variant_without_parameter() {
        EffectiveScalarMultivariateFunction x0=EffectiveScalarMultivariateFunction::coordinate(3,0);
        EffectiveScalarMultivariateFunction x1=EffectiveScalarMultivariateFunction::coordinate(3,1);
        EffectiveScalarMultivariateFunction t=EffectiveScalarMultivariateFunction::coordinate(3,2);

        EffectiveVectorMultivariateFunction f={x0,-x1+2*t};
        ExactBoxType D={ExactIntervalType(-0.25_x,0.25_x),ExactIntervalType(-0.25_x,0.25_x)};
        StepSizeType td(1.0_x);
        StepSizeType hsug(1.0_x);

        StepSizeType h;
        UpperBoxType B;

        std::tie(h,B) = bounder_ptr->compute(f,D,td,suggest(hsug));

        ARIADNE_TEST_COMPARE(h,<=,hsug);
        ARIADNE_TEST_ASSERT(definitely(is_bounded(B)));
    }

    Void test_time_variant_with_parameter() {
        EffectiveScalarMultivariateFunction x0=EffectiveScalarMultivariateFunction::coordinate(4,0);
        EffectiveScalarMultivariateFunction x1=EffectiveScalarMultivariateFunction::coordinate(4,1);
        EffectiveScalarMultivariateFunction t=EffectiveScalarMultivariateFunction::coordinate(4,2);
        EffectiveScalarMultivariateFunction a0=EffectiveScalarMultivariateFunction::coordinate(4,3);

        EffectiveVectorMultivariateFunction f={x0+a0,-x1+2*t};
        ExactBoxType D={ExactIntervalType(-0.25_x,0.25_x),ExactIntervalType(-0.25_x,0.25_x)};
        ExactBoxType A={ExactIntervalType(-0.25_x,0.25_x)};
        StepSizeType td(1.0_x);
        StepSizeType hsug(1.0_x);

        StepSizeType h;
        UpperBoxType B;

        std::tie(h,B) = bounder_ptr->compute(f,D,td,A,suggest(hsug));

        ARIADNE_TEST_COMPARE(h,<=,hsug);
        ARIADNE_TEST_ASSERT(definitely(is_bounded(B)));
    }
};

Int main(Int argc, const char* argv[]) {

    List<BounderHandle> bounders = { EulerBounder() };

    for (BounderHandle bounder : bounders)
        TestBounder(bounder).test();

    return ARIADNE_TEST_FAILURES;
}
