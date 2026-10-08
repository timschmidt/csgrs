/***************************************************************************
 *            test_measurable_function.cpp
 *
 *  Copyright  2020  Pieter Collins
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

#include <cassert>
#include <fstream>
#include <sstream>
#include <string>
#include <iomanip>
#include <stdexcept>
#include <fenv.h>

#include "config.hpp"
#include "function/measurable_function.hpp"
#include "function/formula.hpp"
#include "function/symbolic_function.hpp"
#include "function/taylor_model.hpp"
#include "algebra/algebra.hpp"

#include "numeric/numeric.hpp"

#include "symbolic/variable.hpp"
#include "symbolic/expression.hpp"

#include "../test.hpp"

using namespace std;
using namespace Ariadne;

template<class X> inline OutputStream& operator<<(OutputStream& os, Sequence<X> const& seq) {
    SequenceWriter writer(4u); return os << writer(seq);
}

class TestMeasurableFunction
{
  public:
    Void test();
  private:
    Void test_concept();
    Void test_scalar_univariate_function();
};

Void TestMeasurableFunction::test()
{
    Dyadic::set_default_writer(DecimalWriter());
    ARIADNE_TEST_CALL(test_scalar_univariate_function());
}

Void TestMeasurableFunction::test_concept()
{
}

Void TestMeasurableFunction::test_scalar_univariate_function()
{
    {
        ARIADNE_TEST_CONSTRUCT(UnionOfIntervals<Dyadic>,ivls,({{1,2},{1.5_dy,2.5_dy},{0,0.5_dy}}));
        ARIADNE_TEST_CONSTRUCT(ValidatedOpenSet<Real>,vops,(wrap_open(ivls)));

        RealVariable x("x");
        ValidatedContinuousFunction<Real(Real)> f=make_function(x,x*x);
        ARIADNE_TEST_PRINT(f);
        ARIADNE_TEST_PRINT(vops);
        ValidatedOpenSet<Real> prevops=preimage(f,vops,IntervalDomainType(-1,3),Accuracy(exp2(-4)));
        ARIADNE_TEST_PRINT(prevops);

        IntervalDomainType dom(-1,3);
        Error<FloatDP> err(0.093750_dy,dp);
        FanModel<Real(Real),DP> fmodel(dom,f,err);

        ARIADNE_TEST_PRINT(fmodel);
        ARIADNE_TEST_PRINT(fmodel.preimage(vops));
        ARIADNE_TEST_ASSIGN_CONSTRUCT(ValidatedLowerMeasurableSet<Real>,prim,fmodel.preimage(vops));
        ARIADNE_TEST_PRINT(prim.measure());
    }
}


Int main() {
    TestMeasurableFunction().test();

    return ARIADNE_TEST_FAILURES;
}

