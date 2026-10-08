/***************************************************************************
 *            test_measurable_set.cpp
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
#include "geometry/set.hpp"
#include "geometry/set_wrapper.hpp"
#include "geometry/measurable_set.hpp"

#include "../test.hpp"

using namespace std;
using namespace Ariadne;

template<class X> inline OutputStream& operator<<(OutputStream& os, Sequence<X> const& seq) {
    SequenceWriter writer(4u); return os << writer(seq);
}

class TestMeasurableSet
{
  public:
    Void test();
  private:
    Void test_concept();
    Void test_scalar_set();
};

Void TestMeasurableSet::test()
{
    Dyadic::set_default_writer(DecimalWriter());
    ARIADNE_TEST_CALL(test_scalar_set());
}

Void TestMeasurableSet::test_concept()
{
}

Void TestMeasurableSet::test_scalar_set()
{
    {
        ARIADNE_TEST_CONSTRUCT(UnionOfIntervals<Dyadic>,ivls1,({{1,2},{1.5_dy,2.5_dy},{0,0.5_dy}}));
        ARIADNE_TEST_CONSTRUCT(UnionOfIntervals<Dyadic>,ivls2,({{0.25_dy,2.25_dy}}));
        ARIADNE_TEST_EQUALS(ivls1.measure(),2);
        ARIADNE_TEST_EQUALS(intersection(ivls1,ivls2).measure(),1.5_dy);
        ARIADNE_TEST_EQUALS(join(ivls1,ivls2).measure(),2.5_dy);

        ARIADNE_TEST_CONSTRUCT(UnionOfIntervals<FloatDP>,ivls1dp,({{1.0_dy,2.0_dy},{1.5_dy,2.5_dy},{0.0_dy,0.5_dy}},double_precision));
        ARIADNE_TEST_CONSTRUCT(UnionOfIntervals<FloatMP>,ivls1mp,({{1.0_dy,2.0_dy},{1.5_dy,2.5_dy},{0.0_dy,0.5_dy}},precision(128)));
    }

    {
        ARIADNE_TEST_CONSTRUCT(LowerSequence<UnionOfIntervals<Dyadic>>,lms,([](Natural n){return UnionOfIntervals<Dyadic>({{exp2(-n),1.0_dy+exp2(-(n+1u))}});}));
        ARIADNE_TEST_CONSTRUCT(IncreasingSequence<PositiveDyadic>,ms,(lms.measures()));
    }

    {
        ARIADNE_TEST_CONSTRUCT(UnionOfIntervals<Dyadic>,ivls,({{1,2},{1.5_dy,2.5_dy},{0,0.5_dy}}));
        ARIADNE_TEST_ASSIGN_CONSTRUCT(ValidatedOpenSet<Real>,ops,wrap_open(ivls));


        PositiveDyadic err(0u);
        LowerMeasurableSetModel lmsm(ivls,err);

        static_assert(IsLowerMeasurableSet<UnionOfIntervals<Dyadic>,ValidatedTag,Real>::value);
        static_assert(IsLowerMeasurableSet<LowerMeasurableSetModel<UnionOfIntervals<Dyadic>,Dyadic>,ValidatedTag,Real>::value);

        ValidatedLowerMeasurableSet<Real> ilms(ivls);

    }
}


Int main() {
    TestMeasurableSet().test();

    return ARIADNE_TEST_FAILURES;
}

