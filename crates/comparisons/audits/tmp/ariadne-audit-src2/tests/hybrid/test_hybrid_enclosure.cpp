/***************************************************************************
 *            test_hybrid_enclosure.cpp
 *
 *  Copyright  2008-21  Luca Geretti
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

#include "config.hpp"
#include "../test.hpp"

#include "algebra/algebra.hpp"
#include "function/taylor_function.hpp"
#include "hybrid/hybrid_set.hpp"
#include "hybrid/hybrid_enclosure.hpp"

using namespace std;
using namespace Ariadne;

class TestHybridEnclosure {
  public:
    Void test() {
        ARIADNE_TEST_CALL(test_construct())
        ARIADNE_TEST_CALL(test_space_having_time())
        ARIADNE_TEST_CALL(test_auxiliary())
    }

    void test_construct() {
        HybridEnclosure encl;

        TaylorFunctionFactory function_factory(ThresholdSweeper<FloatDP>(dp,1e-8));
        EnclosureConfiguration config(function_factory);
        HybridEnclosure encl2(config);
        ARIADNE_TEST_PRINT(encl2.configuration())

        DiscreteLocation location;
        RealVariable x("x"), y("y");
        HybridRealBox box(location,{1<=x<=2,0<=y<=1});
        HybridEnclosure encl3(box,config);
        ARIADNE_TEST_EQUALS(encl3.number_of_parameters(),2)
        ARIADNE_TEST_EQUALS(encl3.number_of_constraints(),0)

        Enclosure cencl(config);
        HybridEnclosure(location,RealSpace({x,y}),cencl);
    }

    void test_space_having_time() {
        TaylorFunctionFactory function_factory(ThresholdSweeper<FloatDP>(dp,1e-8));
        EnclosureConfiguration config(function_factory);
        RealVariable x("x"), t("t");
        HybridRealBox box(DiscreteLocation(),{1<=t<=2,0<=x<=1});
        HybridEnclosure encl(box,config);
        ARIADNE_TEST_ASSERT(encl.state_time_auxiliary_space() == RealSpace({t,x}))
    }

    void test_auxiliary() {
        ExactBoxType dom({{0.0_x,2.0_x},{1.0_x,3.0_x}});
        Enclosure encl(dom,EnclosureConfiguration(TaylorFunctionFactory(ThresholdSweeper<FloatDP>(dp,1e-8))));
        ARIADNE_TEST_PRINT(encl.auxiliary_function());
        auto x0 = EffectiveScalarMultivariateFunction::coordinate(2,0);
        auto x1 = EffectiveScalarMultivariateFunction::coordinate(2,1);
        EffectiveVectorMultivariateFunction auxiliary(1,x0+sqr(x1));
        RealVariable x("x"), y("y"), z("z");
        HybridEnclosure he(DiscreteLocation(),RealSpace({x,y}),encl);
        he.set_auxiliary({z},auxiliary);
        ARIADNE_TEST_EQUALS(he.dimension(),3)
        ARIADNE_TEST_EXECUTE(he.function(x))
        ARIADNE_TEST_EXECUTE(he.function(y))
        ARIADNE_TEST_EXECUTE(he.function(z))
        ARIADNE_TEST_EXECUTE(he.function(TimeVariable()))
        ARIADNE_TEST_FAIL(he.function(RealVariable("u")))
        ARIADNE_TEST_EQUALS(he.auxiliary_function().result_size(),1)
        ARIADNE_TEST_EQUALS(he.state_auxiliary_function().result_size(),3)
        ARIADNE_TEST_EQUALS(he.state_time_auxiliary_function().result_size(),4)
        ARIADNE_TEST_EQUALS(he.state_set().dimension(),2)
        ARIADNE_TEST_EQUALS(he.state_time_set().dimension(),3)
        ARIADNE_TEST_EQUALS(he.state_auxiliary_set().dimension(),3)

        ARIADNE_TEST_ASSERT(he.state_auxiliary_space() == RealSpace({x,y,z}))

        ARIADNE_TEST_EQUALS(project(he,RealSpace({x})).dimension(),1)
        ARIADNE_TEST_EQUALS(project(he,RealSpace({y,z})).dimension(),2)
        ARIADNE_TEST_FAIL(project(he,RealSpace({RealVariable("u")})))
    }
};

Int main() {
    TestHybridEnclosure().test();
    return ARIADNE_TEST_FAILURES;
}

