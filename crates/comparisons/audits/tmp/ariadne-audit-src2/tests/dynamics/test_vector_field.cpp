/***************************************************************************
 *            test_vector_field.cpp
 *
 *  Copyright  2006-21  Luca Geretti
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

#include <fstream>
#include <iostream>

#include "config.hpp"
#include "utility/tuple.hpp"
#include "algebra/vector.hpp"
#include "algebra/matrix.hpp"
#include "algebra/algebra.hpp"
#include "function/function.hpp"
#include "function/constraint.hpp"
#include "geometry/box.hpp"
#include "geometry/list_set.hpp"
#include "symbolic/expression_set.hpp"
#include "dynamics/vector_field.hpp"
#include "io/command_line_interface.hpp"

#include "../test.hpp"

using namespace Ariadne;
using namespace std;

class TestVectorField
{
  public:
    Void test() const {
        ARIADNE_TEST_CALL(test_construct());
        ARIADNE_TEST_CALL(test_missing_dynamics());
    }

    Void test_construct() const {

        Real mu=Dyadic(0.5_x);
        RealVariable x("x"), y("y"), z("z");

        VectorField vanderpol({dot(x)=y,dot(y)=mu*(1-x*x)*y-x},{let(z)=sqrt(sqr(x)+sqr(y))});
        ARIADNE_TEST_PRINT(vanderpol);
        ARIADNE_TEST_EQUALS(vanderpol.state_space().dimension(),2);
        ARIADNE_TEST_EQUALS(vanderpol.auxiliary_space().dimension(),1);
    }

    Void test_missing_dynamics() const {

        Real mu=Dyadic(0.5_x);
        RealVariable x("x"), y("y");
        ARIADNE_TEST_FAIL(VectorField(dot(x)=y));
    }
};

Int main(Int argc, const char* argv[])
{
    if (not CommandLineInterface::instance().acquire(argc,argv)) return -1;
    TestVectorField().test();
    return ARIADNE_TEST_FAILURES;
}