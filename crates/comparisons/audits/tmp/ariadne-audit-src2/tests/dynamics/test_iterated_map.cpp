/***************************************************************************
 *            test_iterated_map.cpp
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
#include "function/taylor_model.hpp"
#include "algebra/differential.hpp"
#include "function/constraint.hpp"
#include "function/function.hpp"
#include "function/taylor_function.hpp"
#include "function/formula.hpp"
#include "dynamics/enclosure.hpp"
#include "dynamics/orbit.hpp"
#include "geometry/box.hpp"
#include "geometry/list_set.hpp"
#include "dynamics/iterated_map.hpp"
#include "dynamics/iterated_map_evolver.hpp"
#include "io/figure.hpp"
#include "io/command_line_interface.hpp"

#include "../test.hpp"

using namespace Ariadne;
using namespace std;

class TestIteratedMap
{
  public:
    Void test() const {
        ARIADNE_TEST_CALL(test_construct())
    }

    Void test_construct() const
    {
        Real a=1.5_dyadic; Real b=0.375_dyadic;
        RealVariable x("x"), y("y"), z("z");
        IteratedMap henon({ next(x)=a-x*x+b*y, next(y)=x },{let(z)=x+y});
        ARIADNE_TEST_PRINT(henon);
        ARIADNE_TEST_EQUALS(henon.state_space().dimension(),2)
        ARIADNE_TEST_EQUALS(henon.auxiliary_space().dimension(),1)
        ARIADNE_TEST_EQUALS(henon.state_auxiliary_space().dimension(),3)
    }
};

Int main(Int argc, const char* argv[])
{
    if (not CommandLineInterface::instance().acquire(argc,argv)) return -1;
    TestIteratedMap().test();
    return ARIADNE_TEST_FAILURES;
}
