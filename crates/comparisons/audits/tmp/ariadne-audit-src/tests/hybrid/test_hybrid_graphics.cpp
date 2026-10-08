/***************************************************************************
 *            test_hybrid_graphics.cpp
 *
 *  Copyright  2011-21  Pieter Collins, Luca Geretti
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

#include "config.hpp"
#include "../test.hpp"

#include "foundations/logical.hpp"
#include "numeric/real.hpp"
#include "symbolic/variable.hpp"
#include "symbolic/assignment.hpp"

#include "hybrid/hybrid_graphics.hpp"
#include "hybrid/hybrid_set.hpp"
#include "hybrid/hybrid_paving.hpp"
#include "io/command_line_interface.hpp"

using namespace Ariadne;

class TestHybridGraphics {
  public:

    Void test() const {
        ARIADNE_TEST_CALL(test_boxes());
    }

    Void test_boxes() const {

        RealVariable x("x"),y("y"),z("z");
        DiscreteLocation location(1);
        HybridRealBox hbx1(location,{0<=x<=1,2<=y<=3,5<=z<=7});
        HybridExactBox hbx2(location,{x,y,z},ExactBoxType{{1.0_x,2.0_x},{3.0_x,4.0_x},{6.0_x,8.0_x}});

        HybridFigure hfig;
        hfig.set_locations({location});
        hfig.set_bounds(x,-8,8);
        hfig.set_bounds(y,-8,8);
        hfig.set_bounds(z,-8,8);
        hfig.set_variables(x,y);

        hfig << fill_colour(red) << fill_opacity(1.0) << line_colour(black) << line_width(1.0) << hbx1;
        hfig.write("test_hybrid_graphics1");
        hfig.clear();
        hfig << fill_colour(green) << fill_opacity(0.1) << line_colour(red) << line_width(4.0) << hbx2;
        hfig.write("test_hybrid_graphics2");
    }
};

Int main(Int argc, const char* argv[])
{
    if (not CommandLineInterface::instance().acquire(argc,argv)) return -1;
    TestHybridGraphics().test();

    return ARIADNE_TEST_FAILURES;
}
