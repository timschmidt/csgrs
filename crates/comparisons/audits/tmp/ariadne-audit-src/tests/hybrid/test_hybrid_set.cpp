/***************************************************************************
 *            test_hybrid_set.cpp
 *
 *  Copyright  2008-20  Pieter Collins
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

#include "config.hpp"
#include "../test.hpp"

#include "algebra/vector.hpp"
#include "algebra/matrix.hpp"
#include "function/function.hpp"

#include "geometry/function_set.hpp"
#include "hybrid/hybrid_set.hpp"
#include "hybrid/hybrid_paving.hpp"
#include "io/figure.hpp"

using namespace std;
using namespace Ariadne;


class TestHybridSet {
public:
    Void test() {
        ARIADNE_TEST_CALL(test_hybrid_list_set());
        //ARIADNE_TEST_CALL(test_compile_incorrect_hybrid_paving());
    }

    Void test_hybrid_list_set() {
        HybridListSet<ExactBoxType> hls;
        DiscreteLocation loc1(123);
        DiscreteLocation loc2(105);
        RealVariable x("x"), y("y");
        RealSpace spc1({x});
        RealSpace spc2({x, y});

        ExactBoxType bx1 = {{0, 1}};
        ExactBoxType bx2 = {{2, 3}};
        ExactBoxType bx3 = {{1, 2},
                            {2, 3}};
        ExactBoxType bx4 = {{4, 5},
                            {5, 6}};
        ExactBoxType bx5 = {{6, 7},
                            {8, 9}};
        ARIADNE_TEST_EXECUTE(hls.adjoin(loc1, spc1, bx1));
        ARIADNE_TEST_FAIL(hls.adjoin(loc1, bx3)); // Should fail due to incompatible dimensions
        ARIADNE_TEST_FAIL(hls.adjoin(loc2, bx3)); // Should fail due to unspecified space
        ARIADNE_TEST_EXECUTE(hls.adjoin(loc2, spc2, bx3));
        ARIADNE_TEST_EXECUTE(hls.adjoin(loc2, bx4));
        ARIADNE_TEST_PRINT(hls);
        ARIADNE_TEST_EXECUTE(hls.adjoin(HybridExactBox(loc1, spc1, bx2)));
        ARIADNE_TEST_EXECUTE(hls.adjoin(loc2, bx5));

        ARIADNE_TEST_PRINT(hls);

        HybridListSet<ExactBoxType>::ConstIterator iter = hls.begin();
        ARIADNE_TEST_EQUAL(*iter, HybridExactBox(loc2, spc2, bx3));
        ++iter;
        ARIADNE_TEST_EQUAL(*iter, HybridExactBox(loc2, spc2, bx4));
        ++iter;
        ARIADNE_TEST_EQUAL(*iter, HybridExactBox(loc2, spc2, bx5));
        ++iter;
        ARIADNE_TEST_EQUAL(*iter, HybridExactBox(loc1, spc1, bx1));
        ++iter;
        ARIADNE_TEST_EQUAL(*iter, HybridExactBox(loc1, spc1, bx2));
        ++iter;
        ARIADNE_TEST_ASSERT(iter == hls.end());

        ListSet<ExactBoxType> ls1;
        ls1.adjoin(bx1);
        ls1.adjoin(bx2);
        ListSet<ExactBoxType> ls2;
        ls2.adjoin(bx3);
        ls2.adjoin(bx4);
        ls2.adjoin(bx5);

        HybridListSet<ExactBoxType>::LocationsConstIterator loc_iter = hls.locations_begin();
        ARIADNE_TEST_EQUAL(loc_iter->first, loc2);
        ARIADNE_TEST_EQUAL(loc_iter->second.second, ls2);
        ++loc_iter;
        ARIADNE_TEST_EQUAL(loc_iter->first, loc1);
        ARIADNE_TEST_EQUAL(loc_iter->second.second, ls1);
        ++loc_iter;
        ARIADNE_TEST_ASSERT(loc_iter == hls.locations_end());
    }

    Void test_compile_incorrect_hybrid_paving() {
        Grid g;
        GridTreePaving gts;
        MonolithicHybridSpace hspc;
        DiscreteLocation loc;
        HybridGrid hg(hspc);
        HybridGridTreePaving hgts(hg);
        Figure fig(hgts[loc].bounding_box(), Projection2d(2, 0, 1));
        hg.has_location(loc);
        g = hg[loc];

        hgts.has_location(loc);
        GridTreePaving const &gtscr = hgts[loc];
        draw(fig, gtscr);
    }
};

Int main() {
    TestHybridSet().test();
    return ARIADNE_TEST_FAILURES;
}

