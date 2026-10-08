/***************************************************************************
 *            test_formula.cpp
 *
 *  Copyright  2010-2018  Luca Geretti
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

#include "config.hpp"

#include "numeric/numeric.hpp"
#include "algebra/vector.hpp"
#include "function/formula.hpp"
#include "function/formula.tpl.hpp"

#include "../test.hpp"

#include "function/function.hpp"

using namespace std;
using namespace Ariadne;

class TestFormula
{
    EffectiveFormula two, x, y;

  public:
    TestFormula()
      : two(EffectiveFormula::constant(2.0_x)),
        x(EffectiveFormula::coordinate(0)),
        y(EffectiveFormula::coordinate(1)) { }
  private:
    Void test_construct() {
        EffectiveFormula f1(sqrt(pow(x,2)+pow(y,2))/two);
        ARIADNE_TEST_PRINT(f1);

        Vector<EffectiveFormula> f2({sqrt(pow(x,2)+pow(y,2)), atan(y/x)});
        ARIADNE_TEST_PRINT(f2);
    }

    Void test_identical() {
        ARIADNE_TEST_ASSERT(identical(x,x));
        ARIADNE_TEST_ASSERT(identical(ExactFormula::constant(2.0_x),ExactFormula::constant(2.0_x)));
        ARIADNE_TEST_ASSERT(identical(EffectiveFormula::constant(2.0_x),EffectiveFormula::constant(2.0_x)));
        ARIADNE_TEST_ASSERT(not identical(x*y,y*x));
        ARIADNE_TEST_ASSERT(not identical(x+y,y+x));
        ARIADNE_TEST_ASSERT(identical(cos(x),cos(x)));
        ARIADNE_TEST_ASSERT(identical(pow(x,2),pow(x,2)));
        ARIADNE_TEST_ASSERT(not identical(pow(x,2),pow(x,3)));
    }

    Void test_substitute() {
        EffectiveFormula f(x+cos(y));
        EffectiveFormula s(pow(x,2));
        EffectiveFormula substituted = substitute(f,1,s);
        ARIADNE_TEST_PRINT(substituted);
        ARIADNE_TEST_ASSERT(identical(substituted,x+cos(pow(x,2))));

        Vector<EffectiveFormula> f_v({sqrt(pow(x,2)+pow(y,2)), atan(y/x)});
        List<Pair<Nat,EffectiveFormula>> s_v({{0,x+2},{1,y-1}});
        Vector<EffectiveFormula> substituted_v = substitute(f_v,s_v);
        ARIADNE_TEST_PRINT(substituted_v);
        ARIADNE_TEST_BINARY_PREDICATE(identical,substituted_v[0],sqrt(pow(x+2,2)+pow(y-1,2)));
        ARIADNE_TEST_BINARY_PREDICATE(identical,substituted_v[1],atan((y-1)/(x+2)));
    }

    Void test_simplify() {
        EffectiveFormula z(EffectiveFormula::constant(0));
        ARIADNE_TEST_BINARY_PREDICATE(identical,simplify(sub(neg(z),z)),z);
        EffectiveFormula u(EffectiveFormula::coordinate(2));
        ARIADNE_TEST_BINARY_PREDICATE(identical,simplify(derivative(-u*x*y+2*x,x.ind())),-u*y+2);
    }

    Void test_is_constant_in() {
        ARIADNE_TEST_ASSERT(is_constant_in(x,{1}));
        ARIADNE_TEST_ASSERT(not is_constant_in(x,{0}));
        ARIADNE_TEST_ASSERT(is_constant_in(x*y,{2,3}));
        ARIADNE_TEST_ASSERT(not is_constant_in(cos(x),{0}));
        ARIADNE_TEST_ASSERT(is_constant_in(pow(x,2),{1}));
        ARIADNE_TEST_ASSERT(is_constant_in(two,{0,1}));
        ARIADNE_TEST_ASSERT(not is_constant_in(3*y,{0,1}));
    }

    Void test_is_affine_in() {
        ARIADNE_TEST_ASSERT(is_affine_in(x+y,{0}));
        ARIADNE_TEST_ASSERT(is_affine_in(x+y,{0,1}));
        ARIADNE_TEST_ASSERT(is_affine_in(x+y,{0,1,2}));
        ARIADNE_TEST_ASSERT(is_affine_in(x*y+y,{0}));
        ARIADNE_TEST_ASSERT(is_affine_in(x*y+y,{1}));
        ARIADNE_TEST_ASSERT(not is_affine_in(x*y+y,{0,1}));
        ARIADNE_TEST_ASSERT(is_affine_in(two,{0,1}));
        ARIADNE_TEST_ASSERT(is_affine_in(pow(x,2),{1}));
        ARIADNE_TEST_ASSERT(not is_affine_in(pow(x,2),{0}));
        ARIADNE_TEST_ASSERT(not is_affine_in(x/y,{1}));
        ARIADNE_TEST_ASSERT(is_affine_in(x/y,{0}));
        ARIADNE_TEST_ASSERT(not is_affine_in(sin(x),{0}));
        ARIADNE_TEST_ASSERT(is_affine_in(y*sin(x),{1}));
        ARIADNE_TEST_ASSERT(is_affine_in(Vector<EffectiveFormula>({y*2+x,x+y}),{0,1}));
    }

    Void test_is_additive_in() {
        EffectiveFormula u1(EffectiveFormula::coordinate(2));
        EffectiveFormula u2(EffectiveFormula::coordinate(3));
        ARIADNE_TEST_ASSERT(is_additive_in(Vector<EffectiveFormula>({x+u1,y+u2}),{2,3}));
        ARIADNE_TEST_ASSERT(is_additive_in(Vector<EffectiveFormula>({x+u2,y+u1}),{2,3}));
        ARIADNE_TEST_ASSERT(is_additive_in(Vector<EffectiveFormula>({x+u1,y}),{2}));
        ARIADNE_TEST_ASSERT(is_additive_in(Vector<EffectiveFormula>({x,y+u1}),{2}));
        ARIADNE_TEST_ASSERT(is_additive_in(Vector<EffectiveFormula>({x,y+2*u1}),{2}));
        ARIADNE_TEST_ASSERT(is_additive_in(Vector<EffectiveFormula>({x+u1,y+2*u2}),{2,3}));
        ARIADNE_TEST_ASSERT(not is_additive_in(Vector<EffectiveFormula>({x+u1,y+u1}),{2}));
        ARIADNE_TEST_ASSERT(not is_additive_in(Vector<EffectiveFormula>({x*u1,y+u2}),{2,3}));
        ARIADNE_TEST_ASSERT(not is_additive_in(Vector<EffectiveFormula>({x+u1,y+sqr(u2)}),{2,3}));
    }

  public:
    Void test() {
        ARIADNE_TEST_CALL(test_construct());
        ARIADNE_TEST_CALL(test_identical());
        ARIADNE_TEST_CALL(test_substitute());
        ARIADNE_TEST_CALL(test_simplify());
        ARIADNE_TEST_CALL(test_is_constant_in());
        ARIADNE_TEST_CALL(test_is_affine_in());
        ARIADNE_TEST_CALL(test_is_additive_in());
    }
};


Int main() {
    TestFormula().test();
    return ARIADNE_TEST_FAILURES;
}

