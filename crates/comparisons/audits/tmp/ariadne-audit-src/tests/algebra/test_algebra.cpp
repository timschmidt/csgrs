/***************************************************************************
 *            test_algebra.cpp
 *
 *  Copyright  2023  Pieter Collins
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

#include "algebra/algebra.hpp"
#include "algebra/differential.hpp"

#include "../test.hpp"

using namespace Ariadne;

class TestAlgebra {
    using X=FloatDPApproximation;
    using DX=Differential<X>;
    using AX=Algebra<X>;
    using TAX=TranscendentalAlgebra<X>;
    using EAX=ElementaryAlgebra<X>;
  public:
    TestAlgebra() { }

    void test() const {
        DP pr;
        SizeType as=3;
        SizeType ind=1;
        DegreeType deg=4;
        ARIADNE_TEST_CONSTRUCT(X,cx,(3.0_x,pr));
        ARIADNE_TEST_NAMED_CONSTRUCT(DX,dx,variable(as,deg,cx,ind));
        ARIADNE_TEST_CONSTRUCT(AX,ax,(dx));
        AX rx=ax;
        ARIADNE_TEST_ASSIGN(ax,cx);
        ax=cx;
        ARIADNE_TEST_EXECUTE(ax+ax);
        ax+ax;
        ax-ax;
        ax+=ax;
        rx-=ax;
        ax+=cx;
        ax-=cx;
        ax=-ax;
        ax=ax+ax;
        ax=ax*ax;
        ax=cx+ax;
        ax=cx*ax;
        ax=ax+cx;
        ax=ax-cx;
        ax=ax*cx;
        ARIADNE_TEST_EXECUTE(ax=ax/cx);
        ARIADNE_TEST_ASSIGN(dx,ax.extract<DX>());

        ARIADNE_TEST_CONSTRUCT(TAX,tax,(dx));
        ARIADNE_TEST_ASSIGN(tax,div(tax,tax+cx));
        ARIADNE_TEST_ASSIGN(tax,rec(tax));
        ARIADNE_TEST_ASSIGN(dx,tax.extract<DX>());

    }
};

Int main() {
    TestAlgebra().test();
    return ARIADNE_TEST_FAILURES;
}
