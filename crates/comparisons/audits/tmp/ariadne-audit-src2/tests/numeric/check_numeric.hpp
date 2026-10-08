/***************************************************************************
 *            check_number.hpp
 *
 *  Copyright  2009-20  Pieter Collins
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

#include "../test.hpp"
#include "utility.hpp"
#include "numeric/operators.hpp"

using namespace Ariadne;

namespace Ariadne {


template<class T1, class T2> concept EqualityComparible = HasOperator<Equal,T1,T2>;
template<class T1, class T2> concept LessThanCompartible = HasOperator<Less,T1,T2>;


template<class N> class CheckNumericTypeConcepts {
  public:
    void check_signed_concept() {
        ARIADNE_TEST_CONCEPT(HasOperatorReturning<N,OperatorPlus,N>);
        ARIADNE_TEST_CONCEPT(HasOperatorReturning<N,OperatorMinus,N>);
        ARIADNE_TEST_CONCEPT(HasOperatorReturning<N,Pos,N>);
        ARIADNE_TEST_CONCEPT(HasOperatorReturning<N,Neg,N>);
    }

    void check_semiring_concept() {
        ARIADNE_TEST_CONCEPT(DefaultConstructible<N>);
        ARIADNE_TEST_CONCEPT(Constructible<N,uint>);
        ARIADNE_TEST_CONCEPT(HasOperator<OperatorPlus,N,N>);
        ARIADNE_TEST_CONCEPT(HasOperator<OperatorTimes,N,N>);
        ARIADNE_TEST_CONCEPT(HasOperator<Pow,N,uint>);
        ARIADNE_TEST_CONCEPT(HasOperator<Sqr,N>);
        ARIADNE_TEST_CONCEPT(HasOperator<Add,N,N>);
        ARIADNE_TEST_CONCEPT(HasOperator<Mul,N,N>);
    }

    void check_ring_concept() {
        check_semiring_concept();
        check_signed_concept();
        ARIADNE_TEST_CONCEPT(Constructible<N,int>);
        ARIADNE_TEST_CONCEPT(HasOperator<OperatorMinus,N>);
        ARIADNE_TEST_CONCEPT(HasOperator<OperatorMinus,N,N>);
        ARIADNE_TEST_CONCEPT(HasOperator<Neg,N>);
        ARIADNE_TEST_CONCEPT(HasOperator<Sub,N,N>);
    }

    void check_field_concept() {
        check_ring_concept();
        if (Same<Paradigm<N>,ApproximateTag>) {
            ARIADNE_TEST_CONCEPT(Constructible<N,double>);
        }
        ARIADNE_TEST_CONCEPT(HasOperator<OperatorDivides,N,N>);
        ARIADNE_TEST_CONCEPT(HasOperator<Rec,N>);
        ARIADNE_TEST_CONCEPT(HasOperator<Div,N,N>);
        ARIADNE_TEST_CONCEPT(HasOperator<Pow,N,int>);
    };

    void check_transcendental_concept() {
        ARIADNE_TEST_CONCEPT(HasOperator<Sqrt,N>);
        ARIADNE_TEST_CONCEPT(HasOperator<Exp,N>);
        ARIADNE_TEST_CONCEPT(HasOperator<Log,N>);
        ARIADNE_TEST_CONCEPT(HasOperator<Sin,N>);
        ARIADNE_TEST_CONCEPT(HasOperator<Cos,N>);
        ARIADNE_TEST_CONCEPT(HasOperator<Tan,N>);
        ARIADNE_TEST_CONCEPT(HasOperator<Atan,N>);
    }

    void check_absolute_concept() {
        ARIADNE_TEST_CONCEPT(HasOperator<Abs,N>);
        ARIADNE_TEST_CONCEPT(HasOperator<Max,N,N>);
        ARIADNE_TEST_CONCEPT(HasOperator<Min,N,N>);
    }

    void check_equality_comparible_concept() {
        ARIADNE_TEST_CONCEPT(EqualityComparible<N,N>);
        ARIADNE_TEST_CONCEPT(HasOperator<Equal,N,N>);
        ARIADNE_TEST_CONCEPT(HasOperator<Unequal,N,N>);
    }

    void check_order_concept() {
        ARIADNE_TEST_CONCEPT(LessThanCompartible<N,N>);
        ARIADNE_TEST_CONCEPT(HasOperator<Less,N,N>);
        ARIADNE_TEST_CONCEPT(HasOperator<Leq,N,N>);
        ARIADNE_TEST_CONCEPT(HasOperator<Gtr,N,N>);
        ARIADNE_TEST_CONCEPT(HasOperator<Geq,N,N>);
    }

    void check_comparible_concept() {
        check_equality_comparible_concept();
        check_order_concept();
    }
};

} // namespace Ariadne

