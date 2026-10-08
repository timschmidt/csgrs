/***************************************************************************
 *            algebra/evaluate.hpp
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

/*! \file algebra/evaluate.hpp
 *  \brief Functions for evaluating polynomial expansions.
 */

#ifndef ARIADNE_EVALUATE_HPP
#define ARIADNE_EVALUATE_HPP

namespace Ariadne {

template<class I, class X> class Expansion;
template<class A> class Vector;
class ReverseGradedLexicographicLess;
class ReverseLexicographicKeyLess;

//! \ingroup FunctionModule
//! \brief Evaluate a power series on an algebra using Horner's rule.
template<class X, class A> ArithmeticType<X,A> horner_evaluate(const Expansion<MultiIndex,X>& e, const Vector<A>& y);

//! \ingroup FunctionModule
//! \brief Evaluate a power series on an algebra using Horner's rule.
template<class X, class A> ArithmeticType<X,A> horner_evaluate(const Expansion<DegreeType,X>& e, const A& y);

//! \ingroup FunctionModule
//! \brief Evaluate a power series on an algebra using termwise expansion (slow).
template<class X, class A> ArithmeticType<X,A> power_evaluate(const Expansion<MultiIndex,X>& e, const Vector<A>& y);

template<class X, class A> ArithmeticType<X,A> evaluate(const Expansion<MultiIndex,X>& e, const Vector<A>& y);

template<class X, class A> ArithmeticType<X,A> simple_evaluate(const Expansion<MultiIndex,X>& e, const Vector<A>& y);


template<class X, class A> Vector<ArithmeticType<X,A>> evaluate(const Vector< Expansion<MultiIndex,X> >& e, const Vector<A>& y);

} // namespace Ariadne

#endif /* ARIADNE_EVALUATE_HPP */
