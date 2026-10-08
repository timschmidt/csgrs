/***************************************************************************
 *            symbolic/operations.hpp
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

/*! \file symbolic/operations.hpp
 *  \brief Operations on symbolic expressions
 */

#ifndef ARIADNE_SYMBOLIC_OPERATIONS_HPP
#define ARIADNE_SYMBOLIC_OPERATIONS_HPP

#include <cstdarg>
#include <iosfwd>
#include <iostream>

#include "utility/typedefs.hpp"

namespace Ariadne {

class Identifier;

template<class T> class Variable;
template<class T> class Space;
template<class T> class Expression;
template<class LHS,class RHS> class Assignment;

template<class X> class Vector;
template<class X> class Matrix;
template<class X> class Formula;
template<class X> class Algebra;

using RealVector = Vector<Real>;
using RealMatrix = Matrix<Real>;

typedef Expression<Boolean> DiscretePredicate;
typedef Expression<Kleenean> ContinuousPredicate;
typedef Expression<String> StringExpression;
typedef Expression<Integer> IntegerExpression;
typedef Expression<Real> RealExpression;



template<class X> struct ExpressionNode;

//! \name Methods for building expressions
//! \related Expression
template<class T> struct DeclareExpressionOperations;

template<class A> using NotType = decltype(!declval<A>());
template<class A1, class A2> using AndType = decltype(declval<A1>()&&declval<A2>());
template<class A1, class A2> using OrType = decltype(declval<A1>()||declval<A2>());

template<class A> using UnaryPlusType = decltype(+declval<A>());
template<class A> using UnaryMinusType = decltype(-declval<A>());
template<class A1, class A2> using PlusType = decltype(declval<A1>()+declval<A2>());
template<class A1, class A2> using MinusType = decltype(declval<A1>()-declval<A2>());
template<class A1, class A2> using TimesType = decltype(declval<A1>()*declval<A2>());
template<class A1, class A2> using DivideType = decltype(declval<A1>()/declval<A2>());

template<class A> using SgnType = decltype(sgn(-declval<A>()));

/*
template<template<class> class T>
struct DeclareOperations {
    template<class A> friend T<NotType<A>> operator!(T<A> const& a);
    template<class A1, class A2> friend T<OrType<A1,A2>> operator||(T<A1> const& a1, T<A2> const& a2);
    template<class A1, class A2> friend T<AndType<A1,A2>> operator&&(T<A1> const& a1, T<A2> const& a2);

    template<class A1, class A2> friend T<EqType<A1,A2>> operator==(T<A1> const& a1, T<A2> const& a2);
    template<class A1, class A2> friend T<NeqType<A1,A2>> operator!=(T<A1> const& a1, T<A2> const& a2);
    template<class A1, class A2> friend T<LeqType<A1,A2>> operator<=(T<A1> const& a1, T<A2> const& a2);
    template<class A1, class A2> friend T<GeqType<A1,A2>> operator>=(T<A1> const& a1, T<A2> const& a2);
    template<class A1, class A2> friend T<LtType<A1,A2>> operator< (T<A1> const& a1, T<A2> const& a2);
    template<class A1, class A2> friend T<GtType<A1,A2>> operator> (T<A1> const& a1, T<A2> const& a2);

    template<class A> friend T<UnaryPlusType<A>> operator+(T<A> const& a);
    template<class A> friend T<UnaryMinusType<A>> operator-(T<A> const& a);
    template<class A1, class A2> friend T<PlusType<A1,A2>> operator+(T<A1> const& a1, T<A2> const& a2);
    template<class A1, class A2> friend T<MinusType<A1,A2>> operator-(T<A1> const& a1, T<A2> const& a2);
    template<class A1, class A2> friend T<TimesType<A1,A2>> operator*(T<A1> const& a1, T<A2> const& a2);
    template<class A1, class A2> friend T<DivideType<A1,A2>> operator/(T<A1> const& a1, T<A2> const& a2);

    template<class A1, class A2> friend T<PlusType<A1,A2>> operator+(T<A1> const& a1, A2 const& a2);
    template<class A1, class A2> friend T<MinusType<A1,A2>> operator-(T<A1> const& a1, A2 const& a2);
    template<class A1, class A2> friend T<TimesType<A1,A2>> operator*(T<A1> const& a1, A2 const& a2);
    template<class A1, class A2> friend T<DivideType<A1,A2>> operator/(T<A1> const& a1, A2 const& a2);
    template<class A1, class A2> friend T<PlusType<A1,A2>> operator+(A1 const& a1, T<A2> const& a2);
    template<class A1, class A2> friend T<MinusType<A1,A2>> operator-(A1 const& a1, T<A2> const& a2);
    template<class A1, class A2> friend T<TimesType<A1,A2>> operator*(A1 const& a1, T<A2> const& a2);
    template<class A1, class A2> friend T<DivideType<A1,A2>> operator/(A1 const& a1, T<A2> const& a2);

    template<class A> friend T<SgnType<A>> sgn(T<A> const& a);

    template<class A> friend T<PosType<A>> pos(T<A> const& a);
    template<class A> friend T<NegType<A>> neg(T<A> const& a);
    template<class A> friend T<SqrType<A>> sqr(T<A> const& a);
    template<class A> friend T<RecType<A>> rec(T<A> const& a);
    template<class A1, class A2> friend T<AddType<A1,A2>> add(T<A1> const& a1, T<A2> const& a2);
    template<class A1, class A2> friend T<SubType<A1,A2>> sub(T<A1> const& a1, T<A2> const& a2);
    template<class A1, class A2> friend T<MulType<A1,A2>> mul(T<A1> const& a1, T<A2> const& a2);
    template<class A1, class A2> friend T<DivType<A1,A2>> div(T<A1> const& a1, T<A2> const& a2);
    template<class A1, class A2, class A3> friend T<FmaType<A1,A2,A3>> fma(T<A1> const& a1, T<A2> const& a2, T<E3> const& a3);
    template<class A1, class A2> friend T<PowType<A1,A2>> pow(T<A1> const& a1, T<A2> const& a2);

    template<class A1, class A2> friend T<MinType<A1,A2>> min(T<A1> const& a1, T<A2> const& a2);
    template<class A1, class A2> friend T<MaxType<A1,A2>> max(T<A1> const& a1, T<A2> const& a2);
    template<class A> friend T<AbsType<A>> abs(T<A> const& a);

    template<class A> friend T<SqrtType<A>> sqrt(T<A> const& a);
    template<class A> friend T<ExpType<A>> exp(T<A> const& a);
    template<class A> friend T<LogType<A>> log(T<A> const& a);
    template<class A> friend T<SinType<A>> sin(T<A> const& a);
    template<class A> friend T<CosType<A>> cos(T<A> const& a);
    template<class A> friend T<TanType<A>> tan(T<A> const& a);
    template<class A> friend T<AtanType<A>> atan(T<A> const& a);
};
*/

template<> struct DeclareExpressionOperations<Boolean>  {
    //! \related Expression \brief Logical disjunction.
    friend Expression<Boolean> operator&&(Expression<Boolean> const& e1, Expression<Boolean> const& e2);
    //! \related Expression \brief Logical conjunction.
    friend Expression<Boolean> operator||(Expression<Boolean> const& e1, Expression<Boolean> const& e2);
    //! \related Expression \brief Logical negation.
    friend Expression<Boolean> operator!(Expression<Boolean> const& e);
};

template<> struct DeclareExpressionOperations<Kleenean> {
    //! \related Expression \brief Fuzzy logical disjunction.
    friend Expression<Kleenean> operator&&(Expression<Kleenean> const& e1, Expression<Kleenean> const& e2);
    //! \related Expression \brief Fuzzy logical conjunction.
    friend Expression<Kleenean> operator||(Expression<Kleenean> const& e1, Expression<Kleenean> const& e2);
    //! \related Expression \brief Fuzzy logical negation.
    friend Expression<Kleenean> operator!(Expression<Kleenean> const& e);
};

template<> struct DeclareExpressionOperations<String>  {
    //! \related Expression \brief %String equality.
    friend Expression<Boolean> operator==(Variable<String> v1, const String& s2);
    //! \related Expression \brief %String inequality.
    friend Expression<Boolean> operator!=(Variable<String> v1, const String& s2);
};

template<> struct DeclareExpressionOperations<Integer>  {
    //! \related Expression \brief %Integer equality predicate.
    friend Expression<Boolean> operator==(Expression<Integer> const& e1, Expression<Integer> const& e2);
    //! \related Expression \brief %Integer inequality predicate.
    friend Expression<Boolean> operator!=(Expression<Integer> const& e1, Expression<Integer> const& e2);
    //! \related Expression \brief %Integer comparison predicate (greater or equal).
    friend Expression<Boolean> operator>=(Expression<Integer> const& e1, Expression<Integer> const& e2);
    //! \related Expression \brief %Integer comparison (less or equal)..
    friend Expression<Boolean> operator<=(Expression<Integer> const& e1, Expression<Integer> const& e2);
    //! \related Expression \brief %Integer comparison (greater).
    friend Expression<Boolean> operator> (Expression<Integer> const& e1, Expression<Integer> const& e2);
    //! \related Expression \brief %Integer comparison (less).
    friend Expression<Boolean> operator< (Expression<Integer> const& e1, Expression<Integer> const& e2);

    //! \related Expression \brief %Integer unary plus expression (identity).
    friend Expression<Integer> operator+(Expression<Integer> const& e);
    //! \related Expression \brief %Integer unary minus expression.
    friend Expression<Integer> operator-(Expression<Integer> const& e);
    //! \related Expression \brief %Integer addition expression.
    friend Expression<Integer> operator+(Expression<Integer> const& e1, Expression<Integer> const& e2);
    //! \related Expression \brief %Integer subtraction expression.
    friend Expression<Integer> operator-(Expression<Integer> const& e1, Expression<Integer> const& e2);
    //! \related Expression \brief %Integer multiplication expression.
    friend Expression<Integer> operator*(Expression<Integer> const& e1, Expression<Integer> const& e2);

    friend Expression<Integer>& operator+=(Expression<Integer>& e1, Expression<Integer> const& e2);
    friend Expression<Integer>& operator-=(Expression<Integer>& e1, Expression<Integer> const& e2);
    friend Expression<Integer>& operator*=(Expression<Integer>& e1, Expression<Integer> const& e2);
};

template<> struct DeclareExpressionOperations<Real> {
    //! \related Expression \brief Positivity test.
    //! Returns \c indeterminate if the value cannot be distinguished from zero.
    friend Expression<Kleenean> sgn(Expression<Real> const& e);
    //! \related Expression \brief Fuzzy inequality comparison predicate (less) of real expressions.
    friend Expression<Kleenean> operator<=(Expression<Real> const& e1, Expression<Real> const& e2);
    //! \related Expression \brief Fuzzy inequality comparison predicate (greater) of real expressions.
    friend Expression<Kleenean> operator>=(Expression<Real> const& e1, Expression<Real> const& e2);
    //! \related Expression \brief Fuzzy inequality comparison predicate (less) of real expressions.
    friend Expression<Kleenean> operator< (Expression<Real> const& e1, Expression<Real> const& e2);
    //! \related Expression \brief Fuzzy inequality comparison predicate (greater) of real expressions.
    friend Expression<Kleenean> operator> (Expression<Real> const& e1, Expression<Real> const& e2);

    //! \related Expression \brief Equality comparison predicate of real expressions.
    friend Expression<Kleenean> operator==(Expression<Real> const& e1, Expression<Real> const& e2);
    //! \related Expression \brief Negated equality comparison predicate of real expressions.
    friend Expression<Kleenean> operator!=(Expression<Real> const& e1, Expression<Real> const& e2);

    //! \related Expression \brief %Real unary plus expression.
    friend Expression<Real> operator+(Expression<Real> const& e);
    //! \related Expression \brief %Real unary minus expression.
    friend Expression<Real> operator-(Expression<Real> const& e);
    //! \related Expression \brief %Real addition expression.
    friend Expression<Real> operator+(Expression<Real> const& e1, Expression<Real> const& e2);
    //! \related Expression \brief %Real subtraction expression.
    friend Expression<Real> operator-(Expression<Real> const& e1, Expression<Real> const& e2);
    //! \related Expression \brief %Real multiplication expression.
    friend Expression<Real> operator*(Expression<Real> const& e1, Expression<Real> const& e2);
    //! \related Expression \brief %Real division expression.
    friend Expression<Real> operator/(Expression<Real> const& e1, Expression<Real> const& e2);

    friend Expression<Real>& operator+=(Expression<Real>& e1, Expression<Real> const& e2);
    friend Expression<Real>& operator-=(Expression<Real>& e1, Expression<Real> const& e2);
    friend Expression<Real>& operator*=(Expression<Real>& e1, Expression<Real> const& e2);
    friend Expression<Real>& operator/=(Expression<Real>& e1, Expression<Real> const& e2);

    //! \related Expression \brief %Real zero expression.
    friend Expression<Real> nul(Expression<Real> const& e);
    //! \related Expression \brief %Real unary plus expression.
    //! Equivalent to +\a e.
    friend Expression<Real> pos(Expression<Real> const& e);
    //! \related Expression \brief %Real negation expression.
    //! Equivalent to -\a e.
    friend Expression<Real> neg(Expression<Real> const& e);
    //! \related Expression \brief %Real squaring expression.
    //! Equivalent to \a e*e.
    friend Expression<Real> hlf(Expression<Real> const& e);
    //! \related Expression \brief %Real halving expression.
    //! Equivalent to \a e/2.
    friend Expression<Real> sqr(Expression<Real> const& e);
     //! \related Expression \brief %Real addition expression.
    friend Expression<Real> add(Expression<Real> const& e1, Expression<Real> const& e2);
    //! \related Expression \brief %Real subtraction expression.
    friend Expression<Real> sub(Expression<Real> const& e1, Expression<Real> const& e2);
    //! \related Expression \brief %Real multiplication expression.
    friend Expression<Real> mul(Expression<Real> const& e1, Expression<Real> const& e2);
    //! \related Expression \brief %Real division expression.
    friend Expression<Real> div(Expression<Real> const& e1, Expression<Real> const& e2);
    //! \related Expression \brief %Real integer power expression.
    friend Expression<Real> pow(Expression<Real> const& e, Int n);
    //! \related Expression \brief %Real reciprocal expression.
    //! Equivalent to 1/\a e.
    friend Expression<Real> rec(Expression<Real> const& e);
    //! \related Expression \brief %Real square root expression.
    friend Expression<Real> sqrt(Expression<Real> const& e);
    //! \related Expression \brief %Real exponential expression.
    friend Expression<Real> exp(Expression<Real> const& e);
    //! \related Expression \brief %Real natural logarithm expression.
    friend Expression<Real> log(Expression<Real> const& e);
    //! \related Expression \brief %Real sine expression.
    friend Expression<Real> sin(Expression<Real> const& e);
    //! \related Expression \brief %Real cosine expression.
    friend Expression<Real> cos(Expression<Real> const& e);
    //! \related Expression \brief %Real tangent expression.
    friend Expression<Real> tan(Expression<Real> const& e);
    //! \related Expression \brief %Real arc-sine expression.
    friend Expression<Real> asin(Expression<Real> const& e);
    //! \related Expression \brief %Real arc-cosine expression.
    friend Expression<Real> acos(Expression<Real> const& e);
    //! \related Expression \brief %Real arc-tangent expression.
    friend Expression<Real> atan(Expression<Real> const& e);

    //! \related Expression \brief Real maximum expression.
    friend Expression<Real> max(Expression<Real> const& e1, Expression<Real> const& e2);
    //! \related Expression \brief %Real minimum expression.
    friend Expression<Real> min(Expression<Real> const& e1, Expression<Real> const& e2);
    //! \related Expression \brief %Real absolute value expression.
    friend Expression<Real> abs(Expression<Real> const& e);
};


template<> struct DeclareExpressionOperations<RealVector> {
    //friend Expression<Vector<Real>> expression(Vector<Expression<Real>>);
    //friend Expression<Real> get(Expression<Vector<Real>> const&, SizeType);
    friend Expression<RealVector> operator-(Expression<RealVector> const&);
    friend Expression<RealVector> operator+(Expression<RealVector> const&, Expression<RealVector> const&);
    friend Expression<RealVector> operator-(Expression<RealVector> const&, Expression<RealVector> const&);
    friend Expression<RealVector> operator*(Expression<Real> const&, Expression<RealVector> const&);
    friend Expression<RealVector> operator*(Expression<RealVector> const&, Expression<Real> const&);
    friend Expression<RealVector> operator/(Expression<RealVector> const&, Expression<Real> const&);
};


template<> struct DeclareExpressionOperations<RealMatrix> {
    friend Expression<Matrix<Real>> expression(Matrix<Expression<Real>>);
    friend Expression<Real> get(Expression<Matrix<Real>> const&, SizeType, SizeType);
    friend Expression<RealMatrix> operator-(Expression<RealMatrix> const&);
    friend Expression<RealMatrix> operator+(Expression<RealMatrix> const&, Expression<RealMatrix> const&);
    friend Expression<RealMatrix> operator-(Expression<RealMatrix> const&, Expression<RealMatrix> const&);
    friend Expression<RealMatrix> operator*(Expression<RealMatrix> const&, Expression<RealMatrix> const&);
    friend Expression<RealMatrix> operator*(Expression<Real> const&, Expression<RealMatrix> const&);
    friend Expression<RealMatrix> operator*(Expression<RealMatrix> const&, Expression<Real> const&);
    friend Expression<RealMatrix> operator/(Expression<RealMatrix> const&, Expression<Real> const&);
    friend Expression<RealVector> operator*(Expression<RealMatrix> const&, Expression<RealVector> const&);
    friend Expression<RealMatrix> transpose(Expression<RealMatrix> const&);
    friend Expression<RealMatrix> inverse(Expression<RealMatrix> const&);
    friend Expression<RealMatrix> solve(Expression<RealMatrix> const&);
    friend Expression<RealVector> solve(Expression<RealVector> const&);
};


} // namespace Ariadne

#endif /* ARIADNE_SYMBOLIC_OPERATIONS_HPP */
