/***************************************************************************
 *            numeric/arithmetic.hpp
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

/*! \file numeric/arithmetic.hpp
 *  \brief Arithmetic declarations
 */

#ifndef ARIADNE_ARITHMETIC_HPP
#define ARIADNE_ARITHMETIC_HPP

#include "utility/metaprogramming.hpp"
#include "foundations/logical.decl.hpp"
#include "number.decl.hpp"
#include "sign.hpp"
#include "foundations/logical.hpp" // TODO: Try to remove; needed for specialisation of Boolean DefineMixedComparisonOperators

namespace Ariadne {

using Nat = uint;
using Int = int;

class Natural;
class Integer;

template<class X> X generic_pow(X const& x, Nat m) {
    X r=x; r*=0; r+=1; X p=x; while(m!=0) { if(m%2==1) { r=r*p; } p=p*p; m=m/2; } return r; }
template<class X> X generic_pow(const X& x, Int n) {
    return n>=0 ? generic_pow(x,Nat(n)) : rec(generic_pow(x,Nat(-n))); }



//! \ingroup NumericAlgebraSubModule
//! \brief Declare operators corresponding to \a X being a mutable field.
template<class X, class Y, class NY, class R=X> struct DeclareInplaceMixedDirectedGroupOperators {
};
template<class X, class Y, class NY> struct DeclareInplaceMixedDirectedGroupOperators<X,Y,NY,X>
{
    friend X& operator+=(X& x1, Y const& x2);
    friend X& operator-=(X& x1, NY const& x2);
};
template<class X, class NX, class R=X> struct DeclareInplaceDirectedGroupOperators : DeclareInplaceMixedDirectedGroupOperators<X,X,NX,R> {
};

//! \ingroup NumericAlgebraSubModule
//! \brief Declare operators corresponding to \a X being a mutable semifield.
template<class X, class Y, class QY, class R=X> struct DeclareInplaceMixedDirectedSemifieldOperators {
};
template<class X, class Y, class QY> struct DeclareInplaceMixedDirectedSemifieldOperators<X,Y,QY,X>
{
    friend X& operator+=(X& x1, Y const& x2);
    friend X& operator*=(X& x1, Y const& x2);
    friend X& operator/=(X& x1, QY const& x2);
};
template<class X, class QX, class R=X> struct DeclareInplaceDirectedSemifieldOperators : DeclareInplaceMixedDirectedSemifieldOperators<X,X,QX,R> {
};

//! \ingroup NumericAlgebraSubModule
//! \brief Declare operators corresponding to \a X being a mutable field.
template<class X, class Y, class R=X> struct DeclareInplaceMixedFieldOperators {
};
template<class X, class Y> struct DeclareInplaceMixedFieldOperators<X,Y,X>
{
    //! \brief Inplace addition. May be implemented as \c x1=x1+x2.
    friend X& operator+=(X& x1, Y const& x2);
    //! \brief Inplace subtraction. May be implemented as \c x1=x1-x2.
    friend X& operator-=(X& x1, Y const& x2);
    //! \brief Inplace multiplication. May be implemented as \c x1=x1*x2.
    friend X& operator*=(X& x1, Y const& x2);
    //! \brief Inplace division. May be implemented as \c x1=x1/x2.
    friend X& operator/=(X& x1, Y const& x2);
};
template<class X, class R=X> struct DeclareInplaceFieldOperators : DeclareInplaceMixedFieldOperators<X,X,R> {
};




//! \ingroup NumericAlgebraSubModule
//! \brief Declare operators corresponding to \a X being a (mathematical) ring.
//! \brief \sa DeclareOrderedOperations, DeclareComparisons
//! \details Inheriting from this class declares operators which are found by argument-dependent lookup.
template<class X, class NX, class R=X, class NR=NX> struct DeclareDirectedGroupOperations
    : DeclareInplaceDirectedGroupOperators<X,NX,R>
{
    //!@{
    //! \name Named operations.

    //! \brief Zero \a 0. Preserves accuracy parameters.
    friend X nul(X const& x);
    //! \brief Positive \a +x. Usually constructs a direct copy.
    friend X pos(X const& x);
    //! \brief Negative \a -x. Usually constructs a direct copy with reversed sign.
    friend NX neg(X const& x);
    friend X neg(NX const& x);

    //! \brief Sum \a x1+x2.
    friend R add(X const& x1, X const& x2);
    //! \brief Difference \a x1-x2. May be implemented in terms of add() and neg() as <code>add(x1,neg(x2))</code>.
    friend R sub(X const& x1, NX const& x2);
    friend NR sub(NX const& x1, X const& x2);
    //!@}


    //!@{
    //! \name  Standard overloadable operators.

    //! \brief Positive \a +x. Usually dispatches to <code>pos(x)</code>
    friend X operator+(X const& x);
    //! \brief Negative \a -x. Usually dispatches to <code>neg(x)</code>
    friend NX operator-(X const& x);
    friend X operator-(NX const& x);
    //! \brief Sum \a x1+x2. Usually dispatches to <code>sum(x1,x2)</code>
    friend R operator+(X const& x1, X const& x2);
    //! \brief Difference \a x1-x2. Usually dispatches to <code>sub(x1,x2)</code>
    friend R operator-(X const& x1, NX const& x2);
    friend NR operator-(NX const& x1, X const& x2);
    //!@}
};

//! \ingroup NumericAlgebraSubModule
//! \brief Declare operators corresponding to \a X being a (mathematical) group.
template<class X, class R=X> struct DeclareGroupOperations
    : DeclareDirectedGroupOperations<X,X,R> { };

//! \ingroup NumericAlgebraSubModule
//! \brief Declare operators corresponding to \a X being a (mathematical) ring.
//! \brief \sa DeclareOrderedOperations, DeclareComparisons
//! \details Inheriting from this class declares operators which are found by argument-dependent lookup.
template<class X, class R=X, class PR=R> struct DeclareRingOperations
    : DeclareGroupOperations<X,R>
{
    //!@{
    //! \name Named operations.

    //! \brief Square \a x^2.
    friend PR sqr(X const& x);
    //! \brief Product \a x1*x2.
    friend R mul(X const& x1, X const& x2);
    //! \brief Power \a x<sup>m</sup>. May be implemented in terms of mul() and/or sqr().
    friend R pow(X const& x, Nat m);
    //!@}


    //!@{
    //! \name  Standard overloadable operators.

    //! \brief Product \a x1*x2. Usually dispatches to <code>mul(x1,x2)</code>
    friend R operator*(X const& x1, X const& x2);

    //!@}
};

//! \ingroup NumericAlgebraSubModule
//! \brief Declare operators corresponding to \a X being a (mathematical) field.
//! Includes all operations supported in DeclareRingOperations<X>, and those listed below.
template<class X, class R=X> struct DeclareFieldOperations
    : DeclareRingOperations<X,R>, DeclareInplaceFieldOperators<X,R>
{
    //! \brief Half \a x/2.
    friend R hlf(X const& x);
    //! \brief Reciprocal \a 1/x.
    friend R rec(X const& x);
    //! \brief Quotient \a x1/x2. May be implemented in terms of mul() and rec() as <code>mul(x1,rec(x2))</code>.
    friend R div(X const& x1, X const& x2);
    //! \brief Power \a x<sup>n</sup>. May be implemented using rec() and pow(X,Nat) as  <code> n>=0 ? pow(x,Nat(n)) : rec(pow(x,Nat(-n)))</code>
    friend R pow(X const& x, Int n);

    //! \brief Quotient \a x1*x2. Usually dispatches to <code>div(x1,x2)</code>
    friend R operator/(X const&, X const&);
};

template<class X, class R=X> struct DeclareFieldOperators : DeclareInplaceFieldOperators<X,R> {
    friend R operator+(X const&, X const&);
    friend R operator-(X const&, X const&);
    friend R operator*(X const&, X const&);
    friend R operator/(X const&, X const&);
};

//! \ingroup NumericAlgebraSubModule
//! \brief Declare elementary algebraic and transcendental operations.
template<class X, class R=X> class DeclareMonotoneOperations
{
    //! \brief The square root of \a x, √\a x. Requires \c x>=0.
    friend X sqrt(X const& x);
    //! \brief The natural exponent of \a x, \em e<sup>x</sup>.
    friend X exp(X const& x);
    //! \brief The natural logarithm of \a x. Requires \c x>=0.
    friend X log(X const& x);
    //! \brief The arc-tangent of \a x.
    friend X atan(X const& x);
};

//! \ingroup NumericAlgebraSubModule
//! \brief Declare elementary algebraic and transcendental operations.
template<class X, class R=X> class DeclareTranscendentalOperations
    : DeclareMonotoneOperations<X,R>
{
    //! \brief The sine of \a x.
    friend X sin(X const& x);
    //! \brief The cosine of \a x.
    friend X cos(X const& x);
    //! \brief The tangent of \a x, sin(\a x)/cos(\a x).
    friend X tan(X const& x);
};

//! \ingroup NumericAlgebraSubModule
//! \brief Declare elementary arithmetic, algebraic and transcendental operations.
template<class X, class R=X> class DeclareAnalyticFieldOperations
    : DeclareFieldOperations<X,R>, DeclareTranscendentalOperations<X,R>
{
};

//! \ingroup NumericAlgebraSubModule
//! \brief Declare operators corresponding to \a X being an ordered mutable field.
//! \sa DeclareRingOperations, DeclareFieldOperations, DeclareComparisons
template<class X, class PX=Void> class DeclareLatticeOperations
{
    //! \brief The absolute value of \a x.
    friend PX abs(X const& x);
    //! \brief The mimimum of \a x1 and \a x2.
    friend X min(X const& x1, X const& x2);
    friend PX min(PX const& x1, PX const& x2);
    //! \brief The maximum of \a x1 and \a x2.
    friend X max(X const& x1, X const& x2);
    friend PX max(PX const& x1, X const& x2);
    friend PX max(X const& x1, PX const& x2);
    friend PX max(PX const& x1, PX const& x2);
    //friend U mag(X const& x);
    //friend L mig(X const& x);
};
template<class X> class DeclareLatticeOperations<X,Void>
{
    friend X min(X const& x1, X const& x2);
    friend X max(X const& x1, X const& x2);
};

//! \ingroup NumericAlgebraSubModule
//! \brief Declare comparison operations.
//! \sa DeclareRingOperations, DeclareComparisons
template<class X, class LT, class EQ=LT> struct DeclareComparisonOperations {
    typedef LT GT;
    typedef decltype(not declval<LT>()) GEQ;
    typedef decltype(not declval<GT>()) LEQ;
    typedef decltype(not declval<EQ>()) NEQ;

    //! \brief Tests if \a x1 is equal to \a x2.
    friend EQ eq(X const& x1, X const& x2);
    //! \brief Tests if \a x1 is less than \a x2.
    friend LT lt(X const& x1, X const& x2);
    //! \brief Tests if \a x1 is equal to \a x2. If equality is undecidable, may not return \a true even if values are equal.
    friend EQ  operator==(X const& x1, X const& x2);
    //! \brief Tests if \a x1 is strictly less than \a x2.
    friend LT  operator< (X const& x1, X const& x2);
    //! \brief Tests if \a x1 is not equal to \a x2.
    friend NEQ operator!=(X const& x1, X const& x2);
    //! \brief Tests if \a x1 is strictly greater than \a x2.
    friend GT operator> (X const& x1, X const& x2);
    //! \brief Tests if \a x1 is less than or equal to \a x2.
    friend LEQ operator<=(X const& x1, X const& x2);
    //! \brief Tests if \a x1 is greater than or equal to \a x2.
    friend GEQ operator>=(X const& x1, X const& x2);
};

template<class X, class NX, class LT, class EQ> struct DeclareDirectedComparisonOperations {
    typedef decltype(not declval<EQ>()) NEQ; typedef decltype(not declval<LT>()) GT; typedef LT LEQ; typedef GT GEQ;
    friend LT lt(X const& x1, NX const& x2);
    friend EQ eq(X const& x1, NX const& x2);
    friend EQ  operator==(X const& x1, NX const& x2);
    friend EQ  operator==(NX const& x1, X const& x2);
    friend NEQ operator!=(X const& x1, NX const& x2);
    friend NEQ operator!=(NX const& x1, X const& x2);
    friend LT  operator< (X const& x1, NX const& x2);
    friend GT  operator> (X const& x1, NX const& x2);
    friend LT  operator> (NX const& x1, X const& x2);
    friend GT  operator< (NX const& x1, X const& x2);
    friend LEQ operator<=(X const& x1, NX const& x2);
    friend GEQ operator>=(X const& x1, NX const& x2);
    friend LEQ operator>=(NX const& x1, X const& x2);
    friend GEQ operator<=(NX const& x1, X const& x2);
};

template<class X, class LT, class EQ=LT> class DeclareOrderedAnalyticFieldOperations
    : DeclareFieldOperations<X>, DeclareTranscendentalOperations<X>, DeclareLatticeOperations<X>, DeclareComparisonOperations<X,LT,EQ>
{
};


template<class X, class NX, class Y, class NY, class R=X, class NR=NX> struct DeclareMixedDirectedGroupOperators
    : DeclareInplaceMixedDirectedGroupOperators<X,Y,NY,R>, DeclareInplaceMixedDirectedGroupOperators<NX,NY,Y,NR>
{
    friend R operator+(X const& x1, Y const& y2);
    friend R operator-(X const& x1, NY const& y2);
    friend NR operator+(NX const& x1, NY const& y2);
    friend NR operator-(NX const& x1, Y const& y2);
    friend R operator+(Y const& y1, X const& x2);
    friend R operator-(Y const& y1, NX const& x2);
    friend NR operator+(NY const& y1, NX const& x2);
    friend NR operator-(NY const& y1, X const& x2);
};

template<class X, class QX, class Y, class QY, class R=X, class QR=QX> struct DeclareMixedDirectedSemifieldOperators
    : DeclareInplaceMixedDirectedSemifieldOperators<X,Y,QY,R>, DeclareInplaceMixedDirectedGroupOperators<QX,QY,Y,QR>
{
    friend R operator+(X const& x1, Y const& y2);
    friend R operator*(X const& x1, Y const& y2);
    friend R operator/(X const& x1, QY const& y2);
    friend QR operator+(QX const& x1, QY const& y2);
    friend QR operator*(QX const& x1, QY const& y2);
    friend QR operator/(QX const& x1, Y const& y2);
    friend R operator+(Y const& y1, X const& x2);
    friend R operator*(Y const& y1, X const& x2);
    friend R operator/(Y const& y1, QX const& x2);
    friend QR operator+(QY const& y1, QX const& x2);
    friend QR operator*(QY const& y1, QX const& x2);
    friend QR operator/(QY const& y1, X const& x2);
};

template<class X, class Y, class R=X> struct DeclareMixedFieldOperators
    : DeclareInplaceMixedFieldOperators<X,Y,R>
{
    friend R operator+(X const& x1, Y const& y2);
    friend R operator-(X const& x1, Y const& y2);
    friend R operator*(X const& x1, Y const& y2);
    friend R operator/(X const& x1, Y const& y2);
    friend R operator+(Y const& y1, X const& x2);
    friend R operator-(Y const& y1, X const& x2);
    friend R operator*(Y const& y1, X const& x2);
    friend R operator/(Y const& y1, X const& x2);
};
template<class X, class Y, class R=X> struct DeclareMixedFieldOperations
    : DeclareMixedFieldOperators<X,Y,R>
{
    friend R add(X const& x1, Y const& y2);
    friend R sub(X const& x1, Y const& y2);
    friend R mul(X const& x1, Y const& y2);
    friend R div(X const& x1, Y const& y2);
    friend R add(Y const& y1, X const& x2);
    friend R sub(Y const& y1, X const& x2);
    friend R mul(Y const& y1, X const& x2);
    friend R div(Y const& y1, X const& x2);
};

template<class X, class Y, class R=X> struct DeclareMixedArithmeticOperators : DeclareMixedFieldOperators<X,Y,R> { };
template<class X, class Y, class R=X> struct DeclareMixedArithmeticOperations : DeclareMixedFieldOperations<X,Y,R> { };

template<class X, class NX, class R=X> struct DeclareDirectedNumericOperations
    : DeclareDirectedGroupOperations<X,NX,R>, DeclareMonotoneOperations<X,R>, DeclareLatticeOperations<X>
{
    friend X nul(X const& x);
    friend X pos(X const& x);
    friend X neg(NX const& x);
    friend X hlf(X const& x);
    friend R add(X const& x1, X const& x2);
    friend R sub(X const& x1, NX const& x2);
    friend R sqrt(X const& x);
    friend R exp(X const& x);
    friend R log(X const& x);
    friend R atan(X const& x);
    friend X max(X const& x1, X const& x2);
    friend X min(X const& x1, X const& x2);
};

template<class X, class R=X> struct DeclareNumericOperations
    : DeclareAnalyticFieldOperations<X,R>
{
    friend X nul(X const& x);
    friend X pos(X const& x);
    friend X neg(X const& x);
    friend X hlf(X const& x);
    friend R sqr(X const& x);
    friend R rec(X const& x);
    friend R add(X const& x1, X const& x2);
    friend R sub(X const& x1, X const& x2);
    friend R mul(X const& x1, X const& x2);
    friend R div(X const& x1, X const& x2);
    friend R pow(X const& x, Nat m);
    friend R pow(X const& x, Int n);
    friend R sqrt(X const& x);
    friend R exp(X const& x);
    friend R log(X const& x);
    friend R sin(X const& x);
    friend R cos(X const& x);
    friend R tan(X const& x);
    friend R asin(X const& x);
    friend R acos(X const& x);
    friend R atan(X const& x);
    friend X max(X const& x1, X const& x2);
    friend X min(X const& x1, X const& x2);
    friend X abs(X const& x);
};


template<class X, class NX=X> struct DeclareDirectedNumericOperators {
    friend X operator+(X const& x);
    friend X operator-(NX const& x);
    friend NX operator-(X const& x);
    friend X operator+(X const& x1, X const& x2);
    friend X operator-(X const& x1, NX const& x2);
    friend NX operator-(NX const& x1, X const& x2);
    friend X& operator+=(X& x1, X const& x2);
    friend X& operator-=(X& x1, NX const& x2);

    friend X nul(X const& x);
    friend X pos(X const& x);
    friend X neg(NX const& x);
    friend NX neg(X const& x);
    friend X hlf(X const& x);
    friend X add(X const& x1, X const& x2);
    friend X sub(X const& x1, NX const& x2);
    friend NX sub(NX const& x1, X const& x2);

    friend X max(X const& x1, X const& x2);
    friend X min(X const& x1, X const& x2);
};

template<class X, class QX> struct DeclarePositiveDirectedNumericOperations {
    friend X add(X const&, X const&);
    friend X mul(X const&, X const&);
    friend X pow(X const&, Natural const&);
    friend X div(X const&, QX const&);
    friend QX rec(X const&);
    friend X sqrt(X const&);
    friend X atan(X const&);

    friend X max(X const&, X const&);
    friend X min(X const&, X const&);
};


template<class X, class PX=X> struct DeclareRealOperations
    : DeclareFieldOperators<X>
{
    friend X neg(X const&);
    friend X hlf(X const&);
    friend X add(X const&, X const&);
    friend X sub(X const&, X const&);
    friend X mul(X const&, X const&);
    friend X pow(X const&, Integer const&);
    friend X rec(X const&);
    friend PX sqrt(PX const&);
    friend X sqrt(X const&);
    friend X exp(X const&);
    friend X log(X const&);
    friend X sin(X const&);
    friend X cos(X const&);
    friend X tan(X const&);
    friend X atan(X const&);
    friend PX atan(PX const&);

    friend X max(X const&, X const&);
    friend PX max(X const&, PX const&);
    friend PX max(PX const&, X const&);
    friend X min(X const&, X const&);
    friend PX min(PX const&, PX const&);
    friend PX abs(X const&);
    friend PX dist(X const&, X const&);
};

template<class X, class NX, class PX> struct DeclareDirectedRealOperations {
    friend NX neg(X const&);
    friend X add(X const&, X const&);
    friend X sub(X const&, NX const&);
    friend PX sqrt(PX const&);
    friend PX exp(X const&);
    friend X log(PX const&);
    friend X atan(X const&);

    friend X max(X const&, X const&);
    friend PX max(X const&, PX const&);
    friend PX max(PX const&, X const&);
    friend X min(X const&, X const&);
};

template<class PX> struct DeclarePositiveRealOperations {
    friend PX add(PX const&, PX const&);
    friend PX mul(PX const&, PX const&);
    friend PX div(PX const&, PX const&);
    friend PX rec(PX const&);
    friend PX sqrt(PX const&);
    friend PX atan(PX const&);

    friend PX max(PX const&, PX const&);
    friend PX min(PX const&, PX const&);
};

template<class X, class QX> struct DeclarePositiveDirectedRealOperations {
    friend X add(X const&, X const&);
    friend X mul(X const&, X const&);
    friend X pow(X const&, Natural const&);
    friend X div(X const&, QX const&);
    friend QX rec(X const&);
    friend X sqrt(X const&);
    friend X atan(X const&);

    friend X max(X const&, X const&);
    friend X min(X const&, X const&);
};

template<class T, class NT=T, class QT=NT> struct DeclareArithmeticOperators {
    friend T operator+(T const& t);
    friend T operator-(NT const& t);
    friend NT operator-(T const& t);
    friend T operator+(T const& t1, T const& t2);
    friend T operator-(T const& t1, NT const& t2);
    friend T operator*(T const& t1, T const& t2);
    friend T operator/(T const& t1, QT const& t2);
    friend T& operator+=(T& t1, T const& t2);
    friend T& operator-=(T& t1, NT const& t2);
    friend T& operator*=(T& t1, T const& t2);
    friend T& operator/=(T& t1, QT const& t2);
};
template<class T, class NT=T, class QT=NT> struct DefineArithmeticOperators {
    friend T operator+(T const& t) { return pos(t); }
    friend NT operator-(T const& t) { return neg(t); }
    friend T operator+(T const& t1, T const& t2) { return add(t1,t2); }
    friend T operator-(T const& t1, NT const& t2) { return sub(t1,t2); }
    friend T operator*(T const& t1, T const& t2) { return mul(t1,t2); }
    friend T operator/(T const& t1, NT const& t2) { return div(t1,t2); }
    friend T& operator+=(T& t1, T const& t2) { return t1=add(t1,t2); }
    friend T& operator-=(T& t1, NT const& t2) { return t1=sub(t1,t2); }
    friend T& operator*=(T& t1, T const& t2) { return t1=mul(t1,t2); }
    friend T& operator/=(T& t1, NT const& t2) { return t1=div(t1,t2); }
};


template<class X, class Y, class NY, class R=X> struct DefineInplaceDirectedGroupOperators {
};
template<class X, class Y, class NY> struct DefineInplaceDirectedGroupOperators<X,Y,NY,X> {
    friend X& operator+=(X& x1, Y const& y2) { return x1=operator+(x1,y2); }
    friend X& operator-=(X& x1, NY const& y2) { return x1=operator-(x1,y2); }
};
template<class X, class NX> class DefineDirectedGroupOperators {
    // BUG: Cannot declare these due to bug in Clang
    // friend X operator-(NX const&);
    // friend NX operator-(NX const&, X const&);
    friend X operator+(X const& x) { return pos(x); }
    friend NX operator-(X const& x) { return neg(x); }
    friend X operator+(X const& x1, X const& x2) { return add(x1,x2); }
    friend X operator-(X const& x1, NX const& x2) { return sub(x1,x2); }
    friend X& operator+=(X& x1, X const& x2) { return x1=add(x1,x2); }
    friend X& operator-=(X& x1, NX const& x2) { return x1=sub(x1,x2); }
};

template<class X, class Y, class QY, class R=X> struct DefineInplaceDirectedSemiFieldOperators {
};
template<class X, class Y, class QY> struct DefineInplaceDirectedSemiFieldOperators<X,Y,QY,X> {
    friend X operator+=(X& x1, const Y& y2) { return x1=operator+(x1,y2); }
    friend X operator*=(X& x1, const Y& y2) { return x1=operator*(x1,y2); }
    friend X operator/=(X& x1, const QY& y2) { return x1=operator/(x1,y2); }
};
template<class X, class QX, class R=X, class QR=QX> struct DefineDirectedSemiFieldOperators
    : DefineInplaceDirectedSemiFieldOperators<X,X,QX,R>
{
    friend R operator+(const X& x1, const X& x2) { return add(x1,x2); }
    friend R operator*(const X& x1, const X& x2) { return mul(x1,x2); }
    friend R operator/(const X& x1, const QX& qx2) { return div(x1,qx2); }
    // BUG: Cannot declare this due to bug in Clang
    // friend QR operator/(const QX& qx1, const X& x2);
};
template<class X, class QX, class Y, class QY, class R=X, class QR=QX> struct DefineMixedDirectedSemiFieldOperators
    : DefineInplaceDirectedSemiFieldOperators<X,Y,QY,R>
{
    friend R operator+(const X& x1, const Y& y2) { return operator+(x1,factory(x1).create(y2)); }
    friend R operator*(const X& x1, const Y& y2) { return operator*(x1,factory(x1).create(y2)); }
    friend R operator/(const X& x1, const QY& qy2) { return operator/(x1,create(qy2,x1)); }
    friend R operator+(const Y& y1, const X& x2) { return operator+(factory(x2).create(y1),x2); }
    friend R operator*(const Y& y1, const X& x2) { return operator*(factory(x2).create(y1),x2); }
    friend QR operator/(const QY& y1, const X& x2) { return operator/(factory(x2).create(y1),x2); }
};

template<class X, class R=X> struct DefineSemiFieldOperators
    : DefineDirectedSemiFieldOperators<X,X,R,R>
{
};
template<class X, class Y, class R=X> struct DefineMixedSemiFieldOperators
    : DefineMixedDirectedSemiFieldOperators<X,X,Y,Y,R,R>
{
};

template<class X, class Y=X, class R=X> struct DefineInplaceRingOperators {
};
template<class X, class Y> struct DefineInplaceRingOperators<X,Y,X> {
    friend X& operator+=(X& x1, Y const& y2) { return x1=operator+(x1,y2); }
    friend X& operator-=(X& x1, Y const& y2) { return x1=operator-(x1,y2); }
    friend X& operator*=(X& x1, Y const& y2) { return x1=operator*(x1,y2); }
};
template<class X, class R=X> struct DefineRingOperators : DefineInplaceRingOperators<X,X,R> {
    friend X operator+(X const& x) { return pos(x); }
    friend X operator-(X const& x) { return neg(x); }
    friend R operator+(X const& x1, X const& x2) { return add(x1,x2); }
    friend R operator-(X const& x1, X const& x2) { return sub(x1,x2); }
    friend R operator*(X const& x1, X const& x2) { return mul(x1,x2); }
};

template<class X, class Y=X, class R=X> struct DefineInplaceFieldOperators {
};
template<class X, class Y> struct DefineInplaceFieldOperators<X,Y,X> {
    friend X& operator+=(X& x1, Y const& y2) { return x1=operator+(x1,y2); }
    friend X& operator-=(X& x1, Y const& y2) { return x1=operator-(x1,y2); }
    friend X& operator*=(X& x1, Y const& y2) { return x1=operator*(x1,y2); }
    friend X& operator/=(X& x1, Y const& y2) { return x1=operator/(x1,y2); }
};
template<class X, class R=X> struct DefineFieldOperators : DefineInplaceFieldOperators<X,X,R> {
    friend X operator+(X const& x) { return pos(x); }
    friend X operator-(X const& x) { return neg(x); }
    friend R operator+(X const& x1, X const& x2) { return add(x1,x2); }
    friend R operator-(X const& x1, X const& x2) { return sub(x1,x2); }
    friend R operator*(X const& x1, X const& x2) { return mul(x1,x2); }
    friend R operator/(X const& x1, X const& x2) { return div(x1,x2); }
};

template<class X, class Y, class R=X> struct DefineMixedFieldOperators : DefineInplaceFieldOperators<X,Y,R> {
    friend R operator+(X const& x1, Y const& y2) { return add(x1,y2); }
    friend R operator-(X const& x1, Y const& y2) { return sub(x1,y2); }
    friend R operator*(X const& x1, Y const& y2) { return mul(x1,y2); }
    friend R operator/(X const& x1, Y const& y2) { return div(x1,y2); }
    friend R operator+(Y const& y1, X const& x2) { return add(y1,x2); }
    friend R operator-(Y const& y1, X const& x2) { return sub(y1,x2); }
    friend R operator*(Y const& y1, X const& x2) { return mul(y1,x2); }
    friend R operator/(Y const& y1, X const& x2) { return div(y1,x2); }
};

template<class X, class LT, class EQ=LT> struct DefineComparisonOperators {
    typedef LT GT; typedef decltype(not declval<EQ>()) NEQ; typedef decltype(not declval<LT>()) GEQ; typedef decltype(not declval<GT>()) LEQ;
    friend EQ eq(X const& x1, X const& x2);
    friend LT lt(X const& x1, X const& x2);
    friend EQ  operator==(X const& x1, X const& x2) { return eq(x1,x2); }
    friend NEQ operator!=(X const& x1, X const& x2) { return not eq(x1,x2); }
    friend LT  operator< (X const& x1, X const& x2) { return lt(x1,x2); }
    friend GT  operator> (X const& x1, X const& x2) { return lt(x2,x1); }
    friend LEQ operator<=(X const& x1, X const& x2) { return not lt(x2,x1); }
    friend GEQ operator>=(X const& x1, X const& x2) { return not lt(x1,x2); }
};

template<class X, class Y, class LT, class EQ=LT> struct DefineMixedComparisonOperators {
    typedef LT GT; typedef decltype(not declval<EQ>()) NEQ; typedef decltype(not declval<LT>()) GEQ; typedef decltype(not declval<GT>()) LEQ;
    friend EQ eq(X const& x1, Y const& y2);
    friend LT lt(X const& x1, Y const& y2);
    friend GT gt(X const& x1, Y const& y2);
    friend EQ  operator==(X const& x1, Y const& y2) { return eq(x1,y2); }
    friend NEQ operator!=(X const& x1, Y const& y2) { return not eq(x1,y2); }
    friend LEQ operator<=(X const& x1, Y const& y2) { return not gt(x1,y2); }
    friend GEQ operator>=(X const& x1, Y const& y2) { return not lt(x1,y2); }
    friend LT  operator< (X const& x1, Y const& y2) { return lt(x1,y2); }
    friend GT  operator> (X const& x1, Y const& y2) { return gt(x1,y2); }
    friend EQ  operator==(Y const& y1, X const& x2) { return eq(x2,y1); }
    friend NEQ operator!=(Y const& y1, X const& x2) { return not eq(x2,y1); }
    friend GT  operator< (Y const& y1, X const& x2) { return gt(x2,y1); }
    friend LT  operator> (Y const& y1, X const& x2) { return lt(x2,y1); }
    friend GEQ operator<=(Y const& y1, X const& x2) { return not lt(x2,y1); }
    friend LEQ operator>=(Y const& y1, X const& x2) { return not gt(x2,y1); }
};

template<class X, class Y> struct DefineMixedComparisonOperators<X,Y,Boolean> {
    friend Comparison cmp(X const& x1, Y const& y2);
    friend Boolean eq(X const& x1, Y const& y2) { return cmp(x1,y2)==Comparison::EQUAL; }
    friend Boolean lt(X const& x1, Y const& y2) { return cmp(x1,y2)==Comparison::LESS; }
    friend Boolean gt(X const& x1, Y const& y2) { return cmp(x1,y2)==Comparison::GREATER; }
    friend Boolean operator==(X const& x1, Y const& y2) { return eq(x1,y2); }
    friend Boolean operator!=(X const& x1, Y const& y2) { return not eq(x1,y2); }
    friend Boolean operator<=(X const& x1, Y const& y2) { return not gt(x1,y2); }
    friend Boolean operator>=(X const& x1, Y const& y2) { return not lt(x1,y2); }
    friend Boolean operator< (X const& x1, Y const& y2) { return lt(x1,y2); }
    friend Boolean operator> (X const& x1, Y const& y2) { return gt(x1,y2); }
    friend Boolean operator==(Y const& y1, X const& x2) { return eq(x2,y1); }
    friend Boolean operator!=(Y const& y1, X const& x2) { return not eq(x2,y1); }
    friend Boolean operator< (Y const& y1, X const& x2) { return gt(x2,y1); }
    friend Boolean operator> (Y const& y1, X const& x2) { return lt(x2,y1); }
    friend Boolean operator<=(Y const& y1, X const& x2) { return not lt(x2,y1); }
    friend Boolean operator>=(Y const& y1, X const& x2) { return not gt(x2,y1); }
};

template<class X, class NX, class LT, class EQ> struct DefineDirectedComparisonOperators {
    typedef decltype(not declval<EQ>()) NEQ; typedef decltype(not declval<LT>()) GT; typedef LT LEQ; typedef GT GEQ;
    friend EQ eq(X const& x1, NX const& nx2);
    friend LT lt(X const& x1, NX const& nx2);
    friend EQ  operator==(X const& x1, NX const& nx2) { return eq(x1,nx2); }
    friend NEQ operator!=(X const& x1, NX const& nx2) { return not eq(x1,nx2); }
    friend LT  operator< (X const& x1, NX const& nx2) { return lt(x1,nx2); }
    friend LT  operator> (NX const& nx1, X const& x2) { return lt(x2,nx1); }
    friend GEQ operator>=(X const& x1, NX const& nx2) { return not lt(x1,nx2); }
    friend GEQ operator<=(NX const& nx1, X const& x2) { return not lt(x2,nx1); }

    // BUG: Cannot declare these due to bug in Clang
    // friend EQ  operator==(NX const& nx1, X const& x2);
    // friend NEQ operator!=(NX const& nx1, X const& x2);
    // friend GT  operator< (NX const& nx1, X const& x2);
    // friend GT  operator> (X const& x1, NX const& nx2);
    // friend LEQ operator>=(NX const& nx1, X const& x2);
    // friend LEQ operator<=(X const& x1, NX const& nx2);
};

template<class X, class NY, class LT, class EQ> struct DefineMixedDirectedComparisonOperators {
    typedef decltype(not declval<EQ>()) NEQ; typedef decltype(not declval<LT>()) GT; typedef LT LEQ; typedef GT GEQ;
    friend EQ eq(X const& x1, NY const& y2);
    friend LT lt(X const& x1, NY const& y2);
    friend GT gt(X const& x1, NY const& y2);
    friend EQ  operator==(X const& x1, NY const& ny2) { return eq(x1,ny2); }
    friend NEQ operator!=(X const& x1, NY const& ny2) { return not eq(x1,ny2); }
    friend LT  operator< (X const& x1, NY const& ny2) { return lt(x1,ny2); }
    friend GT  operator> (X const& x1, NY const& ny2) { return gt(x1,ny2); }
    friend LEQ operator<=(X const& x1, NY const& ny2) { return not gt(x1,ny2); }
    friend GEQ operator>=(X const& x1, NY const& ny2) { return not lt(x1,ny2); }
    friend EQ  operator==(NY const& ny1, X const& x2) { return eq(x2,ny1); }
    friend NEQ operator!=(NY const& ny1, X const& x2) { return not eq(x2,ny1); }
    friend GT  operator< (NY const& ny1, X const& x2) { return gt(x2,ny1); }
    friend LT  operator> (NY const& ny1, X const& x2) { return lt(x2,ny1); }
    friend GEQ operator<=(NY const& ny1, X const& x2) { return not lt(x2,ny1); }
    friend LEQ operator>=(NY const& ny1, X const& x2) { return not gt(x2,ny1); }
};




template<class X, class Y, class R=X> struct ProvideConvertedFieldOperations;

template<class X, class Y, class R> struct ProvideConvertedFieldOperations
{
    friend R operator+(X x1, Y y2) { return operator+(R(x1),R(y2)); }
    friend R operator-(X x1, Y y2) { return operator-(R(x1),R(y2)); }
    friend R operator*(X x1, Y y2) { return operator*(R(x1),R(y2)); }
    friend R operator/(X x1, Y y2) { return operator/(R(x1),R(y2)); }
    friend R operator+(Y y1, X x2) { return operator+(R(y1),R(x2)); }
    friend R operator-(Y y1, X x2) { return operator-(R(y1),R(x2)); }
    friend R operator*(Y y1, X x2) { return operator*(R(y1),R(x2)); }
    friend R operator/(Y y1, X x2) { return operator/(R(y1),R(x2)); }
    friend R add(X x1, Y y2) { return add(R(x1),R(y2)); }
    friend R sub(X x1, Y y2) { return sub(R(x1),R(y2)); }
    friend R mul(X x1, Y y2) { return mul(R(x1),R(y2)); }
    friend R div(X x1, Y y2) { return div(R(x1),R(y2)); }
    friend R add(Y y1, X x2) { return add(R(y1),R(x2)); }
    friend R sub(Y y1, X x2) { return sub(R(y1),R(x2)); }
    friend R mul(Y y1, X x2) { return mul(R(y1),R(x2)); }
    friend R div(Y y1, X x2) { return div(R(y1),R(x2)); }
};

template<class X, class Y> struct ProvideConvertedFieldOperations<X,Y,X>
    : DefineInplaceFieldOperators<X,Y,X>
{
    typedef X R;
    friend R operator+(X x1, Y y2) { return operator+(x1,R(y2)); }
    friend R operator-(X x1, Y y2) { return operator-(x1,R(y2)); }
    friend R operator*(X x1, Y y2) { return operator*(x1,R(y2)); }
    friend R operator/(X x1, Y y2) { return operator/(x1,R(y2)); }
    friend R operator+(Y y1, X x2) { return operator+(R(y1),x2); }
    friend R operator-(Y y1, X x2) { return operator-(R(y1),x2); }
    friend R operator*(Y y1, X x2) { return operator*(R(y1),x2); }
    friend R operator/(Y y1, X x2) { return operator/(R(y1),x2); }
    friend R add(X x1, Y y2) { return add(x1,R(y2)); }
    friend R sub(X x1, Y y2) { return sub(x1,R(y2)); }
    friend R mul(X x1, Y y2) { return mul(x1,R(y2)); }
    friend R div(X x1, Y y2) { return div(x1,R(y2)); }
    friend R add(Y y1, X x2) { return add(R(y1),x2); }
    friend R sub(Y y1, X x2) { return sub(R(y1),x2); }
    friend R mul(Y y1, X x2) { return mul(R(y1),x2); }
    friend R div(Y y1, X x2) { return div(R(y1),x2); }
};

template<class X, class R> struct ProvideConvertedFieldOperations<X,X,R> {
    friend R operator+(X const& x1, X const& x2) { return operator+(R(x1),R(x2)); }
    friend R operator-(X const& x1, X const& x2) { return operator-(R(x1),R(x2)); }
    friend R operator*(X const& x1, X const& x2) { return operator*(R(x1),R(x2)); }
    friend R operator/(X const& x1, X const& x2) { return operator/(R(x1),R(x2)); }
    friend R add(X const& x1, X const& x2) { return add(R(x1),R(x2)); }
    friend R sub(X const& x1, X const& x2) { return sub(R(x1),R(x2)); }
    friend R mul(X const& x1, X const& x2) { return mul(R(x1),R(x2)); }
    friend R div(X const& x1, X const& x2) { return div(R(x1),R(x2)); }
};


template<class X, class NX, class Y, class NY, class R=X, class NR=NX> struct ProvideConcreteGenericDirectedGroupOperators
    : DefineInplaceDirectedGroupOperators<X,Y,NY,R>
{
    friend R operator+(const X& x1, const Y& y2) { return operator+(x1,x1.create(y2)); }
    friend R operator-(const X& x1, const NY& ny2) { return operator-(x1,create(ny2,x1)); }
    friend R operator+(const Y& y1, const X& x2) { return operator+(x2.create(y1),x2); }
    friend NR operator-(const NY& ny1, const X& x2) { return operator-(x2.create(ny1),x2); }
};
template<class X, class NX, class Y, class NY, class R=X, class NR=NX> struct ProvideConcreteGenericDirectedGroupOperations
    : ProvideConcreteGenericDirectedGroupOperators<X,NX,Y,NY,R,NR>
{
    friend R add(const X& x1, const Y& y2) { return add(x1,x1.create(y2)); }
    friend R sub(const X& x1, const NY& ny2) { return sub(x1,x1.create(ny2)); }
    friend R add(const Y& y1, const X& x2) { return add(x2.create(y1),x2); }
    friend NR sub(const NY& ny1, const X& x2) { return sub(x2.create(ny1),x2); }
};

template<class X, class QX, class Y, class QY, class R=X, class QR=QX> struct ProvideConcreteGenericDirectedSemiFieldOperators
    : DefineInplaceDirectedSemiFieldOperators<X,Y,QY,R>
{
    friend R operator+(const X& x1, const Y& y2) { return operator+(x1,x1.create(y2)); }
    friend R operator*(const X& x1, const Y& y2) { return operator*(x1,x1.create(y2)); }
    friend R operator/(const X& x1, const QY& qy2) { return operator/(x1,x1.create(qy2)); }
    friend R operator+(const Y& y1, const X& x2) { return operator+(x2.create(y1),x2); }
    friend R operator*(const Y& y1, const X& x2) { return operator*(x2.create(y1),x2); }
    friend QR operator/(const QY& y1, const X& x2) { return operator/(x2.create(y1),x2); }
};
template<class X, class QX, class Y, class QY, class R=X, class QR=QX> struct ProvideConcreteGenericDirectedSemiFieldOperations
    : ProvideConcreteGenericDirectedSemiFieldOperators<X,QX,Y,QY,R,QR>
{
    friend R add(const X& x1, const Y& y2) { return add(x1,x1.create(y2)); }
    friend R mul(const X& x1, const Y& y2) { return mul(x1,x1.create(y2)); }
    friend R div(const X& x1, const QY& y2) { return div(x1,x1.create(y2)); }
    friend R add(const Y& y1, const X& x2) { return add(x2.create(y1),x2); }
    friend R mul(const Y& y1, const X& x2) { return mul(x2.create(y1),x2); }
    friend QR div(const QY& y1, const X& x2) { return div(x2.create(y1),x2); }
};


template<class X, class Y, class R=X> struct ProvideConcreteGenericFieldOperators
    : DefineInplaceFieldOperators<X,Y,R>
{
    //static R create(Y const& y, X const& x) { return X(y,x.precision()); }
    friend decltype(auto) _create(Y const& y, X const& x) { return factory(x).create(y); }
    friend R operator+(const X& x1, const Y& y2) { return operator+(x1,_create(y2,x1)); }
    friend R operator-(const X& x1, const Y& y2) { return operator-(x1,_create(y2,x1)); }
    friend R operator*(const X& x1, const Y& y2) { return operator*(x1,_create(y2,x1)); }
    friend R operator/(const X& x1, const Y& y2) { return operator/(x1,_create(y2,x1)); }
    friend R operator+(const Y& y1, const X& x2) { return operator+(_create(y1,x2),x2); }
    friend R operator-(const Y& y1, const X& x2) { return operator-(_create(y1,x2),x2); }
    friend R operator*(const Y& y1, const X& x2) { return operator*(_create(y1,x2),x2); }
    friend R operator/(const Y& y1, const X& x2) { return operator/(_create(y1,x2),x2); }
};
template<class X, class Y, class R=X> struct ProvideConcreteGenericFieldOperations : ProvideConcreteGenericFieldOperators<X,Y,R> {
    friend R add(const X& x1, const Y& y2) { return add(x1,_create(y2,x1)); }
    friend R sub(const X& x1, const Y& y2) { return sub(x1,_create(y2,x1)); }
    friend R mul(const X& x1, const Y& y2) { return mul(x1,_create(y2,x1)); }
    friend R div(const X& x1, const Y& y2) { return div(x1,_create(y2,x1)); }
    friend R add(const Y& y1, const X& x2) { return add(_create(y1,x2),x2); }
    friend R sub(const Y& y1, const X& x2) { return sub(_create(y1,x2),x2); }
    friend R mul(const Y& y1, const X& x2) { return mul(_create(y1,x2),x2); }
    friend R div(const Y& y1, const X& x2) { return div(_create(y1,x2),x2); }
};

template<class X, class Y> struct ProvideConcreteGenericFieldOperators<X,Y,Void> {
    friend decltype(auto) operator+(X const& x, Y const& y) { return add(x,factory(x).create(y)); }
    friend decltype(auto) operator-(X const& x, Y const& y) { return sub(x,factory(x).create(y)); }
    friend decltype(auto) operator*(X const& x, Y const& y) { return mul(x,factory(x).create(y)); }
    friend decltype(auto) operator/(X const& x, Y const& y) { return div(x,factory(x).create(y)); }
    friend decltype(auto) operator+(Y const& y, X const& x) { return add(factory(x).create(y),x); }
    friend decltype(auto) operator-(Y const& y, X const& x) { return sub(factory(x).create(y),x); }
    friend decltype(auto) operator*(Y const& y, X const& x) { return mul(factory(x).create(y),x); }
    friend decltype(auto) operator/(Y const& y, X const& x) { return div(factory(x).create(y),x); }
    friend decltype(auto) operator+=(X& x, Y const& y) { return x=add(x,factory(x).create(y)); }
    friend decltype(auto) operator-=(X& x, Y const& y) { return x=sub(x,factory(x).create(y)); }
    friend decltype(auto) operator*=(X& x, Y const& y) { return x=mul(x,factory(x).create(y)); }
    friend decltype(auto) operator/=(X& x, Y const& y) { return x=div(x,factory(x).create(y)); }
};
template<class X, class Y> struct ProvideConcreteGenericFieldOperations<X,Y,Void> : ProvideConcreteGenericFieldOperators<X,Y,Void> {
    friend decltype(auto) add(X const& x, Y const& y) { return add(x,factory(x).create(y)); }
    friend decltype(auto) sub(X const& x, Y const& y) { return sub(x,factory(x).create(y)); }
    friend decltype(auto) mul(X const& x, Y const& y) { return mul(x,factory(x).create(y)); }
    friend decltype(auto) div(X const& x, Y const& y) { return div(x,factory(x).create(y)); }
    friend decltype(auto) add(Y const& y, X const& x) { return add(factory(x).create(y),x); }
    friend decltype(auto) sub(Y const& y, X const& x) { return sub(factory(x).create(y),x); }
    friend decltype(auto) mul(Y const& y, X const& x) { return mul(factory(x).create(y),x); }
    friend decltype(auto) div(Y const& y, X const& x) { return div(factory(x).create(y),x); }
};


template<class X> struct ProvideConcreteGenericFieldOperators<X,Void> {
    template<GenericNumber Y> friend decltype(auto) operator+(X const& x, Y const& y) { return add(x,factory(x).create(y)); }
    template<GenericNumber Y> friend decltype(auto) operator-(X const& x, Y const& y) { return sub(x,factory(x).create(y)); }
    template<GenericNumber Y> friend decltype(auto) operator*(X const& x, Y const& y) { return mul(x,factory(x).create(y)); }
    template<GenericNumber Y> friend decltype(auto) operator/(X const& x, Y const& y) { return div(x,factory(x).create(y)); }
    template<GenericNumber Y> friend decltype(auto) operator+(Y const& y, X const& x) { return add(factory(x).create(y),x); }
    template<GenericNumber Y> friend decltype(auto) operator-(Y const& y, X const& x) { return sub(factory(x).create(y),x); }
    template<GenericNumber Y> friend decltype(auto) operator*(Y const& y, X const& x) { return mul(factory(x).create(y),x); }
    template<GenericNumber Y> friend decltype(auto) operator/(Y const& y, X const& x) { return div(factory(x).create(y),x); }
    template<GenericNumber Y> friend decltype(auto) operator+=(X& x, Y const& y) { return x=add(x,factory(x).create(y)); }
    template<GenericNumber Y> friend decltype(auto) operator-=(X& x, Y const& y) { return x=sub(x,factory(x).create(y)); }
    template<GenericNumber Y> friend decltype(auto) operator*=(X& x, Y const& y) { return x=mul(x,factory(x).create(y)); }
    template<GenericNumber Y> friend decltype(auto) operator/=(X& x, Y const& y) { return x=div(x,factory(x).create(y)); }
};
template<class X> struct ProvideConcreteGenericFieldOperations<X,Void> : ProvideConcreteGenericFieldOperators<X,Void> {
    template<GenericNumber Y> friend decltype(auto) add(X const& x, Y const& y) { return add(x,factory(x).create(y)); }
    template<GenericNumber Y> friend decltype(auto) sub(X const& x, Y const& y) { return sub(x,factory(x).create(y)); }
    template<GenericNumber Y> friend decltype(auto) mul(X const& x, Y const& y) { return mul(x,factory(x).create(y)); }
    template<GenericNumber Y> friend decltype(auto) div(X const& x, Y const& y) { return div(x,factory(x).create(y)); }
    template<GenericNumber Y> friend decltype(auto) add(Y const& y, X const& x) { return add(factory(x).create(y),x); }
    template<GenericNumber Y> friend decltype(auto) sub(Y const& y, X const& x) { return sub(factory(x).create(y),x); }
    template<GenericNumber Y> friend decltype(auto) mul(Y const& y, X const& x) { return mul(factory(x).create(y),x); }
    template<GenericNumber Y> friend decltype(auto) div(Y const& y, X const& x) { return div(factory(x).create(y),x); }
};

template<class X, class Y=Void, class R=X> struct ProvideConcreteGenericArithmeticOperators : ProvideConcreteGenericFieldOperators<X,Y,R> { };
template<class X, class Y=Void, class R=X> struct ProvideConcreteGenericArithmeticOperations : ProvideConcreteGenericFieldOperations<X,Y,R> { };

template<class X, class Y=Void, class R=X> struct ProvideConcreteGenericLatticeOperations {
    friend R max(X const& x, Y const& y) { return max(x,factory(x).create(y)); }
    friend R min(X const& x, Y const& y) { return min(x,factory(x).create(y)); }
    friend R max(Y const& y, X const& x) { return max(factory(x).create(y),x); }
    friend R min(Y const& y, X const& x) { return min(factory(x).create(y),x); }
};

template<class X, class Y> struct ProvideConcreteGenericLatticeOperations<X,Y,Void> {
    friend decltype(auto) max(X const& x, Y const& y) { return max(x,factory(x).create(y)); }
    friend decltype(auto) min(X const& x, Y const& y) { return min(x,factory(x).create(y)); }
    friend decltype(auto) max(Y const& y, X const& x) { return max(factory(x).create(y),x); }
    friend decltype(auto) min(Y const& y, X const& x) { return min(factory(x).create(y),x); }
};

template<class X, class Y=Void, class R=X> struct ProvideConcreteGenericElementaryOperations
    : ProvideConcreteGenericArithmeticOperations<X,Y,R>, ProvideConcreteGenericLatticeOperations<X,Y,R> {
};


template<class X, class Y, class R, class LT, class EQ=LT> struct ProvideConvertedComparisonOperations : DefineMixedComparisonOperators<X,Y,LT,EQ> {
    typedef LT GT;
    friend LT lt(const X& x1, const Y& y2) { return lt(R(x1),R(y2)); }
    friend GT gt(const X& x1, const Y& y2) { return lt(R(y2),R(x1)); }
    friend EQ eq(const X& x1, const Y& y2) { return eq(R(x1),R(y2)); }
};

template<class X, class Y, class LT, class EQ=LT> struct ProvideConcreteGenericComparisonOperations : DefineMixedComparisonOperators<X,Y,LT,EQ> {
    typedef LT GT;
    friend LT lt(const X& x1, const Y& y2) { return lt(x1,factory(x1).create(y2)); }
    friend GT gt(const X& x1, const Y& y2) { return lt(factory(x1).create(y2),x1); }
    friend EQ eq(const X& x1, const Y& y2) { return eq(x1,factory(x1).create(y2)); }
};

template<class X, class NY, class LT, class EQ> struct ProvideConcreteGenericDirectedComparisonOperations : DefineMixedDirectedComparisonOperators<X,NY,LT,EQ> {
    typedef decltype(not declval<LT>()) GT;
    friend EQ eq(const X& x1, const NY& ny2) { return eq(x1,factory(x1).create(ny2)); }
    friend LT lt(const X& x1, const NY& ny2) { return lt(x1,factory(x1).create(ny2)); }
    friend GT gt(const X& x1, const NY& ny2) { return lt(factory(x1).create(ny2),x1); }
};

template<class X, class Y, class R, class LT, class EQ=LT> struct ProvideConcreteGenericOperations
    : ProvideConcreteGenericFieldOperations<X,Y,R>, ProvideConcreteGenericComparisonOperations<X,Y,LT,EQ> { };

template<class X, class NX, class Y, class NY, class LT, class EQ> struct ProvideConcreteGenericDirectedOperations
    : ProvideConcreteGenericDirectedGroupOperations<X,NX,Y,NY>, ProvideConcreteGenericDirectedComparisonOperations<X,NY,LT,EQ> { };


template<class X, class QX=X, class R=X, class QR=QX> class ProvideDirectedSemiFieldOperators
{
    friend R operator+(X const& x1, X const& x2) { return add(x1,x2); }
    friend R operator*(X const& x1, X const& x2) { return mul(x1,x2); }
    friend R operator/(X const& x1, QX const& x2) { return div(x1,x2); }
    friend X& operator+=(X& x1, X const& x2) { return x1=add(x1,x2); }
    friend X& operator*=(X& x1, X const& x2) { return x1=mul(x1,x2); }
    friend X& operator/=(X& x1, QX const& x2) { return x1=div(x1,x2); }

    // BUG: Cannot declare these due to bug in Clang
    // friend R rec(QX const&);
    // friend QR rec(X const&);
    // friend QR operator/(QX const&, X const&);
};


template<class X> struct DefineConcreteGenericArithmeticOperations {
    template<GenericNumber Y> friend decltype(auto) add(X const& x, Y const& y) { return add(x,factory(x).create(y)); }
    template<GenericNumber Y> friend decltype(auto) sub(X const& x, Y const& y) { return sub(x,factory(x).create(y)); }
    template<GenericNumber Y> friend decltype(auto) mul(X const& x, Y const& y) { return mul(x,factory(x).create(y)); }
    template<GenericNumber Y> friend decltype(auto) div(X const& x, Y const& y) { return div(x,factory(x).create(y)); }

    template<GenericNumber Y> friend decltype(auto) add(Y const& y, X const& x) { return add(factory(x).create(y),x); }
    template<GenericNumber Y> friend decltype(auto) sub(Y const& y, X const& x) { return sub(factory(x).create(y),x); }
    template<GenericNumber Y> friend decltype(auto) mul(Y const& y, X const& x) { return mul(factory(x).create(y),x); }
    template<GenericNumber Y> friend decltype(auto) div(Y const& y, X const& x) { return div(factory(x).create(y),x); }

    template<GenericNumber Y> friend decltype(auto) max(X const& x, Y const& y) { return max(x,factory(x).create(y)); }
    template<GenericNumber Y> friend decltype(auto) min(X const& x, Y const& y) { return min(x,factory(x).create(y)); }
    template<GenericNumber Y> friend decltype(auto) max(Y const& y, X const& x) { return max(factory(x).create(y),x); }
    template<GenericNumber Y> friend decltype(auto) min(Y const& y, X const& x) { return min(factory(x).create(y),x); }
};


template<class X> struct DefineConcreteGenericArithmeticOperators
    : DefineConcreteGenericArithmeticOperations<X>
{
    template<GenericNumber Y> friend decltype(auto) operator+(X const& x, Y const& y) { return x+factory(x).create(y); }
    template<GenericNumber Y> friend decltype(auto) operator-(X const& x, Y const& y) { return x-factory(x).create(y); }
    template<GenericNumber Y> friend decltype(auto) operator*(X const& x, Y const& y) { return x*factory(x).create(y); }
    template<GenericNumber Y> friend decltype(auto) operator/(X const& x, Y const& y) { return x/factory(x).create(y); }

    template<GenericNumber Y> friend decltype(auto) operator+(Y const& y, X const& x) { return factory(x).create(y)+x; }
    template<GenericNumber Y> friend decltype(auto) operator-(Y const& y, X const& x) { return factory(x).create(y)-x; }
    template<GenericNumber Y> friend decltype(auto) operator*(Y const& y, X const& x) { return factory(x).create(y)*x; }
    template<GenericNumber Y> friend decltype(auto) operator/(Y const& y, X const& x) { return factory(x).create(y)/x; }

    template<GenericNumber Y> friend decltype(auto) operator+=(X& x, Y const& y) { return x+=factory(x).create(y); }
    template<GenericNumber Y> friend decltype(auto) operator-=(X& x, Y const& y) { return x-=factory(x).create(y); }
    template<GenericNumber Y> friend decltype(auto) operator*=(X& x, Y const& y) { return x*=factory(x).create(y); }
    template<GenericNumber Y> friend decltype(auto) operator/=(X& x, Y const& y) { return x/=factory(x).create(y); }
};

template<class X> struct DefineConcreteGenericComparisonOperators {
    template<GenericNumber Y> friend decltype(auto) operator==(X const& x, Y const& y) { return x==factory(x).create(y); }
    template<GenericNumber Y> friend decltype(auto) operator!=(X const& x, Y const& y) { return x!=factory(x).create(y); }
    template<GenericNumber Y> friend decltype(auto) operator<=(X const& x, Y const& y) { return x<=factory(x).create(y); }
    template<GenericNumber Y> friend decltype(auto) operator>=(X const& x, Y const& y) { return x>=factory(x).create(y); }
    template<GenericNumber Y> friend decltype(auto) operator< (X const& x, Y const& y) { return x< factory(x).create(y); }
    template<GenericNumber Y> friend decltype(auto) operator> (X const& x, Y const& y) { return x> factory(x).create(y); }

    template<GenericNumber Y> friend decltype(auto) operator==(Y const& y, X const& x) { return factory(x).create(y)==x; }
    template<GenericNumber Y> friend decltype(auto) operator!=(Y const& y, X const& x) { return factory(x).create(y)!=x; }
    template<GenericNumber Y> friend decltype(auto) operator<=(Y const& y, X const& x) { return factory(x).create(y)<=x; }
    template<GenericNumber Y> friend decltype(auto) operator>=(Y const& y, X const& x) { return factory(x).create(y)>=x; }
    template<GenericNumber Y> friend decltype(auto) operator< (Y const& y, X const& x) { return factory(x).create(y)< x; }
    template<GenericNumber Y> friend decltype(auto) operator> (Y const& y, X const& x) { return factory(x).create(y)> x; }
};

template<class X> struct DefineConcreteGenericOperators
    : DefineConcreteGenericArithmeticOperators<X>, DefineConcreteGenericComparisonOperators<X> { };



template<class T> struct NumericTraits;

template<class X> class Operations {
    typedef decltype(add(declval<X>(),declval<X>())) R;
    typedef decltype(neg(declval<X>())) NX;
    typedef typename NumericTraits<X>::OppositeType QX;
    typedef decltype(rec(declval<X>())) QR;
    typedef R PR;
    typedef decltype(abs(declval<X>())) PX;
    typedef decltype(eq(declval<X>(),declval<NX>())) EQ;
    typedef decltype(lt(declval<X>(),declval<NX>())) LT;
  public:
    static X _nul(X const& x);
    static X _pos(X const& x);
    static NX _neg(X const& x);
    static X _hlf(X const& x);
    static PR _sqr(X const& x);
    static QR _rec(X const& x);
    static R _add(X const& x1, X const& x2);
    static R _sub(X const& x1, NX const& x2);
    static R _mul(X const& x1, X const& x2);
    static R _div(X const& x1, QX const& x2);
    static R _pow(X const& x, Nat m);
    static R _pow(X const& x, Int n);
    static R _sqrt(X const& x);
    static R _exp(X const& x);
    static R _log(X const& x);
    static R _sin(X const& x);
    static R _cos(X const& x);
    static R _tan(X const& x);
    static R _asin(X const& x);
    static R _acos(X const& x);
    static R _atan(X const& x);
    static X _max(X const& x1, X const& x2);
    static X _min(X const& x1, X const& x2);
    static PX _abs(X const& x);
    static EQ _eq(X const& x1, NX const& x2);
    static LT _lt(X const& x1, NX const& x2);
    static OutputStream& _write(OutputStream& os, X const& x);
    static InputStream& _read(InputStream& is, X& x);
};

template<class X, class R=X> struct DispatchNumericOperations
    : DefineFieldOperators<X,R>
{
    typedef Operations<X> OperationsType;
    friend X nul(X const& x) { return OperationsType::_nul(x); }
    friend X pos(X const& x) { return OperationsType::_pos(x); }
    friend X neg(X const& x) { return OperationsType::_neg(x); }
    friend X hlf(X const& x) { return OperationsType::_hlf(x); }
    friend R sqr(X const& x) { return OperationsType::_sqr(x); }
    friend R rec(X const& x) { return OperationsType::_rec(x); }
    friend R add(X const& x1, X const& x2) { return OperationsType::_add(x1,x2); }
    friend R sub(X const& x1, X const& x2) { return OperationsType::_sub(x1,x2); }
    friend R mul(X const& x1, X const& x2) { return OperationsType::_mul(x1,x2); }
    friend R div(X const& x1, X const& x2) { return OperationsType::_div(x1,x2); }
    friend R pow(X const& x, Nat m) { return OperationsType::_pow(x,m); }
    friend R pow(X const& x, Int n) { return OperationsType::_pow(x,n); }
    friend R sqrt(X const& x) { return OperationsType::_sqrt(x); }
    friend R exp(X const& x) { return OperationsType::_exp(x); }
    friend R log(X const& x) { return OperationsType::_log(x); }
    friend R sin(X const& x) { return OperationsType::_sin(x); }
    friend R cos(X const& x) { return OperationsType::_cos(x); }
    friend R tan(X const& x) { return OperationsType::_tan(x); }
    friend R asin(X const& x) { return OperationsType::_asin(x); }
    friend R acos(X const& x) { return OperationsType::_acos(x); }
    friend R atan(X const& x) { return OperationsType::_atan(x); }
    friend X max(X const& x1, X const& x2) { return OperationsType::_max(x1,x2); }
    friend X min(X const& x1, X const& x2) { return OperationsType::_min(x1,x2); }
    friend X abs(X const& x) { return OperationsType::_abs(x); }
};

template<class X, class NX, class R=X, class NR=NX> struct DispatchDirectedNumericOperations
    : DefineDirectedGroupOperators<X,NX>
{
    typedef Operations<X> OperationsType;
    friend X nul(X const& x) { return OperationsType::_nul(x); }
    friend X pos(X const& x) { return OperationsType::_pos(x); }
    friend NX neg(X const& x) { return OperationsType::_neg(x); }
    friend X hlf(X const& x) { return OperationsType::_hlf(x); }
    friend R add(X const& x1, X const& x2) { return OperationsType::_add(x1,x2); }
    friend R sub(X const& x1, NX const& x2) { return OperationsType::_sub(x1,x2); }
    friend R sqrt(X const& x) { return OperationsType::_sqrt(x); }
    friend R exp(X const& x) { return OperationsType::_exp(x); }
    friend R log(X const& x) { return OperationsType::_log(x); }
    friend R atan(X const& x) { return OperationsType::_atan(x); }
    friend X max(X const& x1, X const& x2) { return OperationsType::_max(x1,x2); }
    friend X min(X const& x1, X const& x2) { return OperationsType::_min(x1,x2); }
};

template<class X, class QX, class R=X, class QR=QX> struct DispatchPositiveDirectedNumericOperations
    : ProvideDirectedSemiFieldOperators<X,QX,R,QR>
{
    typedef Operations<X> OperationsType;
    friend X nul(X const& x) { return OperationsType::_nul(x); }
    friend X hlf(X const& x) { return OperationsType::_hlf(x); }
    friend R sqr(X const& x) { return OperationsType::_sqr(x); }
    friend QR rec(X const& x) { return OperationsType::_rec(x); }
    friend R add(X const& x1, X const& x2) { return OperationsType::_add(x1,x2); }
    friend R mul(X const& x1, X const& x2) { return OperationsType::_mul(x1,x2); }
    friend R div(X const& x1, QX const& x2) { return OperationsType::_div(x1,x2); }
    friend R pow(X const& x, Nat m) { return OperationsType::_pow(x,m); }
    friend R sqrt(X const& x) { return OperationsType::_sqrt(x); }
    friend R atan(X const& x) { return OperationsType::_atan(x); }
    friend X max(X const& x1, X const& x2) { return OperationsType::_max(x1,x2); }
    friend X min(X const& x1, X const& x2) { return OperationsType::_min(x1,x2); }
    friend X abs(X const& x) { return OperationsType::_abs(x); }
};

template<class X, class R=X> struct DispatchPositiveNumericOperations
    : DispatchPositiveDirectedNumericOperations<X,X,R,R>
{
};

template<class X, class LT, class EQ=LT> struct DispatchComparisonOperations
    : DefineComparisonOperators<X,LT,EQ>
{
    typedef Operations<X> OperationsType;
    friend EQ eq(X const& x1, X const& x2) { return OperationsType::_eq(x1,x2); }
    friend LT lt(X const& x1, X const& x2) { return OperationsType::_lt(x1,x2); }
};

template<class X, class NX, class LT, class EQ> struct DispatchDirectedComparisonOperations
    : DefineDirectedComparisonOperators<X,NX,LT,EQ>
{
    typedef Operations<X> OperationsType;
    friend EQ eq(X const& x1, NX const& nx2) { return OperationsType::_eq(x1,nx2); }
    friend LT lt(X const& x1, NX const& nx2) { return OperationsType::_lt(x1,nx2); }
};



} // namespace Ariadne

#endif
