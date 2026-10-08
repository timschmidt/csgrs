/***************************************************************************
 *            algebra/operations.hpp
 *
 *  Copyright  2010-20  Pieter Collins
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

/*! \file algebra/operations.hpp
 *  \brief Provide common algebra operations automatically.
 */

#ifndef ARIADNE_ALGEBRA_OPERATIONS_HPP
#define ARIADNE_ALGEBRA_OPERATIONS_HPP

#include "numeric/operators.hpp"
#include "numeric/arithmetic.hpp"

namespace Ariadne {

template<class A, class X=typename A::NumericType> struct AlgebraOperations;

template<class A> class GradedAlgebraOperations {
    typedef typename A::NumericType X;
  private:
    static A _compose(const Series<X>& f, const A& a) { return A::_compose(f,a); }
  public:
    static A apply(Hlf op, A const& a) {
        return a/2; }
    static A apply(Sqr op, A const& a) {
        return a*a; }
    static A apply(Div op, A const& a1, A const& a2) {
        return mul(a1,rec(a2)); }
    static A apply(Pow op, A const& a, Nat m) {
        A s=a; A r=s.create_constant(1); while(m) { if(m%2) { r*=s; } s=sqr(s); m/=2; } return r; }
    static A apply(Pow op, A const& a, Int n) {
        if(n>=0) { return pow(a,Nat(n)); } else { return rec(pow(a,Nat(-n))); } }

    template<class OP> static A apply(OP op, const A& a) { return _compose(Series<X>(op,a.value()),a); }
};

template<class A> class NormedAlgebraOperations {
    typedef typename A::NumericType X;
  public:
    static A apply(Hlf, A const& a) {
        return a/2; }
    static A apply(Sqr, A const& a) {
        return a*a; }
    static A apply(Pow, A const& a, Nat m) {
        A s=a; A r=s.create_constant(1); while(m) { if(m%2) { r*=s; } s=sqr(s); m/=2; } return r; }
    static A apply(Pow, A const& a, Int n) {
        if(n>=0) { return pow(a,Nat(n)); } else { return rec(pow(a,Nat(-n))); } }
    static A apply(Div, A const& a1, A const& a2) {
        return mul(a1,rec(a2)); }

    static A apply(Rec, const A& a);
    static A apply(Sqrt, const A& a);
    static A apply(Exp, const A& a);
    static A apply(Log, const A& a);
    static A apply(Sin, const A& a);
    static A apply(Cos, const A& a);
    static A apply(Tan, const A& a);
    static A apply(Asin, const A& a);
    static A apply(Acos, const A& a);
    static A apply(Atan, const A& a);
};

template<class V, class X> struct DeclareVectorOperators
{
    friend V operator+(V const& v);
    friend V operator-(V const& v);
    friend V operator+(V const& v1, V const& v2);
    friend V operator-(V const& v1, V const& v2);
    friend V operator*(V const& v1, X const& x2);
    friend V operator/(V const& v1, X const& x2);
    friend V operator*(X const& x1, V const& v2);
    friend V& operator+=(V& v1, V const& v2);
    friend V& operator-=(V& v1, V const& v2);
    friend V& operator*=(V& v1, X const& x2);
    friend V& operator/=(V& v1, X const& x2);
};

template<class VA, class A, class VX, class X> struct DeclareVectorAlgebraOperators
    : DeclareVectorOperators<VA,A>
{
    friend VA operator+(VA const& va1, VX const& vx2);
    friend VA operator+(VX const& vx1, VA const& va2);
    friend VA operator-(VA const& va1, VX const& vx2);
    friend VA operator-(VX const& vx1, VA const& va2);
    friend VA operator*(VA const& va1, X const& x2);
    friend VA operator/(VA const& va1, X const& x2);
    friend VA operator*(X const& x1, VA const& va2);
    friend VA& operator+=(VA& va1, VX const& vx2);
    friend VA& operator-=(VA& va1, VX const& vx2);
    friend VA& operator*=(VA& va1, X const& x2);
    friend VA& operator/=(VA& va1, X const& x2);
};

template<class A, class X> struct DeclareAlgebraOperators
    : DeclareVectorOperators<A,X>
{
    friend A operator*(A const& a1, A const& a2);
    friend A& operator*=(A& a1, A const& a2);
    friend A operator+(A const& a1, X const& x2);
    friend A operator-(A const& a1, X const& x2);
    friend A operator+(X const& x1, A const& a2);
    friend A operator-(X const& x1, A const& a2);
    friend A& operator+=(A& a1, X const& x2);
    friend A& operator-=(A& a1, X const& x2);
};

template<class A, class X> struct DeclareAlgebraOperations
    : DeclareAlgebraOperators<A,X>
{
    friend A nul(A const& a);
    friend A pos(A const& a);
    friend A neg(A const& a);
    friend A hlf(A const& a);
    friend A sqr(A const& a);
    friend A add(A const& a1, A const& a2);
    friend A sub(A const& a1, A const& a2);
    friend A mul(A const& a1, A const& a2);

    friend A add(A const& a1, X const& x2);
    friend A sub(A const& a1, X const& x2);
    friend A mul(A const& a1, X const& x2);
    friend A div(A const& a1, X const& x2);
    friend A add(X const& x1, A const& a2);
    friend A sub(X const& x1, A const& a2);
    friend A mul(X const& x1, A const& a2);
    friend A pow(A const& a, Nat m);
    friend A pow(A const& a, Int n);
};

template<class A, class X> struct DeclareDivisionAlgebraOperations : DeclareAlgebraOperations<A,X> {
    friend A operator/(A const& a1, A const& a2);
    friend A& operator/=(A& a1, A const& a2);
    friend A operator/(X const& x1, A const& a2);

    friend A rec(A const& a);
    friend A div(A const& a1, A const& a2);
    friend A div(X const& x1, A const& a2);
    friend A pow(A const& a, Int n);
};

template<class A, class X> struct DeclareTranscendentalAlgebraOperations : DeclareDivisionAlgebraOperations<A,X> {
    friend A sqrt(A const& a);
    friend A exp(A const& a);
    friend A log(A const& a);
    friend A sin(A const& a);
    friend A cos(A const& a);
    friend A tan(A const& a);
    friend A asin(A const& a);
    friend A acos(A const& a);
    friend A atan(A const& a);
};

template<class A, class X> struct DeclareLatticeAlgebraOperations {
    friend A max(A const& a1, A const& a2);
    friend A min(A const& a1, A const& a2);
    friend A abs(A const& a);
};

template<class A, class X> struct DeclareElementaryAlgebraOperations
    : DeclareTranscendentalAlgebraOperations<A,X>, DeclareLatticeAlgebraOperations<A,X> {
};



template<class A, class Y> struct DispatchMixedAlgebraNumberOperations;

template<class A, class X, class Y> struct DispatchConcreteGenericAlgebraNumberOperators {
    friend X create(Y const& y, A const& a) { return X(y,a.precision()); }
    friend A& operator+=(A& a1, Y const& y2) { X x2=create(y2,a1); return a1+=x2; }
    friend A& operator-=(A& a1, Y const& y2) { X x2=create(y2,a1); return a1-=x2; }
    friend A& operator*=(A& a1, Y const& y2) { X x2=create(y2,a1); return a1*=x2; }
    friend A& operator/=(A& a1, Y const& y2) { X x2=create(y2,a1); return a1/=x2; }
    friend A operator+(A a1, Y const& y2) { a1+=y2; return a1; }
    friend A operator-(A a1, Y const& y2) { a1-=y2; return a1; }
    friend A operator*(A a1, Y const& y2) { a1*=y2; return a1; }
    friend A operator/(A a1, Y const& y2) { a1/=y2; return a1; }
    friend A operator+(Y const& y1, A a2) { return a2+y1; }
    friend A operator-(Y const& y1, A a2) { return (-a2)+y1; }
    friend A operator*(Y const& y1, A a2) { return a2*y1; }
    friend A operator/(Y const& y1, A a2) { return rec(a2)*y1; }
};

template<class A, class X, class Y> struct DispatchConcreteGenericAlgebraNumberOperations : DispatchConcreteGenericAlgebraNumberOperators<A,X,Y> {
    friend A add(A a1, Y const& y2) { a1+=y2; return a1; }
    friend A sub(A a1, Y const& y2) { a1-=y2; return a1; }
    friend A mul(A a1, Y const& y2) { a1*=y2; return a1; }
    friend A div(A a1, Y const& y2) { a1/=y2; return a1; }
    friend A add(Y const& y1, A a2) { return a2+y1; }
    friend A sub(Y const& y1, A a2) { return (-a2)+y1; }
    friend A mul(Y const& y1, A a2) { return a2*y1; }
    friend A div(Y const& y1, A a2) { return rec(a2)*y1; }
};


template<class A, class X> struct DispatchAlgebraOperators
{
    typedef AlgebraOperations<A,X> OperationsType;
   public:
    friend A operator+(A const& a) { return pos(a); }
    friend A operator-(A const& a) { return neg(a); }
    friend A operator+(A const& a1, A const& a2) { return add(a1,a2); }
    friend A operator-(A const& a1, A const& a2) { return sub(a1,a2); }
    friend A operator*(A const& a1, A const& a2) { return mul(a1,a2); }
    friend A& operator+=(A& a1, A const& a2) { return a1=add(a1,a2); }
    friend A& operator-=(A& a1, A const& a2) { return a1=sub(a1,a2); }
    friend A& operator*=(A& a1, A const& a2) { return a1=mul(a1,a2); }
    friend A operator+(A const& a1, X const& x2) { return add(a1,x2); }
    friend A operator-(A const& a1, X const& x2) { return sub(a1,x2); }
    friend A operator*(A const& a1, X const& x2) { return mul(a1,x2); }
    friend A operator/(A const& a1, X const& x2) { return div(a1,x2); }
    friend A& operator+=(A& a1, X const& x2) { return a1=add(a1,x2); }
    friend A& operator-=(A& a1, X const& x2) { return a1=sub(a1,x2); }
    friend A& operator*=(A& a1, X const& x2) { return a1=mul(a1,x2); }
    friend A& operator/=(A& a1, X const& x2) { return a1=div(a1,x2); }
    friend A operator+(X const& x1, A const& a2) { return add(x1,a2); }
    friend A operator-(X const& x1, A const& a2) { return sub(x1,a2); }
    friend A operator*(X const& x1, A const& a2) { return mul(x1,a2); }
};

template<class A, class X> struct DispatchAlgebraOperations
    : DispatchAlgebraOperators<A,X>
{
    typedef AlgebraOperations<A,X> OperationsType;
   public:
    friend A nul(A const& a) { return OperationsType::apply(Nul(),a); }
    friend A pos(A const& a) { return OperationsType::apply(Pos(),a); }
    friend A neg(A const& a) { return OperationsType::apply(Neg(),a); }
    friend A hlf(A const& a) { return OperationsType::apply(Hlf(),a); }
    friend A sqr(A const& a) { return OperationsType::apply(Sqr(),a); }
    friend A add(A const& a1, A const& a2) { return OperationsType::apply(Add(),a1, a2); }
    friend A sub(A const& a1, A const& a2) { return OperationsType::apply(Sub(),a1, a2); }
    friend A mul(A const& a1, A const& a2) { return OperationsType::apply(Mul(),a1, a2); }

    friend A add(A const& a1, X const& x2) { return OperationsType::apply(Add(),a1,x2); }
    friend A sub(A const& a1, X const& x2) { return OperationsType::apply(Add(),a1,neg(x2)); }
    friend A mul(A const& a1, X const& x2) { return OperationsType::apply(Mul(),a1,x2); }
    friend A div(A const& a1, X const& x2) { return OperationsType::apply(Mul(),a1,rec(x2)); }
    friend A add(X const& x1, A const& a2) { return OperationsType::apply(Add(),a2,x1); }
    friend A sub(X const& x1, A const& a2) { return OperationsType::apply(Add(),neg(a2),x1); }
    friend A mul(X const& x1, A const& a2) { return OperationsType::apply(Mul(),a2,x1); }
    friend A pow(A const& a, Nat m) { return GradedAlgebraOperations<A>::apply(Pow(),a, m); }
};

template<class A, class X> struct DispatchTranscendentalAlgebraOperations : DispatchAlgebraOperations<A,X> {
    typedef AlgebraOperations<A,X> OperationsType;
  public:
    // BUG: Cannot declare these due to bug in Clang
    // friend A operator/(A const& a1, X const& x2);
    // friend A div(A const& a1, X const& x2);
    // friend A pow(A const& a, Nat m);

    friend A operator/(A const& a1, A const& a2) { return div(a1,a2); }
    friend A& operator/=(A& a1, A const& a2) { return a1=div(a1,a2); }
    friend A operator/(X const& x1, A const& a2) { return div(x1,a2); }
    friend A div(A const& a1, A const& a2) { return OperationsType::apply(Div(),a1, a2); }

    friend A rec(A const& a) { return OperationsType::apply(Rec(),a); }
    friend A div(X const& x1, A const& a2) { return x1*rec(a2); }
    friend A pow(A const& a, Int n) { return OperationsType::apply(Pow(),a, n); }

    friend A sqrt(A const& a) { return OperationsType::apply(Sqrt(),a); }
    friend A exp(A const& a) { return OperationsType::apply(Exp(),a); }
    friend A log(A const& a) { return OperationsType::apply(Log(),a); }
    friend A sin(A const& a) { return OperationsType::apply(Sin(),a); }
    friend A cos(A const& a) { return OperationsType::apply(Cos(),a); }
    friend A tan(A const& a) { return OperationsType::apply(Tan(),a); }
    friend A asin(A const& a) { return OperationsType::apply(Asin(),a); }
    friend A acos(A const& a) { return OperationsType::apply(Acos(),a); }
    friend A atan(A const& a) { return OperationsType::apply(Atan(),a); }
};

template<class A, class X> struct DispatchLatticeAlgebraOperations {
    typedef AlgebraOperations<A,X> OperationsType;
    friend A max(A const& a1, A const& a2) { return OperationsType::apply(Max(),a1,a2); }
    friend A min(A const& a1, A const& a2) { return OperationsType::apply(Min(),a1,a2); }
    friend A abs(A const& a) { return OperationsType::apply(Abs(),a); }

    friend A max(A const& a1, X const& x2) { return OperationsType::apply(Max(),a1,x2); }
    friend A min(A const& a1, X const& x2) { return OperationsType::apply(Min(),a1,x2); }
    friend A max(X const& x1, A const& a2) { return OperationsType::apply(Max(),x1,a2); }
    friend A min(X const& x1, A const& a2) { return OperationsType::apply(Min(),x1,a2); }

};

template<class A, class X> struct DispatchElementaryAlgebraOperations
    : DispatchTranscendentalAlgebraOperations<A,X>, DispatchLatticeAlgebraOperations<A,X> {
};

struct Factorial {
    Nat _n;
    Factorial(Nat n) : _n(n) { }
    operator FloatDPBounds() { FloatDPBounds r(1,dp); for(Nat i=1; i<=_n; ++i) { r*=i; } return r; }
    friend FloatDPBounds rec(Factorial x) { return rec(FloatDPBounds(x)); }
};


} // namespace Ariadne

#endif /* ARIADNE_ALGEBRA_OPERATIONS_HPP */
