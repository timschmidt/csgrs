/***************************************************************************
 *            algebra/algebra_concepts.hpp
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

/*! \file algebra/algebra_concepts.hpp
 *  \brief Concepts for algebraic structures.
 */

#ifndef ARIADNE_ALGEBRA_CONCEPTS_HPP
#define ARIADNE_ALGEBRA_CONCEPTS_HPP

#include "utility/metaprogramming.hpp"
#include "numeric/concepts.hpp"

namespace Ariadne {

//! \brief Concept for a vector over a ring or field \a X.
template<class V, class X> concept AVectorOver = Ring<X> and requires (V const& v, X x, typename V::IndexType i) {
  { declval<V>()[declval<typename V::IndexType>] = declval<X>() };
  { v[i] } -> ConvertibleTo<X>;
  { +v } -> ConvertibleTo<V>;
  { -v } -> ConvertibleTo<V>;
  { v+v } -> ConvertibleTo<V>;
  { v-v } -> ConvertibleTo<V>;
  { x*v } -> ConvertibleTo<V>;
  { v*x } -> ConvertibleTo<V>;
  { v/x } -> ConvertibleTo<V>;
};

//! \brief Concept for a unital algebra over a field \a X.
template<class A, class X> concept AnAlgebraOver = Field<X> and Assignable<A,X> and requires (A const& a, X const& x, Nat m) {
    { operator+(a) } -> SameAsOrConvertibleTo<A>;
    { operator-(a) } -> SameAsOrConvertibleTo<A>;
    { operator+(a,a) } -> SameAsOrConvertibleTo<A>;
    { operator-(a,a) } -> SameAsOrConvertibleTo<A>;
    { operator*(a,a) } -> SameAsOrConvertibleTo<A>;
    { operator+(x,a) } -> SameAsOrConvertibleTo<A>;
    { operator-(x,a) } -> SameAsOrConvertibleTo<A>;
    { operator*(x,a) } -> SameAsOrConvertibleTo<A>;
    { operator+(a,x) } -> SameAsOrConvertibleTo<A>;
    { operator-(a,x) } -> SameAsOrConvertibleTo<A>;
    { operator*(a,x) } -> SameAsOrConvertibleTo<A>;
    { operator/(a,x) } -> SameAsOrConvertibleTo<A>;
    { operator+=(declval<A&>(),a) } -> SameAs<A&>;
    { operator-=(declval<A&>(),a) } -> SameAs<A&>;
    { operator+=(declval<A&>(),x) } -> SameAs<A&>;
    { operator-=(declval<A&>(),x) } -> SameAs<A&>;
    { operator*=(declval<A&>(),x) } -> SameAs<A&>;
    { operator/=(declval<A&>(),x) } -> SameAs<A&>;

    { nul(a) } -> SameAsOrConvertibleTo<A>;
    { pos(a) } -> SameAsOrConvertibleTo<A>;
    { neg(a) } -> SameAsOrConvertibleTo<A>;
    { sqr(a) } -> SameAsOrConvertibleTo<A>;
    { hlf(a) } -> SameAsOrConvertibleTo<A>;
    { add(a,a) } -> SameAsOrConvertibleTo<A>;
    { sub(a,a) } -> SameAsOrConvertibleTo<A>;
    { mul(a,a) } -> SameAsOrConvertibleTo<A>;
    { add(a,x) } -> SameAsOrConvertibleTo<A>;
    { mul(a,x) } -> SameAsOrConvertibleTo<A>;
    { pow(a,m) } -> SameAsOrConvertibleTo<A>;
};

template<class A, class X> concept ATranscendentalAlgebraOver = AnAlgebraOver<A,X> and Transcendental<A>;

template<class A, class X> concept AnElementaryAlgebraOver = AnAlgebraOver<A,X> and Transcendental<A> and requires(A a, X x) {
    { max(a,a) } -> SameAsOrConvertibleTo<A>;
    { min(a,a) } -> SameAsOrConvertibleTo<A>;
    { max(a,x) } -> SameAsOrConvertibleTo<A>;
    { min(a,x) } -> SameAsOrConvertibleTo<A>;
    { max(x,a) } -> SameAsOrConvertibleTo<A>;
    { min(x,a) } -> SameAsOrConvertibleTo<A>;
};

//! \brief Concept for a unital algebra over a field \a X supporting inplace operations.
template<class A, class X> concept AnInplaceAlgebraOver = Field<X> and requires (A& r, X const& c, A const& a) {
  { r.iadd(c) };
  { r.imul(c) };
  { r.isma(c,a) };
  { r.ifma(a,a) };
};

//! \brief Concept for a unital algebra.
template<class A> concept AnAlgebra = requires { typename A::NumericType; } and AnAlgebraOver<A,typename A::NumericType>;


template<class X> class AlgebraArchetype {
    using ALG = AlgebraArchetype<X>;
  public:
    typedef X NumericType;
    ALG& operator=(X const&);
    friend ALG operator+(ALG const&);
    friend ALG operator-(ALG const&);
    friend ALG operator+(ALG const&, ALG const&);
    friend ALG operator-(ALG const&, ALG const&);
    friend ALG operator*(ALG const&, ALG const&);
    friend ALG operator+(X const&, ALG const&);
    friend ALG operator-(X const&, ALG const&);
    friend ALG operator*(X const&, ALG const&);
    friend ALG operator+(ALG const&, X const&);
    friend ALG operator-(ALG const&, X const&);
    friend ALG operator*(ALG const&, X const&);
    friend ALG operator/(ALG const&, X const&);

    friend ALG& operator+=(ALG&, ALG const&);
    friend ALG& operator-=(ALG&, ALG const&);
    friend ALG& operator+=(ALG&, X const&);
    friend ALG& operator-=(ALG&, X const&);
    friend ALG& operator*=(ALG&, X const&);
    friend ALG& operator/=(ALG&, X const&);

    friend ALG nul(ALG const&);
    friend ALG pos(ALG const&);
    friend ALG neg(ALG const&);
    friend ALG sqr(ALG const&);
    friend ALG hlf(ALG const&);
    friend ALG add(ALG const&, ALG const&);
    friend ALG sub(ALG const&, ALG const&);
    friend ALG mul(ALG const&, ALG const&);
    friend ALG add(X const&, ALG const&);
    friend ALG sub(X const&, ALG const&);
    friend ALG mul(X const&, ALG const&);
    friend ALG add(ALG const&, X const&);
    friend ALG sub(ALG const&, X const&);
    friend ALG mul(ALG const&, X const&);
    friend ALG div(ALG const&, X const&);
    friend ALG pow(ALG const&, Nat const&);

    friend OutputStream& operator<<(OutputStream& os, AlgebraArchetype<X> const&);
};

template<class X> class InplaceAlgebraArchetype {
    using ALG = InplaceAlgebraArchetype<X>;
  public:
    typedef X NumericType;
    ALG& operator=(X const&);
    ALG create() const;
    ALG create_constant(X const&) const;
    ALG& iadd(X const&);
    ALG& imul(X const&);
    ALG& isma(X const&, ALG const&);
    ALG& ifma(ALG const&, ALG const&);
    friend OutputStream& operator<<(OutputStream& os, ALG const& a);
};


} // namespace Ariadne

#endif /* ARIADNE_ALGEBRA_CONCEPTS_HPP */
