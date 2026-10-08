/***************************************************************************
 *            numeric/concepts.hpp
 *
 *  Copyright  2020  Pieter Collins
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

/*! \file numeric/concepts.hpp
 *  \brief
 */

#pragma once

#include "../utility/typedefs.hpp"
#include "../foundations/paradigm.hpp"

#ifndef ARIADNE_NUMERIC_CONCEPTS_HPP
#define ARIADNE_NUMERIC_CONCEPTS_HPP

namespace Ariadne {

enum class Comparison : ComparableEnumerationType;

template<class L> concept IsLogical = Constructible<L,Bool> and requires(L l) {
    { not (not l) } -> ConvertibleTo<L>;
    { l and l } -> ConvertibleTo<L>;
    { l or l } -> ConvertibleTo<L>;
};

template<class X> using UnaryPlusType = decltype(+declval<X>());
template<class X> using UnaryMinusType = decltype(-declval<X>());
template<class X1, class X2=X1> using PlusType = decltype(declval<X1>()+declval<X2>());
template<class X1, class X2=X1> using MinusType = decltype(declval<X1>()-declval<X2>());
template<class X1, class X2=X1> using TimesType = decltype(declval<X1>()*declval<X2>());
template<class X1, class X2=X1> using DividesType = decltype(declval<X1>()/declval<X2>());

template<class X> concept HasUnaryPlus = requires(X x) { +x; };
template<class X> concept HasUnaryMinus = requires(X x) { -x; };
template<class X1, class X2=X1> concept HasPlus = requires(X1 x1, X2 x2) { x1+x2; };
template<class X1, class X2=X1> concept HasMinus = requires(X1 x1, X2 x2) { x1-x2; };
template<class X1, class X2=X1> concept HasTimes = requires(X1 x1, X2 x2) { x1*x2; };
template<class X1, class X2=X1> concept HasDivides = requires(X1 x1, X2 x2) { x1/x2; };

template<class X> concept HasPos = requires(X x) { pos(x); };
template<class X> concept HasNeg = requires(X x) { neg(x); };
template<class X1, class X2=X1> concept HasAdd = requires(X1 x1, X2 x2) { add(x1,x2); };
template<class X1, class X2=X1> concept HasSub = requires(X1 x1, X2 x2) { sub(x1,x2); };
template<class X1, class X2=X1> concept HasMul = requires(X1 x1, X2 x2) { mul(x1,x2); };
template<class X1, class X2=X1> concept HasDiv = requires(X1 x1, X2 x2) { div(x1,x2); };
template<class X1, class X2=X1> concept HasEq = requires(X1 x1, X2 x2) { eq(x1,x2); };
template<class X1, class X2=X1> concept HasLt = requires(X1 x1, X2 x2) { lt(x1,x2); };
template<class X1, class X2=X1> concept HasCmp = requires(X1 x1, X2 x2) { { cmp(x1,x2) } -> SameAs<Comparison>; };

template<class X> using PosType = decltype(pos(declval<X>()));
template<class X> using NegType = decltype(neg(declval<X>()));
template<class X> using RecType = decltype(rec(declval<X>()));
template<class X1, class X2=X1> using AddType = decltype(add(declval<X1>(),declval<X2>()));
template<class X1, class X2=X1> using SubType = decltype(sub(declval<X1>(),declval<X2>()));
template<class X1, class X2=X1> using MulType = decltype(mul(declval<X1>(),declval<X2>()));
template<class X1, class X2=X1> using DivType = decltype(div(declval<X1>(),declval<X2>()));
template<class X1, class X2=X1> using MaxType = decltype(max(declval<X1>(),declval<X2>()));
template<class X1, class X2=X1> using MinType = decltype(min(declval<X1>(),declval<X2>()));

template<class Y> concept Lattice = requires(Y y) {
//    { abs(y) } -> ConvertibleTo<Y>;
    { max(y,y) } -> SameAs<Y>;
    { min(y,y) } -> SameAs<Y>;
} and  ((not requires(Y y) { { neg(y) } -> SameAs<Y>;}) or (requires(Y y) { { abs(y) } -> ConvertibleTo<Y>; }));

template<class Y1, class Y2=Y1> concept Comparible = requires(Y1 y1, Y2 y2) {
    { cmp(y1,y2) } -> SameAs<Comparison>;
    { cmp(y2,y1) } -> SameAs<Comparison>;
};

template<class Y1, class Y2=Y1> concept Ordered = Comparible<Y1,Y2> or requires(Y1 y1, Y2 y2) {
    lt(y1,y2);
    lt(y2,y1);
};

template<class Y1, class Y2=Y1> concept Equality = requires(Y1 y1, Y2 y2) {
    eq(y1,y2);
    eq(y2,y1);
};

template<class Y, class NY> concept DirectedGroups = requires(Y y, NY ny) {
    { neg(y) } -> SameAs<NY>;
    { neg(ny) } -> SameAs<Y>;
    { add(y,y) } -> SameAs<Y>;
    { add(ny,ny) } -> SameAs<NY>;
    { sub(y,ny) } -> SameAs<Y>;
    { sub(ny,y) } -> SameAs<NY>;
};

template<class Y> concept DirectedGroup = DirectedGroups<Y,NegType<Y>>;

template<class Y, class QY> concept DirectedSemiFields = requires(Y y, QY qy) {
    { rec(y) } -> SameAs<QY>;
    { rec(qy) } -> SameAs<Y>;

    { add(y,y) } -> SameAs<Y>;
    { add(qy,qy) } -> SameAs<QY>;
    { mul(y,y) } -> SameAs<Y>;
    { mul(qy,qy) } -> SameAs<QY>;
    { div(y,qy) } -> SameAs<Y>;
    { div(qy,y) } -> SameAs<QY>;
};

template<class Y> concept DirectedSemiField = DirectedSemiFields<Y,RecType<Y>>;

template<class Y> concept AbelianGroup = (not BuiltinArithmetic<Y>) and requires(Y y) {
    { nul(y) } -> SameAs<Y>;
    { pos(y) } -> SameAs<Y>;
    { neg(y) } -> SameAs<Y>;
    { add(y,y) } -> SameAs<Y>;
    { sub(y,y) } -> SameAs<Y>;
};

template<class Y> concept Ring = AbelianGroup<Y> and Assignable<Y,Int> and requires(Y y) {
    { sqr(y) } -> ConvertibleTo<Y>;
    { mul(y,y) } -> SameAs<Y>;
//    { fma(y, y, y) } -> SameAs<Y>;
};

template<class Y> concept DyadicRing = Ring<Y> and requires(Y y) {
    { hlf(y) } -> SameAs<Y>;
};

template<class Y> concept Field = DyadicRing<Y> and requires(Y y) {
    { rec(y) } -> SameAs<Y>;
    { div(y,y) } -> SameAs<Y>;
};

template<class Y> concept Monotone = requires(Y y) {
    { sqrt(y) } -> ConvertibleTo<Y>;
    { exp(y) } -> ConvertibleTo<Y>;
    { log(y) } -> ConvertibleTo<Y>;
    { atan(y) } -> ConvertibleTo<Y>;
};

template<class Y> concept Trigonometric = requires(Y y) {
    { sin(y) } -> SameAs<Y>;
    { cos(y) } -> SameAs<Y>;
    { tan(y) } -> SameAs<Y>;
};

template<class Y> concept Transcendental = Monotone<Y> and Trigonometric<Y>;

template<class Y> concept TranscendentalField = Field<Y> and Transcendental<Y>;


template<class Y> concept OrderedLattice = Ordered<Y> and Lattice<Y>;

template<class Y> concept LatticeRing = Lattice<Y> and Ring<Y>;
template<class Y> concept LatticeField = Lattice<Y> and Field<Y>;

template<class Y> concept OrderedLatticeRing = OrderedLattice<Y> and Ring<Y>;
template<class Y> concept OrderedLatticeField = OrderedLattice<Y> and Field<Y>;
template<class Y> concept OrderedLatticeTranscendentalField = OrderedLattice<Y> and TranscendentalField<Y>;


template<class X> concept RoundedRing = requires(typename X::RoundingModeType rnd, X x, X x1, X x2) {
    { nul(x) } -> SameAs<X>;
    { pos(x) } -> SameAs<X>;
    { neg(x) } -> SameAs<X>;
    { hlf(x) } -> SameAs<X>;
    { add(rnd, x1, x2) } -> SameAs<X>;
    { sub(rnd, x1, x2) } -> SameAs<X>;
    { mul(rnd, x1, x2) } -> SameAs<X>;
    { fma(rnd, x, x1, x2) } -> SameAs<X>;
};

template<class X> concept RoundedField = RoundedRing<X> and requires(typename X::RoundingModeType rnd, X x, X x1, X x2) {
    { rec(rnd, x) } -> SameAs<X>;
    { div(rnd, x1, x2) } -> SameAs<X>;
};

template<class X> concept RoundedTranscendental = requires(typename X::RoundingModeType rnd, X x) {
    { sqrt(rnd, x) } -> ConvertibleTo<X>;
    { exp(rnd, x) } -> ConvertibleTo<X>;
    { log(rnd, x) } -> ConvertibleTo<X>;
    { atan(rnd, x) } -> ConvertibleTo<X>;

    { sin(rnd, x) } -> SameAs<X>;
    { cos(rnd, x) } -> SameAs<X>;
    { tan(rnd, x) } -> SameAs<X>;
};

template<class X> concept RoundedTranscendentalField = RoundedField<X> and RoundedTranscendental<X>;

template<class X> class Bounds;

template<class X> concept WidenTranscendentalField = requires(X x, X x1, X x2) {
    { add(x1,x2) } -> ConvertibleTo<Bounds<X>>;
    { sub(x1,x2) } -> ConvertibleTo<Bounds<X>>;
    { mul(x1,x2) } -> ConvertibleTo<Bounds<X>>;
    { div(x1,x2) } -> ConvertibleTo<Bounds<X>>;

    { sqrt(x) } -> ConvertibleTo<Bounds<X>>;
    { exp(x) } -> ConvertibleTo<Bounds<X>>;
    { log(x) } -> ConvertibleTo<Bounds<X>>;
    { atan(x) } -> ConvertibleTo<Bounds<X>>;

    { sin(x) } -> SameAs<Bounds<X>>;
    { cos(x) } -> SameAs<Bounds<X>>;
    { tan(x) } -> SameAs<Bounds<X>>;
};

template<class X> concept RoundedLatticeRing = Lattice<X> and RoundedRing<X>;
template<class X> concept RoundedLatticeField = Lattice<X> and RoundedField<X>;

template<class X> concept IsRoundedRing = RoundedRing<X>;
template<class X> concept IsRoundedField = RoundedField<X>;
template<class X> concept IsRoundedLatticeRing = RoundedLatticeRing<X>;
template<class X> concept IsRoundedLatticeField = RoundedLatticeField<X>;


template<class X1, class X2> concept AreMixedLattice = requires(X1 const& x1, X2 const& x2) {
    max(x1,x2);
    { max(x2,x1) } -> SameAs<decltype(max(x1,x2))>;
    min(x1,x2);
    min(x2,x1);
};

template<class X1, class X2> concept AreMixedRing = requires(X1 const& x1, X2 const& x2) {
    -x1;
    -x2;
    x1+x2;
    { x2+x1 } -> SameAs<decltype(x1+x2)>;
    { x1-x2 } -> SameAs<decltype(x1+(-x2))>;
    { x2-x1 } -> SameAs<decltype(x2+(-x1))>;
    { x1-x2 } -> SameAs<decltype(x1+(-x2))>;
    { x2-x1 } -> SameAs<decltype(x2+(-x1))>;
    x1*x2;
    { x2*x1 } -> SameAs<decltype(x1*x2)>;
};

template<class X1, class X2> concept AreMixedField = AreMixedRing<X1,X2> and requires(X1 const& x1, X2 const& x2) {
    x1/x2;
    x2/x1;
};

template<class X1, class X2> concept AreMixedOrdered = requires(X1 const& x1, X2 const& x2) {
    x1 == x2;
    { x2 == x1 } -> SameAs<decltype(x1==x2)>;
    { x1 != x2 } -> SameAs<decltype(!(x1==x2))>;
    { x2 != x1 } -> SameAs<decltype(x2!=x1)>;
    x1 <= x2;
    x2 <= x1;
    { x1 >= x2 } -> SameAs<decltype(x2<=x1)>;
    { x2 >= x1 } -> SameAs<decltype(x1<=x2)>;
    { x1 <  x2 } -> SameAs<decltype(!(x2<=x1))>;
    { x2 <  x1 } -> SameAs<decltype(!(x1<=x2))>;
    { x1 >  x2 } -> SameAs<decltype(x2< x1)>;
    { x2 >  x1 } -> SameAs<decltype(x2< x1)>;
};

template<class X1, class X2> concept AreMixedOrderedLattice = AreMixedOrdered<X1,X2> and AreMixedLattice<X1,X2>;

template<class X1, class X2> concept AreMixedLatticeRing = AreMixedLattice<X1,X2> and AreMixedRing<X1,X2>;
template<class X1, class X2> concept AreMixedLatticeField = AreMixedLattice<X1,X2> and AreMixedField<X1,X2>;

template<class X1, class X2> concept AreMixedOrderedLatticeRing = AreMixedOrderedLattice<X1,X2> and AreMixedRing<X1,X2>;
template<class X1, class X2> concept AreMixedOrderedLatticeField = AreMixedOrderedLattice<X1,X2> and AreMixedField<X1,X2>;

template<class X> concept IsOrderedLatticeRing = OrderedLatticeRing<X> and AreMixedOrderedLatticeRing<X,X>;

} // namespace Ariadne

#endif
