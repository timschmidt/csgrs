/***************************************************************************
 *            geometry_concepts.hpp
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

#ifndef ARIADNE_GEOMETRY_CONCEPTS_HPP
#define ARIADNE_GEOMETRY_CONCEPTS_HPP

#include <iosfwd>

#include "utility/metaprogramming.hpp"
#include "foundations/logical.decl.hpp"

namespace Ariadne {

using OutputStream = std::ostream;

template<class T> struct SetTraits;
template<class T> using DimensionOfType = typename SetTraits<T>::DimensionType;
template<class T> using BasicSetType = typename SetTraits<T>::BasicSetType;
template<class T> using BoundingSetType = typename SetTraits<T>::BoundingSetType;


template<class W> concept Writable = requires(OutputStream& os, W const& w) {
    { os << w } -> SameAs<OutputStream&>;
};

static_assert(SameAs<LogicalType<ExactTag>,Boolean>);
static_assert(SameAs<LogicalType<EffectiveTag>,Kleenean>);
static_assert(SameAs<LogicalType<ValidatedTag>,ValidatedKleenean>);
static_assert(SameAs<LogicalType<ApproximateTag>,ApproximateKleenean>);


template<class S, class T> concept ASetBase = CopyConstructible<T> and Writable<T> and requires(S s) {
    { s.dimension() } -> SameAs<typename SetTraits<T>::DimensionType>;
};

template<class S, class P, class T> concept ABoundedSet = ASetBase<S,T> and requires(S s, BasicSetType<T> bs) {
    { s.bounding_box() } -> CastableTo<BoundingSetType<T>>;
    { s.inside(bs) } -> Convertible<LowerLogicalType<P>>;
};

template<class S, class P, class T> concept AnOvertSet = ASetBase<S,T> and requires(S s, BasicSetType<T> bs) {
    { s.overlaps(bs) } -> Convertible<LowerLogicalType<P>>;
};

template<class S, class P, class T> concept AnOpenSet = AnOvertSet<S,P,T> and requires(S s, BasicSetType<T> bs) {
    { s.covers(bs) } -> Convertible<LowerLogicalType<P>>;
};

template<class S, class P, class T> concept AClosedSet = ASetBase<S,T> and requires(S s, BasicSetType<T> bs) {
    { s.separated(bs) } -> Convertible<LowerLogicalType<P>>;
};

template<class S, class P, class T> concept ACompactSet = AClosedSet<S,P,T> and ABoundedSet<S,P,T>;

template<class S, class P, class T> concept ARegularSet = AnOpenSet<S,P,T> and AClosedSet<S,P,T>;

template<class S, class P, class T> concept ALocatedSet = AnOvertSet<S,P,T> and ACompactSet<S,P,T>;

template<class S, class P, class T> concept ARegularLocatedSet = ARegularSet<S,P,T> and ALocatedSet<S,P,T>;

} // namespace Ariadne

#endif // ARIADNE_GEOMETRY_CONCEPTS_HPP
