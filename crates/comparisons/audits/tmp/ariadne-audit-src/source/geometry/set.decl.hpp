/***************************************************************************
 *            geometry/set.hpp
 *
 *  Copyright  2013-20  Pieter Collins
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

#ifndef ARIADNE_SET_DECL_HPP
#define ARIADNE_SET_DECL_HPP

namespace Ariadne {

struct EffectiveTag;
struct ValidatedTag;
struct ApproximateTag;

class Real;
template<class X> using Scalar = X;
template<class X> class Vector;
using RealScalar = Scalar<Real>;
using RealVector = Vector<Real>;

template<class T> class SetBase;
using EuclideanSetBase = SetBase<RealVector>;

//! \ingroup GeometryModule SetSubModule
//! \brief %Bounded sets.
template<class P, class T> class BoundedSet;
//! \ingroup GeometryModule SetSubModule
//! \brief %Open sets, defined by a verifyable membership predicate.
template<class P, class T> class OpenSet;
//! \ingroup GeometryModule SetSubModule
//! \brief %Closed sets, defined by a falsifyable membership predicate.
template<class P, class T> class ClosedSet;
//! \ingroup GeometryModule SetSubModule
//! \brief %Overt sets, defined by a verifyable intersection with open sets.
template<class P, class T> class OvertSet;
//! \ingroup GeometryModule SetSubModule
//! \brief %Compact sets, defined by a verifyable disjointness with closed sets.
template<class P, class T> class CompactSet;
//! \ingroup GeometryModule SetSubModule
//! \brief %Regular (open and closed) sets.
template<class P, class T> class RegularSet;
//! \ingroup GeometryModule SetSubModule
//! \brief %Located (overt and compact) sets.
template<class P, class T> class LocatedSet;
//! \ingroup GeometryModule SetSubModule
//! \brief %Regular located sets; equivalently bounded regular or open located.
template<class P, class T> class RegularLocatedSet;

template<class P> using EuclideanBoundedSet = BoundedSet<P,RealVector>;
template<class P> using EuclideanOpenSet = OpenSet<P,RealVector>;
template<class P> using EuclideanClosedSet = ClosedSet<P,RealVector>;
template<class P> using EuclideanOvertSet = OvertSet<P,RealVector>;
template<class P> using EuclideanCompactSet = CompactSet<P,RealVector>;
template<class P> using EuclideanRegularSet = RegularSet<P,RealVector>;
template<class P> using EuclideanLocatedSet = LocatedSet<P,RealVector>;
template<class P> using EuclideanRegularLocatedSet = RegularLocatedSet<P,RealVector>;

template<class T> using EffectiveBoundedSet = BoundedSet<EffectiveTag,T>;
template<class T> using EffectiveOpenSet = OpenSet<EffectiveTag,T>;
template<class T> using EffectiveClosedSet = ClosedSet<EffectiveTag,T>;
template<class T> using EffectiveOvertSet = OvertSet<EffectiveTag,T>;
template<class T> using EffectiveCompactSet = CompactSet<EffectiveTag,T>;
template<class T> using EffectiveRegularSet = RegularSet<EffectiveTag,T>;
template<class T> using EffectiveLocatedSet = LocatedSet<EffectiveTag,T>;
template<class T> using EffectiveRegularLocatedSet = RegularLocatedSet<EffectiveTag,T>;

template<class T> using ValidatedBoundedSet = BoundedSet<ValidatedTag,T>;
template<class T> using ValidatedOpenSet = OpenSet<ValidatedTag,T>;
template<class T> using ValidatedClosedSet = ClosedSet<ValidatedTag,T>;
template<class T> using ValidatedOvertSet = OvertSet<ValidatedTag,T>;
template<class T> using ValidatedCompactSet = CompactSet<ValidatedTag,T>;
template<class T> using ValidatedRegularSet = RegularSet<ValidatedTag,T>;
template<class T> using ValidatedLocatedSet = LocatedSet<ValidatedTag,T>;
template<class T> using ValidatedRegularLocatedSet = RegularLocatedSet<ValidatedTag,T>;

using EffectiveEuclideanBoundedSet = BoundedSet<EffectiveTag,RealVector>;
using EffectiveEuclideanOpenSet = OpenSet<EffectiveTag,RealVector>;
using EffectiveEuclideanClosedSet = ClosedSet<EffectiveTag,RealVector>;
using EffectiveEuclideanOvertSet = OvertSet<EffectiveTag,RealVector>;
using EffectiveEuclideanCompactSet = CompactSet<EffectiveTag,RealVector>;
using EffectiveEuclideanRegularSet = RegularSet<EffectiveTag,RealVector>;
using EffectiveEuclideanLocatedSet = LocatedSet<EffectiveTag,RealVector>;
using EffectiveEuclideanRegularLocatedSet = RegularLocatedSet<EffectiveTag,RealVector>;

using ValidatedEuclideanBoundedSet = BoundedSet<ValidatedTag,RealVector>;
using ValidatedEuclideanOpenSet = OpenSet<ValidatedTag,RealVector>;
using ValidatedEuclideanClosedSet = ClosedSet<ValidatedTag,RealVector>;
using ValidatedEuclideanOvertSet = OvertSet<ValidatedTag,RealVector>;
using ValidatedEuclideanCompactSet = CompactSet<ValidatedTag,RealVector>;
using ValidatedEuclideanRegularSet = RegularSet<ValidatedTag,RealVector>;
using ValidatedEuclideanLocatedSet = LocatedSet<ValidatedTag,RealVector>;
using ValidatedEuclideanRegularLocatedSet = RegularLocatedSet<ValidatedTag,RealVector>;




template<class T> class SetInterfaceBase;
using EuclideanSetInterfaceBase = SetInterfaceBase<RealVector>;

template<class P, class T> class BoundedSetInterface;
template<class P, class T> class OpenSetInterface;
template<class P, class T> class ClosedSetInterface;
template<class P, class T> class OvertSetInterface;
template<class P, class T> class CompactSetInterface;
template<class P, class T> class RegularSetInterface;
template<class P, class T> class LocatedSetInterface;
template<class P, class T> class RegularLocatedSetInterface;
template<class P, class T> using SetInterface = RegularLocatedSetInterface<P, T>;

template<class P> using EuclideanBoundedSetInterface = BoundedSetInterface<P,RealVector>;
template<class P> using EuclideanOpenSetInterface = OpenSetInterface<P,RealVector>;
template<class P> using EuclideanClosedSetInterface = ClosedSetInterface<P,RealVector>;
template<class P> using EuclideanOvertSetInterface = OvertSetInterface<P,RealVector>;
template<class P> using EuclideanCompactSetInterface = CompactSetInterface<P,RealVector>;
template<class P> using EuclideanRegularSetInterface = RegularSetInterface<P,RealVector>;
template<class P> using EuclideanLocatedSetInterface = LocatedSetInterface<P,RealVector>;
template<class P> using EuclideanRegularLocatedSetInterface = RegularLocatedSetInterface<P,RealVector>;

template<class T> using EffectiveBoundedSetInterface = BoundedSetInterface<EffectiveTag,T>;
template<class T> using EffectiveOpenSetInterface = OpenSetInterface<EffectiveTag,T>;
template<class T> using EffectiveClosedSetInterface = ClosedSetInterface<EffectiveTag,T>;
template<class T> using EffectiveOvertSetInterface = OvertSetInterface<EffectiveTag,T>;
template<class T> using EffectiveCompactSetInterface = CompactSetInterface<EffectiveTag,T>;
template<class T> using EffectiveRegularSetInterface = RegularSetInterface<EffectiveTag,T>;
template<class T> using EffectiveLocatedSetInterface = LocatedSetInterface<EffectiveTag,T>;
template<class T> using EffectiveRegularLocatedSetInterface = RegularLocatedSetInterface<EffectiveTag,T>;

template<class T> using ValidatedBoundedSetInterface = BoundedSetInterface<ValidatedTag,T>;
template<class T> using ValidatedOpenSetInterface = OpenSetInterface<ValidatedTag,T>;
template<class T> using ValidatedClosedSetInterface = ClosedSetInterface<ValidatedTag,T>;
template<class T> using ValidatedOvertSetInterface = OvertSetInterface<ValidatedTag,T>;
template<class T> using ValidatedCompactSetInterface = CompactSetInterface<ValidatedTag,T>;
template<class T> using ValidatedRegularSetInterface = RegularSetInterface<ValidatedTag,T>;
template<class T> using ValidatedLocatedSetInterface = LocatedSetInterface<ValidatedTag,T>;
template<class T> using ValidatedRegularLocatedSetInterface = RegularLocatedSetInterface<ValidatedTag,T>;

using EffectiveEuclideanBoundedSetInterface = BoundedSetInterface<EffectiveTag,RealVector>;
using EffectiveEuclideanOpenSetInterface = OpenSetInterface<EffectiveTag,RealVector>;
using EffectiveEuclideanClosedSetInterface = ClosedSetInterface<EffectiveTag,RealVector>;
using EffectiveEuclideanOvertSetInterface = OvertSetInterface<EffectiveTag,RealVector>;
using EffectiveEuclideanCompactSetInterface = CompactSetInterface<EffectiveTag,RealVector>;
using EffectiveEuclideanRegularSetInterface = RegularSetInterface<EffectiveTag,RealVector>;
using EffectiveEuclideanLocatedSetInterface = LocatedSetInterface<EffectiveTag,RealVector>;
using EffectiveEuclideanRegularLocatedSetInterface = RegularLocatedSetInterface<EffectiveTag,RealVector>;
using EffectiveEuclideanSetInterface = SetInterface<EffectiveTag,RealVector>;

using ValidatedEuclideanBoundedSetInterface = BoundedSetInterface<ValidatedTag,RealVector>;
using ValidatedEuclideanOpenSetInterface = OpenSetInterface<ValidatedTag,RealVector>;
using ValidatedEuclideanClosedSetInterface = ClosedSetInterface<ValidatedTag,RealVector>;
using ValidatedEuclideanOvertSetInterface = OvertSetInterface<ValidatedTag,RealVector>;
using ValidatedEuclideanCompactSetInterface = CompactSetInterface<ValidatedTag,RealVector>;
using ValidatedEuclideanRegularSetInterface = RegularSetInterface<ValidatedTag,RealVector>;
using ValidatedEuclideanLocatedSetInterface = LocatedSetInterface<ValidatedTag,RealVector>;
using ValidatedEuclideanRegularLocatedSetInterface = RegularLocatedSetInterface<ValidatedTag,RealVector>;
using ValidatedEuclideanSetInterface = RegularLocatedSetInterface<ValidatedTag,RealVector>;


} // namespace Ariadne


#endif // ARIADNE_SET_DECL_HPP
