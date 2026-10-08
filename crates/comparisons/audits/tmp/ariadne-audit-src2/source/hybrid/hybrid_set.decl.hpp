/***************************************************************************
 *            hybrid_set.decl.hpp
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

/*! \file hybrid_set.decl.hpp
 *  \brief Forward declarations of sets in hybrid spaces.
 */

#ifndef ARIADNE_HYBRID_SET_DECL_HPP
#define ARIADNE_HYBRID_SET_DECL_HPP


namespace Ariadne {

class HybridSetBase;

template<class P> class HybridBoundedSet;
template<class P> class HybridOpenSet;
template<class P> class HybridClosedSet;
template<class P> class HybridOvertSet;
template<class P> class HybridCompactSet;
template<class P> class HybridRegularSet;
template<class P> class HybridLocatedSet;
template<class P> class HybridRegularLocatedSet;


template<class UB> class VariableInterval;
using FloatVariableInterval = VariableInterval<FloatDP>;


template<class UB> class HybridVariableInterval;
template<class IVL> class HybridVariablesBox;

class HybridBoxSet;
class HybridConstraintSet;
class HybridBoundedConstraintSet;
using HybridSet = HybridBoundedConstraintSet;


template<class X> class HybridPoint;
using HybridRealPoint = HybridPoint<Real>;
using HybridExactPoint = HybridPoint<FloatDP>;
using HybridApproximatePoint = HybridPoint<FloatDPApproximation>;

template<class IVL> class HybridBox;
using HybridRealBox = HybridBox<RealInterval>;
using HybridExactBox = HybridBox<ExactIntervalType>;
using HybridUpperBox = HybridBox<UpperIntervalType>;

using HybridExactBoxType = HybridExactBox;
using HybridUpperBoxType = HybridUpperBox;

template<class IVL> class HybridBoxes;
using HybridExactBoxes = HybridBoxes<ExactIntervalType>;
using HybridUpperBoxes = HybridBoxes<UpperIntervalType>;

using HybridExactBoxesType = HybridExactBoxes;
using HybridUpperBoxesType = HybridUpperBoxes;

template<class EBS> class HybridListSet;
class HybridGridTreePaving;

} // namespace Ariadne

#endif // ARIADNE_HYBRID_SET_DECL_HPP
