/***************************************************************************
 *            interval.decl.hpp
 *
 *  Copyright  2008-20  Alberto Casagrande, Pieter Collins
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

#ifndef ARIADNE_INTERVAL_DECL_HPP
#define ARIADNE_INTERVAL_DECL_HPP

#include "numeric/float.decl.hpp"
#include "numeric/number.decl.hpp"

namespace Ariadne {

template<class U> class Interval;
template<class U> class VariableInterval;

template<class IVL> struct IsInterval : False { };
template<class UB> struct IsInterval<Interval<UB>> : True { };

template<class IVL> concept AnInterval = IsInterval<IVL>::value;

//!@{
//! \relates Interval
//! \name Type shorthands
template<class F> using ExactInterval = Interval<F>; //!< <p/>
template<class F> using BallInterval = Interval<Ball<F>>; //!< <p/>
template<class F> using BoundsInterval = Interval<Bounds<F>>; //!< <p/>
template<class F> using UpperInterval = Interval<UpperBound<F>>; //!< <p/>
template<class F> using LowerInterval = Interval<LowerBound<F>>; //!< <p/>
template<class F> using ApproximateInterval = Interval<Approximation<F>>; //!< <p/>

template<class PR> using FloatExactInterval = Interval<Float<PR>>; //!< <p/>
template<class PR> using FloatBallInterval = Interval<FloatBall<PR>>; //!< <p/>
template<class PR> using FloatBoundsInterval = Interval<FloatBounds<PR>>; //!< <p/>
template<class PR> using FloatUpperInterval = Interval<FloatUpperBound<PR>>; //!< <p/>
template<class PR> using FloatLowerInterval = Interval<FloatLowerBound<PR>>; //!< <p/>
template<class PR> using FloatApproximateInterval = Interval<FloatApproximation<PR>>; //!< <p/>
//!@}

//!@{
//! \relates Interval
//! \name Type synonyms
using DyadicInterval = Interval<Dyadic>; //!< <p/>
using DecimalInterval = Interval<Decimal>; //!< <p/>
using RationalInterval = Interval<Rational>; //!< <p/>
using RealInterval = Interval<Real>; //!< <p/>

using FloatDPExactInterval = FloatExactInterval<DoublePrecision>; //!< <p/>
using FloatDPBallInterval = FloatBallInterval<DoublePrecision>; //!< <p/>
using FloatDPBoundsInterval = FloatBoundsInterval<DoublePrecision>; //!< <p/>
using FloatDPUpperInterval = FloatUpperInterval<DoublePrecision>; //!< <p/>
using FloatDPLowerInterval = FloatLowerInterval<DoublePrecision>; //!< <p/>
using FloatDPApproximateInterval = FloatApproximateInterval<DoublePrecision>; //!< <p/>

using FloatMPExactInterval = FloatExactInterval<MultiplePrecision>; //!< <p/>
using FloatMPBallInterval = FloatBallInterval<MultiplePrecision>; //!< <p/>
using FloatMPBoundsInterval = FloatBoundsInterval<MultiplePrecision>; //!< <p/>
using FloatMPUpperInterval = FloatUpperInterval<MultiplePrecision>; //!< <p/>
using FloatMPLowerInterval = FloatLowerInterval<MultiplePrecision>; //!< <p/>
using FloatMPApproximateInterval = FloatApproximateInterval<MultiplePrecision>; //!< <p/>
//!@}


//!@{
//! \ingroup GeometryModule
//! \name Type definitions

//! The type used for the domain of a univariate function. \ingroup GeometryModule
typedef FloatDPExactInterval IntervalDomainType;
//! The type used for an over-approximation to the range of a validated scalar function. \ingroup GeometryModule
typedef FloatDPUpperInterval IntervalValidatedRangeType;
//! The type used for an approximation to the range of an approximate scalar function. \ingroup GeometryModule
typedef FloatDPApproximateInterval IntervalApproximateRangeType;

//! \brief The type used for testing characteristics of one-dimensional sets. \ingroup GeometryModule
typedef FloatDPExactInterval ExactIntervalType;
//! \brief The type used for the bounding interval of validated one-dimensional sets. \ingroup GeometryModule
typedef FloatDPUpperInterval UpperIntervalType;
//! \brief The type used for testing boundedness of validated one-dimensional sets. \ingroup GeometryModule
typedef FloatDPLowerInterval LowerIntervalType;
//! \brief The type used for the bounding interval of approximate one-dimensional sets. \ingroup GeometryModule
typedef FloatDPApproximateInterval ApproximateIntervalType;
//!@}


} // namespace Ariadne


#endif
