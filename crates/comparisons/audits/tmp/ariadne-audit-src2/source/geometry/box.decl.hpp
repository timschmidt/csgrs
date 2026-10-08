/***************************************************************************
 *            box.decl.hpp
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

#ifndef ARIADNE_BOX_DECL_HPP
#define ARIADNE_BOX_DECL_HPP

#include "geometry/interval.decl.hpp"

namespace Ariadne {

template<class IVL> class Box;
template<class IVL> class VariablesBox;

//!@{
//! \relates Box
//! \name Type synonyms
using DyadicBox = Box<DyadicInterval>; //!< <p/>
using DecimalBox = Box<DecimalInterval>; //!< <p/>
using RationalBox = Box<RationalInterval>; //!< <p/>
using RealBox = Box<RealInterval>; //!< <p/>

using FloatDPExactBox = Box<FloatDPExactInterval>; //!< <p/>
using FloatDPBallBox = Box<FloatDPBallInterval>; //!< <p/>
using FloatDPBoundsBox = Box<FloatDPBoundsInterval>; //!< <p/>
using FloatDPUpperBox = Box<FloatDPUpperInterval>; //!< <p/>
using FloatDPLowerBox = Box<FloatDPLowerInterval>; //!< <p/>
using FloatDPApproximateBox = Box<FloatDPApproximateInterval>; //!< <p/>

using FloatMPExactBox = Box<FloatMPExactInterval>; //!< <p/>
using FloatMPBallBox = Box<FloatMPBallInterval>; //!< <p/>
using FloatMPBoundsBox = Box<FloatMPBoundsInterval>; //!< <p/>
using FloatMPUpperBox = Box<FloatMPUpperInterval>; //!< <p/>
using FloatMPLowerBox = Box<FloatMPLowerInterval>; //!< <p/>
using FloatMPApproximateBox = Box<FloatMPApproximateInterval>; //!< <p/>
//!@}


//!@{
//! \ingroup GeometryModule
//! \name Type definitions

//! The type used for the domain of a multivariate function. \ingroup GeometryModule
typedef FloatDPExactBox BoxDomainType;
//! The type used for an over-approximation to the range of a validated vector function. \ingroup GeometryModule
typedef FloatDPUpperBox BoxValidatedRangeType;
//! The type used for an approximation to the range of an approximate vector function. \ingroup GeometryModule
typedef FloatDPApproximateBox BoxApproximateRangeType;

//! \brief The type used for the bounding box of validated sets in Euclidean space. \ingroup GeometryModule
typedef FloatDPUpperBox BoundingBoxType;

//! \brief The type used for testing properties of sets in Euclidean space. \ingroup GeometryModule
typedef FloatDPExactBox ExactBoxType;
//! \brief The type used for the bounding box of validated sets in Euclidean space. \ingroup GeometryModule
typedef FloatDPUpperBox UpperBoxType;
//! \brief The type used for testing boundedness of sets in Euclidean space. \ingroup GeometryModule
typedef FloatDPLowerBox LowerBoxType;
//! \brief The type used for the bounding box of approximate sets in Euclidean space. \ingroup GeometryModule
typedef FloatDPApproximateBox ApproximateBoxType;
//!@}

template<class IVL> class BoxSet;
//!@{
//! \relates BoxSet
//! \name Type synonyms
typedef BoxSet<ExactIntervalType> ExactBoxSetType; //!< <p/>
typedef BoxSet<UpperIntervalType> UpperBoxSetType; //!< <p/>
typedef BoxSet<ApproximateIntervalType> ApproximateBoxSetType; //!< <p/>
typedef BoxSet<RealInterval> RealBoxSet; //!< <p/>
//!@}

} // namespace Ariadne


#endif /* ARIADNE_BOX_HPP */
