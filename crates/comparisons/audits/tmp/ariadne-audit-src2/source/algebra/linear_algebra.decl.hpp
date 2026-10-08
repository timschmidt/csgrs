/***************************************************************************
 *            algebra/linear_algebra.decl.hpp
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

/*! \file algebra/linear_algebra.decl.hpp
 *  \brief
 */

#include "numeric/number.decl.hpp"
#include "numeric/float.decl.hpp"

#ifndef ARIADNE_LINEAR_ALGEBRA_DECL_HPP
#define ARIADNE_LINEAR_ALGEBRA_DECL_HPP

namespace Ariadne {

template<class X> using Scalar = X;
template<class X> class Vector;
template<class X> class Covector;
template<class X> class Matrix;
template<class X> class DiagonalMatrix;
template<class X> class SymmetricMatrix;

//! \relates Vector
//! \name Type synonyms
//!@{
using DyadicVector = Vector<Dyadic>; //!< <p/>
using RationalVector = Vector<Rational>; //!< <p/>
using RealVector = Vector<Real>; //!< <p/>
using FloatDPVector = Vector<FloatDP>; //!< <p/>
using RawFloatDPVector = Vector<RawFloatDP>; //!< <p/>
using FloatDPApproximationVector = Vector<FloatDPApproximation>; //!< <p/>
using FloatDPBoundsVector = Vector<FloatDPBounds>; //!< <p/>
using FloatDPBallVector = Vector<FloatDPBall>; //!< <p/>
using FloatDPVector = Vector<FloatDP>; //!< <p/>
using FloatMPVector = Vector<FloatMP>; //!< <p/>
using RawFloatMPVector = Vector<RawFloatMP>; //!< <p/>
using FloatMPApproximationVector = Vector<FloatMPApproximation>; //!< <p/>
using FloatMPBoundsVector = Vector<FloatMPBounds>; //!< <p/>
using FloatMPBallVector = Vector<FloatMPBall>; //!< <p/>
using FloatMPVector = Vector<FloatMP>; //!< <p/>
using FloatMPDPBallVector = Vector<FloatMPDPBall>; //!< <p/>
template<class F> using ApproximationVector = Vector<Approximation<F>>; //!< <p/>
//!@}


//! \relates Covector
//! \name Type synonyms
//!@{
using DyadicCovector = Covector<Dyadic>; //!< <p/>
using RationalCovector = Covector<Rational>; //!< <p/>
using RealCovector = Covector<Real>; //!< <p/>
using FloatDPCovector = Covector<FloatDP>; //!< <p/>
using RawFloatDPCovector = Covector<RawFloatDP>; //!< <p/>
using FloatDPApproximationCovector = Covector<FloatDPApproximation>; //!< <p/>
using FloatDPBoundsCovector = Covector<FloatDPBounds>; //!< <p/>
using FloatDPBallCovector = Covector<FloatDPBall>; //!< <p/>
using FloatDPCovector = Covector<FloatDP>; //!< <p/>
using FloatMPCovector = Covector<FloatMP>; //!< <p/>
using RawFloatMPCovector = Covector<RawFloatMP>; //!< <p/>
using FloatMPApproximationCovector = Covector<FloatMPApproximation>; //!< <p/>
using FloatMPBoundsCovector = Covector<FloatMPBounds>; //!< <p/>
using FloatMPBallCovector = Covector<FloatMPBall>; //!< <p/>
using FloatMPCovector = Covector<FloatMP>; //!< <p/>
using FloatMPDPBallCovector = Covector<FloatMPDPBall>; //!< <p/>
//!@}


//! \relates Matrix
//! \name Type synonyms
//!@{
using DyadicMatrix = Matrix<Dyadic>; //!< <p/>
using RationalMatrix = Matrix<Rational>; //!< <p/>
using RealMatrix = Matrix<Real>; //!< <p/>
using FloatDPMatrix = Matrix<FloatDP>; //!< <p/>
using RawFloatDPMatrix = Matrix<RawFloatDP>; //!< <p/>
using FloatDPApproximationMatrix = Matrix<FloatDPApproximation>; //!< <p/>
using FloatDPBoundsMatrix = Matrix<FloatDPBounds>; //!< <p/>
using FloatDPBallMatrix = Matrix<FloatDPBall>; //!< <p/>
using FloatDPMatrix = Matrix<FloatDP>; //!< <p/>
using FloatMPApproximationMatrix = Matrix<FloatMPApproximation>; //!< <p/>
using FloatMPBoundsMatrix = Matrix<FloatMPBounds>; //!< <p/>
using FloatMPBallMatrix = Matrix<FloatMPBall>; //!< <p/>
using FloatMPMatrix = Matrix<FloatMP>; //!< <p/>
using FloatMPDPBallMatrix = Matrix<FloatMPDPBall>; //!< <p/>
//!@}


//! \relates DiagonalMatrix
//! \name Type synonyms
//!@{
using FloatDPApproximationDiagonalMatrix = DiagonalMatrix<FloatDPApproximation>; //!< <p/>
using FloatDPBoundsDiagonalMatrix = DiagonalMatrix<FloatDPBounds>; //!< <p/>
using FloatDPBallDiagonalMatrix = DiagonalMatrix<FloatDPBall>; //!< <p/>
using FloatDPDiagonalMatrix = DiagonalMatrix<FloatDP>; //!< <p/>
using FloatMPApproximationDiagonalMatrix = DiagonalMatrix<FloatMPApproximation>; //!< <p/>
using FloatMPBoundsDiagonalMatrix = DiagonalMatrix<FloatMPBounds>; //!< <p/>
using FloatMPBallDiagonalMatrix = DiagonalMatrix<FloatMPBall>; //!< <p/>
using FloatMPDiagonalMatrix = DiagonalMatrix<FloatMP>; //!< <p/>
using FloatMPDPBallDiagonalMatrix = DiagonalMatrix<FloatMPDPBall>; //!< <p/>
//!@}

} // namespace Ariadne

#endif
