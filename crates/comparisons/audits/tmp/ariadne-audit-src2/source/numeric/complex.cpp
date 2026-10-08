/***************************************************************************
 *            numeric/complex.cpp
 *
 *  Copyright  2019-20  Pieter Collins
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


#include "rational.hpp"
#include "real.hpp"
#include "complex.hpp"

#include "floatdp.hpp"
#include "floatmp.hpp"

#include "float_approximation.hpp"
#include "float_lower_bound.hpp"
#include "float_upper_bound.hpp"
#include "float_bounds.hpp"
#include "float_ball.hpp"
#include "float_error.hpp"

namespace Ariadne {

namespace Constants {
const Complex<Integer> i = Complex<Integer>(0,1);
}

//template class Complex<Rational>;
template class Complex<Real>;
//template class Complex<FloatDPBall>;
template class Complex<FloatDPBounds>;
template class Complex<FloatDPApproximation>;
//template class Complex<FloatMPBall>;
template class Complex<FloatMPBounds>;
template class Complex<FloatMPApproximation>;

} // namespace Ariadne
