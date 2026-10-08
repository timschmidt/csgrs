/***************************************************************************
 *            numeric/float_literals.hpp
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

/*! \file numeric/float_literals.hpp
 *  \brief Inclusion header for floating-point extended literals.
 */

#ifndef ARIADNE_FLOAT_LITERALS_HPP
#define ARIADNE_FLOAT_LITERALS_HPP

#include "float.decl.hpp"

namespace Ariadne {

FloatDPError operator"" _error(long double lx);
FloatDP operator"" _exact(long double lx);
FloatDPBall operator"" _near(long double lx);
FloatDPUpperBound operator"" _upper(long double lx);
FloatDPLowerBound operator"" _lower(long double lx);
FloatDPApproximation operator"" _approx(long double lx);

} // namespace Ariadne

#endif
