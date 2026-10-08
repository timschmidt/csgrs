/***************************************************************************
 *            function/affine.cpp
 *
 *  Copyright  2009-20  Pieter Collins
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

#include "numeric/numeric.hpp"
#include "config.hpp"

#include "numeric/numeric.hpp"
#include "algebra/vector.hpp"
#include "function/function.hpp"
#include "function/affine.hpp"

namespace Ariadne {

template class Affine<FloatDPApproximation>;
template class Affine<FloatDPBounds>;
template struct ProvideAlgebraOperations<Affine<FloatDPApproximation>,FloatDPApproximation>;
template struct ProvideAlgebraOperations<Affine<FloatDPBounds>,FloatDPBounds>;

template class Affine<FloatMPApproximation>;
template class Affine<FloatMPBounds>;
template struct ProvideAlgebraOperations<Affine<FloatMPApproximation>,FloatMPApproximation>;
template struct ProvideAlgebraOperations<Affine<FloatMPBounds>,FloatMPBounds>;

} //namespace Ariadne


