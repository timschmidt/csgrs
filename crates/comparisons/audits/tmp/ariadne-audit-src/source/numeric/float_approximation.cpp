/***************************************************************************
 *            numeric/float_approximation.cpp
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

#include "float_approximation.hpp"
#include "float_approximation.tpl.hpp"

#include "floatdp.hpp"
#include "floatmp.hpp"

namespace Ariadne {

template class Approximation<FloatDP>;
template class Operations<Approximation<FloatDP>>;
template class Approximation<FloatMP>;
template class Operations<Approximation<FloatMP>>;

template<> String class_name<Approximation<FloatDP>>() { return "FloatDPApproximation"; }
template<> String class_name<Approximation<FloatMP>>() { return "FloatMPApproximation"; }

} // namespace Ariadne
