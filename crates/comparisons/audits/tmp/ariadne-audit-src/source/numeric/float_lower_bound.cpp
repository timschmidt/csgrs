/***************************************************************************
 *            numeric/float_lower_bound.cpp
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

#include "float_lower_bound.hpp"
#include "float_lower_bound.tpl.hpp"

#include "floatdp.hpp"
#include "floatmp.hpp"

namespace Ariadne {

LowerBound<Dyadic>::LowerBound(LowerBound<FloatDP> const& x) : LowerBound(Dyadic(x.raw())) { }
LowerBound<Dyadic>::LowerBound(LowerBound<FloatMP> const& x) : LowerBound(Dyadic(x.raw())) { }
LowerBound<FloatDP> LowerBound<Dyadic>::get(DoublePrecision pr) const { return LowerBound<FloatDP>(FloatDP(this->raw(),down,pr)); }
LowerBound<FloatMP> LowerBound<Dyadic>::get(MultiplePrecision pr) const { return LowerBound<FloatMP>(FloatMP(this->raw(),down,pr)); }

template class LowerBound<FloatDP>;
template class Operations<LowerBound<FloatDP>>;
template class LowerBound<FloatMP>;
template class Operations<LowerBound<FloatMP>>;

template<> String class_name<LowerBound<FloatDP>>() { return "FloatDPLowerBound"; }
template<> String class_name<LowerBound<FloatMP>>() { return "FloatMPLowerBound"; }

} // namespace Ariadne
