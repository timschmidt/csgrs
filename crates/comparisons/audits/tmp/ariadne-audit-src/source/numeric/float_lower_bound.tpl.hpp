/***************************************************************************
 *            float_lower_bound.tpl.hpp
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


#ifndef ARIADNE_FLOAT_LOWER_BOUND_TPL_HPP
#define ARIADNE_FLOAT_LOWER_BOUND_TPL_HPP

#include "float_lower_bound.hpp"

#include "float_bounds.hpp"
#include "float_upper_bound.hpp"

#include "lower_number.hpp"
#include "number_wrapper.hpp"

namespace Ariadne {

template<class F> LowerBound<F>::LowerBound(F const& x, PR pr) : _l(x,downward,pr) {}
template<class F> LowerBound<F>::LowerBound(Bounds<F> const& x, PR pr) : _l(x._l,downward,pr) {}
template<class F> LowerBound<F>::LowerBound(LowerBound<F> const& x, PR pr) : _l(x._l,downward,pr) {}

template<class F> LowerBound<F>::LowerBound(Bounds<F> const& x) : LowerBound<F>(x.lower_raw()) { }

template<class F> LowerBound<F>::LowerBound(Real const& r, PR pr) : LowerBound(r.get(pr)) {}
template<class F> LowerBound<F>::LowerBound(ValidatedLowerNumber const& y, PR pr) : LowerBound(y.get(pr)) {}
template<class F> LowerBound<F>::operator ValidatedLowerNumber() const {
    return ValidatedLowerNumber(Handle<const NumberInterface>(new NumberWrapper<LowerBound<F>>(*this)));
}
template<class F> auto LowerBound<F>::generic() const -> GenericType { return this->operator GenericType(); }

} // namespace Ariadne

#endif
