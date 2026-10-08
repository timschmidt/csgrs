/***************************************************************************
 *            float_upper_bound.tpl.hpp
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


#ifndef ARIADNE_FLOAT_UPPER_BOUND_TPL_HPP
#define ARIADNE_FLOAT_UPPER_BOUND_TPL_HPP

#include "float_upper_bound.hpp"

#include "float_error.hpp"
#include "float_bounds.hpp"
#include "float_lower_bound.hpp"

#include "upper_number.hpp"
#include "number_wrapper.hpp"

namespace Ariadne {

template<class F> UpperBound<F>::UpperBound(F const& x, PR pr) : _u(x,downward,pr) {}
template<class F> UpperBound<F>::UpperBound(Bounds<F> const& x, PR pr) : _u(x._u,downward,pr) {}
template<class F> UpperBound<F>::UpperBound(UpperBound<F> const& x, PR pr) : _u(x._u,upward,pr) {}

template<class F> UpperBound<F>::UpperBound(Bounds<F> const& x) : UpperBound<F>(x.upper_raw()) { }
template<class F> UpperBound<F>::UpperBound(Error<F> const& x) : UpperBound<F>(x.raw()) { }

template<class F> UpperBound<F>::UpperBound(Real const& r, PR pr) : UpperBound(r.get(pr)) {}
template<class F> UpperBound<F>::UpperBound(ValidatedUpperNumber const& y, PR pr) : UpperBound(y.get(pr)) {}
template<class F> UpperBound<F>::operator ValidatedUpperNumber() const {
    return ValidatedUpperNumber(Handle<const NumberInterface>(new NumberWrapper<UpperBound<F>>(*this)));
}
template<class F> auto UpperBound<F>::generic() const -> GenericType { return this->operator GenericType(); }

} // namespace Ariadne

#endif
