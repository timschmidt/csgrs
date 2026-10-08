/***************************************************************************
 *            float_approximation.tpl.hpp
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


#ifndef ARIADNE_FLOAT_APPROXIMATION_TPL_HPP
#define ARIADNE_FLOAT_APPROXIMATION_TPL_HPP

#include "float_approximation.hpp"

#include "float_error.hpp"
#include "float_bounds.hpp"
#include "float_upper_bound.hpp"
#include "float_lower_bound.hpp"

#include "number_wrapper.hpp"

namespace Ariadne {

template<class F> Nat Approximation<F>::output_places = 4;

template<class F> Approximation<F>::Approximation(LowerBound<F> const& x) : Approximation<F>(x.raw()) { }
template<class F> Approximation<F>::Approximation(UpperBound<F> const& x) : Approximation<F>(x.raw()) { }
template<class F> Approximation<F>::Approximation(Bounds<F> const& x) : Approximation<F>(x.value_raw()) { }
template<class F> Approximation<F>::Approximation(Error<F> const& x) : Approximation<F>(x.raw()) { }

template<class F> Approximation<F>::Approximation(Real const& r, PR pr) : Approximation<F>(r.get(pr)) {}
template<class F> Approximation<F>::Approximation(ApproximateNumber const& y, PR pr) : Approximation<F>(y.get(pr)) {}
template<class F> Approximation<F>::operator ApproximateNumber() const { return ApproximateNumber(new NumberWrapper<Approximation<F>>(*this));}


} // namespace Ariadne

#endif
