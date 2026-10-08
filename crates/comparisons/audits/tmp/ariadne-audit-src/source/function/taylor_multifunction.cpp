/***************************************************************************
 *            taylor_multifunction.cpp
 *
 *  Copyright 2008-12  Pieter Collins
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include "../config.hpp"

#include "taylor_multifunction.hpp"
#include "multifunction.hpp"

#include "../numeric/numeric.hpp"
#include "../algebra/vector.hpp"
#include "../algebra/matrix.hpp"
#include "../algebra/algebra.hpp"
#include "../algebra/multi_index.hpp"
#include "../function/polynomial.hpp"
#include "../algebra/differential.hpp"
#include "../algebra/evaluate.hpp"
#include "../geometry/interval.hpp"
#include "../geometry/box.hpp"
#include "../geometry/set_wrapper.hpp"

#include "taylor_model.tpl.hpp"
#include "scaled_function_patch.tpl.hpp"
#include "../geometry/box.tpl.hpp"



namespace Ariadne {

template<class F> auto ScaledFunctionPatchMixin<ValidatedIntervalTaylorModel<F>>::
_call(Argument<Number<P>> const& x) const -> CompactSet<P,RES>
{
    Argument<Bounds<F>> cx(x,this->_upcast().precision());
    Interval<UpperBound<F>> ivl=this->_upcast().operator()(cx);
    IntervalSet<UpperBound<F>> ivls(std::move(ivl));
    return CompactSet<P,Real>(std::make_shared<CompactSetWrapper<IntervalSet<UpperBound<F>>,ValidatedTag,Real>>(ivls));
}

template<class F> auto VectorScaledFunctionPatchMixin<ValidatedIntervalTaylorModel<F>>::
_call(Argument<Number<P>> const& x) const -> CompactSet<P,RES>
{
    typedef Box<Interval<UpperBound<F>>> BoxType;
    Vector<Bounds<F>> cx(x,this->_upcast().precision());
    BoxType rbx=this->_upcast().operator()(cx);
    return CompactSet<P,RealVector>(std::make_shared<CompactSetWrapper<BoxType,ValidatedTag,RealVector>>(rbx));
}

template class ScaledFunctionPatch<ValidatedIntervalTaylorModelDP>;
template class VectorScaledFunctionPatch<ValidatedIntervalTaylorModelDP>;
template class ScaledFunctionPatch<ValidatedIntervalTaylorModelMP>;
template class VectorScaledFunctionPatch<ValidatedIntervalTaylorModelMP>;

template auto ScaledFunctionPatchMixin<ValidatedIntervalTaylorModel<FloatDP>>::
_call(Argument<Number<P>> const& x) const -> CompactSet<P,RES>;
template auto ScaledFunctionPatchMixin<ValidatedIntervalTaylorModel<FloatMP>>::
_call(Argument<Number<P>> const& x) const -> CompactSet<P,RES>;

template auto VectorScaledFunctionPatchMixin<ValidatedIntervalTaylorModel<FloatDP>>::
_call(Argument<Number<P>> const& x) const -> CompactSet<P,RES>;
template auto VectorScaledFunctionPatchMixin<ValidatedIntervalTaylorModel<FloatMP>>::
_call(Argument<Number<P>> const& x) const -> CompactSet<P,RES>;

} // namespace Ariadne

