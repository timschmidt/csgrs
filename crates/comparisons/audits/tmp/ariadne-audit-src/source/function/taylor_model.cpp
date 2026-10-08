/***************************************************************************
 *            function/taylor_model.cpp
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

#include "numeric/numeric.hpp"
#include "function/taylor_model.tpl.hpp"

namespace Ariadne {

template<> String class_name<UnknownError<FloatDP>>() { return "UnknownError<FloatDP>"; }
template<> String class_name<UnknownError<FloatMP>>() { return "UnknownError<FloatMP>"; }


template class Series<FloatDPBounds>;
template class Series<FloatMPBounds>;

template class SweeperBase<FloatDP>;
template class SweeperBase<FloatMP>;
template class RelativeSweeperBase<FloatDP>;
template class RelativeSweeperBase<FloatMP>;

template class TaylorModel<ValidatedTag,FloatDP>;
template struct AlgebraOperations<TaylorModel<ValidatedTag,FloatDP>>;
template class NormedAlgebraOperations<TaylorModel<ValidatedTag,FloatDP>>;

template class TaylorModel<ValidatedTag,FloatDPBounds>;
template struct AlgebraOperations<TaylorModel<ValidatedTag,FloatDPBounds>>;
template class NormedAlgebraOperations<TaylorModel<ValidatedTag,FloatDPBounds>>;

template class TaylorModel<ApproximateTag,FloatDP>;
template struct AlgebraOperations<TaylorModel<ApproximateTag,FloatDP>>;
template class NormedAlgebraOperations<TaylorModel<ApproximateTag,FloatDP>>;


template class TaylorModel<ValidatedTag,FloatMP>;
template struct AlgebraOperations<TaylorModel<ValidatedTag,FloatMP>>;
template class NormedAlgebraOperations<TaylorModel<ValidatedTag,FloatMP>>;

template class TaylorModel<ValidatedTag,FloatMPBounds>;
template struct AlgebraOperations<TaylorModel<ValidatedTag,FloatMPBounds>>;
template class NormedAlgebraOperations<TaylorModel<ValidatedTag,FloatMPBounds>>;

template class TaylorModel<ApproximateTag,FloatMP>;
template struct AlgebraOperations<TaylorModel<ApproximateTag,FloatMP>>;
template class NormedAlgebraOperations<TaylorModel<ApproximateTag,FloatMP>>;


template class TaylorModel<ValidatedTag,FloatDPUpperInterval>;
template struct AlgebraOperations<TaylorModel<ValidatedTag,FloatDPUpperInterval>>;
template class NormedAlgebraOperations<TaylorModel<ValidatedTag,FloatDPUpperInterval>>;

template class TaylorModel<ValidatedTag,FloatMPUpperInterval>;
template struct AlgebraOperations<TaylorModel<ValidatedTag,FloatMPUpperInterval>>;
template class NormedAlgebraOperations<TaylorModel<ValidatedTag,FloatMPUpperInterval>>;

//template TaylorModel<ValidatedTag,FloatDP> value_coefficients(TaylorModel<ValidatedTag,FloatDPBounds> const&);
//template TaylorModel<ValidatedTag,FloatDPBounds> exact_coefficients(TaylorModel<ValidatedTag,FloatDPBounds> const&);
//template TaylorModel<ValidatedTag,FloatMP> value_coefficients(TaylorModel<ValidatedTag,FloatMPBounds> const&);
//template TaylorModel<ValidatedTag,FloatMPBounds> exact_coefficients(TaylorModel<ValidatedTag,FloatMPBounds> const&);



} // namespace Ariadne
