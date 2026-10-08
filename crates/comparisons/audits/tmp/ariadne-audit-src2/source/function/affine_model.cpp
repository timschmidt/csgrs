/***************************************************************************
 *            function/affine_model.cpp
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

#include "algebra/algebra.hpp"
#include "algebra/vector.hpp"
#include "function/function.hpp"
#include "function/formula.hpp"
#include "function/taylor_model.hpp"
#include "function/affine_model.hpp"

#include "function/affine.hpp"
#include "function/taylor_function.hpp"

#include "algebra/expansion.inl.hpp"
#include "function/affine_model.tpl.hpp"


namespace Ariadne {

template class AffineModel<ApproximateTag,FloatDP>;
template class AffineModel<ApproximateTag,FloatMP>;

template class AffineModel<ValidatedTag,FloatDP>;
template class AffineModel<ValidatedTag,FloatMP>;

template struct AlgebraOperations<AffineModel<ApproximateTag,FloatDP>>;
template struct AlgebraOperations<AffineModel<ApproximateTag,FloatMP>>;

template struct AlgebraOperations<AffineModel<ValidatedTag,FloatDP>>;
template struct AlgebraOperations<AffineModel<ValidatedTag,FloatMP>>;

template AffineModel<ValidatedTag,FloatDP> affine_model(const ExactBoxType&, const ScalarMultivariateFunction<ValidatedTag>&, DoublePrecision);

} //namespace Ariadne


