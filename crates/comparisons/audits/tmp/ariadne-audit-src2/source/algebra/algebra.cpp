/***************************************************************************
 *            algebra/algebra.cpp
 *
 *  Copyright  2011-20  Pieter Collins
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

#include "algebra.hpp"
#include "algebra_concepts.hpp"
#include "algebra_mixin.hpp"
#include "algebra_operations.tpl.hpp"

#include "algebra_wrapper.hpp"

namespace Ariadne {

// FIXME: Resolve _copy and _create_copy in Algebra / Handle
template class Algebra<ValidatedNumber>;

static_assert(AnAlgebraOver<AlgebraArchetype<Real>,Real>);
//template class AlgebraWrapper<AlgebraArchetype<Real>,Real>;

static_assert(AnInplaceAlgebraOver<InplaceAlgebraArchetype<Real>,Real>);
//template class AlgebraWrapper<InplaceAlgebraArchetype<Real>,Real>;

} // namespace Ariadne


#include "differential.hpp"

namespace Ariadne {

template class AlgebraWrapper<Differential<FloatDPBounds>,FloatDPBounds>;
template class AlgebraWrapper<Differential<FloatMPBounds>,FloatMPBounds>;

} // namespace Ariadne
