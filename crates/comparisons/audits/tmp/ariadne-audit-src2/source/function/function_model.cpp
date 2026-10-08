/***************************************************************************
 *            function/function_model.cpp
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

#include "function/function_model.hpp"
#include "function/function_model.tpl.hpp"
#include "function/taylor_model.hpp"

#include "algebra/algebra.hpp"

#include "function/formula.hpp"

namespace Ariadne {

template class FunctionModelFactoryInterface<ValidatedTag,DoublePrecision,DoublePrecision>;
template class FunctionModelFactoryInterface<ValidatedTag,MultiplePrecision,MultiplePrecision>;

template class FunctionModel<ValidatedTag,RealScalar(RealScalar),DoublePrecision>;
template class FunctionModel<ValidatedTag,RealScalar(RealScalar),MultiplePrecision>;
template class FunctionModel<ValidatedTag,RealVector(RealScalar),DoublePrecision>;
template class FunctionModel<ValidatedTag,RealVector(RealScalar),MultiplePrecision>;

template class FunctionModel<ValidatedTag,RealScalar(RealVector),DoublePrecision>;
template class FunctionModel<ValidatedTag,RealScalar(RealVector),MultiplePrecision>;
template class FunctionModel<ValidatedTag,RealVector(RealVector),DoublePrecision>;
template class FunctionModel<ValidatedTag,RealVector(RealVector),MultiplePrecision>;

} // namespace Ariadne
