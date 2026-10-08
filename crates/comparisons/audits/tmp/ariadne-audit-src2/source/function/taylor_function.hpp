/***************************************************************************
 *            function/taylor_function.hpp
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

/*! \file function/taylor_function.hpp
 *  \brief Over-approximations of functions based on Taylor expansions.
 */

#ifndef ARIADNE_TAYLOR_FUNCTION_HPP
#define ARIADNE_TAYLOR_FUNCTION_HPP

#include <iosfwd>
#include "utility/container.hpp"
#include "utility/exceptions.hpp"
#include "numeric/numeric.hpp"
#include "algebra/vector.hpp"
#include "function/taylor_model.hpp"

#include "function/function_interface.hpp"
#include "function/function_mixin.hpp"
#include "function/function_model.hpp"
#include "function/scaled_function_patch.hpp"

namespace Ariadne {

template<class P, class F> class TaylorModel;
class TaylorFunctionFactory;

//! \ingroup FunctionModelSubModule
//! \name Template shorthands and type synonyms for Taylor function models
//!@{
//! \ingroup FunctionModelSubModule
template<class P, class F> using ScalarMultivariateTaylorFunctionModel = ScalarScaledFunctionPatch<TaylorModel<P,F>>; //!< <p/> \ingroup FunctionModelSubModule
template<class P, class F> using VectorMultivariateTaylorFunctionModel = VectorScaledFunctionPatch<TaylorModel<P,F>>; //!< <p/> \ingroup FunctionModelSubModule
template<class F> using ValidatedScalarMultivariateTaylorFunctionModel = ScalarScaledFunctionPatch<ValidatedTaylorModel<F>>; //!< <p/> \ingroup FunctionModelSubModule
template<class F> using ValidatedVectorMultivariateTaylorFunctionModel = VectorScaledFunctionPatch<ValidatedTaylorModel<F>>; //!< <p/> \ingroup FunctionModelSubModule
template<class F> using ApproximateScalarMultivariateTaylorFunctionModel = ScalarScaledFunctionPatch<ApproximateTaylorModel<F>>; //!< <p/> \ingroup FunctionModelSubModule
template<class F> using ApproximateVectorMultivariateTaylorFunctionModel = VectorScaledFunctionPatch<ApproximateTaylorModel<F>>; //!< <p/> \ingroup FunctionModelSubModule

using ValidatedScalarMultivariateTaylorFunctionModelDP = ScalarScaledFunctionPatch<ValidatedTaylorModelDP>; //!< <p/> \ingroup FunctionModelSubModule
using ValidatedVectorMultivariateTaylorFunctionModelDP = VectorScaledFunctionPatch<ValidatedTaylorModelDP>; //!< <p/> \ingroup FunctionModelSubModule
using ApproximateScalarMultivariateTaylorFunctionModelDP = ScalarScaledFunctionPatch<ApproximateTaylorModelDP>; //!< <p/> \ingroup FunctionModelSubModule
using ApproximateVectorMultivariateTaylorFunctionModelDP = VectorScaledFunctionPatch<ApproximateTaylorModelDP>; //!< <p/> \ingroup FunctionModelSubModule

using ValidatedScalarMultivariateTaylorFunctionModelMP = ScalarScaledFunctionPatch<ValidatedTaylorModelMP>; //!< <p/> \ingroup FunctionModelSubModule
using ValidatedVectorMultivariateTaylorFunctionModelMP = VectorScaledFunctionPatch<ValidatedTaylorModelMP>; //!< <p/> \ingroup FunctionModelSubModule
using ApproximateScalarMultivariateTaylorFunctionModelMP = ScalarScaledFunctionPatch<ApproximateTaylorModelMP>; //!< <p/> \ingroup FunctionModelSubModule
using ApproximateVectorMultivariateTaylorFunctionModelMP = VectorScaledFunctionPatch<ApproximateTaylorModelMP>; //!< <p/> \ingroup FunctionModelSubModule

//! \ingroup FunctionModelSubModule
template<class P, class F> using ScalarMultivariateBoundsTaylorFunctionModel = ScalarScaledFunctionPatch<TaylorModel<P,Bounds<F>>>; //!< <p/> \ingroup FunctionModelSubModule
template<class P, class F> using VectorMultivariateBoundsTaylorFunctionModel = VectorScaledFunctionPatch<TaylorModel<P,Bounds<F>>>; //!< <p/> \ingroup FunctionModelSubModule
template<class F> using ValidatedScalarMultivariateBoundsTaylorFunctionModel = ScalarScaledFunctionPatch<ValidatedBoundsTaylorModel<F>>; //!< <p/> \ingroup FunctionModelSubModule
template<class F> using ValidatedVectorMultivariateBoundsTaylorFunctionModel = VectorScaledFunctionPatch<ValidatedBoundsTaylorModel<F>>; //!< <p/> \ingroup FunctionModelSubModule

using ValidatedScalarMultivariateBoundsTaylorFunctionModelDP = ScalarScaledFunctionPatch<ValidatedBoundsTaylorModelDP>; //!< <p/> \ingroup FunctionModelSubModule
using ValidatedVectorMultivariateBoundsTaylorFunctionModelDP = VectorScaledFunctionPatch<ValidatedBoundsTaylorModelDP>; //!< <p/> \ingroup FunctionModelSubModule

using ValidatedScalarMultivariateBoundsTaylorFunctionModelMP = ScalarScaledFunctionPatch<ValidatedBoundsTaylorModelMP>; //!< <p/> \ingroup FunctionModelSubModule
using ValidatedVectorMultivariateBoundsTaylorFunctionModelMP = VectorScaledFunctionPatch<ValidatedBoundsTaylorModelMP>; //!< //!@}

class TaylorFunctionFactory
    : public ScaledFunctionPatchFactory<TaylorModel<ValidatedTag,FloatDP>>
{
    typedef TaylorModel<ValidatedTag,FloatDP> M;
    typedef BoxDomainType D;
  public:
    typedef typename M::SweeperType SweeperType;
    using ScaledFunctionPatchFactory<M>::ScaledFunctionPatchFactory;
    explicit TaylorFunctionFactory(SweeperType sweeper) : ScaledFunctionPatchFactory<M>(sweeper) { }
    SweeperType sweeper() const { return this->properties(); }
    friend OutputStream& operator<<(OutputStream& os, TaylorFunctionFactory const& factory) { return os << "TaylorFunctionFactory( sweeper=" << factory.sweeper() << " )"; }
};

FunctionModelFactoryInterface<ValidatedTag,DoublePrecision>* make_taylor_function_factory();
FunctionModelFactoryInterface<ValidatedTag,DoublePrecision>* make_taylor_function_factory(Sweeper<FloatDP> const& sweeper);
FunctionPatchFactoryInterface<ValidatedTag>* make_taylor_function_patch_factory();
FunctionPatchFactoryInterface<ValidatedTag>* make_taylor_function_patch_factory(Sweeper<FloatDP> const& sweeper);

} // namespace Ariadne

#endif // ARIADNE_TAYLOR_FUNCTION_HPP
