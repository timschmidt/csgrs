/***************************************************************************
 *            function_mixin.tpl.hpp
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

#ifndef ARIADNE_FUNCTION_MIXIN_TCC
#define ARIADNE_FUNCTION_MIXIN_TCC

#include "numeric/numeric.hpp"
#include "algebra/vector.hpp"
#include "algebra/differential.hpp"
#include "function/taylor_model.hpp"
#include "function/formula.hpp"
#include "algebra/algebra.hpp"

#include "function/function_mixin.hpp"

namespace Ariadne {

template<class X> Scalar<X> create_result(SizeOne n, X z) { return z; }
template<class X> Vector<X> create_result(SizeType n, X z) { return Vector<X>(n,z); }

template<class F, class SIG> template<class X> auto
FunctionMixin<F,Void,SIG>::_base_call(const ElementType<D,X>& x) const -> ElementType<C,X> {
    const F& f=static_cast<const F&>(*this);
    return f(x);
}

/*
template<class F, class D> template<class X> auto
FunctionMixin<F,Void,D,IntervalDomainType>::_base_call(const ElementType<D,X>& x) const -> X {
    ElementType<IntervalDomainType,X> r=create_result<X>(this->codomain().dimension(),zero_element(x));
    static_cast<const F*>(this)->_call(r,x); return r;
}
*/

template<class F,class SIG> auto
FunctionMixin<F,ApproximateTag,SIG>::_call(const Argument<ApproximateNumber>& x) const -> Result<ApproximateNumber> {
    return this->_base_call(x); }
template<class F,class SIG> auto
FunctionMixin<F,ApproximateTag,SIG>::_call(const Argument<FloatDPApproximation>& x) const -> Result<FloatDPApproximation> {
    return this->_base_call(x); }
template<class F,class SIG> auto
FunctionMixin<F,ApproximateTag,SIG>::_call(const Argument<FloatMPApproximation>& x) const -> Result<FloatMPApproximation> {
    return this->_base_call(x); }
template<class F,class SIG> auto
FunctionMixin<F,ApproximateTag,SIG>::_call(const Argument<Differential<FloatDPApproximation>>& x) const -> Result<Differential<FloatDPApproximation>> {
    return this->_base_call(x); }
template<class F,class SIG> auto
FunctionMixin<F,ApproximateTag,SIG>::_call(const Argument<Differential<FloatMPApproximation>>& x) const -> Result<Differential<FloatMPApproximation>> {
    return this->_base_call(x); }
template<class F,class SIG> auto
FunctionMixin<F,ApproximateTag,SIG>::_call(const Argument<TaylorModel<ApproximateTag,FloatDP>>& x) const -> Result<TaylorModel<ApproximateTag,FloatDP>> {
    return this->_base_call(x); }
template<class F,class SIG> auto
FunctionMixin<F,ApproximateTag,SIG>::_call(const Argument<TaylorModel<ApproximateTag,FloatMP>>& x) const -> Result<TaylorModel<ApproximateTag,FloatMP>> {
    return this->_base_call(x); }
template<class F,class SIG> auto
FunctionMixin<F,ApproximateTag,SIG>::_call(const Argument<ElementaryAlgebra<ApproximateNumber>>& x) const -> Result<ElementaryAlgebra<ApproximateNumber>> {
    return this->_base_call(x); }
template<class F,class SIG> auto
FunctionMixin<F,ApproximateTag,SIG>::_call(const Argument<Formula<ApproximateNumber>>& x) const -> Result<Formula<ApproximateNumber>> {
    return this->_base_call(x); }

template<class F,class SIG> auto
FunctionMixin<F,ValidatedTag,SIG>::_call(const Argument<ValidatedNumber>& x) const -> Result<ValidatedNumber> {
    return this->_base_call(x); }
template<class F,class SIG> auto
FunctionMixin<F,ValidatedTag,SIG>::_call(const Argument<FloatDPBounds>& x) const -> Result<FloatDPBounds> {
    return this->_base_call(x); }
template<class F,class SIG> auto
FunctionMixin<F,ValidatedTag,SIG>::_call(const Argument<FloatMPBounds>& x) const -> Result<FloatMPBounds> {
    return this->_base_call(x); }
template<class F,class SIG> auto
FunctionMixin<F,ValidatedTag,SIG>::_call(const Argument<Differential<FloatDPBounds>>& x) const -> Result<Differential<FloatDPBounds>> {
    return this->_base_call(x); }
template<class F,class SIG> auto
FunctionMixin<F,ValidatedTag,SIG>::_call(const Argument<Differential<FloatMPBounds>>& x) const -> Result<Differential<FloatMPBounds>> {
    return this->_base_call(x); }
template<class F,class SIG> auto
FunctionMixin<F,ValidatedTag,SIG>::_call(const Argument<TaylorModel<ValidatedTag,FloatDP>>& x) const -> Result<TaylorModel<ValidatedTag,FloatDP>> {
    return this->_base_call(x); }
template<class F,class SIG> auto
FunctionMixin<F,ValidatedTag,SIG>::_call(const Argument<TaylorModel<ValidatedTag,FloatMP>>& x) const -> Result<TaylorModel<ValidatedTag,FloatMP>> {
    return this->_base_call(x); }
template<class F,class SIG> auto
FunctionMixin<F,ValidatedTag,SIG>::_call(const Argument<TaylorModel<ValidatedTag,FloatDPBounds>>& x) const -> Result<TaylorModel<ValidatedTag,FloatDPBounds>> {
    return this->_base_call(x); }
template<class F,class SIG> auto
FunctionMixin<F,ValidatedTag,SIG>::_call(const Argument<TaylorModel<ValidatedTag,FloatMPBounds>>& x) const -> Result<TaylorModel<ValidatedTag,FloatMPBounds>> {
    return this->_base_call(x); }
template<class F,class SIG> auto
FunctionMixin<F,ValidatedTag,SIG>::_call(const Argument<TaylorModel<ValidatedTag,FloatDPUpperInterval>>& x) const -> Result<TaylorModel<ValidatedTag,FloatDPUpperInterval>> {
    return this->_base_call(x); }
template<class F,class SIG> auto
FunctionMixin<F,ValidatedTag,SIG>::_call(const Argument<TaylorModel<ValidatedTag,FloatMPUpperInterval>>& x) const -> Result<TaylorModel<ValidatedTag,FloatMPUpperInterval>> {
    return this->_base_call(x); }
template<class F,class SIG> auto
FunctionMixin<F,ValidatedTag,SIG>::_call(const Argument<ElementaryAlgebra<ValidatedNumber>>& x) const -> Result<ElementaryAlgebra<ValidatedNumber>> {
    return this->_base_call(x); }
template<class F,class SIG> auto
FunctionMixin<F,ValidatedTag,SIG>::_call(const Argument<Formula<ValidatedNumber>>& x) const -> Result<Formula<ValidatedNumber>> {
    return this->_base_call(x); }

template<class F,class SIG> auto
FunctionMixin<F,ValidatedTag,SIG>::_call(const Argument<ValidatedScalarMultivariateFunction>& x) const -> Result<ValidatedScalarMultivariateFunction> {
    return this->_base_call(x); }

template<class F,class SIG> auto
FunctionMixin<F,EffectiveTag,SIG>::_call(const Argument<EffectiveNumber>& x) const -> Result<EffectiveNumber> {
    return this->_base_call(x); }
template<class F,class SIG> auto
FunctionMixin<F,EffectiveTag,SIG>::_call(const Argument<Real>& x) const -> Result<Real> {
    return this->_base_call(x); }
template<class F,class SIG> auto
FunctionMixin<F,EffectiveTag,SIG>::_call(const Argument<ElementaryAlgebra<Real>>& x) const -> Result<ElementaryAlgebra<Real>> {
    return this->_base_call(x); }
template<class F,class SIG> auto
FunctionMixin<F,EffectiveTag,SIG>::_call(const Argument<Formula<Real>>& x) const -> Result<Formula<Real>> {
    return this->_base_call(x); }
template<class F,class SIG> auto
FunctionMixin<F,EffectiveTag,SIG>::_call(const Argument<ElementaryAlgebra<EffectiveNumber>>& x) const -> Result<ElementaryAlgebra<EffectiveNumber>> {
    return this->_base_call(x); }
template<class F,class SIG> auto
FunctionMixin<F,EffectiveTag,SIG>::_call(const Argument<Formula<EffectiveNumber>>& x) const -> Result<Formula<EffectiveNumber>> {
    return this->_base_call(x); }

} // namespace Ariadne


#include "function/function_wrapper.hpp"

namespace Ariadne {

template<class F, class P, class... ARGS> auto
FunctionWrapperGetterMixin<F,P,RealVector(ARGS...)>::_get(SizeType i) const -> ScalarFunctionInterface<P,ARGS...>* {
    auto fp=static_cast<F const*>(this);
    if (not fp) { ARIADNE_THROW(std::runtime_error,"_get","bad cast"); }
    auto fi=(*fp)[i];
    if constexpr (DerivedFrom<decltype(fi),ScalarFunctionInterface<P,ARGS...>>) { return heap_copy(fi); }
    else { return make_function_wrapper<P,RealScalar(ARGS...)>(fi); }
}

} // namespace Ariadne

#endif
