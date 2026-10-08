/***************************************************************************
 *            function/function_mixin.hpp
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

#ifndef ARIADNE_FUNCTION_MIXIN_HPP
#define ARIADNE_FUNCTION_MIXIN_HPP

#include "function/function_interface.hpp"

// Adaptors for classes to conform to the Function interface.

namespace Ariadne {

typedef Differential<FloatDPApproximation> ApproximateDifferentialDP;
typedef Differential<FloatDPBounds> ValidatedDifferentialDP;
typedef UnivariateDifferential<FloatDPApproximation> ApproximateUnivariateDifferentialDP;
typedef UnivariateDifferential<FloatDPBounds> ValidatedUnivariateDifferentialDP;
typedef TaylorModel<ApproximateTag,FloatDP> ApproximateTaylorModelDP;
typedef TaylorModel<ValidatedTag,FloatDP> ValidatedTaylorModelDP;
typedef TaylorModel<ApproximateTag,FloatMP> ApproximateTaylorModelMP;
typedef TaylorModel<ValidatedTag,FloatMP> ValidatedTaylorModelMP;

typedef TaylorModel<ValidatedTag,FloatDPUpperInterval> ValidatedIntervalTaylorModelDP;

typedef Formula<ApproximateNumber> ApproximateFormula;
typedef Formula<ValidatedNumber> ValidatedFormula;
typedef Formula<EffectiveNumber> EffectiveFormula;
typedef Algebra<ApproximateNumber> ApproximateAlgebra;
typedef Algebra<ValidatedNumber> ValidatedAlgebra;
typedef Algebra<EffectiveNumber> EffectiveAlgebra;
typedef ElementaryAlgebra<ApproximateNumber> ApproximateElementaryAlgebra;
typedef ElementaryAlgebra<ValidatedNumber> ValidatedElementaryAlgebra;
typedef ElementaryAlgebra<EffectiveNumber> EffectiveElementaryAlgebra;

template<class F, class P, class SIG> class FunctionMixin { };
template<class F, class P, class... ARGS> class ScalarFunctionMixin;
template<class F, class P, class... ARGS> class VectorFunctionMixin;
template<class F, class P> using ScalarUnivariateFunctionMixin = ScalarFunctionMixin<F,P,RealScalar>;
template<class F, class P> using VectorUnivariateFunctionMixin = VectorFunctionMixin<F,P,RealScalar>;
template<class F, class P> using ScalarMultivariateFunctionMixin = ScalarFunctionMixin<F,P,RealVector>;
template<class F, class P> using VectorMultivariateFunctionMixin = VectorFunctionMixin<F,P,RealVector>;

template<class T> T* heap_copy(const T& t) { return new T(t); }
template<class T> T* heap_move(T&& t) { return new T(std::move(t)); }

template<class P, class D> ScalarFunctionInterface<P,D>* heap_copy(ScalarFunction<P,D> const& f) { return f.raw_pointer()->_clone(); }



template<class D> D make_domain(SizeType d);
template<> inline IntervalDomainType make_domain(SizeType d) { assert(d==1u); return IntervalDomainType(-inf,+inf); }
template<> inline BoxDomainType make_domain(SizeType d) { return BoxDomainType(d,IntervalDomainType(-inf,+inf)); }

template<class F, class SIG>
class FunctionMixin<F,Void,SIG>
    : public virtual FunctionInterface<Void,SIG>
{
    using D=typename SignatureTraits<SIG>::DomainType;
    using C=typename SignatureTraits<SIG>::CodomainType;
  public:
    typedef typename FunctionInterface<Void,SIG>::DomainType DomainType;
    typedef typename FunctionInterface<Void,SIG>::CodomainType CodomainType;
    typedef typename FunctionInterface<Void,SIG>::ArgumentSizeType ArgumentSizeType;
    typedef typename FunctionInterface<Void,SIG>::ResultSizeType ResultSizeType;
  protected:
    FunctionMixin() { }
    template<class X> ElementType<C,X> _base_call(const ElementType<D,X>& x) const;
  public:
    virtual ArgumentSizeType argument_size() const override = 0;
    virtual ResultSizeType result_size() const override = 0;
  private:
    virtual OutputStream& _write(OutputStream& os) const override { return os << static_cast<F const&>(*this); }
    virtual OutputStream& _repr(OutputStream& os) const override { return os << representation(static_cast<F const&>(*this)); }
};


template<class F, class SIG>
class FunctionMixin<F,ApproximateTag,SIG>
    : public virtual FunctionInterface<ApproximateTag,SIG>
    , public FunctionMixin<F,Void,SIG>
{
    using P=ApproximateTag;
    using D=typename SignatureTraits<SIG>::DomainType;
    using C=typename SignatureTraits<SIG>::CodomainType;
    template<class X> using Argument = typename SignatureTraits<SIG>::template Argument<X>;
    template<class X> using Result = typename SignatureTraits<SIG>::template Result<X>;
  public:
    virtual FunctionInterface<ApproximateTag,SIG>* _clone() const override;
    virtual FunctionInterface<ApproximateTag,SIG>* _derivative(ElementIndexType<D> i) const override;
    virtual Result<ApproximateNumber> _call(const Argument<ApproximateNumber>& x) const override;
    virtual Result<FloatDPApproximation> _call(const Argument<FloatDPApproximation>& x) const override;
    virtual Result<FloatMPApproximation> _call(const Argument<FloatMPApproximation>& x) const override;
    virtual Result<Differential<FloatDPApproximation>> _call(const Argument<Differential<FloatDPApproximation>>& x) const override;
    virtual Result<Differential<FloatMPApproximation>> _call(const Argument<Differential<FloatMPApproximation>>& x) const override;
    virtual Result<TaylorModel<ApproximateTag,FloatDP>> _call(const Argument<TaylorModel<ApproximateTag,FloatDP>>& x) const override;
    virtual Result<TaylorModel<ApproximateTag,FloatMP>> _call(const Argument<TaylorModel<ApproximateTag,FloatMP>>& x) const override;
    virtual Result<Formula<ApproximateNumber>> _call(const Argument<Formula<ApproximateNumber>>& x) const override;
    virtual Result<ElementaryAlgebra<ApproximateNumber>> _call(const Argument<ElementaryAlgebra<ApproximateNumber>>& x) const override;
};

// A wrapper for classes with non-static _compute and _compute_approx methods
template<class F, class SIG>
class FunctionMixin<F,ValidatedTag,SIG>
    : public virtual FunctionInterface<ValidatedTag,SIG>
    , public FunctionMixin<F,ApproximateTag,SIG>
{
    using D=typename SignatureTraits<SIG>::DomainType;
    using C=typename SignatureTraits<SIG>::CodomainType;
    template<class X> using Argument = typename SignatureTraits<SIG>::template Argument<X>;
    template<class X> using Result = typename SignatureTraits<SIG>::template Result<X>;
  public:
    using FunctionMixin<F,ApproximateTag,SIG>::_call;
    virtual FunctionInterface<ValidatedTag,SIG>* _clone() const override;
    virtual FunctionInterface<ValidatedTag,SIG>* _derivative(ElementIndexType<D> i) const override;
    virtual Result<ValidatedNumber> _call(const Argument<ValidatedNumber>& x) const override;
    virtual Result<FloatDPBounds> _call(const Argument<FloatDPBounds>& x) const override;
    virtual Result<FloatMPBounds> _call(const Argument<FloatMPBounds>& x) const override;
    virtual Result<Differential<FloatDPBounds>> _call(const Argument<Differential<FloatDPBounds>>& x) const override;
    virtual Result<Differential<FloatMPBounds>> _call(const Argument<Differential<FloatMPBounds>>& x) const override;
    virtual Result<TaylorModel<ValidatedTag,FloatDP>> _call(const Argument<TaylorModel<ValidatedTag,FloatDP>>& x) const override;
    virtual Result<TaylorModel<ValidatedTag,FloatMP>> _call(const Argument<TaylorModel<ValidatedTag,FloatMP>>& x) const override;
    virtual Result<TaylorModel<ValidatedTag,FloatDPBounds>> _call(const Argument<TaylorModel<ValidatedTag,FloatDPBounds>>& x) const override;
    virtual Result<TaylorModel<ValidatedTag,FloatMPBounds>> _call(const Argument<TaylorModel<ValidatedTag,FloatMPBounds>>& x) const override;
    virtual Result<TaylorModel<ValidatedTag,FloatDPUpperInterval>> _call(const Argument<TaylorModel<ValidatedTag,FloatDPUpperInterval>>& x) const override;
    virtual Result<TaylorModel<ValidatedTag,FloatMPUpperInterval>> _call(const Argument<TaylorModel<ValidatedTag,FloatMPUpperInterval>>& x) const override;
    virtual Result<Formula<ValidatedNumber>> _call(const Argument<Formula<ValidatedNumber>>& x) const override;
    virtual Result<ElementaryAlgebra<ValidatedNumber>> _call(const Argument<ElementaryAlgebra<ValidatedNumber>>& x) const override;

    virtual Result<ValidatedScalarMultivariateFunction> _call(const Argument<ValidatedScalarMultivariateFunction>& x) const override;

};

// A wrapper for classes with non-static _compute and _compute_approx methods
template<class F, class SIG>
class FunctionMixin<F,EffectiveTag,SIG>
    : public virtual FunctionInterface<EffectiveTag,SIG>
    , public FunctionMixin<F,ValidatedTag,SIG>
{
    using D=typename SignatureTraits<SIG>::DomainType;
    using C=typename SignatureTraits<SIG>::CodomainType;
    template<class X> using Argument = typename SignatureTraits<SIG>::template Argument<X>;
    template<class X> using Result = typename SignatureTraits<SIG>::template Result<X>;
  public:
    using FunctionMixin<F,ValidatedTag,SIG>::_call;
    virtual FunctionInterface<EffectiveTag,SIG>* _clone() const override;
    virtual FunctionInterface<EffectiveTag,SIG>* _derivative(ElementIndexType<D> i) const override;
    virtual Result<EffectiveNumber> _call(const Argument<EffectiveNumber>& x) const override;
    virtual Result<Real> _call(const Argument<Real>& x) const override;
    virtual Result<ElementaryAlgebra<Real>> _call(const Argument<ElementaryAlgebra<Real>>& x) const override;
    virtual Result<Formula<Real>> _call(const Argument<Formula<Real>>& x) const override;
    virtual Result<ElementaryAlgebra<EffectiveNumber>> _call(const Argument<ElementaryAlgebra<EffectiveNumber>>& x) const override;
    virtual Result<Formula<EffectiveNumber>> _call(const Argument<Formula<EffectiveNumber>>& x) const override;
};





template<class F, class P, class SIG> class FunctionGetterMixin;

template<class F, class P, class... ARGS> class FunctionGetterMixin<F,P,RealScalar(ARGS...)> {
};

template<class F, class P, class... ARGS> class FunctionGetterMixin<F,P,RealVector(ARGS...)>
    : public virtual VectorOfFunctionInterface<P,ARGS...>
{
    virtual ScalarFunctionInterface<P,ARGS...>* _get(SizeType i) const override;
};


template<class F, class P, class... ARGS> class ScalarFunctionMixin
    : public FunctionMixin<F,P,RealScalar(ARGS...)> { };

template<class F, class P, class... ARGS> class VectorFunctionMixin
    : public FunctionMixin<F,P,RealVector(ARGS...)>
    , public FunctionGetterMixin<F,P,RealVector(ARGS...)>
{
};

template<class F, class P, class... ARGS> auto
FunctionGetterMixin<F,P,RealVector(ARGS...)>::_get(SizeType i) const -> ScalarFunctionInterface<P,ARGS...>* {
    auto fi=static_cast<F const&>(*this)[i]; return heap_copy(fi); }


template<class F,class SIG> FunctionInterface<ApproximateTag,SIG>* FunctionMixin<F,ApproximateTag,SIG>::_clone() const {
    return new F(static_cast<const F&>(*this)); }
template<class F,class SIG> FunctionInterface<ValidatedTag,SIG>* FunctionMixin<F,ValidatedTag,SIG>::_clone() const {
    return new F(static_cast<const F&>(*this)); }
template<class F,class SIG> FunctionInterface<EffectiveTag,SIG>* FunctionMixin<F,EffectiveTag,SIG>::_clone() const {
    return new F(static_cast<const F&>(*this)); }

template<class T> inline T* _heap_move(T&& t) { return new T(std::forward<T>(t)); }
template<class P, class SIG> inline FunctionInterface<P,SIG>* _heap_move(Function<P,SIG>&& f) {
    return f.raw_pointer()->_clone(); }

template<class F,class SIG> FunctionInterface<ApproximateTag,SIG>* FunctionMixin<F,ApproximateTag,SIG>::_derivative(ElementIndexType<D> j) const {
    if constexpr (BaseOf<FunctionInterface<ApproximateTag,SIG>,decltype(derivative(static_cast<const F&>(*this),j))>) {
        return _heap_move(derivative(static_cast<const F&>(*this),j)); }
    else { assert(false); std::abort(); } }
template<class F,class SIG> FunctionInterface<ValidatedTag,SIG>* FunctionMixin<F,ValidatedTag,SIG>::_derivative(ElementIndexType<D> j) const {
    if constexpr (BaseOf<FunctionInterface<ValidatedTag,SIG>,decltype(derivative(static_cast<const F&>(*this),j))>) {
        return _heap_move(derivative(static_cast<const F&>(*this),j)); }
    else { assert(false); std::abort(); } }
template<class F,class SIG> FunctionInterface<EffectiveTag,SIG>* FunctionMixin<F,EffectiveTag,SIG>::_derivative(ElementIndexType<D> j) const {
    if constexpr (BaseOf<FunctionInterface<EffectiveTag,SIG>,decltype(derivative(static_cast<const F&>(*this),j))>) {
        return _heap_move(derivative(static_cast<const F&>(*this),j)); }
    else { assert(false); std::abort(); } }

} // namespace Ariadne

#endif // ARIADNE_FUNCTION_MIXIN_HPP
