/***************************************************************************
 *            function_concepts.hpp
 *
 *  Copyright  2022  Pieter Collins
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

#ifndef ARIADNE_FUNCTION_CONCEPTS_HPP
#define ARIADNE_FUNCTION_CONCEPTS_HPP


namespace Ariadne {

template<class P, class F> class TaylorModel;
template<class F> using ApproximateTaylorModel = TaylorModel<ApproximateTag,F>;
template<class F> using ValidatedTaylorModel = TaylorModel<ValidatedTag,F>;

template<class Y> class ElementaryAlgebra;

using ScalarUnivariate = RealScalar(RealScalar);
using VectorUnivariate = RealVector(RealScalar);
using ScalarMultivariate = RealScalar(RealVector);
using VectorMultivariate = RealVector(RealVector);


template<class F, template<class>class A, template<class>class R, class T> concept CallableOn = requires(F f, A<T> a) {
    { f(a) } -> SameAs<R<T>>;
};

template<class F, template<class>class Argument, template<class> class Result> concept IsApproximateCallable = requires(F const f) {
    { f(declval<Argument<FloatDPApproximation>>()) } -> SameAs<Result<FloatDPApproximation>>;
    { f(declval<Argument<FloatMPApproximation>>()) } -> SameAs<Result<FloatMPApproximation>>;
    { f(declval<Argument<Differential<FloatDPApproximation>>>()) } -> SameAs<Result<Differential<FloatDPApproximation>>>;
    { f(declval<Argument<Differential<FloatMPApproximation>>>()) } -> SameAs<Result<Differential<FloatMPApproximation>>>;
    { f(declval<Argument<TaylorModel<ApproximateTag,FloatDP>>>()) } -> SameAs<Result<TaylorModel<ApproximateTag,FloatDP>>>;
    { f(declval<Argument<TaylorModel<ApproximateTag,FloatMP>>>()) } -> SameAs<Result<TaylorModel<ApproximateTag,FloatMP>>>;

    { f(declval<Argument<Formula<ApproximateNumber>>>()) } -> SameAs<Result<Formula<ApproximateNumber>>>;
    { f(declval<Argument<ElementaryAlgebra<ApproximateNumber>>>()) } -> SameAs<Result<ElementaryAlgebra<ApproximateNumber>>>;
};

template<class F, template<class>class Argument, template<class> class Result>
concept IsValidatedCallable = IsApproximateCallable<F,Argument,Result> and requires(F const f) {
    { f(declval<Argument<FloatDPBounds>>()) } -> SameAs<Result<FloatDPBounds>>;
    { f(declval<Argument<FloatMPBounds>>()) } -> SameAs<Result<FloatMPBounds>>;
    { f(declval<Argument<Differential<FloatDPBounds>>>()) } -> SameAs<Result<Differential<FloatDPBounds>>>;
    { f(declval<Argument<Differential<FloatMPBounds>>>()) } -> SameAs<Result<Differential<FloatMPBounds>>>;
    { f(declval<Argument<ValidatedTaylorModel<FloatDP>>>()) } -> SameAs<Result<ValidatedTaylorModel<FloatDP>>>;
    { f(declval<Argument<ValidatedTaylorModel<FloatMP>>>()) } -> SameAs<Result<ValidatedTaylorModel<FloatMP>>>;
    { f(declval<Argument<ValidatedTaylorModel<FloatDPUpperInterval>>>()) } -> SameAs<Result<ValidatedTaylorModel<FloatDPUpperInterval>>>;
    { f(declval<Argument<ValidatedTaylorModel<FloatMPUpperInterval>>>()) } -> SameAs<Result<ValidatedTaylorModel<FloatMPUpperInterval>>>;

    { f(declval<Argument<Formula<ValidatedNumber>>>()) } -> SameAs<Result<Formula<ValidatedNumber>>>;
    { f(declval<Argument<ElementaryAlgebra<ValidatedNumber>>>()) } -> SameAs<Result<ElementaryAlgebra<ValidatedNumber>>>;

    { f(declval<Argument<FloatDP>>()) } -> ConvertibleTo<Result<FloatDPBounds>>;
    { f(declval<Argument<FloatMP>>()) } -> ConvertibleTo<Result<FloatMPBounds>>;
};

template<class F, template<class>class Argument, template<class> class Result>
concept IsEffectiveCallable = IsValidatedCallable<F,Argument,Result> and requires(F const f) {
    { f(declval<Argument<Real>>()) } -> SameAs<Result<Real>>;
    { f(declval<Argument<ElementaryAlgebra<Real>>>()) } -> SameAs<Result<ElementaryAlgebra<Real>>>;
    { f(declval<Argument<Formula<Real>>>()) } -> SameAs<Result<Formula<Real>>>;
    { f(declval<Argument<ElementaryAlgebra<EffectiveNumber>>>()) } -> SameAs<Result<ElementaryAlgebra<EffectiveNumber>>>;
    { f(declval<Argument<Formula<EffectiveNumber>>>()) } -> SameAs<Result<Formula<EffectiveNumber>>>;
};

template<class F, class P, template<class>class Argument, template<class>class Result>
concept IsCallable
    = ((not Same<P,ApproximateTag>) or IsApproximateCallable<F,Argument,Result>)
        and ((not Same<P,ValidatedTag>) or IsValidatedCallable<F,Argument,Result>)
        and ((not Same<P,EffectiveTag>) or IsEffectiveCallable<F,Argument,Result>);

template<class F, class P, class SIG>
concept IsCallableSignature = IsCallable<F,P, SignatureTraits<SIG>::template Argument, SignatureTraits<SIG>::template Result>;

template<class F, class SIG> concept IsBaseFunction = requires(F const& f)
{
    { f.argument_size() } -> SameAs<typename SignatureTraits<SIG>::ArgumentSizeType>;
    { f.result_size() } -> SameAs<typename SignatureTraits<SIG>::ResultSizeType>;
    { f.domain() } -> SameAs<typename SignatureTraits<SIG>::EntireDomainType>;
};

template<class F, class SIG> concept IsApproximateFunction = IsBaseFunction<F,SIG> and IsCallableSignature<F,ApproximateTag,SIG>;
template<class F, class SIG> concept IsValidatedFunction = IsApproximateFunction<F,SIG> and IsCallableSignature<F,ValidatedTag,SIG>;
template<class F, class SIG> concept IsEffectiveFunction = IsValidatedFunction<F,SIG> and IsCallableSignature<F,EffectiveTag,SIG>;

template<class F, class P, class SIG> concept AFunction
    = ((not Same<P,ApproximateTag>) or IsApproximateFunction<F,SIG>)
        and ((not Same<P,ValidatedTag>) or IsValidatedFunction<F,SIG>)
        and ((not Same<P,EffectiveTag>) or IsEffectiveFunction<F,SIG>);

// FIXME: Provided to avoid AFunction concept checking conformance before argument classes are defined.
template<class F, class SIG> concept IsFunctionClass = Same<F,Function<ApproximateTag,SIG>> or Same<F,Function<ValidatedTag,SIG>> or Same<F,Function<EffectiveTag,SIG>> or Same<F,FunctionPatch<ApproximateTag,SIG>> or Same<F,FunctionPatch<ValidatedTag,SIG>>;


template<class F, class P, class SIG> concept IsEntireFunction = AFunction<F,P,SIG> and requires(F const& f) {
    { f.domain() } -> ConvertibleTo<typename SignatureTraits<SIG>::EntireDomainType>;
};

template<class F, class P, class SIG> concept IsFunctionPatch = AFunction<F,P,SIG> and requires(F const& f) {
    { f.domain() } -> ConvertibleTo<typename SignatureTraits<SIG>::BoundedDomainType>;
};



} // namespace Ariadne

#endif // ARIADNE_FUNCTION_CONCEPTS_HPP
