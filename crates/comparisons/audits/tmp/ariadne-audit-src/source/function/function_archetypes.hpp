/***************************************************************************
 *            function_archetypes.hpp
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

#ifndef ARIADNE_FUNCTION_ARCHETYPES_HPP
#define ARIADNE_FUNCTION_ARCHETYPES_HPP


namespace Ariadne {

template<class P, class SIG> class FunctionArchetype;

template<class SIG> class FunctionArchetype<ApproximateTag,SIG>
{
    using P=ApproximateTag;
  public:
    using DomainType = typename SignatureTraits<SIG>::DomainType;
    using CodomainType = typename SignatureTraits<SIG>::CodomainType;
    using ArgumentSizeType = typename SignatureTraits<SIG>::ArgumentSizeType;
    using ResultSizeType = typename SignatureTraits<SIG>::ResultSizeType;
    using ArgumentIndexType = typename SignatureTraits<SIG>::ArgumentIndexType;
    template<class X> using Argument = typename SignatureTraits<SIG>::template Argument<X>;
    template<class X> using Result = typename SignatureTraits<SIG>::template Result<X>;

    ArgumentSizeType argument_size() const { assert(false); }
    ResultSizeType result_size() const { assert(false); }

    Result<FloatDPApproximation> operator() (const Argument<FloatDPApproximation>& x) const { assert(false); }
    Result<FloatMPApproximation> operator() (const Argument<FloatMPApproximation>& x) const { assert(false); }
    Result<Differential<FloatDPApproximation>> operator() (const Argument< Differential<FloatDPApproximation> >& x) const { assert(false); }
    Result<Differential<FloatMPApproximation>> operator() (const Argument< Differential<FloatMPApproximation> >& x) const { assert(false); }
    Result<TaylorModel<ApproximateTag,FloatDP>> operator() (const Argument< TaylorModel<ApproximateTag,FloatDP> >& x) const { assert(false); }
    Result<TaylorModel<ApproximateTag,FloatMP>> operator() (const Argument< TaylorModel<ApproximateTag,FloatMP> >& x) const { assert(false); }
    Result<Formula<ApproximateNumber>> operator() (const Argument< Formula<ApproximateNumber> >& x) const { assert(false); }
    Result<ElementaryAlgebra<ApproximateNumber>> operator() (const Argument< ElementaryAlgebra<ApproximateNumber> >& x) const { assert(false); }

    friend FunctionArchetype<P,SIG> derivative(FunctionArchetype<P,SIG>, SizeType) { assert(false); }

    friend OutputStream& operator<<(OutputStream& os, FunctionArchetype<P,SIG> const& f) { assert(false); }

    template<class X> Result<X> _call(Argument<X> const& x) const { return (*this)(x); }
};



template<class SIG> class FunctionArchetype<ValidatedTag,SIG>
    : public FunctionArchetype<ApproximateTag,SIG>
{
    using P=ValidatedTag;
  public:

    using FunctionArchetype<ApproximateTag,SIG>::operator();
    using FunctionArchetype<ApproximateTag,SIG>::_call;

    Scalar<FloatDPBounds> operator() (const Vector<FloatDPBounds>& x) const { assert(false); }
    Scalar<FloatMPBounds> operator() (const Vector<FloatMPBounds>& x) const { assert(false); }
    Scalar<Differential<FloatDPBounds>> operator() (const Vector< Differential<FloatDPBounds> >& x) const { assert(false); }
    Scalar<Differential<FloatMPBounds>> operator() (const Vector< Differential<FloatMPBounds> >& x) const { assert(false); }
    Scalar<TaylorModel<ValidatedTag,FloatDP>> operator() (const Vector< TaylorModel<ValidatedTag,FloatDP> >& x) const { assert(false); }
    Scalar<TaylorModel<ValidatedTag,FloatMP>> operator() (const Vector< TaylorModel<ValidatedTag,FloatMP> >& x) const { assert(false); }
    Scalar<TaylorModel<ValidatedTag,FloatDPUpperInterval>> operator() (const Vector<TaylorModel<ValidatedTag,FloatDPUpperInterval>>& x) const { assert(false); }
    Scalar<TaylorModel<ValidatedTag,FloatMPUpperInterval>> operator() (const Vector<TaylorModel<ValidatedTag,FloatMPUpperInterval>>& x) const { assert(false); }

    Scalar<Formula<ValidatedNumber>> operator() (const Vector< Formula<ValidatedNumber> >& x) const { assert(false); }
    Scalar<ElementaryAlgebra<ValidatedNumber>> operator() (const Vector< ElementaryAlgebra<ValidatedNumber> >& x) const { assert(false); }

    Scalar<Function<ValidatedTag,SIG>> operator() (const Vector< Function<ValidatedTag,SIG> >& x) const { assert(false); }

    inline Scalar<FloatDPBounds> operator() (const Vector<FloatDP>& x) const { assert(false); }
    inline Scalar<FloatMPBounds> operator() (const Vector<FloatMP>& x) const { assert(false); }

    friend FunctionArchetype<P,SIG> derivative(FunctionArchetype<P,SIG>, SizeType) { assert(false); }
};

template<class SIG> class FunctionArchetype<EffectiveTag,SIG>
    : public FunctionArchetype<ValidatedTag,SIG>
{
    using P=EffectiveTag;
  public:
    using FunctionArchetype<ValidatedTag,SIG>::operator();
    using FunctionArchetype<ValidatedTag,SIG>::_call;

    Scalar<Real> operator() (const Vector<Real>& x) const { assert(false); }
    Scalar<ElementaryAlgebra<Real>> operator() (const Vector<ElementaryAlgebra<Real>>& x) const { assert(false); }
    Scalar<Formula<Real>> operator() (const Vector<Formula<Real>>& x) const { assert(false); }
    Scalar<ElementaryAlgebra<EffectiveNumber>> operator() (const Vector<ElementaryAlgebra<EffectiveNumber>>& x) const { assert(false); }
    Scalar<Formula<EffectiveNumber>> operator() (const Vector<Formula<EffectiveNumber>>& x) const { assert(false); }

    friend FunctionArchetype<P,SIG> derivative(FunctionArchetype<P,SIG>, SizeType) { assert(false); }
};

} // namespace Ariadne




#include "function_concepts.hpp"

namespace Ariadne {

using ScalarMultivariate=RealScalar(RealVector);

static_assert(IsApproximateFunction<FunctionArchetype<ApproximateTag,ScalarMultivariate>,ScalarMultivariate>);
static_assert(IsValidatedFunction<FunctionArchetype<ValidatedTag,ScalarMultivariate>,ScalarMultivariate>);
static_assert(IsEffectiveFunction<FunctionArchetype<EffectiveTag,ScalarMultivariate>,ScalarMultivariate>);

static_assert(AFunction<FunctionArchetype<ApproximateTag,ScalarMultivariate>,ApproximateTag,ScalarMultivariate>);
static_assert(AFunction<FunctionArchetype<ValidatedTag,ScalarMultivariate>,ValidatedTag,ScalarMultivariate>);
static_assert(AFunction<FunctionArchetype<EffectiveTag,ScalarMultivariate>,EffectiveTag,ScalarMultivariate>);

} // namespace Ariadne

#endif // ARIADNE_FUNCTION_ARCHETYPES_HPP
