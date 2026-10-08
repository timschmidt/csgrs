/***************************************************************************
 *            function/function_traits.hpp
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

/*! \file function/function_traits.hpp
 *  \brief Traits giving relevant associated types for functions.
 */

#ifndef ARIADNE_FUNCTION_TRAITS_HPP
#define ARIADNE_FUNCTION_TRAITS_HPP

#include <iosfwd>

#include "utility/declarations.hpp"
#include "function/function.decl.hpp"

namespace Ariadne {


template<template<class>class T> struct LinearAlgebraTraits;
template<> struct LinearAlgebraTraits<Scalar> {
    template<class X> using Type=Scalar<X>;
    typedef Scalar<Real> Kind;
    typedef SizeOne SizeType;
    typedef IndexZero IndexType;
    typedef RealVariable SpaceType;
};
template<> struct LinearAlgebraTraits<Vector> {
    template<class X> using Type=Vector<X>;
    typedef Vector<Real> Kind;
    typedef Ariadne::SizeType SizeType;
    typedef Ariadne::SizeType IndexType;
    typedef RealSpace SpaceType;
};
template<> struct LinearAlgebraTraits<Matrix> {
    template<class X> using Type=Matrix<X>;
    typedef Matrix<Real> Kind;
    typedef Pair<Ariadne::SizeType,Ariadne::SizeType> SizeType;
    typedef Pair<Ariadne::SizeType,Ariadne::SizeType> IndexType;
};
using ScalarTraits = LinearAlgebraTraits<Scalar>;
using VectorTraits = LinearAlgebraTraits<Vector>;
using MatrixTraits = LinearAlgebraTraits<Matrix>;

template<class R> struct ValueTraits;
template<> struct ValueTraits<Scalar<Real>> : LinearAlgebraTraits<Scalar> { };
template<template<class>class T> struct ValueTraits<T<Real>> : LinearAlgebraTraits<T> { };

using DimensionOne = SizeOne;
class UnitInterval;
class UnitBox;

template<class... T> struct DomainTraits;
template<> struct DomainTraits<RealScalar> : public ScalarTraits {
    typedef RealDomain EntireSetType;
    typedef UnitInterval UnitSetType;
    typedef RealDomain EntireDomainType;
    typedef FloatDPExactInterval BoundedDomainType;
    typedef FloatDPUpperInterval BoundedRangeType;
    template<class FLT> using ConcreteRangeType = Interval<UpperBound<FLT>>;
};
template<> struct DomainTraits<RealVector> : public VectorTraits {
    typedef EuclideanDomain EntireSetType;
    typedef UnitBox UnitSetType;
    typedef EuclideanDomain EntireDomainType;
    typedef FloatDPExactBox BoundedDomainType;
    typedef FloatDPUpperBox BoundedRangeType;
    template<class FLT> using ConcreteRangeType = Box<Interval<UpperBound<FLT>>>;
};
template<class... ARGS> using EntireDomainType = typename DomainTraits<ARGS...>::EntireDomainType;
template<class... ARGS> using BoundedDomainType = typename DomainTraits<ARGS...>::BoundedDomainType;

template<class S> struct ElementTraits;
template<class S> using ElementKind = typename ElementTraits<S>::Kind;
template<class S, class X> using ElementType = typename ValueTraits<ElementKind<S>>::template Type<X>;
template<class S> using ElementSizeType = typename ValueTraits<ElementKind<S>>::SizeType;
template<class S> using ElementIndexType = typename ValueTraits<ElementKind<S>>::IndexType;


template<class SIG> struct SignatureTraits;
template<class RES, class ARG> struct SignatureTraits<RES(ARG)> {
    typedef ARG ArgumentKind;
    typedef RES ResultKind;

    template<class X> using Argument = typename ValueTraits<ARG>::template Type<X>;
    template<class X> using Result = typename ValueTraits<RES>::template Type<X>;

    typedef typename ValueTraits<ARG>::SizeType ArgumentSizeType;
    typedef typename ValueTraits<ARG>::IndexType ArgumentIndexType;
    typedef typename ValueTraits<RES>::SizeType ResultSizeType;
    typedef typename ValueTraits<RES>::IndexType ResultIndexType;

    typedef typename ValueTraits<ARG>::SpaceType ArgumentSpaceType;

        typedef typename DomainTraits<ARG>::EntireDomainType DomainType;
    typedef typename DomainTraits<ARG>::EntireDomainType EntireDomainType;
    typedef typename DomainTraits<ARG>::BoundedDomainType BoundedDomainType;
        typedef typename DomainTraits<RES>::EntireDomainType CodomainType;
    typedef typename DomainTraits<RES>::EntireDomainType EntireCodomainType;
    typedef typename DomainTraits<RES>::BoundedDomainType BoundedCodomainType;

    typedef typename DomainTraits<RES>::BoundedRangeType BoundedRangeType;
    template<class FLT> using ConcreteRangeType = typename DomainTraits<RES>::template ConcreteRangeType<FLT>;
};

template<template<class>class LIN> struct CartesianProductBoxTypedef;
template<> struct CartesianProductBoxTypedef<Scalar> { template<class IVL> using Type = IVL; };
template<> struct CartesianProductBoxTypedef<Vector> { template<class IVL> using Type = Box<IVL>; };
template<template<class>class LIN, class IVL> using CartesianProductBoxType = typename CartesianProductBoxTypedef<LIN>::template Type<IVL>;

template<class P, class PR, class PRE=PR> struct FunctionModelTraits;

template<class F> class UnknownError;

template<class PR, class PRE> struct FunctionModelTraits<ValidatedTag,PR,PRE> {
    typedef RawFloat<PR> F; typedef RawFloat<PRE> FE;
    typedef F ValueType; typedef Error<FE> ErrorType;
    typedef PositiveUpperBound<F> NormType;
    typedef Interval<UpperBound<F>> RangeType;
    typedef Bounds<F> NumericType;
    typedef ValidatedNumber GenericNumericType;
    typedef F RawFloatType;
};
template<class PR> struct FunctionModelTraits<ApproximateTag,PR> {
    typedef RawFloat<PR> F;
    typedef Approximation<F> ValueType; typedef UnknownError<F> ErrorType;
    typedef PositiveApproximation<F> NormType;
    typedef Interval<Approximation<F>> RangeType;
    typedef Approximation<F> NumericType; typedef ApproximateNumber GenericNumericType;
    typedef F RawFloatType;
};

template<class P, class PR, class PRE=PR> using CanonicalNumericType = typename FunctionModelTraits<P,PR,PRE>::NumericType;
template<class P, class PR> using CanonicalCoefficientType = typename FunctionModelTraits<P,PR>::CoefficientType;
template<class P, class PRE> using CanonicalErrorType = typename FunctionModelTraits<P,PRE,PRE>::ErrorType;

template<class P> using CanonicalNumeric64Type = typename FunctionModelTraits<P,DoublePrecision>::NumericType;
template<class P> using CanonicalCoefficient64Type = typename FunctionModelTraits<P,DoublePrecision>::CoefficientType;
template<class P> using CanonicalError64Type = typename FunctionModelTraits<P,DoublePrecision,DoublePrecision>::ErrorType;

template<class X> using PrecisionType = typename X::PrecisionType;
template<class X> using ErrorPrecisionType = typename X::ErrorPrecisionType;



} // namespace Ariadne

#endif /* ARIADNE_FUNCTION_TRAITS_HPP */
