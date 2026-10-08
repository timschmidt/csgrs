/***************************************************************************
 *            function/function_patch_interface.hpp
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

/*! \file function/function_patch_interface.hpp
 *  \brief Interface for functions on bounded sets.
 */

#ifndef ARIADNE_FUNCTION_PATCH_INTERFACE_HPP
#define ARIADNE_FUNCTION_PATCH_INTERFACE_HPP

#include "../numeric/number.decl.hpp"
#include "../function/function.decl.hpp"

#include "../numeric/operators.hpp"
#include "../numeric/numeric.hpp"

#include "../algebra/algebra_interface.hpp"

#include "../function/domain.hpp"
#include "../function/function_interface.hpp"


namespace Ariadne {

typedef Interval<FloatDPUpperBound> IntervalRangeType;

template<class P, class SIG> class FunctionPatch;

template<class P> class FunctionPatchFactoryInterface;
template<class P, class... ARGS> class FunctionPatchCreatorInterface;

template<class P, class SIG> class FunctionPatchInterface;
template<class P, class... ARGS> using ScalarFunctionPatchInterface = FunctionPatchInterface<P,RealScalar(ARGS...)>;
template<class P, class... ARGS> using VectorFunctionPatchInterface = FunctionPatchInterface<P,RealVector(ARGS...)>;
template<class P> using ScalarUnivariateFunctionPatchInterface = FunctionPatchInterface<P,RealScalar(RealScalar)>;
template<class P> using VectorUnivariateFunctionPatchInterface = FunctionPatchInterface<P,RealVector(RealScalar)>;
template<class P> using ScalarMultivariateFunctionPatchInterface = FunctionPatchInterface<P,RealScalar(RealVector)>;
template<class P> using VectorMultivariateFunctionPatchInterface = FunctionPatchInterface<P,RealVector(RealVector)>;

using ValidatedFunctionPatchFactoryInterface = FunctionPatchFactoryInterface<ValidatedTag>;

template<class P, class SIG> class FunctionPatchAlgebraInterface;

template<class P, class... ARGS> class FunctionPatchAlgebraInterface<P,RealScalar(ARGS...)>
    : public virtual ElementaryAlgebraInterface<Number<P>>
{ };

template<class P, class... ARGS> class FunctionPatchAlgebraInterface<P,RealVector(ARGS...)>
{
    using RES=RealVector; using SIG=RES(ARGS...);
  public:
    typedef SizeType ResultIndexType;
    virtual ScalarFunctionPatchInterface<P,ARGS...>* _get(ResultIndexType) const = 0;
    virtual Void _set(ResultIndexType, ScalarFunctionPatchInterface<P,ARGS...> const&) = 0;
    virtual Void _adjoin(const ScalarFunctionPatchInterface<P,ARGS...>& f2) = 0;
    virtual VectorFunctionPatchInterface<P,ARGS...>* _join(const VectorFunctionPatchInterface<P,ARGS...>& f2) const = 0;
    virtual VectorFunctionPatchInterface<P,ARGS...>* _combine(const VectorFunctionPatchInterface<P,ARGS...>& f2) const = 0;
};

template<class P, class SIG> class FunctionPatchInterface
    : public virtual FunctionInterface<P,SIG>
    , public virtual FunctionPatchAlgebraInterface<P,SIG>
{
    using RES=typename SignatureTraits<SIG>::ResultKind; using ARG=typename SignatureTraits<SIG>::ArgumentKind;
    static_assert(Same<RES,RealScalar> or Same<RES,RealVector>);
    static_assert(Same<ARG,RealScalar> or Same<ARG,RealVector>);
  public:
    typedef typename SignatureTraits<SIG>::BoundedCodomainType CodomainType;
    typedef typename SignatureTraits<SIG>::BoundedDomainType DomainType;
    typedef typename SignatureTraits<SIG>::BoundedRangeType RangeType;
    typedef PositiveValidatedUpperNumber NormType;
    typedef PositiveValidatedUpperNumber ErrorType;
    typedef ExactNumber CoefficientType;
    typedef ElementSizeType<DomainType> ArgumentSizeType;
    typedef ElementSizeType<CodomainType> ResultSizeType;
    typedef ElementIndexType<DomainType> ArgumentIndexType;

    template<class X> using Argument = typename SignatureTraits<SIG>::template Argument<X>;
    template<class X> using Result = typename SignatureTraits<SIG>::template Result<X>;
  public:
    virtual Result<ErrorType> const _errors() const = 0;
//    virtual CoefficientType const _gradient_value(ArgumentIndexType) const = 0;
    virtual ErrorType const _error() const = 0;
    virtual NormType const _norm() const = 0;
    virtual RangeType const _range() const = 0;

    virtual Void _clobber() = 0;
    virtual Bool _refines(const FunctionPatchInterface<P,SIG>& fp) const = 0;

    virtual FunctionPatchFactoryInterface<P>* _factory() const { ARIADNE_NOT_IMPLEMENTED; }

    virtual DomainType const domain() const = 0;
    virtual CodomainType const codomain() const = 0;

    virtual Result<Number<P>> _unchecked_evaluate(const Argument<Number<P>>& x) const = 0;
    virtual Result<CanonicalNumericType<P,DP>> _unchecked_evaluate(const Argument<CanonicalNumericType<P,DP>>& x) const = 0;
    virtual Result<CanonicalNumericType<P,MP>> _unchecked_evaluate(const Argument<CanonicalNumericType<P,MP>>& x) const = 0;
    virtual FunctionPatchInterface<P,SIG>* _partial_evaluate(SizeType j, const Number<P>& c) const = 0;

    virtual ScalarFunctionPatchInterface<P,ARG>* _compose(const ScalarFunction<P,RES>& f) const = 0;
    virtual VectorFunctionPatchInterface<P,ARG>* _compose(const VectorFunction<P,RES>& f) const = 0;
    virtual ScalarFunctionPatchInterface<P,ARG>* _unchecked_compose(const ScalarFunction<P,RES>& f) const = 0;
    virtual VectorFunctionPatchInterface<P,ARG>* _unchecked_compose(const VectorFunction<P,RES>& f) const = 0;

    virtual FunctionPatchInterface<P,SIG>* _clone() const override = 0;
    virtual FunctionPatchInterface<P,SIG>* _create() const = 0;
    virtual FunctionPatchInterface<P,SIG>* _embed(const DomainType& d1, const DomainType& d2) const = 0;
    virtual FunctionPatchInterface<P,SIG>* _restriction(const DomainType& d) const = 0;

    virtual FunctionPatchInterface<P,SIG>* _derivative(ArgumentIndexType j) const override = 0;
    virtual FunctionPatchInterface<P,SIG>* _antiderivative(ArgumentIndexType j) const = 0;
    virtual FunctionPatchInterface<P,SIG>* _antiderivative(ArgumentIndexType j, Number<P> c) const = 0;

    friend OutputStream& operator<<(OutputStream& os, FunctionPatchInterface<P,SIG> const& f) {
        return os << static_cast<FunctionInterface<P,SIG>const&>(f); }
};


template<class P> class FunctionPatchFactoryInterface
{
    typedef RealScalar SARG;
    typedef RealVector VARG;
  public:
    typedef IntervalDomainType ScalarDomainType;
    typedef BoxDomainType VectorDomainType;
    virtual ~FunctionPatchFactoryInterface() = default;
    virtual FunctionPatchFactoryInterface<P>* clone() const = 0;
    virtual OutputStream& _write(OutputStream& os) const = 0;
    friend OutputStream& operator<<(OutputStream& os, FunctionPatchFactoryInterface<P> const& factory) { factory._write(os); return os; }
  private: public:
    virtual ScalarFunctionPatchInterface<P,SARG>* _create(const ScalarDomainType& domain, const ScalarUnivariateFunctionInterface<P>& function) const = 0;
    virtual VectorFunctionPatchInterface<P,SARG>* _create(const ScalarDomainType& domain, const VectorUnivariateFunctionInterface<P>& function) const = 0;
    virtual ScalarFunctionPatchInterface<P,VARG>* _create(const VectorDomainType& domain, const ScalarMultivariateFunctionInterface<P>& function) const = 0;
    virtual VectorFunctionPatchInterface<P,VARG>* _create(const VectorDomainType& domain, const VectorMultivariateFunctionInterface<P>& function) const = 0;

    virtual ScalarFunctionPatchInterface<P,SARG>* _create_zero(const ScalarDomainType& domain) const = 0;
    virtual ScalarFunctionPatchInterface<P,SARG>* _create_constant(const ScalarDomainType& domain, const Number<P>& value) const = 0;
    virtual VectorFunctionPatchInterface<P,SARG>* _create_zeros(SizeType rsize, const ScalarDomainType& domain) const = 0;
    virtual VectorFunctionPatchInterface<P,SARG>* _create_constants(const ScalarDomainType& domain, const Vector<Number<P>>& values) const = 0;
    virtual ScalarFunctionPatchInterface<P,SARG>* _create_identity(const ScalarDomainType& domain) const = 0;

    virtual ScalarFunctionPatchInterface<P,VARG>* _create_zero(const VectorDomainType& domain) const = 0;
    virtual ScalarFunctionPatchInterface<P,VARG>* _create_constant(const VectorDomainType& domain, const Number<P>& value) const = 0;
    virtual ScalarFunctionPatchInterface<P,VARG>* _create_coordinate(const VectorDomainType& domain, SizeType index) const = 0;
    virtual VectorFunctionPatchInterface<P,VARG>* _create_zeros(SizeType rsize, const VectorDomainType& domain) const = 0;
    virtual VectorFunctionPatchInterface<P,VARG>* _create_constants(const VectorDomainType& domain, const Vector<Number<P>>& values) const = 0;
    virtual VectorFunctionPatchInterface<P,VARG>* _create_projection(const VectorDomainType& domain, Range indices) const = 0;
    virtual VectorFunctionPatchInterface<P,VARG>* _create_identity(const VectorDomainType& domain) const = 0;
  public:
    ScalarFunctionPatch<P,VARG> create(const VectorDomainType& domain, const ScalarMultivariateFunctionInterface<P>& function) const {
        return ScalarFunctionPatch<P,VARG>(this->_create(domain,function)); }
    VectorFunctionPatch<P,VARG> create(const VectorDomainType& domain, const VectorMultivariateFunctionInterface<P>& function) const {
        return VectorFunctionPatch<P,VARG>(this->_create(domain,function)); }

    ScalarFunctionPatch<P,VARG> create_zero(const VectorDomainType& domain) const {
        return ScalarFunctionPatch<P,VARG>(_create_zero(domain)); }
    ScalarFunctionPatch<P,VARG> create_constant(const VectorDomainType& domain, const Number<P>& value) const {
        return ScalarFunctionPatch<P,VARG>(_create_constant(domain,value)); }
    ScalarFunctionPatch<P,VARG> create_coordinate(const VectorDomainType& domain, SizeType index) const {
        return ScalarFunctionPatch<P,VARG>(_create_coordinate(domain,index)); }
    VectorFunctionPatch<P,VARG> create_zeros(SizeType rsize, const VectorDomainType& domain) const {
        return VectorFunctionPatch<P,VARG>(_create_zeros(rsize,domain)); }
    VectorFunctionPatch<P,VARG> create_constants(const VectorDomainType& domain, const Vector<Number<P>>& values) const {
        return VectorFunctionPatch<P,VARG>(_create_constants(domain,values)); }
    VectorFunctionPatch<P,VARG> create_projection(const VectorDomainType& domain, Range indices) const {
        return VectorFunctionPatch<P,VARG>(_create_projection(domain,indices)); }
    VectorFunctionPatch<P,VARG> create_identity(const VectorDomainType& domain) const {
        return VectorFunctionPatch<P,VARG>(_create_identity(domain)); }

    ScalarFunctionPatch<P,VARG> create_identity(const ScalarDomainType& domain) const;
};


} // namespace Ariadne

#endif
