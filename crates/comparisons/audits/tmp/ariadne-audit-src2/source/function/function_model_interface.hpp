/***************************************************************************
 *            function/function_model_interface.hpp
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

/*! \file function/function_model_interface.hpp
 *  \brief Interface for functions on bounded sets.
 */

#ifndef ARIADNE_FUNCTION_MODEL_INTERFACE_HPP
#define ARIADNE_FUNCTION_MODEL_INTERFACE_HPP

#include "algebra/algebra_interface.hpp"

#include "function/function.decl.hpp"
#include "function/function_interface.hpp"
#include "function/function_patch_interface.hpp"

#include "algebra/range.hpp"
#include "numeric/operators.hpp"

namespace Ariadne {

template<class P, class PR, class PRE> class FunctionModelFactoryInterface;
template<class P, class ARG, class PR, class PRE> class FunctionModelCreatorInterface;

template<class P, class SIG, class PR, class PRE> class FunctionModelInterface;

template<class P, class SIG, class PR, class PRE> class FunctionModelAlgebraInterface;

template<class P, class ARG, class PR, class PRE> class FunctionModelAlgebraInterface<P,RealScalar(ARG),PR,PRE>
    : public virtual ElementaryAlgebraInterface<CanonicalNumericType<P,PR,PRE>>
{
    static_assert(Same<ARG,RealScalar> or Same<ARG,RealVector>);
    typedef RealScalar RES; typedef RES SIG(ARG);
    typedef BoundedDomainType<ARG> D; typedef BoundedDomainType<RES> C;
    typedef RawFloat<PR> F;
  public:
    typedef D DomainType;
    typedef C CodomainType;
    typedef typename FunctionModelTraits<P,PR,PRE>::ValueType ValueType;
  public:
    virtual ValueType const _value() const = 0;
};

template<class P, class ARG, class PR, class PRE> using ScalarFunctionModelInterface = FunctionModelInterface<P,RealScalar(ARG),PR,PRE>;
template<class P, class ARG, class PR, class PRE> using VectorFunctionModelInterface = FunctionModelInterface<P,RealVector(ARG),PR,PRE>;

template<class P, class ARG, class PR, class PRE> using ScalarFunctionModelAlgebraInterface = FunctionModelAlgebraInterface<P,RealScalar(ARG),PR,PRE>;
template<class P, class ARG, class PR, class PRE> using VectorFunctionModelAlgebraInterface = FunctionModelAlgebraInterface<P,RealVector(ARG),PR,PRE>;

template<class P, class ARG, class PR, class PRE> class FunctionModelAlgebraInterface<P,RealVector(ARG),PR,PRE>
    : public virtual WritableInterface
{
  public:
    typedef typename FunctionModelTraits<P,PR,PRE>::ValueType ValueType;
    typedef typename FunctionModelTraits<P,PR,PRE>::ErrorType ErrorType;
  public:
    virtual Void _concrete_set(SizeType, ScalarFunctionModelInterface<P,ARG,PR,PRE> const&) = 0;
    virtual ScalarFunctionModelInterface<P,ARG,PR,PRE>* _concrete_get(SizeType) const = 0;
    virtual VectorFunctionModelInterface<P,ARG,PR,PRE>* _concrete_join(const VectorFunctionModelInterface<P,ARG,PR,PRE>& f2) const = 0;
    virtual VectorFunctionModelInterface<P,ARG,PR,PRE>* _concrete_combine(const VectorFunctionModelInterface<P,ARG,PR,PRE>& f2) const = 0;
    virtual Void _concrete_adjoin(const ScalarFunctionModelInterface<P,ARG,PR,PRE>& f2) = 0;

    virtual Vector<ValueType> const _concrete_values() const = 0;
    virtual Vector<ErrorType> const _concrete_errors() const = 0;
};

template<class P, class SIG, class PR, class PRE> class FunctionModelInterface
    : public virtual FunctionInterface<P,SIG>
    , public virtual FunctionPatchInterface<P,SIG>
    , public virtual FunctionModelAlgebraInterface<P,SIG,PR,PRE>
{
    using RES=typename SignatureTraits<SIG>::ResultKind; using ARG=typename SignatureTraits<SIG>::ArgumentKind;
    static_assert(Same<ARG,RealScalar> or Same<ARG,RealVector>,"");
    static_assert(Same<RES,RealScalar> or Same<RES,RealVector>,"");
    using C=BoundedDomainType<RES>; using D=BoundedDomainType<ARG>;
    static_assert(Same<D,IntervalDomainType> or Same<D,BoxDomainType>,"");
    static_assert(Same<C,IntervalDomainType> or Same<C,BoxDomainType>,"");

    template<class X> using Argument = typename FunctionInterface<P,SIG>::template Argument<X>;
    template<class X> using Result = typename FunctionInterface<P,SIG>::template Result<X>;

    using FLT = Float<PR>;
  public:
    typedef D DomainType;
    typedef C CodomainType;
    typedef typename SignatureTraits<SIG>::template ConcreteRangeType<FLT> RangeType;
    typedef typename FunctionModelTraits<P,PR,PRE>::NormType NormType;
    typedef typename FunctionModelTraits<P,PR,PRE>::ValueType ValueType;
    typedef typename FunctionModelTraits<P,PR,PRE>::ErrorType ErrorType;
    typedef typename FunctionModelTraits<P,PR,PRE>::NumericType NumericType;
    typedef typename FunctionModelTraits<P,PR,PRE>::GenericNumericType GenericNumericType;
    typedef typename SignatureTraits<SIG>::ArgumentIndexType ArgumentIndexType;
  public:
    virtual DomainType const domain() const = 0;
    virtual CodomainType const codomain() const = 0;
    virtual RangeType const range() const = 0;
    virtual NormType const _concrete_norm() const = 0;
    virtual ErrorType const _concrete_error() const = 0;

    virtual Void clobber() = 0;

    virtual Result<CanonicalNumericType<P,PR,PRE>> _concrete_unchecked_evaluate(const Argument<CanonicalNumericType<P,PR,PRE>>& x) const = 0;
    virtual FunctionModelInterface<P,SIG,PR,PRE>* _concrete_partial_evaluate(SizeType j, const CanonicalNumericType<P,PR,PRE>& c) const = 0;
    virtual FunctionModelInterface<P,SIG,PR,PRE>* _embed(const DomainType& d1, const DomainType& d2) const = 0;

    virtual FunctionModelFactoryInterface<P,PR,PRE>* _factory() const = 0;
    virtual FunctionModelInterface<P,SIG,PR,PRE>* _clone() const = 0;
    virtual FunctionModelInterface<P,SIG,PR,PRE>* _create() const = 0;
    inline FunctionModelInterface<P,SIG,PR,PRE>* _copy() const { return this->_clone(); }

    virtual FunctionModelInterface<P,SIG,PR,PRE>* _restriction(const DomainType& d) const = 0;

    virtual FunctionModelInterface<P,SIG,PR,PRE>* _derivative(ArgumentIndexType j) const = 0;
    virtual FunctionModelInterface<P,SIG,PR,PRE>* _antiderivative(ArgumentIndexType j) const = 0;
    virtual FunctionModelInterface<P,SIG,PR,PRE>* _antiderivative(ArgumentIndexType j, Number<P>) const = 0;
    virtual FunctionModelInterface<P,SIG,PR,PRE>* _concrete_antiderivative(ArgumentIndexType j, CanonicalNumericType<P,PR,PRE> c) const = 0;

    using FunctionPatchInterface<P,SIG>::_compose;
    using FunctionPatchInterface<P,SIG>::_unchecked_compose;
    virtual ScalarFunctionModelInterface<P,ARG,PR,PRE>* _compose(const ScalarFunction<P,RES>& f) const = 0;
    virtual VectorFunctionModelInterface<P,ARG,PR,PRE>* _compose(const VectorFunction<P,RES>& f) const = 0;
    virtual ScalarFunctionModelInterface<P,ARG,PR,PRE>* _unchecked_compose(const ScalarFunction<P,RES>& f) const = 0;
    virtual VectorFunctionModelInterface<P,ARG,PR,PRE>* _unchecked_compose(const VectorFunction<P,RES>& f) const = 0;

    virtual Boolean _concrete_refines(const FunctionModelInterface<P,SIG,PR,PRE>& f) const = 0;
    virtual Boolean _concrete_inconsistent(const FunctionModelInterface<P,SIG,PR,PRE>& f) const = 0;
    virtual FunctionModelInterface<P,SIG,PR,PRE>* _concrete_refinement(const FunctionModelInterface<P,SIG,PR,PRE>& f) const = 0;

    virtual OutputStream& _write(OutputStream&) const = 0;

    friend OutputStream& operator<<(OutputStream& os, FunctionModelInterface<P,SIG,PR,PRE> const& f) {
        return f._write(os); }
};



template<class P, class PR, class PRE> class FunctionModelFactoryInterface
    : public FunctionPatchFactoryInterface<P>
{
    typedef RealScalar SARG;
    typedef RealVector VARG;

    typedef BoxDomainType VD;
    typedef IntervalDomainType SD;
    friend class FunctionModelFactory<P,PR,PRE>;
  public:
    typedef SD ScalarDomainType;
    typedef VD VectorDomainType;
    virtual ~FunctionModelFactoryInterface() = default;
    virtual FunctionModelFactoryInterface<P,PR,PRE>* clone() const = 0;
    inline FunctionModelFactoryInterface<P,PR,PRE>* _copy() const { return this->clone(); }
    virtual OutputStream& _write(OutputStream& os) const = 0;
    friend OutputStream& operator<<(OutputStream& os, FunctionModelFactoryInterface<P,PR,PRE> const& factory) { factory._write(os); return os; }
  private:
    using FunctionPatchFactoryInterface<P>::_create;
    virtual CanonicalNumericType<P,PR,PRE> _create(const Number<P>& number) const = 0;
    virtual ScalarFunctionModelInterface<P,VARG,PR,PRE>* _create(const VectorDomainType& domain, const ScalarMultivariateFunctionInterface<P>& function) const = 0;
    virtual VectorFunctionModelInterface<P,VARG,PR,PRE>* _create(const VectorDomainType& domain, const VectorMultivariateFunctionInterface<P>& function) const = 0;

    virtual ScalarFunctionModelInterface<P,VARG,PR,PRE>* _create_zero(const VectorDomainType& domain) const = 0;
    virtual ScalarFunctionModelInterface<P,VARG,PR,PRE>* _create_constant(const VectorDomainType& domain, const Number<P>& value) const = 0;
    virtual ScalarFunctionModelInterface<P,VARG,PR,PRE>* _create_coordinate(const VectorDomainType& domain, SizeType index) const = 0;
    virtual VectorFunctionModelInterface<P,VARG,PR,PRE>* _create_zeros(SizeType rsize, const VectorDomainType& domain) const = 0;
    virtual VectorFunctionModelInterface<P,VARG,PR,PRE>* _create_constants(const VectorDomainType& domain, const Vector<Number<P>>& values) const = 0;
    virtual VectorFunctionModelInterface<P,VARG,PR,PRE>* _create_projection(const VectorDomainType& domain, Range indices) const = 0;
    virtual VectorFunctionModelInterface<P,VARG,PR,PRE>* _create_identity(const VectorDomainType& domain) const = 0;
  public:
    CanonicalNumericType<P,PR,PRE> create(const Number<P>& number) const {
        return CanonicalNumericType<P,PR,PRE>(this->_create(number)); }
    ScalarMultivariateFunctionModel<P,PR,PRE> create(const VectorDomainType& domain, const ScalarMultivariateFunctionInterface<P>& function) const {
        return ScalarMultivariateFunctionModel<P,PR,PRE>(this->_create(domain,function)); }
    VectorMultivariateFunctionModel<P,PR,PRE> create(const VectorDomainType& domain, const VectorMultivariateFunctionInterface<P>& function) const {
        return VectorMultivariateFunctionModel<P,PR,PRE>(this->_create(domain,function)); }

    CanonicalNumericType<P,PR,PRE> create_number(const Number<P>& number) const {
        return CanonicalNumericType<P,PR,PRE>(this->_create(number)); }

    ScalarMultivariateFunctionModel<P,PR,PRE> create_zero(const VectorDomainType& domain) const {
        return ScalarMultivariateFunctionModel<P,PR,PRE>(_create_zero(domain)); }
    ScalarMultivariateFunctionModel<P,PR,PRE> create_constant(const VectorDomainType& domain, const Number<P>& value) const {
        return ScalarMultivariateFunctionModel<P,PR,PRE>(_create_constant(domain,value)); }
    ScalarMultivariateFunctionModel<P,PR,PRE> create_constant(const VectorDomainType& domain, const CanonicalNumericType<P,PR,PRE>& value) const {
        return ScalarMultivariateFunctionModel<P,PR,PRE>(_create_constant(domain,Number<P>(value))); }
    ScalarMultivariateFunctionModel<P,PR,PRE> create_coordinate(const VectorDomainType& domain, SizeType index) const {
        return ScalarMultivariateFunctionModel<P,PR,PRE>(_create_coordinate(domain,index)); }
    VectorMultivariateFunctionModel<P,PR,PRE> create_zeros(SizeType rsize, const VectorDomainType& domain) const {
        return VectorMultivariateFunctionModel<P,PR,PRE>(_create_zeros(rsize,domain)); }
    VectorMultivariateFunctionModel<P,PR,PRE> create_constants(const VectorDomainType& domain, const Vector<Number<P>>& values) const {
        return VectorMultivariateFunctionModel<P,PR,PRE>(_create_constants(domain,values)); }
    VectorMultivariateFunctionModel<P,PR,PRE> create_projection(const VectorDomainType& domain, Range indices) const {
        return VectorMultivariateFunctionModel<P,PR,PRE>(_create_projection(domain,indices)); }
    VectorMultivariateFunctionModel<P,PR,PRE> create_identity(const VectorDomainType& domain) const {
        return VectorMultivariateFunctionModel<P,PR,PRE>(_create_identity(domain)); }

    ScalarMultivariateFunctionModel<P,PR,PRE> create_identity(const ScalarDomainType& domain) const;
};


} // namespace Ariadne

#endif
