/***************************************************************************
 *            function/function_model_mixin.hpp
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

/*! \file function/function_model_mixin.hpp
 *  \brief Mixin for concrete functions on bounded domains.
 */

#ifndef ARIADNE_FUNCTION_MODEL_MIXIN_HPP
#define ARIADNE_FUNCTION_MODEL_MIXIN_HPP

#include <cstdarg>
#include <iosfwd>
#include <iostream>

#include "function/function_model_interface.hpp"
#include "function/function_model.hpp"

#include "numeric/operators.hpp"
#include "numeric/numeric.hpp"
#include "algebra/vector.hpp"
#include "algebra/matrix.hpp"
#include "algebra/operations.hpp"
#include "algebra/algebra_mixin.hpp"

#include "function/domain.hpp"

#include "function/function_interface.hpp"
#include "function/function_mixin.hpp"
#include "function/function.hpp"

namespace Ariadne {

template<class FM, class P, class SIG, class PR=DoublePrecision, class PRE=PR> class FunctionModelMixin;
template<class FM, class P, class PR=DoublePrecision, class PRE=PR> using ScalarMultivariateFunctionModelMixin = FunctionModelMixin<FM,P,RealScalar(RealVector),PR,PRE>;
template<class FM, class P, class PR=DoublePrecision, class PRE=PR> using VectorMultivariateFunctionModelMixin = FunctionModelMixin<FM,P,RealVector(RealVector),PR,PRE>;

template<class FM, class P, class ARG, class PR, class PRE> class FunctionModelMixin<FM,P,RealScalar(ARG),PR,PRE>
    : public virtual ScalarFunctionModelInterface<P,ARG,PR,PRE>
    , public ScalarFunctionMixin<FM,P,ARG>
    , public ElementaryAlgebraMixin<FM,CanonicalNumericType<P,PR,PRE>>
    , public ElementaryAlgebraMixin<FM,Number<P>>
{
    static_assert(Same<ARG,RealScalar> or Same<ARG,RealVector>);
    using RES = RealScalar; using SIG=RES(ARG);
    using C=BoundedDomainType<RES>; using D=BoundedDomainType<ARG>;
    using X = typename FunctionModelInterface<P,SIG,PR,PRE>::NumericType;
  public:
    typedef FunctionPatchInterface<P,SIG> GenericInterface;
    typedef typename GenericInterface::ErrorType GenericErrorType;
    typedef typename GenericInterface::NormType GenericNormType;
    typedef typename GenericInterface::RangeType GenericRangeType;
  public:
    typedef FunctionModelInterface<P,SIG,PR,PRE> Interface;
    typedef typename Interface::ValueType ValueType;
    typedef typename Interface::ErrorType ErrorType;
    typedef typename Interface::NormType NormType;
    typedef typename Interface::RangeType RangeType;
  public:
    ScalarFunctionModelInterface<P,ARG,PR,PRE>* _clone() const override {
        return new FM(static_cast<const FM&>(*this)); }

    ScalarFunctionModelInterface<P,ARG,PR,PRE>* _create_copy() const override {
        return new FM(static_cast<const FM&>(*this)); }
    ScalarFunctionModelInterface<P,ARG,PR,PRE>* _create_zero() const override {
        return new FM(factory(static_cast<FM const&>(*this)).create_zero()); }
    ScalarFunctionModelInterface<P,ARG,PR,PRE>* _create_constant(Number<P> const& c) const override {
        return new FM(factory(static_cast<FM const&>(*this)).create_constant(c)); }
    ScalarFunctionModelInterface<P,ARG,PR,PRE>* _create_constant(CanonicalNumericType<P,PR,PRE> const& c) const override {
        return new FM(factory(static_cast<FM const&>(*this)).create_constant(c)); }

    GenericRangeType const _range() const override {
        return static_cast<GenericRangeType>(static_cast<const FM&>(*this).range()); }
    GenericErrorType const _errors() const override {
        return static_cast<const FM&>(*this).error(); }
    GenericErrorType const _error() const override {
        return static_cast<const FM&>(*this).error(); }
    GenericNormType const _norm() const override {
        return static_cast<GenericNormType>(norm(static_cast<const FM&>(*this))); }

    ValueType const _value() const override {
        return static_cast<const FM&>(*this).value(); }
    ErrorType const _concrete_error() const override {
        return static_cast<const FM&>(*this).error(); }
    NormType const _concrete_norm() const override {
        return norm(static_cast<const FM&>(*this)); }
    Void _clobber() override {
        static_cast<FM&>(*this).clobber(); }
    ScalarFunctionModelInterface<P,ARG,PR,PRE>* _derivative(SizeType j) const override {
        return new FM(derivative(static_cast<const FM&>(*this),j)); }
    ScalarFunctionModelInterface<P,ARG,PR,PRE>* _antiderivative(SizeType j) const override {
        return new FM(antiderivative(static_cast<const FM&>(*this),j)); }
    ScalarFunctionModelInterface<P,ARG,PR,PRE>* _antiderivative(SizeType j, Number<P> c) const override {
        return new FM(antiderivative(static_cast<const FM&>(*this),j,c)); }
    ScalarFunctionModelInterface<P,ARG,PR,PRE>* _concrete_antiderivative(SizeType j, CanonicalNumericType<P,PR,PRE> c) const override {
        return new FM(antiderivative(static_cast<const FM&>(*this),j,c)); }
    ScalarFunctionModelInterface<P,ARG,PR,PRE>* _restriction(const BoxDomainType& d) const override {
        return new FM(restriction(static_cast<const FM&>(*this),d)); }
    Number<P> _unchecked_evaluate(const Vector<Number<P>>& x) const override {
        return unchecked_evaluate(static_cast<const FM&>(*this),x); }
    CanonicalNumericType<P,DP> _unchecked_evaluate(const Vector<CanonicalNumericType<P,DP>>& x) const override {
        auto r = unchecked_evaluate(static_cast<const FM&>(*this),x);
        if constexpr (Convertible<decltype(r),CanonicalNumericType<P,DP>>) { return r; } else { ARIADNE_FAIL_MSG(""); }
    }
    CanonicalNumericType<P,MP> _unchecked_evaluate(const Vector<CanonicalNumericType<P,MP>>& x) const override {
        auto r = unchecked_evaluate(static_cast<const FM&>(*this),x);
        if constexpr (Convertible<decltype(r),CanonicalNumericType<P,MP>>) { return r; } else {ARIADNE_FAIL_MSG(""); }
    }
    CanonicalNumericType<P,PR,PRE> _concrete_unchecked_evaluate(const Vector<CanonicalNumericType<P,PR,PRE>>& x) const override {
        return unchecked_evaluate(static_cast<const FM&>(*this),x); }
    ScalarFunctionModelInterface<P,ARG,PR,PRE>* _partial_evaluate(SizeType j, const Number<P>& c) const override {
        return heap_copy(partial_evaluate(static_cast<const FM&>(*this),j,c)); }
    ScalarFunctionModelInterface<P,ARG,PR,PRE>* _concrete_partial_evaluate(SizeType j, const CanonicalNumericType<P,PR,PRE>& c) const override {
        return heap_copy(partial_evaluate(static_cast<const FM&>(*this),j,c)); }

    ScalarFunctionModelInterface<P,ARG,PR,PRE>* _compose(const ScalarUnivariateFunction<P>& f) const override {
        ARIADNE_NOT_IMPLEMENTED; }
    VectorFunctionModelInterface<P,ARG,PR,PRE>* _compose(const VectorUnivariateFunction<P>& f) const override {
        ARIADNE_NOT_IMPLEMENTED; }
    ScalarFunctionModelInterface<P,ARG,PR,PRE>* _unchecked_compose(const ScalarUnivariateFunction<P>& f) const override {
        ARIADNE_NOT_IMPLEMENTED; }
    VectorFunctionModelInterface<P,ARG,PR,PRE>* _unchecked_compose(const VectorUnivariateFunction<P>& f) const override {
        ARIADNE_NOT_IMPLEMENTED; }


    ScalarFunctionModelInterface<P,ARG,PR,PRE>* _embed(const BoxDomainType& d1, const BoxDomainType& d2) const override {
        return new FM(embed(d1,static_cast<const FM&>(*this),d2)); }
    Bool _refines(FunctionPatchInterface<P,SIG> const& fp) const override {
        ARIADNE_ASSERT(dynamic_cast<const FM*>(&fp)); return refines(static_cast<const FM&>(*this),dynamic_cast<const FM&>(fp)); }
    Boolean _concrete_refines(const ScalarFunctionModelInterface<P,ARG,PR,PRE>& f) const override {
        ARIADNE_ASSERT(dynamic_cast<const FM*>(&f)); return refines(static_cast<const FM&>(*this),dynamic_cast<const FM&>(f)); }
    Boolean _concrete_inconsistent(const ScalarFunctionModelInterface<P,ARG,PR,PRE>& f) const override {
        ARIADNE_ASSERT(dynamic_cast<const FM*>(&f)); return inconsistent(static_cast<const FM&>(*this),dynamic_cast<const FM&>(f)); }
    ScalarFunctionModelInterface<P,ARG,PR,PRE>* _concrete_refinement(const ScalarFunctionModelInterface<P,ARG,PR,PRE>& f) const override {
        ARIADNE_ASSERT(dynamic_cast<const FM*>(&f)); return new FM(refinement(static_cast<const FM&>(*this),dynamic_cast<const FM&>(f))); }

    OutputStream& _write(OutputStream& os) const override {
        return os << static_cast<FM const&>(*this); }
};


template<class FM, class P, class ARG, class PR, class PRE> class FunctionModelMixin<FM,P,RealVector(ARG),PR,PRE>
    : public virtual VectorFunctionModelInterface<P,ARG,PR,PRE>
    , public VectorFunctionMixin<FM,P,ARG>
{
    using RES = RealVector; using SIG=RES(ARG);
    using C = BoundedDomainType<RES>; using D = BoundedDomainType<ARG>;
    using X = typename FunctionModelInterface<P,SIG,PR,PRE>::NumericType;
  public:
    typedef FunctionPatchInterface<P,SIG> GenericInterface;
    typedef typename GenericInterface::ErrorType GenericErrorType;
    typedef typename GenericInterface::NormType GenericNormType;
    typedef typename GenericInterface::RangeType GenericRangeType;
  public:
    typedef FunctionModelInterface<P,SIG,PR,PRE> Interface;
    typedef typename Interface::ValueType ValueType;
    typedef typename Interface::ErrorType ErrorType;
    typedef typename Interface::NormType NormType;
    typedef typename Interface::RangeType RangeType;

    typedef typename Element<FM>::Type ScalarMultivariateFunctionType;
  public:
    virtual VectorFunctionModelInterface<P,ARG,PR,PRE>* _clone() const override { return new FM(static_cast<const FM&>(*this)); }
    virtual Void _set(SizeType i, const ScalarFunctionPatchInterface<P,ARG>& sf) override {
        if(!dynamic_cast<const typename FM::ScalarMultivariateFunctionType*>(&sf)) {
            ARIADNE_FAIL_MSG("Cannot set element of VectorMultivariateFunctionModel "<<*this<<" to "<<sf); }
        static_cast<FM&>(*this).FM::set(i,dynamic_cast<const ScalarMultivariateFunctionType&>(sf)); }
    virtual Void _concrete_set(SizeType i, const ScalarFunctionModelInterface<P,ARG,PR,PRE>& sf) override {
        if(!dynamic_cast<const typename FM::ScalarMultivariateFunctionType*>(&sf)) {
            ARIADNE_FAIL_MSG("Cannot set element of VectorMultivariateFunctionModel "<<*this<<" to "<<sf); }
        static_cast<FM&>(*this).FM::set(i,dynamic_cast<const ScalarMultivariateFunctionType&>(sf)); }
    GenericRangeType const _range() const override {
        return static_cast<GenericRangeType>(static_cast<const FM&>(*this).range()); }
    Vector<GenericErrorType> const _errors() const override {
        return static_cast<const FM&>(*this).errors(); }
    GenericErrorType const _error() const override {
        return static_cast<const FM&>(*this).error(); }
    GenericNormType const _norm() const override {
         return static_cast<GenericNormType>(norm(static_cast<const FM&>(*this))); }
    Vector<ValueType> const _concrete_values() const override {
        return static_cast<const FM&>(*this).values(); }
    Vector<ErrorType> const _concrete_errors() const override {
        return static_cast<const FM&>(*this).errors(); }
    ErrorType const _concrete_error() const override {
        return static_cast<const FM&>(*this).error(); }
    NormType const _concrete_norm() const override {
         return norm(static_cast<const FM&>(*this)); }
    virtual VectorFunctionModelInterface<P,ARG,PR,PRE>* _derivative(SizeType j) const override {
        ARIADNE_NOT_IMPLEMENTED; }
    virtual VectorFunctionModelInterface<P,ARG,PR,PRE>* _antiderivative(SizeType j) const override {
        return new FM(antiderivative(static_cast<const FM&>(*this),j)); }
    virtual VectorFunctionModelInterface<P,ARG,PR,PRE>* _antiderivative(SizeType j, Number<P> c) const override {
        return new FM(antiderivative(static_cast<const FM&>(*this),j,c)); }
    virtual VectorFunctionModelInterface<P,ARG,PR,PRE>* _concrete_antiderivative(SizeType j, CanonicalNumericType<P,PR,PRE> c) const override {
        return new FM(antiderivative(static_cast<const FM&>(*this),j,c)); }
    VectorFunctionModelInterface<P,ARG,PR,PRE>* _embed(const BoxDomainType& d1, const BoxDomainType& d2) const override {
        return heap_copy(embed(d1,static_cast<const FM&>(*this),d2)); }
    VectorFunctionModelInterface<P,ARG,PR,PRE>* _restriction(const BoxDomainType& d) const override {
        return new FM(restriction(static_cast<const FM&>(*this),d)); }
    Void _adjoin(const ScalarFunctionPatchInterface<P,ARG>& f) override {
        static_cast<FM&>(*this).FM::adjoin(dynamic_cast<const ScalarMultivariateFunctionType&>(f)); }
    Void _concrete_adjoin(const ScalarFunctionModelInterface<P,ARG,PR,PRE>& f) override {
        static_cast<FM&>(*this).FM::adjoin(dynamic_cast<const ScalarMultivariateFunctionType&>(f)); }
    VectorFunctionModelInterface<P,ARG,PR,PRE>* _join(const VectorFunctionPatchInterface<P,ARG>& f) const override {
        return heap_copy(join(static_cast<const FM&>(*this),dynamic_cast<const FM&>(f))); }
    VectorFunctionModelInterface<P,ARG,PR,PRE>* _concrete_join(const VectorFunctionModelInterface<P,ARG,PR,PRE>& f) const override {
        return heap_copy(join(static_cast<const FM&>(*this),dynamic_cast<const FM&>(f))); }
    VectorFunctionModelInterface<P,ARG,PR,PRE>* _combine(const VectorFunctionPatchInterface<P,ARG>& f) const override {
        return heap_copy(combine(static_cast<const FM&>(*this),dynamic_cast<const FM&>(f))); }
    VectorFunctionModelInterface<P,ARG,PR,PRE>* _concrete_combine(const VectorFunctionModelInterface<P,ARG,PR,PRE>& f) const override {
        return heap_copy(combine(static_cast<const FM&>(*this),dynamic_cast<const FM&>(f))); }
    Vector<Number<P>> _unchecked_evaluate(const Vector<Number<P>>& y) const override {
        return unchecked_evaluate(static_cast<const FM&>(*this),y); }
    Vector<CanonicalNumericType<P,DP>> _unchecked_evaluate(const Vector<CanonicalNumericType<P,DP>>& x) const override {
        auto pr=x.zero_element().precision(); return Vector<CanonicalNumericType<P,DP>>(unchecked_evaluate(static_cast<const FM&>(*this),x),pr); }
    Vector<CanonicalNumericType<P,MP>> _unchecked_evaluate(const Vector<CanonicalNumericType<P,MP>>& x) const override {
        auto pr=x.zero_element().precision(); return Vector<CanonicalNumericType<P,MP>>(unchecked_evaluate(static_cast<const FM&>(*this),x),pr); }
    Vector<CanonicalNumericType<P,PR,PRE>> _concrete_unchecked_evaluate(const Vector<CanonicalNumericType<P,PR,PRE>>& x) const override {
        return unchecked_evaluate(static_cast<const FM&>(*this),x); }
    ScalarFunctionModelInterface<P,ARG,PR,PRE>* _compose(const ScalarMultivariateFunction<P>& f) const override {
        return heap_copy(compose(f,static_cast<const FM&>(*this))); }
    VectorFunctionModelInterface<P,ARG,PR,PRE>* _compose(const VectorMultivariateFunction<P>& f) const override {
        return heap_copy(compose(f,static_cast<const FM&>(*this))); }

    ScalarFunctionModelInterface<P,ARG,PR,PRE>* _unchecked_compose(const ScalarMultivariateFunction<P>& f) const override {
        return heap_copy(compose(f,static_cast<const FM&>(*this))); }
    VectorFunctionModelInterface<P,ARG,PR,PRE>* _unchecked_compose(const VectorMultivariateFunction<P>& f) const override {
        return heap_copy(compose(f,static_cast<const FM&>(*this))); }

    VectorFunctionModelInterface<P,ARG,PR,PRE>* _partial_evaluate(SizeType j, const Number<P>& c) const override {
        return heap_copy(partial_evaluate(static_cast<const FM&>(*this),j,c)); }
    VectorFunctionModelInterface<P,ARG,PR,PRE>* _concrete_partial_evaluate(SizeType j, const CanonicalNumericType<P,PR,PRE>& c) const override {
        return heap_copy(partial_evaluate(static_cast<const FM&>(*this),j,c)); }

    Void _clobber() override {
        return static_cast<FM&>(*this).clobber(); }

    Bool _refines(FunctionPatchInterface<P,SIG> const& fp) const override {
        ARIADNE_ASSERT(dynamic_cast<const FM*>(&fp)); return refines(static_cast<const FM&>(*this),dynamic_cast<const FM&>(fp)); }
    Boolean _concrete_inconsistent(const VectorFunctionModelInterface<P,ARG,PR,PRE>& f) const override {
        ARIADNE_ASSERT(dynamic_cast<const FM*>(&f)); return inconsistent(static_cast<const FM&>(*this),dynamic_cast<const FM&>(f)); }
    Boolean _concrete_refines(const VectorFunctionModelInterface<P,ARG,PR,PRE>& f) const override {
        ARIADNE_ASSERT(dynamic_cast<const FM*>(&f)); return refines(static_cast<const FM&>(*this),dynamic_cast<const FM&>(f)); }
    VectorFunctionModelInterface<P,ARG,PR,PRE>* _concrete_refinement(const VectorFunctionModelInterface<P,ARG,PR,PRE>& f) const override {
        ARIADNE_ASSERT(dynamic_cast<const FM*>(&f)); return new FM(refinement(static_cast<const FM&>(*this),dynamic_cast<const FM&>(f))); }

    OutputStream& _write(OutputStream& os) const override {
        return os << static_cast<FM const&>(*this); }

};


template<class FCTRY, class P, class PR, class PRE> class FunctionModelFactoryMixin
    : public FunctionModelFactoryInterface<P,PR,PRE>
{
    typedef RealScalar SARG;
    typedef RealVector VARG;
    typedef BoxDomainType VD;
    typedef IntervalDomainType SD;
    friend class FunctionModelFactory<P,PR,PRE>;
  public:
    typedef VD VectorDomainType;
    typedef SD ScalarDomainType;
  public:
    virtual FunctionModelFactoryInterface<P,PR,PRE>* clone() const override { return new FCTRY(this->upcast()); }
    virtual OutputStream& _write(OutputStream& os) const override { return os << this->upcast(); }
/*
    CanonicalNumericType<P,PR,PRE> create(const Number<P>& number) const;
    ScalarFunctionModel<P,ARG,PR,PRE> create(const BoxDomainType& domain, const ScalarMultivariateFunctionInterface<P>& function) const;
    VectorFunctionModel<P,ARG,PR,PRE> create(const BoxDomainType& domain, const VectorMultivariateFunctionInterface<P>& function) const;
    ScalarFunctionModel<P,ARG,PR,PRE> create_zero(const BoxDomainType& domain) const;
    VectorFunctionModel<P,ARG,PR,PRE> create_zeros(SizeType result_size, const BoxDomainType& domain) const;
    ScalarFunctionModel<P,ARG,PR,PRE> create_constant(const BoxDomainType& domain, const Number<P>& value) const;
    ScalarFunctionModel<P,ARG,PR,PRE> create_constant(const BoxDomainType& domain, const CanonicalNumericType<P,PR,PRE>& value) const;
    VectorFunctionModel<P,ARG,PR,PRE> create_constants(const BoxDomainType& domain, const Vector<Number<P>>& values) const;
    VectorFunctionModel<P,ARG,PR,PRE> create_constants(const BoxDomainType& domain, const Vector<CanonicalNumericType<P,PR,PRE>>& values) const;
    ScalarFunctionModel<P,ARG,PR,PRE> create_coordinate(const BoxDomainType& domain, SizeType index) const;
    ScalarFunctionModel<P,ARG,PR,PRE> create_identity(const IntervalDomainType& domain) const;
    VectorFunctionModel<P,ARG,PR,PRE> create_identity(const BoxDomainType& domain) const;
    CanonicalNumericType<P,PR,PRE> create_number(const Number<P>& number) const;
*/
  private:
    template<class T> static inline T* heap_move(T&& t) { return new T(std::forward<T>(t)); }
    inline FCTRY const& upcast() const { return static_cast<FCTRY const&>(*this); }

    virtual CanonicalNumericType<P,PR,PRE> _create(const Number<P>& number) const override {
        return this->upcast().create(number); }
    virtual ScalarUnivariateFunctionPatchInterface<P>* _create(const ScalarDomainType& domain, const ScalarUnivariateFunctionInterface<P>& function) const override {
        ARIADNE_NOT_IMPLEMENTED; } // return heap_move(this->upcast().create(domain,function)); };
    virtual VectorUnivariateFunctionPatchInterface<P>* _create(const ScalarDomainType& domain, const VectorUnivariateFunctionInterface<P>& function) const override {
        ARIADNE_NOT_IMPLEMENTED; } // return heap_move(this->upcast().create(domain,function)); };
    virtual ScalarFunctionModelInterface<P,VARG,PR,PRE>* _create(const VectorDomainType& domain, const ScalarMultivariateFunctionInterface<P>& function) const override {
        return heap_move(this->upcast().create(domain,function)); };
    virtual VectorFunctionModelInterface<P,VARG,PR,PRE>* _create(const VectorDomainType& domain, const VectorMultivariateFunctionInterface<P>& function) const override {
         return heap_move(this->upcast().create(domain,function)); };

    virtual ScalarUnivariateFunctionPatchInterface<P>* _create_zero(const ScalarDomainType& domain) const override {
        ARIADNE_NOT_IMPLEMENTED; } // return heap_move(this->upcast().create_zero(domain)); };
    virtual ScalarUnivariateFunctionPatchInterface<P>* _create_constant(const ScalarDomainType& domain, const Number<P>& value) const override {
        ARIADNE_NOT_IMPLEMENTED; } // return heap_move(this->upcast().create_constant(domain,value)); };
    virtual VectorUnivariateFunctionPatchInterface<P>* _create_zeros(SizeType rsize, const ScalarDomainType& domain) const override {
        ARIADNE_NOT_IMPLEMENTED; } // return heap_move(this->upcast().create_zeros(rsize,domain)); };
    virtual VectorUnivariateFunctionPatchInterface<P>* _create_constants(const ScalarDomainType& domain, const Vector<Number<P>>& values) const override {
        ARIADNE_NOT_IMPLEMENTED; } // return heap_move(this->upcast().create_constants(domain,values)); };
    virtual ScalarUnivariateFunctionPatchInterface<P>* _create_identity(const ScalarDomainType& domain) const override {
        ARIADNE_NOT_IMPLEMENTED; } // return heap_move(this->upcast().create_identity(domain)); };


    virtual ScalarFunctionModelInterface<P,VARG,PR,PRE>* _create_zero(const VectorDomainType& domain) const override {
        return heap_move(this->upcast().create_zero(domain));

    };
    virtual ScalarFunctionModelInterface<P,VARG,PR,PRE>* _create_constant(const VectorDomainType& domain, const Number<P>& value) const override {
        return heap_move(this->upcast().create_constant(domain,value)); };
    virtual ScalarFunctionModelInterface<P,VARG,PR,PRE>* _create_coordinate(const VectorDomainType& domain, SizeType index) const override {
        return heap_move(this->upcast().create_coordinate(domain,index)); };
    virtual VectorFunctionModelInterface<P,VARG,PR,PRE>* _create_zeros(SizeType rsize, const VectorDomainType& domain) const override {
        return heap_move(this->upcast().create_zeros(rsize,domain)); };
    virtual VectorFunctionModelInterface<P,VARG,PR,PRE>* _create_constants(const VectorDomainType& domain, const Vector<Number<P>>& values) const override {
        return heap_move(this->upcast().create_constants(domain,values)); };
    virtual VectorFunctionModelInterface<P,VARG,PR,PRE>* _create_projection(const VectorDomainType& domain, Range indices) const override {
        return heap_move(this->upcast().create_projection(domain,indices)); };
    virtual VectorFunctionModelInterface<P,VARG,PR,PRE>* _create_identity(const VectorDomainType& domain) const override {
        return heap_move(this->upcast().create_identity(domain)); };
};


} // namespace Ariadne

#endif
