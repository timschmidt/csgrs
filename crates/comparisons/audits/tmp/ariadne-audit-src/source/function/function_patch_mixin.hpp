/***************************************************************************
 *            function/function_patch_mixin.hpp
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

/*! \file function/function_patch_mixin.hpp
 *  \brief Mixin for functions on bounded sets.
 */

#ifndef ARIADNE_FUNCTION_PATCH_MIXIN_HPP
#define ARIADNE_FUNCTION_PATCH_MIXIN_HPP

#include "../numeric/number.decl.hpp"
#include "../function/function.decl.hpp"

#include "../numeric/operators.hpp"
#include "../numeric/numeric.hpp"

#include "../algebra/vector.hpp"
#include "../algebra/matrix.hpp"
#include "../algebra/range.hpp"
#include "../algebra/operations.hpp"
#include "../algebra/algebra_interface.hpp"
#include "../algebra/algebra_mixin.hpp"

#include "../function/domain.hpp"
#include "../function/function_interface.hpp"
#include "../function/function_patch_interface.hpp"
#include "../function/function_mixin.hpp"


namespace Ariadne {

template<class FP, class P, class SIG> class FunctionPatchMixin;
template<class FP, class P, class... ARGS> using ScalarFunctionPatchMixin = FunctionPatchMixin<FP,P,RealScalar(ARGS...)>;
template<class FP, class P, class... ARGS> using VectorFunctionPatchMixin = FunctionPatchMixin<FP,P,RealVector(ARGS...)>;
template<class FP, class P> using ScalarUnivariateFunctionPatchMixin = FunctionPatchMixin<FP,P,RealScalar(RealScalar)>;
template<class FP, class P> using VectorUnivariateFunctionPatchMixin = FunctionPatchMixin<FP,P,RealVector(RealScalar)>;
template<class FP, class P> using ScalarMultivariateFunctionPatchMixin = FunctionPatchMixin<FP,P,RealScalar(RealVector)>;
template<class FP, class P> using VectorMultivariateFunctionPatchMixin = FunctionPatchMixin<FP,P,RealVector(RealVector)>;

template<class FP, class P, class... ARGS> class FunctionPatchMixin<FP,P,RealScalar(ARGS...)>
    : public virtual ScalarFunctionPatchInterface<P,ARGS...>
    , public ScalarFunctionMixin<FP,P,ARGS...>
    , public ElementaryAlgebraMixin<FP,Number<P>>
{
    using RES=RealScalar; using SIG=RES(ARGS...);
    using D=typename ScalarFunctionPatchInterface<P,ARGS...>::DomainType;
    using C=typename ScalarFunctionPatchInterface<P,ARGS...>::CodomainType;
  public:
    typedef D DomainType;
    typedef C CodomainType;
    using typename FunctionPatchInterface<P,SIG>::ErrorType;
    using typename FunctionPatchInterface<P,SIG>::NormType;
    typedef typename SignatureTraits<SIG>::BoundedRangeType RangeType;
    typedef Number<P> X;
  public:
    DomainType const domain() const override { return static_cast<FP const&>(*this).domain(); }
    CodomainType const codomain() const override { return static_cast<FP const&>(*this).codomain(); }

    ScalarFunctionPatchInterface<P,ARGS...>* _clone() const override {
        return new FP(static_cast<const FP&>(*this)); }

    ScalarFunctionPatchInterface<P,ARGS...>* _create_copy() const override {
        return new FP(static_cast<const FP&>(*this)); }
    ScalarFunctionPatchInterface<P,ARGS...>* _create_zero() const override {
        return new FP(factory(static_cast<FP const&>(*this)).create_zero()); }
    ScalarFunctionPatchInterface<P,ARGS...>* _create_constant(Number<P> const& c) const override {
        return new FP(factory(static_cast<FP const&>(*this)).create_constant(c)); }


    RangeType const _range() const override {
        return RangeType(static_cast<const FP&>(*this).range()); }
    ErrorType const _error() const override {
        return static_cast<ErrorType>(static_cast<const FP&>(*this).error()); }
    Scalar<ErrorType> const _errors() const override {
        return static_cast<ErrorType>(static_cast<const FP&>(*this).error()); }

    inline virtual Void _clobber() override {
        static_cast<FP&>(*this).clobber(); }
    inline virtual Bool _refines(FunctionPatchInterface<P,SIG> const& fp) const override {
        ARIADNE_ASSERT(dynamic_cast<const FP*>(&fp)); return refines(static_cast<const FP&>(*this),dynamic_cast<const FP&>(fp)); }
    inline virtual ScalarFunctionPatchInterface<P,ARGS...>* _compose(ScalarFunction<P,RES> const& f) const override {
        return heap_move(compose(f,static_cast<const FP&>(*this))); }
    inline virtual VectorFunctionPatchInterface<P,ARGS...>* _compose(VectorFunction<P,RES> const& f) const override {
        return heap_move(compose(f,static_cast<const FP&>(*this))); }
    inline virtual ScalarFunctionPatchInterface<P,ARGS...>* _unchecked_compose(ScalarFunction<P,RES> const& f) const override {
        return this->_compose(f); }
    inline virtual VectorFunctionPatchInterface<P,ARGS...>* _unchecked_compose(VectorFunction<P,RES> const& f) const override {
        return this->_compose(f); }

    NormType const _norm() const override {
        return static_cast<NormType>(norm(static_cast<const FP&>(*this))); }
    ScalarFunctionPatchInterface<P,ARGS...>* _derivative(SizeType j) const override {
        return new FP(derivative(static_cast<const FP&>(*this),j)); }
    ScalarFunctionPatchInterface<P,ARGS...>* _antiderivative(SizeType j) const override {
        return new FP(antiderivative(static_cast<const FP&>(*this),j)); }
    ScalarFunctionPatchInterface<P,ARGS...>* _antiderivative(SizeType j, Number<P> c) const override {
        return new FP(antiderivative(static_cast<const FP&>(*this),j,c)); }
     ScalarFunctionPatchInterface<P,ARGS...>* _restriction(const BoxDomainType& d) const override {
        return new FP(restriction(static_cast<const FP&>(*this),d)); }
    Number<P> _unchecked_evaluate(const Vector<Number<P>>& x) const override {
        return unchecked_evaluate(static_cast<const FP&>(*this),x); }
    CanonicalNumericType<P,DP> _unchecked_evaluate(const Vector<CanonicalNumericType<P,DP>>& x) const override {
        auto r = unchecked_evaluate(static_cast<const FP&>(*this),x);
        if constexpr (Convertible<decltype(r),CanonicalNumericType<P,DP>>) { return r; } else { ARIADNE_FAIL_MSG(""); }
    }
    CanonicalNumericType<P,MP> _unchecked_evaluate(const Vector<CanonicalNumericType<P,MP>>& x) const override {
        auto r = unchecked_evaluate(static_cast<const FP&>(*this),x);
        if constexpr (Convertible<decltype(r),CanonicalNumericType<P,MP>>) { return r; } else {ARIADNE_FAIL_MSG(""); }
    }
    ScalarFunctionPatchInterface<P,ARGS...>* _partial_evaluate(SizeType j, const Number<P>& c) const override {
        return heap_copy(partial_evaluate(static_cast<const FP&>(*this),j,c)); }
    ScalarFunctionPatchInterface<P,ARGS...>* _embed(const BoxDomainType& d1, const BoxDomainType& d2) const override {
        return new FP(embed(d1,static_cast<const FP&>(*this),d2)); }

    OutputStream& _write(OutputStream& os) const override {
        return os << static_cast<FP const&>(*this); }
};


template<class FP, class P, class... ARGS> class FunctionPatchMixin<FP,P,RealVector(ARGS...)>
    : public virtual VectorFunctionPatchInterface<P,ARGS...>
    , public VectorFunctionMixin<FP,P,ARGS...>
{
    using RES=RealVector; using SIG=RES(ARGS...);
    using C=BoxDomainType;
    using D=typename ScalarFunctionPatchInterface<P,ARGS...>::DomainType;
  public:
    typedef D DomainType;
    typedef C CodomainType;
    using typename FunctionPatchInterface<P,SIG>::ErrorType;
    using typename FunctionPatchInterface<P,SIG>::NormType;
    typedef typename VectorFunctionPatchInterface<P,ARGS...>::RangeType RangeType;
    typedef typename Element<FP>::Type ScalarMultivariateFunctionType;
  public:
    DomainType const domain() const override {
        return static_cast<FP const&>(*this).domain(); }
    CodomainType const codomain() const override {
        return static_cast<FP const&>(*this).codomain(); }

    ErrorType const _error() const override {
        return static_cast<ErrorType>(static_cast<const FP&>(*this).error()); }
    Vector<ErrorType> const _errors() const override {
        return static_cast<Vector<ErrorType>>(static_cast<const FP&>(*this).errors()); }

    inline virtual Void _clobber() override {
        static_cast<FP&>(*this).clobber(); }
    inline virtual Bool _refines(FunctionPatchInterface<P,SIG> const& fp) const override {
        ARIADNE_ASSERT(dynamic_cast<const FP*>(&fp)); return refines(static_cast<const FP&>(*this),dynamic_cast<const FP&>(fp)); }
    inline virtual ScalarFunctionPatchInterface<P,ARGS...>* _compose(ScalarFunction<P,RES> const& f) const override {
        return heap_move(compose(f,static_cast<const FP&>(*this))); }
    inline virtual VectorFunctionPatchInterface<P,ARGS...>* _compose(VectorFunction<P,RES> const& f) const override {
        return heap_move(compose(f,static_cast<const FP&>(*this))); }
    // FIXME: Unchecked compose dispatches to compose
    inline virtual ScalarFunctionPatchInterface<P,ARGS...>* _unchecked_compose(ScalarFunction<P,RES> const& f) const override {
        return this->_compose(f); }
    inline virtual VectorFunctionPatchInterface<P,ARGS...>* _unchecked_compose(VectorFunction<P,RES> const& f) const override {
        return this->_compose(f); }

    virtual VectorFunctionPatchInterface<P,ARGS...>* _clone() const override { return new FP(static_cast<const FP&>(*this)); }
    virtual Void _set(SizeType i, const ScalarFunctionPatchInterface<P,ARGS...>& sf) override {
        if(!dynamic_cast<const typename FP::ScalarMultivariateFunctionType*>(&sf)) {
            ARIADNE_FAIL_MSG("Cannot set element of VectorMultivariateFunctionPatch "<<*this<<" to "<<sf<<"\n"); }
        static_cast<FP&>(*this).FP::set(i,dynamic_cast<const ScalarMultivariateFunctionType&>(sf)); }
    virtual VectorFunctionPatchInterface<P,ARGS...>* _derivative(SizeType j) const override {
        ARIADNE_NOT_IMPLEMENTED; }
    virtual VectorFunctionPatchInterface<P,ARGS...>* _antiderivative(SizeType j) const override {
        ARIADNE_NOT_IMPLEMENTED; }
    virtual VectorFunctionPatchInterface<P,ARGS...>* _antiderivative(SizeType j, Number<P> c) const override {
        ARIADNE_NOT_IMPLEMENTED; }
    RangeType const _range() const override {
        return RangeType(static_cast<const FP&>(*this).range()); }
    NormType const _norm() const override {
         return norm(static_cast<const FP&>(*this)); }
    VectorFunctionPatchInterface<P,ARGS...>* _embed(const BoxDomainType& d1, const BoxDomainType& d2) const override {
        return heap_copy(embed(d1,static_cast<const FP&>(*this),d2)); }
    VectorFunctionPatchInterface<P,ARGS...>* _restriction(const BoxDomainType& d) const override {
        return new FP(restriction(static_cast<const FP&>(*this),d)); }
    Void _adjoin(const ScalarFunctionPatchInterface<P,ARGS...>& f) override {
        static_cast<FP&>(*this).FP::adjoin(dynamic_cast<const ScalarMultivariateFunctionType&>(f)); }
    VectorFunctionPatchInterface<P,ARGS...>* _join(const VectorFunctionPatchInterface<P,ARGS...>& f) const override {
        return heap_copy(join(static_cast<const FP&>(*this),dynamic_cast<const FP&>(f))); }
    VectorFunctionPatchInterface<P,ARGS...>* _combine(const VectorFunctionPatchInterface<P,ARGS...>& f) const override {
        return heap_copy(combine(static_cast<const FP&>(*this),dynamic_cast<const FP&>(f))); }
    Vector<Number<P>> _unchecked_evaluate(const Vector<Number<P>>& x) const override {
        return unchecked_evaluate(static_cast<const FP&>(*this),x); }
    Vector<CanonicalNumericType<P,DP>> _unchecked_evaluate(const Vector<CanonicalNumericType<P,DP>>& x) const override {
        auto r = unchecked_evaluate(static_cast<const FP&>(*this),x);
        if constexpr (Convertible<decltype(r),Vector<CanonicalNumericType<P,DP>>>) { return r; } else { ARIADNE_FAIL_MSG(""); }
    }
    Vector<CanonicalNumericType<P,MP>> _unchecked_evaluate(const Vector<CanonicalNumericType<P,MP>>& x) const override {
        auto r = unchecked_evaluate(static_cast<const FP&>(*this),x);
        if constexpr (Convertible<decltype(r),Vector<CanonicalNumericType<P,MP>>>) { return r; } else { ARIADNE_FAIL_MSG(""); }
    }

    VectorFunctionPatchInterface<P,ARGS...>* _partial_evaluate(SizeType j, const Number<P>& c) const override {
        return heap_copy(partial_evaluate(static_cast<const FP&>(*this),j,c)); }
};


template<class FCTRY, class P> class FunctionPatchFactoryMixin
    : public FunctionPatchFactoryInterface<P>
{
    typedef BoxDomainType VD;
    typedef IntervalDomainType SD;
  public:
    typedef VD VectorDomainType;
    typedef SD ScalarDomainType;


    virtual FunctionPatchFactoryInterface<P>* clone() const override { return new FCTRY(this->upcast()); }
    virtual OutputStream& _write(OutputStream& os) const override { return os << this->upcast(); }
  private:
    template<class T> static inline T* heap_move(T&& t) { return new T(std::forward<T>(t)); }
    inline FCTRY const& upcast() const { return static_cast<FCTRY const&>(*this); }

    virtual ScalarUnivariateFunctionPatchInterface<P>* _create(const ScalarDomainType& domain, const ScalarUnivariateFunctionInterface<P>& function) const override {
        ARIADNE_NOT_IMPLEMENTED; } // return heap_move(this->upcast().create(domain,function)); };
    virtual VectorUnivariateFunctionPatchInterface<P>* _create(const ScalarDomainType& domain, const VectorUnivariateFunctionInterface<P>& function) const override {
        ARIADNE_NOT_IMPLEMENTED; } // return heap_move(this->upcast().create(domain,function)); };

    virtual ScalarMultivariateFunctionPatchInterface<P>* _create(const VectorDomainType& domain, const ScalarMultivariateFunctionInterface<P>& function) const override {
        return heap_move(this->upcast().create(domain,function)); };
    virtual VectorMultivariateFunctionPatchInterface<P>* _create(const VectorDomainType& domain, const VectorMultivariateFunctionInterface<P>& function) const override {
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

    virtual ScalarMultivariateFunctionPatchInterface<P>* _create_zero(const VectorDomainType& domain) const override {
        return heap_move(this->upcast().create_zero(domain)); };
    virtual ScalarMultivariateFunctionPatchInterface<P>* _create_constant(const VectorDomainType& domain, const Number<P>& value) const override {
        return heap_move(this->upcast().create_constant(domain,value)); };
    virtual ScalarMultivariateFunctionPatchInterface<P>* _create_coordinate(const VectorDomainType& domain, SizeType index) const override {
        return heap_move(this->upcast().create_coordinate(domain,index)); };
    virtual VectorMultivariateFunctionPatchInterface<P>* _create_zeros(SizeType rsize, const VectorDomainType& domain) const override {
        return heap_move(this->upcast().create_zeros(rsize,domain)); };
    virtual VectorMultivariateFunctionPatchInterface<P>* _create_constants(const VectorDomainType& domain, const Vector<Number<P>>& values) const override {
        return heap_move(this->upcast().create_constants(domain,values)); };
    virtual VectorMultivariateFunctionPatchInterface<P>* _create_projection(const VectorDomainType& domain, Range indices) const override {
        return heap_move(this->upcast().create_projection(domain,indices)); };
    virtual VectorMultivariateFunctionPatchInterface<P>* _create_identity(const VectorDomainType& domain) const override {
        return heap_move(this->upcast().create_identity(domain)); };
};


} // namespace Ariadne

#endif
