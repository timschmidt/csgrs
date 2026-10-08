/***************************************************************************
 *            function/function_patch.hpp
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

/*! \file function/function_patch.hpp
 *  \brief Functions on bounded sets.
 */

#ifndef ARIADNE_FUNCTION_PATCH_HPP
#define ARIADNE_FUNCTION_PATCH_HPP

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
#include "../function/function.hpp"


namespace Ariadne {

template<class P> class FunctionPatchFactory;
using ValidatedFunctionPatchFactory = FunctionPatchFactory<ValidatedTag>;

// FIXME: Extend with univariate case
template<class P> class FunctionPatchFactory {
    SharedPointer<FunctionPatchFactoryInterface<P>> _ptr;
    typedef RealScalar SARG;
    typedef RealVector VARG;
  public:
    typedef FunctionPatchFactoryInterface<ValidatedTag> Interface;
    typedef P Paradigm;
    typedef IntervalDomainType ScalarDomainType;
    typedef BoxDomainType VectorDomainType;

    operator const FunctionPatchFactoryInterface<P>& () const { return *_ptr; }
    SharedPointer<FunctionPatchFactoryInterface<P>> managed_pointer() const { return this->_ptr; }

    explicit FunctionPatchFactory(FunctionPatchFactoryInterface<P>* p) : _ptr(p) { }
    explicit FunctionPatchFactory(SharedPointer<FunctionPatchFactoryInterface<P>> p) : _ptr(p) { }

    ScalarUnivariateFunctionPatch<P> create(ScalarDomainType const& dom, ScalarUnivariateFunction<P> const& f) const {
        return ScalarUnivariateFunctionPatch<P>(this->_ptr->_create(dom,f)); }
    VectorUnivariateFunctionPatch<P> create(ScalarDomainType const& dom, VectorUnivariateFunction<P> const& f) const {
        return VectorUnivariateFunctionPatch<P>(this->_ptr->_create(dom,f)); }
    ScalarMultivariateFunctionPatch<P> create(VectorDomainType const& dom, ScalarMultivariateFunction<P> const& f) const {
        return ScalarMultivariateFunctionPatch<P>(this->_ptr->_create(dom,f)); }
    VectorMultivariateFunctionPatch<P> create(VectorDomainType const& dom, VectorMultivariateFunction<P> const& f) const {
        return VectorMultivariateFunctionPatch<P>(this->_ptr->_create(dom,f)); }

    ScalarFunctionPatch<P,VARG> create_zero(VectorDomainType const& dom) const {
        return ScalarFunctionPatch<P,VARG>(this->_ptr->_create_zero(dom)); }
    ScalarFunctionPatch<P,VARG> create_constant(VectorDomainType const& dom, Number<P> const& c) const {
        return ScalarFunctionPatch<P,VARG>(this->_ptr->_create_constant(dom,c)); }
    template<class X> requires Constructible<Number<P>,X> ScalarFunctionPatch<P,VARG> create_constant(VectorDomainType const& dom, X const& c) const {
        return this->create_constant(dom,Number<P>(c)); }
    VectorFunctionPatch<P,VARG> create_constants(VectorDomainType const& dom, Vector<Number<P>> const& c) const {
        return VectorFunctionPatch<P,VARG>(this->_ptr->_create_constants(dom,c)); }
    ScalarFunctionPatch<P,VARG> create_coordinate(VectorDomainType const& dom, SizeType index) const {
        return ScalarFunctionPatch<P,VARG>(this->_ptr->_create_coordinate(dom,index)); }
    VectorFunctionPatch<P,VARG> create_zeros(SizeType rsize, VectorDomainType const& dom) const {
        return VectorFunctionPatch<P,VARG>(this->_ptr->_create_zeros(rsize,dom)); }
    VectorFunctionPatch<P,VARG> create_projection(VectorDomainType const& dom, Range indices) const {
        return VectorFunctionPatch<P,VARG>(this->_ptr->_create_projection(dom,indices)); }
    VectorFunctionPatch<P,VARG> create_identity(VectorDomainType const& dom) const {
        return VectorFunctionPatch<P,VARG>(this->_ptr->_create_identity(dom)); }

    ScalarFunctionPatch<P,SARG> create_zero(ScalarDomainType const& dom) const {
        return ScalarFunctionPatch<P,SARG>(this->_ptr->_create_zero(dom)); }
    VectorFunctionPatch<P,SARG> create_zeros(SizeType rsize, ScalarDomainType const& dom) const {
        return VectorFunctionPatch<P,SARG>(this->_ptr->_create_zeros(rsize,dom)); }
    ScalarFunctionPatch<P,SARG> create_constant(ScalarDomainType const& dom, Number<P> const& c) const {
        return ScalarFunctionPatch<P,SARG>(this->_ptr->_create_constant(dom,c)); }
    VectorFunctionPatch<P,SARG> create_constants(ScalarDomainType const& dom, Vector<Number<P>> const& c) const {
        return VectorFunctionPatch<P,SARG>(this->_ptr->_create_constants(dom,c)); }
    ScalarFunctionPatch<P,SARG> create_identity(ScalarDomainType const& dom) const {
        return ScalarFunctionPatch<P,SARG>(this->_ptr->_create_identity(dom)); }

    friend OutputStream& operator<<(OutputStream& os, FunctionPatchFactory<P> const& factory) { return factory._ptr->_write(os); }
};

template<class P> auto inline
FunctionPatchFactoryInterface<P>::create_identity(IntervalDomainType const& dom) const -> ScalarFunctionPatch<P,VARG> {
    return ScalarFunctionPatch<P,VARG>(this->create_coordinate(BoxDomainType(1u,dom),0u)); }

template<class FCTRY, class ARG> class FunctionPatchCreator {
    typedef typename FCTRY::Paradigm P;
  public:
    typedef FCTRY FactoryType;
    typedef typename DomainTraits<ARG>::BoundedDomainType DomainType;
    typedef P Paradigm;

    explicit FunctionPatchCreator(DomainType domain, FactoryType factory) : _factory(factory), _domain(domain) { }

    decltype(auto) create(ScalarFunction<P,ARG> const& f) { return this->_factory.create(this->_domain,f); }
    decltype(auto) create(VectorFunction<P,ARG> const& f) { return this->_factory.create(this->_domain,f); }
    decltype(auto) create_zero() { return this->_factory.create_zero(this->_domain); }
    decltype(auto) create_zeros(SizeType n) { return this->_factory.create_zeros(n,this->_domain); }
    decltype(auto) create_constant(Number<P> const& c) const { return this->_factory.create_constant(this->_domain,c); }
    decltype(auto) create_identity() { return this->_factory.create_identity(this->_domain); }
  protected:
    FactoryType _factory;
    DomainType _domain;
};

template<class F, class P, class SIG> concept FunctionPatchConcept = requires (F f) {
    typename F::DomainType;
    f.domain();
};


//! \ingroup FunctionModule
//! \ingroup FunctionPatchSubModule
//! \brief Generic functions on bounded domains.
template<class P, class SIG> class FunctionPatch;

//! \ingroup FunctionPatchSubModule
//! \brief Generic scalar functions on bounded domains.
template<class P, class... ARGS> class FunctionPatch<P,RealScalar(ARGS...)>
    : public DispatchElementaryAlgebraOperations<ScalarFunctionPatch<P,ARGS...>, Number<P>>
{
    using RES=RealScalar; using SIG=RES(ARGS...);
  public:
    typedef Function<P,SIG> GenericType;
    typedef P Paradigm;
    typedef FunctionPatchInterface<P,SIG> Interface;
    typedef typename FunctionPatchInterface<P,SIG>::DomainType DomainType;
    typedef typename FunctionPatchInterface<P,SIG>::CodomainType CodomainType;
    typedef typename FunctionPatchInterface<P,SIG>::RangeType RangeType;
    typedef ExactNumber CoefficientType;
    typedef PositiveValidatedUpperNumber ErrorType;
    typedef Number<P> NumericType;
    typedef PositiveValidatedUpperNumber NormType;
    typedef typename FunctionPatchInterface<P,SIG>::ResultSizeType ResultSizeType;
    typedef typename FunctionPatchInterface<P,SIG>::ArgumentSizeType ArgumentSizeType;
    typedef typename FunctionPatchInterface<P,SIG>::ArgumentIndexType ArgumentIndexType;

    template<class X> using Argument = typename SignatureTraits<SIG>::template Argument<X>;
    template<class X> using Result = typename SignatureTraits<SIG>::template Result<X>;
  public:
    clone_on_copy_ptr< FunctionPatchInterface<P,SIG> > _ptr;
  public:
    FunctionPatch() : _ptr() { }
    explicit FunctionPatch(FunctionPatchInterface<P,SIG>* p) : _ptr(p) { }
    FunctionPatch(const SharedPointer<const FunctionPatchInterface<P,SIG>> p) : _ptr(p->_clone()) { }
    FunctionPatch(const FunctionPatch<P,SIG>& f) : _ptr(f._ptr) { }
    FunctionPatch& operator=(const FunctionPatch<P,SIG>& f) { this->_ptr=f._ptr; return *this; }
        FunctionPatch(const FunctionPatchInterface<P,SIG>& f) : _ptr(f._clone()) { }
    FunctionPatch(const Function<P,SIG>& f) : _ptr(dynamic_cast<FunctionPatchInterface<P,SIG>*>(f.raw_pointer()->_clone())) { }
    template<class PR, class PRE> FunctionPatch(FunctionModel<P,SIG,PR,PRE> fm);
    operator Function<P,SIG>() const { return Function<P,SIG>(this->_ptr->_clone()); } // DEPRECATED
    friend Function<P,SIG> cast_unchecked(FunctionPatch<P,SIG> const& fp) {
        return Function<P,SIG>(fp._ptr->_clone()); }
    operator FunctionPatchInterface<P,SIG>& () { return *_ptr; }
    operator const FunctionPatchInterface<P,SIG>& () const { return *_ptr; }
    const FunctionPatchInterface<P,SIG>* raw_pointer() const { return _ptr.operator->(); }
    SharedPointer<FunctionPatchInterface<P,SIG>> managed_pointer() const {
        return SharedPointer<FunctionPatchInterface<P,SIG>>(this->_ptr->_clone()); }
    FunctionPatchInterface<P,SIG>& reference() { return *_ptr; }
    const FunctionPatchInterface<P,SIG>& reference() const { return *_ptr; }

    ScalarFunctionPatch<P,ARGS...>& operator=(const Number<P>& c);
    ScalarFunctionPatch<P,ARGS...>& operator=(const ScalarFunction<P,ARGS...>& f);
    ScalarFunctionPatch<P,ARGS...>& operator=(const ScalarFunctionPatchInterface<P,ARGS...>& f);

    inline ResultSizeType result_size() const { return this->_ptr->result_size(); }
    inline ArgumentSizeType argument_size() const { return this->_ptr->argument_size(); }
    template<class X> X operator() (const Argument<X>& x) const {
        return this->_ptr->_call(x); }
    template<class X> X evaluate(const Argument<X>& x) const {
        return this->_ptr->_call(x); }
    inline DomainType const domain() const { return this->_ptr->domain(); }
    inline CodomainType const codomain() const { return this->_ptr->codomain(); }
    inline RangeType const range() const { return this->_ptr->_range(); }

//    inline CoefficientType value() const { return this->_ptr->value(); }
    inline ErrorType error() const { return this->_ptr->_error(); }
    inline Void clobber() { return this->_ptr->_clobber(); }

    friend FunctionPatchCreator<FunctionPatchFactory<P>,ARGS...> factory(ScalarFunctionPatch<P,ARGS...> const& f) {
        FunctionPatchFactory<P> factory(f._ptr->_factory());
        return FunctionPatchCreator<FunctionPatchFactory<P>,ARGS...>(f.domain(),factory); }
  public:
    friend ScalarFunctionPatch<P,ARGS...> operator+(ScalarFunctionPatch<P,ARGS...> const& fp1, ScalarFunction<P,ARGS...> const& f2) {
        return fp1+factory(fp1).create(f2); }
    friend ScalarFunctionPatch<P,ARGS...> operator+(ScalarFunction<P,ARGS...> const& f1, ScalarFunctionPatch<P,ARGS...> const& fp2) {
        return factory(fp2).create(f1)+fp2; }
    friend ScalarFunctionPatch<P,ARGS...> operator-(ScalarFunctionPatch<P,ARGS...> const& fp1, ScalarFunction<P,ARGS...> const& f2) {
        return fp1-factory(fp1).create(f2); }
    friend ScalarFunctionPatch<P,ARGS...> operator-(ScalarFunction<P,ARGS...> const& f1, ScalarFunctionPatch<P,ARGS...> const& fp2) {
        return factory(fp2).create(f1)-fp2; }
    friend ScalarFunctionPatch<P,ARGS...> operator*(ScalarFunctionPatch<P,ARGS...> const& fp1, ScalarFunction<P,ARGS...> const& f2) {
        return fp1*factory(fp1).create(f2); }
    friend ScalarFunctionPatch<P,ARGS...> operator*(ScalarFunction<P,ARGS...> const& f1, ScalarFunctionPatch<P,ARGS...> const& fp2) {
        return factory(fp2).create(f1)*fp2; }
    friend ScalarFunctionPatch<P,ARGS...> operator/(ScalarFunctionPatch<P,ARGS...> const& fp1, ScalarFunction<P,ARGS...> const& f2) {
        return fp1/factory(fp1).create(f2); }
    friend ScalarFunctionPatch<P,ARGS...> operator/(ScalarFunction<P,ARGS...> const& f1, ScalarFunctionPatch<P,ARGS...> const& fp2) {
        return factory(fp2).create(f1)/fp2; }

    friend VectorFunctionPatch<P,ARGS...> operator*(ScalarFunctionPatch<P,ARGS...> const& fp1, VectorFunction<P,ARGS...> const& f2) {
        return fp1*factory(fp1).create(f2); }
    friend VectorFunctionPatch<P,ARGS...> operator*(VectorFunction<P,ARGS...> const& f1, ScalarFunctionPatch<P,ARGS...> const& fp2) {
        return factory(fp2).create(f1)*fp2; }
    friend VectorFunctionPatch<P,ARGS...> operator/(VectorFunction<P,ARGS...> const& f1, ScalarFunctionPatch<P,ARGS...> const& fp2) {
        return factory(fp2).create(f1)/fp2; }
  public:
    friend Number<P> evaluate(const ScalarFunctionPatch<P,ARGS...>& f, const Vector<Number<P>>& x) {
        return f._ptr->_call(x); }
    friend CanonicalNumericType<P,DP> evaluate(const ScalarFunctionPatch<P,ARGS...>& f, const Vector<CanonicalNumericType<P,DP>>& x) {
        return f._ptr->_call(x); }
    friend CanonicalNumericType<P,MP> evaluate(const ScalarFunctionPatch<P,ARGS...>& f, const Vector<CanonicalNumericType<P,MP>>& x) {
        return f._ptr->_call(x); }
    friend Number<P> unchecked_evaluate(const ScalarFunctionPatch<P,ARGS...>& f, const Vector<Number<P>>& x) {
        return f._ptr->_unchecked_evaluate(x); }
    friend CanonicalNumericType<P,DP> unchecked_evaluate(const ScalarFunctionPatch<P,ARGS...>& f, const Vector<CanonicalNumericType<P,DP>>& x) {
        return f._ptr->_unchecked_evaluate(x); }
    friend CanonicalNumericType<P,MP> unchecked_evaluate(const ScalarFunctionPatch<P,ARGS...>& f, const Vector<CanonicalNumericType<P,MP>>& x) {
        return f._ptr->_unchecked_evaluate(x); }

    friend ScalarFunctionPatch<P,ARGS...> partial_evaluate(const ScalarFunctionPatch<P,ARGS...>& f, SizeType j, const Number<P>& c) {
        return ScalarFunctionPatch<P,ARGS...>(f._ptr->_partial_evaluate(j,c)); }

    friend NormType norm(const ScalarFunctionPatch<P,ARGS...>& f) {
        return f._ptr->_generic_norm(); }
    friend ScalarFunctionPatch<P,ARGS...> derivative(const ScalarFunctionPatch<P,ARGS...>& f, SizeType j) {
        return ScalarFunctionPatch<P,ARGS...>(f._ptr->_derivative(j)); }
    friend ScalarFunctionPatch<P,ARGS...> antiderivative(const ScalarFunctionPatch<P,ARGS...>& f, SizeType j) {
        return ScalarFunctionPatch<P,ARGS...>(f._ptr->_antiderivative(j)); }
    friend ScalarFunctionPatch<P,ARGS...> antiderivative(const ScalarFunctionPatch<P,ARGS...>& f, SizeType j, Number<P> c) {
        return ScalarFunctionPatch<P,ARGS...>(f._ptr->_antiderivative(j,c)); }

    friend ScalarFunctionPatch<P,ARGS...> embed(const DomainType& d1, const ScalarFunctionPatch<P,ARGS...>& f, const DomainType& d2) {
        return ScalarFunctionPatch<P,ARGS...>(f._ptr->_embed(d1,d2)); }
    friend ScalarFunctionPatch<P,ARGS...> embed(const DomainType& d, const ScalarFunctionPatch<P,ARGS...>& f) {
        return embed(d,f,DomainType()); }
    friend ScalarFunctionPatch<P,ARGS...> embed(const ScalarFunctionPatch<P,ARGS...>& f, const BoxDomainType& d) {
        return embed(DomainType(),f,d); }
    friend ScalarFunctionPatch<P,ARGS...> embed(const ScalarFunctionPatch<P,ARGS...>& f, const IntervalDomainType& d) {
        return embed(f,DomainType(1,d)); }
    friend ScalarFunctionPatch<P,ARGS...> restriction(const ScalarFunctionPatch<P,ARGS...>& f, const DomainType& d) {
        return ScalarFunctionPatch<P,ARGS...>(f._ptr->_restriction(d)); }
    friend inline ScalarFunction<P,ARGS...> cast_unrestricted(ScalarFunctionPatch<P,ARGS...> const& f) {
        return ScalarFunction<P,ARGS...>(std::dynamic_pointer_cast<ScalarFunctionInterface<P,ARGS...>>(f.managed_pointer())); }

    friend VectorFunctionPatch<P,ARGS...> join(const ScalarFunctionPatch<P,ARGS...>& f1, const ScalarFunctionPatch<P,ARGS...>& f2) {
        return join(VectorFunctionPatch<P,ARGS...>(1,f1),f2); }
    friend VectorFunctionPatch<P,ARGS...> combine(const ScalarFunctionPatch<P,ARGS...>& f1, const ScalarFunctionPatch<P,ARGS...>& f2);
  public:
    friend inline ScalarFunctionPatch<P,ARGS...> compose(const ScalarUnivariateFunction<P>& f, const ScalarFunctionPatch<P,ARGS...>& g) {
        return ScalarFunctionPatch<P,ARGS...>(g._ptr->_compose(f)); }
    friend inline ScalarFunctionPatch<P,ARGS...> compose(const ScalarUnivariateFunctionPatch<P>& f, const ScalarFunctionPatch<P,ARGS...>& g) {
        return ScalarFunctionPatch<P,ARGS...>(g._ptr->_compose(cast_unchecked(f))); }
    friend inline VectorFunctionPatch<P,ARGS...> compose(const VectorUnivariateFunction<P>& f, const ScalarFunctionPatch<P,ARGS...>& g) {
        return VectorFunctionPatch<P,ARGS...>(g._ptr->_compose(f)); }
    friend inline VectorFunctionPatch<P,ARGS...> compose(const VectorUnivariateFunctionPatch<P>& f, const ScalarFunctionPatch<P,ARGS...>& g) {
        return VectorFunctionPatch<P,ARGS...>(g._ptr->_compose(cast_unchecked(f))); }

    friend inline ScalarFunctionPatch<P,ARGS...> unchecked_compose(const ScalarUnivariateFunctionPatch<P>& f, const ScalarFunctionPatch<P,ARGS...>& g) {
//        return ScalarFunctionPatch<P,ARGS...>(g._ptr->_unchecked_compose(f)); }
        return ScalarFunctionPatch<P,ARGS...>(g._ptr->_compose(cast_unchecked(f))); }
    friend inline VectorFunctionPatch<P,ARGS...> unchecked_compose(const VectorUnivariateFunctionPatch<P>& f, const ScalarFunctionPatch<P,ARGS...>& g) {
        return VectorFunctionPatch<P,ARGS...>(g._ptr->_compose(cast_unchecked(f))); }
  public:
    friend Bool inconsistent(ScalarFunctionPatch<P,ARGS...> const& f1, ScalarFunctionPatch<P,ARGS...> const& f2);
    friend Bool refines(ScalarFunctionPatch<P,ARGS...> const& f1, ScalarFunctionPatch<P,ARGS...> const& f2) {
        return f1._ptr->_refines(f2); }
    friend ScalarFunctionPatch<P,ARGS...> refinement(ScalarFunctionPatch<P,ARGS...> const& f1, ScalarFunctionPatch<P,ARGS...> const& f2);
  public:
    friend OutputStream& operator<<(OutputStream& os, const ScalarFunctionPatch<P,ARGS...>& f) {
        return os <<  f.operator ScalarFunction<P,ARGS...>(); }
};

template<class P, class... ARGS> struct AlgebraOperations<ScalarFunctionPatch<P,ARGS...>> {

    typedef ScalarFunctionPatch<P,ARGS...> FM; typedef Number<P> X;
    typedef ScalarFunctionPatchInterface<P,ARGS...> FMI;

    static FM apply(BinaryElementaryOperator op, const FM& f1, const FM& f2) {
        return FM(&dynamic_cast<FMI&>(*f1._ptr->_apply(op,*f2._ptr))); }
    static FM apply(BinaryElementaryOperator op, const FM& f1, const X& c2) {
        return FM(&dynamic_cast<FMI&>(*f1._ptr->_apply(op,c2))); }
    static FM apply(BinaryElementaryOperator op, const X& c1, const FM& f2) {
        return FM(&dynamic_cast<FMI&>(*f2._ptr->_rapply(op,c1))); }
    static FM apply(UnaryElementaryOperator op, const FM& f) {
        return FM(&dynamic_cast<FMI&>(*f._ptr->_apply(op))); }
    static FM apply(GradedElementaryOperator op, const FM& f, Int n) {
        return FM(&dynamic_cast<FMI&>(*f._ptr->_apply(op,n))); }
};


template<class P, class... ARGS> inline auto
FunctionPatch<P,RealScalar(ARGS...)>::operator=(const Number<P>& c) -> ScalarFunctionPatch<P,ARGS...>& {
    return (*this)=factory(*this).create_constant(c); }
template<class P, class... ARGS> inline auto
FunctionPatch<P,RealScalar(ARGS...)>::operator=(const ScalarFunction<P,ARGS...>& f) -> ScalarFunctionPatch<P,ARGS...>& {
    return (*this)=factory(*this).create(f); }
template<class P, class... ARGS> inline auto
FunctionPatch<P,RealScalar(ARGS...)>::operator=(const ScalarFunctionPatchInterface<P,ARGS...>& f) -> ScalarFunctionPatch<P,ARGS...>& {
    return (*this)=ScalarFunctionPatch<P,ARGS...>(f._clone()); }



template<class V> struct Element;

template<class M> class ScaledFunctionPatch;
template<class M> class VectorScaledFunctionPatch;

template<class P, class... ARGS> class VectorFunctionPatchElement
    : public DispatchTranscendentalAlgebraOperations<ScalarFunctionPatch<P,ARGS...>, Number<P>>
    , public ProvideConcreteGenericArithmeticOperators<ScalarFunctionPatch<P,ARGS...>>
{
    VectorFunctionPatch<P,ARGS...>* _p; SizeType _i;
  public:
    typedef typename ScalarFunctionPatch<P,ARGS...>::GenericType GenericType;
    operator const ScalarFunctionPatch<P,ARGS...> () const;
    VectorFunctionPatchElement(VectorFunctionPatch<P,ARGS...>* p, SizeType i) : _p(p), _i(i) { }
    VectorFunctionPatchElement<P,ARGS...>& operator=(const ScalarFunctionPatch<P,ARGS...>& sf) {
        _p->set(_i,sf); return *this; }
    VectorFunctionPatchElement<P,ARGS...>& operator=(const VectorFunctionPatchElement<P,ARGS...>& sf) {
        return this->operator=(static_cast<ScalarFunctionPatch<P,ARGS...>const>(sf)); }
    Void clobber() { ScalarFunctionPatch<P,ARGS...> sf=_p->get(_i); sf.clobber(); _p->set(_i,sf); }
    const ScalarFunctionPatch<P,ARGS...> model() const { ScalarFunctionPatch<P,ARGS...> sf=_p->get(_i); return sf.model(); }
    const PositiveValidatedUpperNumber error() const { ScalarFunctionPatch<P,ARGS...> sf=_p->get(_i); return sf.error(); }
    decltype(auto) argument_size() const { return _p->get(_i).argument_size(); }
    decltype(auto) domain() const { return _p->get(_i).domain(); }
    IntervalRangeType range() const { return _p->get(_i).range(); }
    friend Bool inconsistent(ScalarFunctionPatch<P,ARGS...> const& f1, ScalarFunctionPatch<P,ARGS...> const& f2);
    friend Bool refines(ScalarFunctionPatch<P,ARGS...> const& f1, ScalarFunctionPatch<P,ARGS...> const& f2);
    friend ScalarFunctionPatch<P,ARGS...> refinement(ScalarFunctionPatch<P,ARGS...> const& f1, ScalarFunctionPatch<P,ARGS...> const&);
    friend inline OutputStream& operator<<(OutputStream& os, const VectorFunctionPatchElement<P,ARGS...>& function) {
        return os << static_cast< const ScalarFunctionPatch<P,ARGS...> >(function); }
};

//! \ingroup FunctionPatchSubModule
//! \brief Generic vector functions on bounded domains.
template<class P, class... ARGS> class FunctionPatch<P,RealVector(ARGS...)>
{
    using RES=RealVector; using SIG=RES(ARGS...);
  public:
    clone_on_copy_ptr< VectorFunctionPatchInterface<P,ARGS...> > _ptr;
  public:
    typedef P Paradigm;
    typedef FunctionPatchInterface<P,SIG> Interface;
    typedef VectorFunction<P,ARGS...> GenericType;
    typedef typename FunctionPatchInterface<P,SIG>::DomainType DomainType;
    typedef typename FunctionPatchInterface<P,SIG>::CodomainType CodomainType;
    typedef typename FunctionPatchInterface<P,SIG>::RangeType RangeType;
    typedef ExactNumber CoefficientType;
    typedef PositiveValidatedUpperNumber ErrorType;
    typedef Number<P> NumericType;
    typedef PositiveValidatedUpperNumber NormType;
    typedef typename FunctionPatchInterface<P,SIG>::ResultSizeType ResultSizeType;
    typedef typename FunctionPatchInterface<P,SIG>::ArgumentSizeType ArgumentSizeType;
    typedef typename FunctionPatchInterface<P,SIG>::ArgumentIndexType ArgumentIndexType;
    template<class X> using Argument = typename SignatureTraits<SIG>::template Argument<X>;
    template<class X> using Result = typename SignatureTraits<SIG>::template Result<X>;
  public:
    inline FunctionPatch() : _ptr() { }
    inline FunctionPatch(SharedPointer<const FunctionPatchInterface<P,SIG>> vfp)
        : _ptr(vfp->_clone()) { }
    inline FunctionPatch(SizeType n, const ScalarFunctionPatchInterface<P,ARGS...>& sf) {
        FunctionPatchFactory<P> factory(sf._factory()); *this=factory.create_zeros(n,sf.domain());
        for(SizeType i=0; i!=n; ++i) { (*this)[i]=sf; } }
    inline FunctionPatch(Array<ScalarFunctionPatch<P,ARGS...>> const& asf)
        : FunctionPatch(asf.size(),asf[0]) { for(SizeType i=0; i!=asf.size(); ++i) { (*this)[i]=asf[i]; } }
    inline FunctionPatch(List<ScalarFunctionPatch<P,ARGS...>> const& lsf)
        : FunctionPatch(lsf.size(),lsf[0]) { for(SizeType i=0; i!=lsf.size(); ++i) { (*this)[i]=lsf[i]; } }
    inline explicit FunctionPatch(FunctionPatchInterface<P,SIG>* p) : _ptr(p) { }
    template<class PR, class PRE> FunctionPatch(FunctionModel<P,SIG,PR,PRE> fm);
    inline FunctionPatch(const FunctionPatchInterface<P,SIG>& f) : _ptr(f._clone()) { }
    inline FunctionPatch(const FunctionPatch<P,SIG>& f) : _ptr(f._ptr) { }
    inline FunctionPatch& operator=(const FunctionPatch<P,SIG>& f) { this->_ptr=f._ptr; return *this; }
    inline operator const FunctionPatchInterface<P,SIG>& () const { return *_ptr; }
    inline operator Function<P,SIG> () const { return Function<P,SIG>(*_ptr); } // DEPRECATED
    friend inline Function<P,SIG> cast_unchecked(FunctionPatch<P,SIG> const& fp) {
        return Function<P,SIG>(*fp._ptr); }
    inline const FunctionPatchInterface<P,SIG>* raw_pointer() const { return _ptr.operator->(); }
    SharedPointer<FunctionPatchInterface<P,SIG>> managed_pointer() const {
        return SharedPointer<FunctionPatchInterface<P,SIG>>(this->_ptr->_clone()); }
    inline const FunctionPatchInterface<P,SIG>& reference() const { return *_ptr; }
    inline FunctionPatchInterface<P,SIG>& reference() { return *_ptr; }

    inline ResultSizeType result_size() const { return this->_ptr->result_size(); }
    inline ArgumentSizeType argument_size() const { return this->_ptr->argument_size(); }
    inline SizeType size() const { return this->_ptr->result_size(); }
    template<class XX> inline Vector<XX> operator()(const Argument<XX>& v) const { return this->_ptr->_call(v); }
    template<class XX> inline Vector<XX> evaluate(const Argument<XX>& v) const { return this->_ptr->_call(v); }
    inline ScalarFunctionPatch<P,ARGS...> const get(SizeType i) const { return ScalarFunctionPatch<P,ARGS...>(this->_ptr->_get(i)); }
    inline Void set(SizeType i, ScalarFunctionPatch<P,ARGS...> const& sf) { this->_ptr->_set(i,sf); }
    inline ScalarFunctionPatch<P,ARGS...> const operator[](SizeType i) const { return this->get(i); }
    inline VectorFunctionPatchElement<P,ARGS...> operator[](SizeType i) { return VectorFunctionPatchElement<P,ARGS...>(this,i); }
    inline VectorFunctionPatch<P,ARGS...> operator[](Range rng) { VectorFunctionPatch<P,ARGS...> r=factory(*this).create_zeros(rng.size());
        for(SizeType i=0; i!=rng.size(); ++i) { r[i]=this->operator[](rng[i]); } return r; }
    inline DomainType const domain() const { return this->_ptr->domain(); }
    inline CodomainType const codomain() const { return this->_ptr->codomain(); }
    inline RangeType const range() const { return this->_ptr->_range(); }
    inline Vector<ErrorType> const errors() const { return this->_ptr->_errors(); }
    inline ErrorType const error() const { return this->_ptr->_error(); }
    inline Void clobber() { this->_ptr->_clobber(); }
    inline Matrix<NumericType> const jacobian(const Vector<NumericType>& x) const;

  public:
    friend FunctionPatchCreator<FunctionPatchFactory<P>,ARGS...> factory(VectorFunctionPatch<P,ARGS...> const& f) {
        FunctionPatchFactory<P> factory(f._ptr->_factory());
        return FunctionPatchCreator<FunctionPatchFactory<P>,ARGS...>(f.domain(),factory); }
  public:
    friend inline ScalarFunctionPatch<P,ARGS...> compose(const ScalarMultivariateFunction<P>& f, const VectorFunctionPatch<P,ARGS...>& g) {
        return ScalarFunctionPatch<P,ARGS...>(g._ptr->_compose(f)); }
    friend inline ScalarFunctionPatch<P,ARGS...> compose(const ScalarMultivariateFunctionPatch<P>& f, const VectorFunctionPatch<P,ARGS...>& g) {
        return ScalarFunctionPatch<P,ARGS...>(g._ptr->_compose(cast_unchecked(f))); }
    friend inline VectorFunctionPatch<P,ARGS...> compose(const VectorMultivariateFunction<P>& f, const VectorFunctionPatch<P,ARGS...>& g) {
        return VectorFunctionPatch<P,ARGS...>(g._ptr->_compose(f)); }
    friend inline VectorFunctionPatch<P,ARGS...> compose(const VectorMultivariateFunctionPatch<P>& f, const VectorFunctionPatch<P,ARGS...>& g) {
        return VectorFunctionPatch<P,ARGS...>(g._ptr->_compose(cast_unchecked(f))); }

    friend inline ScalarFunctionPatch<P,ARGS...> unchecked_compose(const ScalarMultivariateFunctionPatch<P>& f, const VectorFunctionPatch<P,ARGS...>& g) {
//        return ScalarFunctionPatch<P,ARGS...>(g._ptr->_unchecked_compose(f)); }
        return ScalarFunctionPatch<P,ARGS...>(g._ptr->_compose(cast_unchecked(f))); }
    friend inline VectorFunctionPatch<P,ARGS...> unchecked_compose(const VectorMultivariateFunctionPatch<P>& f, const VectorFunctionPatch<P,ARGS...>& g) {
        return VectorFunctionPatch<P,ARGS...>(g._ptr->_compose(cast_unchecked(f))); }

    friend inline VectorFunctionPatch<P,ARGS...> operator+(const VectorFunctionPatch<P,ARGS...>& f) {
        return VectorFunctionPatch<P,ARGS...>(f._ptr->_clone()); }
    friend inline VectorFunctionPatch<P,ARGS...> operator-(const VectorFunctionPatch<P,ARGS...>& f) {
        VectorFunctionPatch<P,ARGS...> r=f; for(SizeType i=0; i!=r.size(); ++i) { r[i]=-f[i]; } return r; }
    friend inline VectorFunctionPatch<P,ARGS...> operator+(const VectorFunctionPatch<P,ARGS...>& f1, const VectorFunctionPatch<P,ARGS...>& f2) {
        VectorFunctionPatch<P,ARGS...> r=f1; for(SizeType i=0; i!=r.size(); ++i) { r[i]=f1[i]+f2[i]; } return r; }
    friend inline VectorFunctionPatch<P,ARGS...> operator-(const VectorFunctionPatch<P,ARGS...>& f1, const VectorFunctionPatch<P,ARGS...>& f2) {
        VectorFunctionPatch<P,ARGS...> r=f1; for(SizeType i=0; i!=r.size(); ++i) { r[i]=f1[i]-f2[i]; } return r; }
    friend inline VectorFunctionPatch<P,ARGS...> operator+(const VectorFunctionPatch<P,ARGS...>& f1, const Vector<Number<P>>& c2) {
        VectorFunctionPatch<P,ARGS...> r=f1; for(SizeType i=0; i!=r.size(); ++i) { r[i]=f1[i]+c2[i]; } return r; }
    friend inline VectorFunctionPatch<P,ARGS...> operator-(const VectorFunctionPatch<P,ARGS...>& f1, const Vector<Number<P>>& c2) {
        VectorFunctionPatch<P,ARGS...> r=f1; for(SizeType i=0; i!=r.size(); ++i) { r[i]=f1[i]-c2[i]; } return r; }
    friend inline VectorFunctionPatch<P,ARGS...> operator*(const ScalarFunctionPatch<P,ARGS...>& f1, const VectorFunctionPatch<P,ARGS...>& f2) {
        VectorFunctionPatch<P,ARGS...> r=f2; for(SizeType i=0; i!=r.size(); ++i) { r[i]=f1*f2[i]; } return r; }
    friend inline VectorFunctionPatch<P,ARGS...> operator*(const VectorFunctionPatch<P,ARGS...>& f1, const ScalarFunctionPatch<P,ARGS...>& f2) {
        VectorFunctionPatch<P,ARGS...> r=f1; for(SizeType i=0; i!=r.size(); ++i) { r[i]=f1[i]*f2; } return r; }
    friend inline VectorFunctionPatch<P,ARGS...> operator/(const VectorFunctionPatch<P,ARGS...>& f1, const ScalarFunctionPatch<P,ARGS...>& f2) {
        VectorFunctionPatch<P,ARGS...> r=f1; for(SizeType i=0; i!=r.size(); ++i) { r[i]=f1[i]/f2; } return r; }
    friend inline VectorFunctionPatch<P,ARGS...> operator+(const Vector<Number<P>>& c1, const VectorFunctionPatch<P,ARGS...>& f2) {
        VectorFunctionPatch<P,ARGS...> r=f2; for(SizeType i=0; i!=r.size(); ++i) { r[i]=c1[i]+f2[i]; } return r; }
    friend inline VectorFunctionPatch<P,ARGS...> operator-(const Vector<Number<P>>& c1, const VectorFunctionPatch<P,ARGS...>& f2) {
        VectorFunctionPatch<P,ARGS...> r=f2; for(SizeType i=0; i!=r.size(); ++i) { r[i]=c1[i]-f2[i]; } return r; }
    friend inline VectorFunctionPatch<P,ARGS...> operator*(const VectorFunctionPatch<P,ARGS...>& f1, const Number<P>& c2) {
        VectorFunctionPatch<P,ARGS...> r=f1; for(SizeType i=0; i!=r.size(); ++i) { r[i]=f1[i]*c2; } return r; }
    friend inline VectorFunctionPatch<P,ARGS...> operator*(const Number<P>& c1, const VectorFunctionPatch<P,ARGS...>& f2) {
        VectorFunctionPatch<P,ARGS...> r=f2; for(SizeType i=0; i!=r.size(); ++i) { r[i]=c1*f2[i]; } return r; }

    friend inline VectorFunctionPatch<P,ARGS...> operator+(const VectorFunctionPatch<P,ARGS...>& f1, const VectorMultivariateFunction<P>& f2) {
        return f1+factory(f1).create(f2); }
    friend inline VectorFunctionPatch<P,ARGS...> operator-(const VectorFunctionPatch<P,ARGS...>& f1, const VectorMultivariateFunction<P>& f2) {
        return f1-factory(f1).create(f2); }
    friend inline VectorFunctionPatch<P,ARGS...> operator*(const VectorFunctionPatch<P,ARGS...>& f1, const ScalarMultivariateFunction<P>& f2) {
        return f1*factory(f1).create(f2); }
    friend inline VectorFunctionPatch<P,ARGS...> operator/(const VectorFunctionPatch<P,ARGS...>& f1, const ScalarMultivariateFunction<P>& f2) {
        return f1/factory(f1).create(f2); }
    friend inline VectorFunctionPatch<P,ARGS...> operator+(const VectorMultivariateFunction<P>& f1, const VectorFunctionPatch<P,ARGS...>& f2) {
        return factory(f2).create(f1)+f2; }
    friend inline VectorFunctionPatch<P,ARGS...> operator-(const VectorMultivariateFunction<P>& f1, const VectorFunctionPatch<P,ARGS...>& f2) {
        return factory(f2).create(f1)-f2; }
    friend inline VectorFunctionPatch<P,ARGS...> operator*(const ScalarMultivariateFunction<P>& f1, const VectorFunctionPatch<P,ARGS...>& f2) {
        return factory(f2).create(f1)*f2; }
    friend inline VectorFunctionPatch<P,ARGS...> operator/(const ScalarMultivariateFunction<P>& f1, const VectorFunctionPatch<P,ARGS...>& f2) {
        return factory(f2).create(f1)/f2; }


  public:
    friend NormType norm(const VectorFunctionPatch<P,ARGS...>& f) {
        return f._ptr->_norm(); }
    friend VectorFunctionPatch<P,ARGS...> embed(const DomainType& d1, const VectorFunctionPatch<P,ARGS...>& f, const DomainType& d2) {
        return VectorFunctionPatch<P,ARGS...>(f._ptr->_embed(d1,d2)); }
    friend VectorFunctionPatch<P,ARGS...> embed(const DomainType& d, const VectorFunctionPatch<P,ARGS...>& f) {
        return embed(d,f,DomainType()); }
    friend VectorFunctionPatch<P,ARGS...> embed(const VectorFunctionPatch<P,ARGS...>& f, const BoxDomainType& d) {
        return embed(DomainType(),f,d); }
    friend VectorFunctionPatch<P,ARGS...> embed(const VectorFunctionPatch<P,ARGS...>& f, const IntervalDomainType& d) {
        return embed(f,DomainType(1,d)); }
    friend VectorFunctionPatch<P,ARGS...> restriction(const VectorFunctionPatch<P,ARGS...>& f, const DomainType& d) {
        return VectorFunctionPatch<P,ARGS...>(f._ptr->_restriction(d)); }
    friend inline VectorFunction<P,ARGS...> cast_unrestricted(VectorFunctionPatch<P,ARGS...> const& f) {
        return VectorFunction<P,ARGS...>(std::dynamic_pointer_cast<VectorFunctionInterface<P,ARGS...>>(f.managed_pointer())); }

    friend Vector<Number<P>> evaluate(const VectorFunctionPatch<P,ARGS...>& f, const Vector<Number<P>>& x) {
        return f._ptr->_call(x); }
    friend Vector<CanonicalNumericType<P,DP>> evaluate(const VectorFunctionPatch<P,ARGS...>& f, const Vector<CanonicalNumericType<P,DP>>& x) {
        return f._ptr->_call(x); }
    friend Vector<CanonicalNumericType<P,MP>> evaluate(const VectorFunctionPatch<P,ARGS...>& f, const Vector<CanonicalNumericType<P,MP>>& x) {
        return f._ptr->_call(x); }

    friend Vector<Number<P>> unchecked_evaluate(const VectorFunctionPatch<P,ARGS...>& f, const Vector<Number<P>>& x) {
        return f._ptr->_unchecked_evaluate(x); }
    friend Vector<CanonicalNumericType<P,DP>> unchecked_evaluate(const VectorFunctionPatch<P,ARGS...>& f, const Vector<CanonicalNumericType<P,DP>>& x) {
        return f._ptr->_unchecked_evaluate(x); }
    friend Vector<CanonicalNumericType<P,MP>> unchecked_evaluate(const VectorFunctionPatch<P,ARGS...>& f, const Vector<CanonicalNumericType<P,MP>>& x) {
        return f._ptr->_unchecked_evaluate(x); }

    friend VectorFunctionPatch<P,ARGS...> partial_evaluate(const VectorFunctionPatch<P,ARGS...>& f, SizeType j, const Number<P>& c) {
        return VectorFunctionPatch<P,ARGS...>(f._ptr->_partial_evaluate(j,c)); }

    friend OutputStream& operator<<(OutputStream& os, const VectorFunctionPatch<P,ARGS...>& f) {
        return os <<  f.operator VectorFunction<P,ARGS...>(); }

    friend ScalarFunctionPatch<P,ARGS...> unchecked_compose(const ScalarMultivariateFunction<P>& f, const VectorFunctionPatch<P,ARGS...>& g) {
        ScalarFunctionPatchInterface<P,ARGS...> const* fptr = dynamic_cast<ScalarFunctionPatchInterface<P,ARGS...> const*>(f.raw_pointer());
        if(fptr) { return unchecked_compose(ScalarFunctionPatch<P,ARGS...>(*fptr),g); } else { return compose(f,g); } }
    friend VectorFunctionPatch<P,ARGS...> unchecked_compose(const VectorMultivariateFunction<P>& f, const VectorFunctionPatch<P,ARGS...>& g) {
        VectorFunctionPatchInterface<P,ARGS...> const* fptr = dynamic_cast<VectorFunctionPatchInterface<P,ARGS...> const*>(f.raw_pointer());
        if(fptr) { return unchecked_compose(VectorFunctionPatch<P,ARGS...>(*fptr),g); } else { return compose(f,g); } }

    friend VectorFunctionPatch<P,ARGS...> antiderivative(const VectorFunctionPatch<P,ARGS...>& f, SizeType j) {
        VectorFunctionPatch<P,ARGS...> r(f);
        for(SizeType i=0; i!=r.size(); ++i) { r[i]=antiderivative(f[i],j); }
        return r;
    }

    friend VectorFunctionPatch<P,ARGS...> antiderivative(const VectorFunctionPatch<P,ARGS...>& f, SizeType j, const Number<P>& c) {
        VectorFunctionPatch<P,ARGS...> r(f);
        for(SizeType i=0; i!=r.size(); ++i) { r[i]=antiderivative(f[i],j,c); }
        return r;
    }

    friend VectorFunctionPatch<P,ARGS...> join(const ScalarFunctionPatch<P,ARGS...>& f1, const ScalarFunctionPatch<P,ARGS...>& f2);
    friend VectorFunctionPatch<P,ARGS...> join(const ScalarFunctionPatch<P,ARGS...>& f1, const VectorFunctionPatch<P,ARGS...>& f2) {
        return join(VectorFunctionPatch<P,ARGS...>(1u,f1),f2); }
    friend VectorFunctionPatch<P,ARGS...> join(const VectorFunctionPatch<P,ARGS...>& f1, const ScalarFunctionPatch<P,ARGS...>& f2) {
        VectorFunctionPatch<P,ARGS...> r=VectorFunctionPatch<P,ARGS...>(f1._ptr->_clone()); r._ptr->_adjoin(f2); return r; }
    friend VectorFunctionPatch<P,ARGS...> join(const VectorFunctionPatch<P,ARGS...>& f1, const VectorFunctionPatch<P,ARGS...>& f2) {
        return VectorFunctionPatch<P,ARGS...>(f1._ptr->_join(f2)); }

    friend VectorFunctionPatch<P,ARGS...> combine(const ScalarFunctionPatch<P,ARGS...>& f1, const ScalarFunctionPatch<P,ARGS...>& f2) {
        return VectorFunctionPatch<P,ARGS...>(1,f1)._ptr->_combine(VectorFunctionPatch<P,ARGS...>(1,f2)); };
    friend VectorFunctionPatch<P,ARGS...> combine(const ScalarFunctionPatch<P,ARGS...>& f1, const VectorFunctionPatch<P,ARGS...>& f2) {
        return VectorFunctionPatch<P,ARGS...>(1,f1)._ptr->_combine(f2); };
    friend VectorFunctionPatch<P,ARGS...> combine(const VectorFunctionPatch<P,ARGS...>& f1, const ScalarFunctionPatch<P,ARGS...>& f2) {
        return VectorFunctionPatch<P,ARGS...>(f1._ptr->_combine(VectorFunctionPatch<P,ARGS...>(1,f2))); };
    friend VectorFunctionPatch<P,ARGS...> combine(const VectorFunctionPatch<P,ARGS...>& f1, const VectorFunctionPatch<P,ARGS...>& f2) {
        return VectorFunctionPatch<P,ARGS...>(f1._ptr->_combine(f2)); }
  public:
    friend Bool inconsistent(VectorFunctionPatch<P,ARGS...> const& f1, VectorFunctionPatch<P,ARGS...> const& f2);
    friend Bool refines(VectorFunctionPatch<P,ARGS...> const& f1, VectorFunctionPatch<P,ARGS...> const& f2) {
        return f1._ptr->_refines(f2); }
    friend VectorFunctionPatch<P,ARGS...> refinement(VectorFunctionPatch<P,ARGS...> const& f1, VectorFunctionPatch<P,ARGS...> const& f2);
};


template<class P, class... ARGS> VectorFunctionPatchElement<P,ARGS...>::operator const ScalarFunctionPatch<P,ARGS...> () const {
    return ScalarFunctionPatch<P,ARGS...>(_p->get(_i)); }


template<class P, class SIG> class UncheckedFunctionPatch {
    FunctionPatch<P,SIG> _fp;
  public:
    template<class... ARGS> auto operator() (ARGS... x) const { return unchecked_evaluate(this->_fp, x...); }
};

} // namespace Ariadne

#endif
