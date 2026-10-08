/***************************************************************************
 *            algebra/algebra_mixin.hpp
 *
 *  Copyright  2010-20  Pieter Collins
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

/*! \file algebra/algebra_wrapper.hpp
 *  \brief Mixin class providing operations for (Banach) algebras.
 */

#ifndef ARIADNE_ALGEBRA_WRAPPER_HPP
#define ARIADNE_ALGEBRA_WRAPPER_HPP

#include "algebra/algebra_interface.hpp"
#include "algebra/algebra_mixin.hpp"
#include "numeric/operators.hpp"

namespace Ariadne {

template<class A, class X=NumericType<A>> class AlgebraWrapper;

template<class A, class X> requires AnAlgebraOver<A,X> class AlgebraWrapper<A,X>
    : public virtual AlgebraInterface<X>
    , public A
{
    static AlgebraInterface<X>* heap_move(A&& a) { return new AlgebraWrapper<A,X>(std::forward<A>(a)); }
  public:
    typedef X NumericType;

    AlgebraWrapper(A&& a) : A(std::move(a)) { }
    AlgebraWrapper(A const& a) : A(a) { }
    virtual AlgebraInterface<X>* _copy() const override { return new AlgebraWrapper<A>(*this); }
    virtual AlgebraInterface<X>* _create_zero() const override { return heap_move(nul(*this)); }
    virtual AlgebraInterface<X>* _create_constant(X const& c) const override { A a=nul(*this); a=c; return heap_move(std::move(a)); }
    virtual AlgebraInterface<X>* _create_copy() const override { return new AlgebraWrapper<A>(*this); }
    virtual Void _iadd(const X& c) override{
        static_cast<A&>(*this)+=c; }
    virtual Void _imul(const X& c) override{
        static_cast<A&>(*this)*=c; }
    virtual Void _isma(const X& c, const AlgebraInterface<X>& a) override {
        static_cast<A&>(*this)+=c*dynamic_cast<const A&>(a); }
    virtual Void _ifma(const AlgebraInterface<X>& a1, const AlgebraInterface<X>& a2) override {
        static_cast<A&>(*this)+=dynamic_cast<const A&>(a1)*dynamic_cast<const A&>(a2); }
    virtual AlgebraInterface<X>* _apply(UnaryRingOperator op) const override {
        return heap_move(op(static_cast<const A&>(*this))); }
    virtual AlgebraInterface<X>* _apply(BinaryRingOperator op, AlgebraInterface<X> const& a) const override {
        return heap_move(op(static_cast<const A&>(*this),dynamic_cast<AlgebraWrapper<A,X>const&>(a))); }
    virtual AlgebraInterface<X>* _apply(BinaryFieldOperator op, X const& c) const override {
        return heap_move(op(static_cast<const A&>(*this),c)); }
    virtual AlgebraInterface<X>* _rapply(BinaryRingOperator op, X const& c) const override {
        return heap_move(op(c,static_cast<const A&>(*this))); };
    virtual AlgebraInterface<X>* _apply(GradedRingOperator op, Nat m) const override {
        return heap_move(op(static_cast<const A&>(*this),m)); };
    virtual OutputStream& _write(OutputStream& os) const override { os << static_cast<const A&>(*this); return os; }
};

template<class A, class X> requires AnInplaceAlgebraOver<A,X> class AlgebraWrapper<A,X>
    : public virtual AlgebraInterface<X>
    , public A
{
    static AlgebraInterface<X>* heap_move(A&& a) { return new AlgebraWrapper<A,X>(std::forward<A>(a)); }
  public:
    typedef X NumericType;

    AlgebraWrapper(A&& a) : A(std::move(a)) { }
    AlgebraWrapper(A const& a) : A(a) { }
    virtual AlgebraInterface<X>* _copy() const override { return new AlgebraWrapper<A>(*this); }
    virtual AlgebraInterface<X>* _create_zero() const override { return heap_move(this->A::create()); }
    virtual AlgebraInterface<X>* _create_constant(X const& c) const override { return heap_move(this->A::create_constant(c)); }
    virtual AlgebraInterface<X>* _create_copy() const override { return new AlgebraWrapper<A>(*this); }
    virtual Void _iadd(const X& c) override{ static_cast<A*>(this)->A::iadd(c); }
    virtual Void _imul(const X& c) override{ static_cast<A*>(this)->A::imul(c); }
    virtual Void _isma(const X& c, const AlgebraInterface<X>& x) override {
        static_cast<A*>(this)->A::isma(c,dynamic_cast<const A&>(x)); }
    virtual Void _ifma(const AlgebraInterface<X>& x1, const AlgebraInterface<X>& x2) override {
        static_cast<A*>(this)->A::ifma(dynamic_cast<const A&>(x1),dynamic_cast<const A&>(x2)); }
    virtual AlgebraInterface<X>* _apply(Neg op) const override {
        ARIADNE_NOT_IMPLEMENTED; }
    virtual AlgebraInterface<X>* _apply(BinaryRingOperator op, AlgebraInterface<X> const& c) const override {
        ARIADNE_NOT_IMPLEMENTED; }
    virtual AlgebraInterface<X>* _apply(BinaryFieldOperator op, X const& c) const override {
        ARIADNE_NOT_IMPLEMENTED; }
    virtual AlgebraInterface<X>* _rapply(BinaryRingOperator op, X const& c) const override {
        ARIADNE_NOT_IMPLEMENTED; }
    virtual AlgebraInterface<X>* _apply(GradedRingOperator op, Nat m) const override {
        ARIADNE_NOT_IMPLEMENTED; }
    virtual OutputStream& _write(OutputStream& os) const override { os << static_cast<const A&>(*this); return os; }
};


template<class A, class X=NumericType<A>> class NormedAlgebraWrapper
    : public virtual NormedAlgebraInterface<X>
    , public AlgebraWrapper<A,X>
{
    using typename AlgebraInterface<X>::ErrorType;
    using AlgebraWrapper<A,X>::AlgebraWrapper;
    virtual NormedAlgebraInterface<X>* _create_ball(ErrorType r) const { return new A(static_cast<const A&>(*this).A::create_ball(r)); }
    virtual NormedAlgebraInterface<X>* _create_constant(X c) const { return new A(static_cast<const A&>(*this).A::create_constant(c)); }
    virtual NormedAlgebraInterface<X>* _create_zero() const { return new A(static_cast<const A&>(*this).A::create()); }
    virtual NormedAlgebraInterface<X>* _create_copy() const { return new A(static_cast<const A&>(*this)); }
};

template<class A, class X=NumericType<A>> class GradedAlgebraWrapper
    : public virtual GradedAlgebraInterface<X>
    , public AlgebraWrapper<A,X>
{
    using AlgebraWrapper<A,X>::AlgebraWrapper;
    virtual GradedAlgebraInterface<X>* _create_zero() const { return new A(static_cast<const A&>(*this).A::create()); }
    virtual GradedAlgebraInterface<X>* _create_copy() const { return new A(static_cast<const A&>(*this)); }
    virtual GradedAlgebraInterface<X>* _apply(const Series<X>& f) const { return new A(compose(f,static_cast<const A&>(*this))); }
};

template<class A, class X=NumericType<A>> class SymbolicAlgebraWrapper
    : public virtual SymbolicAlgebraInterface<X>
    , public AlgebraWrapper<A,X>
{
    using AlgebraWrapper<A,X>::AlgebraWrapper;
    using SymbolicAlgebraInterface<X>::_apply;
    using AlgebraWrapper<A,X>::_apply;
    virtual SymbolicAlgebraInterface<X>* _create_zero() const { return new A(static_cast<const A&>(*this).A::create()); }
    virtual SymbolicAlgebraInterface<X>* _create_copy() const { return new SymbolicAlgebraWrapper<A>(*this); }
    virtual SymbolicAlgebraInterface<X>* _apply(UnaryElementaryOperator op) { return new A(op,static_cast<const A&>(*this)); }
};


template<class A, class X> class TranscendentalAlgebraWrapper
    : public TranscendentalAlgebraMixin<TranscendentalAlgebraWrapper<A,X>,X>
    , public A
{
  public:
    using A::A;
    using TranscendentalAlgebraMixin<TranscendentalAlgebraWrapper<A,X>,X>::_apply;
    using TranscendentalAlgebraMixin<TranscendentalAlgebraWrapper<A,X>,X>::_rapply;
    TranscendentalAlgebraWrapper(A const& a) : A(a) { }
    friend OutputStream& operator<<(OutputStream& os, TranscendentalAlgebraWrapper<A,X> const& a) { return os << static_cast<A const&>(a); }
};

template<class A, class X> class ElementaryAlgebraWrapper
    : public ElementaryAlgebraMixin<ElementaryAlgebraWrapper<A,X>,X>
    , public A
{
  public:
    using A::A;
    using ElementaryAlgebraMixin<ElementaryAlgebraWrapper<A,X>,X>::_apply;
    using ElementaryAlgebraMixin<ElementaryAlgebraWrapper<A,X>,X>::_rapply;
    ElementaryAlgebraWrapper(A const& a) : A(a) { }
    friend OutputStream& operator<<(OutputStream& os, ElementaryAlgebraWrapper<A,X> const& a) { return os << static_cast<A const&>(a); }
};


} // namespace Ariadne

#endif /* ARIADNE_ALGEBRA_WRAPPER_HPP */
