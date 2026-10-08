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

/*! \file algebra/algebra_mixin.hpp
 *  \brief Mixin class providing operations for (Banach) algebras.
 */

#ifndef ARIADNE_ALGEBRA_MIXIN_HPP
#define ARIADNE_ALGEBRA_MIXIN_HPP

#include "algebra/algebra_interface.hpp"
#include "algebra/algebra_concepts.hpp"
#include "numeric/operators.hpp"

namespace Ariadne {

template<class A, class X> class AlgebraMixin
    : public virtual AlgebraInterface<X>
{
    static_assert(AnInplaceAlgebraOver<A,X> or AnAlgebraOver<A,X>);
    typedef X NumericType;
  private:
    template<class OP> static AlgebraInterface<X>* _eval(OP op, AlgebraMixin<A,X> const& am1, AlgebraInterface<X> const& ai2) {
        AlgebraMixin<A,X>const* amp2 = dynamic_cast<AlgebraMixin<A,X>const*>(&ai2); assert(amp2);
        A const& a1=static_cast<A const&>(am1); A const& a2=static_cast<A const&>(*amp2);
        return new A(op(a1,a2)); }
    template<class OP> static AlgebraInterface<X>* _eval(OP op, AlgebraMixin<A,X> const& am, X const& c) {
        A const& a=static_cast<A const&>(am); return new A(op(a,c)); }
    template<class OP> static AlgebraInterface<X>* _eval(OP op, X const& c, AlgebraMixin<A,X> const& am) {
        A const& a=static_cast<A const&>(am); return new A(op(c,a)); }
    template<class OP> static AlgebraInterface<X>* _eval(OP op, AlgebraMixin<A,X> const& am) {
        A const& a=static_cast<A const&>(am); return new A(op(a)); }
    template<class OP> static AlgebraInterface<X>* _eval(OP op, AlgebraMixin<A,X> const& am, Nat m) {
        A const& a=static_cast<A const&>(am); return new A(op(a,m)); }
  public:
    virtual AlgebraInterface<X>* _copy() const { return new A(static_cast<const A&>(*this)); }
    virtual AlgebraInterface<X>* _create_zero() const { return new A(static_cast<const A&>(*this).A::create_zero()); }
    virtual AlgebraInterface<X>* _create_constant(X const& c) const { return new A(static_cast<const A&>(*this).A::create_constant(c)); }
    virtual AlgebraInterface<X>* _create_copy() const { return new A(static_cast<const A&>(*this)); }
    virtual Void _iadd(const X& c) {
        if constexpr (AnInplaceAlgebraOver<A,X>) { static_cast<A*>(this)->A::iadd(c); } else { static_cast<A&>(*this) += c; } }
    virtual Void _imul(const X& c) {
        if constexpr (AnInplaceAlgebraOver<A,X>) { static_cast<A*>(this)->A::imul(c); } else { static_cast<A&>(*this) *= c; } }
    virtual Void _isma(const X& c, const AlgebraInterface<X>& x) {
        if constexpr (AnInplaceAlgebraOver<A,X>) { static_cast<A*>(this)->A::isma(c, dynamic_cast<const A&>(x)); }
        else { static_cast<A&>(*this) += c * dynamic_cast<const A&>(x); } }
    virtual Void _ifma(const AlgebraInterface<X>& x1, const AlgebraInterface<X>& x2) {
        if constexpr (AnInplaceAlgebraOver<A,X>) { static_cast<A*>(this)->A::isma(dynamic_cast<const A&>(x1), dynamic_cast<const A&>(x2)); }
        else { static_cast<A&>(*this) += dynamic_cast<const A&>(x1) * dynamic_cast<const A&>(x2); } }

    virtual AlgebraInterface<X>* _apply(UnaryRingOperator op) const { return _eval(op,*this); }
    virtual AlgebraInterface<X>* _apply(BinaryRingOperator op, AlgebraInterface<X> const& other) const { return _eval(op,*this,other); }
    virtual AlgebraInterface<X>* _apply(BinaryFieldOperator op, X const& cnst) const { return _eval(op,*this,cnst); }
    virtual AlgebraInterface<X>* _rapply(BinaryRingOperator op, X const& cnst) const { return _eval(op,cnst,*this); }
    virtual AlgebraInterface<X>* _apply(GradedRingOperator op, Nat m) const { return _eval(op,*this,m); }

    virtual OutputStream& _write(OutputStream& os) const { return os << *static_cast<A const*>(this); }
};

template<class A, class X> class TranscendentalAlgebraMixin
    : public virtual TranscendentalAlgebraInterface<X>
{
    typedef X NumericType;
  private:
    static A const& _cast(TranscendentalAlgebraMixin<A,X> const& am) {
        return static_cast<A const&>(am); }
    static A const& _cast(TranscendentalAlgebraInterface<X> const& ai) {
        return static_cast<A const&>(dynamic_cast<TranscendentalAlgebraMixin<A,X>const&>(ai)); }
    static A* _heap_move(A&& a) { return new A(std::move(a)); }
    template<class OP> static TranscendentalAlgebraInterface<X>* _eval(OP op, TranscendentalAlgebraMixin<A,X> const& am1, TranscendentalAlgebraInterface<X> const& ai2) {
        TranscendentalAlgebraMixin<A,X>const* amp2 = dynamic_cast<TranscendentalAlgebraMixin<A,X>const*>(&ai2); assert(amp2);
        A const& a1=static_cast<A const&>(am1); A const& a2=static_cast<A const&>(*amp2);
        return new A(op(a1,a2)); }
    template<class OP> static TranscendentalAlgebraInterface<X>* _eval(OP op, TranscendentalAlgebraMixin<A,X> const& am, X const& c) {
        A const& a=static_cast<A const&>(am); return new A(op(a,c)); }
    template<class OP> static TranscendentalAlgebraInterface<X>* _eval(OP op, X const& c, TranscendentalAlgebraMixin<A,X> const& am) {
        A const& a=static_cast<A const&>(am); return new A(op(c,a)); }
    template<class OP> static TranscendentalAlgebraInterface<X>* _eval(OP op, TranscendentalAlgebraMixin<A,X> const& am) {
        A const& a=static_cast<A const&>(am); return new A(op(a)); }
    template<class OP> static TranscendentalAlgebraInterface<X>* _eval(OP op, TranscendentalAlgebraMixin<A,X> const& am, Nat m) {
        A const& a=static_cast<A const&>(am); return new A(op(a,m)); }
  public:
    virtual TranscendentalAlgebraInterface<X>* _create_zero() const override {
        return new A(static_cast<const A&>(*this).A::create_zero()); }
    virtual TranscendentalAlgebraInterface<X>* _create_constant(X const& c) const override {
        return new A(static_cast<const A&>(*this).A::create_constant(c)); }
    virtual TranscendentalAlgebraInterface<X>* _create_copy() const override {
        return new A(static_cast<const A&>(*this)); }

    virtual TranscendentalAlgebraInterface<X>* _apply(BinaryFieldOperator op, TranscendentalAlgebraInterface<X> const& other) const override {
        return _heap_move(op(_cast(*this),_cast(other))); }
    virtual TranscendentalAlgebraInterface<X>* _apply(UnaryTranscendentalOperator op) const override {
        return _heap_move(op(_cast(*this))); }
    virtual TranscendentalAlgebraInterface<X>* _apply(BinaryFieldOperator op, X const& cnst) const override {
        return _heap_move(op(_cast(*this),cnst)); }
    virtual TranscendentalAlgebraInterface<X>* _rapply(BinaryFieldOperator op, X const& cnst) const override {
        return _heap_move(op(cnst,_cast(*this))); }
    virtual TranscendentalAlgebraInterface<X>* _apply(GradedFieldOperator op, Int n) const override {
        return _heap_move(op(_cast(*this),n)); }

    virtual OutputStream& _write(OutputStream& os) const override { return os << *static_cast<A const*>(this); }
};

template<class A, class X> class ElementaryAlgebraMixin
    : public virtual ElementaryAlgebraInterface<X>
{
    typedef X NumericType;
  private:
    static A const& _cast(ElementaryAlgebraMixin<A,X> const& am) {
        return static_cast<A const&>(am); }
    static A const& _cast(ElementaryAlgebraInterface<X> const& ai) {
        return static_cast<A const&>(dynamic_cast<ElementaryAlgebraMixin<A,X>const&>(ai)); }
    static A* _heap_move(A&& a) { return new A(std::move(a)); }
    template<class OP> static ElementaryAlgebraInterface<X>* _eval(OP op, ElementaryAlgebraMixin<A,X> const& am1, ElementaryAlgebraInterface<X> const& ai2) {
        ElementaryAlgebraMixin<A,X>const* amp2 = dynamic_cast<ElementaryAlgebraMixin<A,X>const*>(&ai2); assert(amp2);
        A const& a1=static_cast<A const&>(am1); A const& a2=static_cast<A const&>(*amp2);
        return new A(op(a1,a2)); }
    template<class OP> static ElementaryAlgebraInterface<X>* _eval(OP op, ElementaryAlgebraMixin<A,X> const& am, X const& c) {
        A const& a=static_cast<A const&>(am); return new A(op(a,c)); }
    template<class OP> static ElementaryAlgebraInterface<X>* _eval(OP op, X const& c, ElementaryAlgebraMixin<A,X> const& am) {
        A const& a=static_cast<A const&>(am); return new A(op(c,a)); }
    template<class OP> static ElementaryAlgebraInterface<X>* _eval(OP op, ElementaryAlgebraMixin<A,X> const& am) {
        A const& a=static_cast<A const&>(am); return new A(op(a)); }
    template<class OP> static ElementaryAlgebraInterface<X>* _eval(OP op, ElementaryAlgebraMixin<A,X> const& am, Nat m) {
        A const& a=static_cast<A const&>(am); return new A(op(a,m)); }
  public:
    virtual ElementaryAlgebraInterface<X>* _create_zero() const override {
        return new A(static_cast<const A&>(*this).A::create_zero()); }
    virtual ElementaryAlgebraInterface<X>* _create_constant(X const& c) const override {
        return new A(static_cast<const A&>(*this).A::create_constant(c)); }
    virtual ElementaryAlgebraInterface<X>* _create_copy() const override {
        return new A(static_cast<const A&>(*this)); }

    virtual ElementaryAlgebraInterface<X>* _apply(BinaryElementaryOperator op, ElementaryAlgebraInterface<X> const& other) const override {
        return _heap_move(op(_cast(*this),_cast(other))); }
    virtual ElementaryAlgebraInterface<X>* _apply(UnaryElementaryOperator op) const override {
        return _heap_move(op(_cast(*this))); }
    virtual ElementaryAlgebraInterface<X>* _apply(BinaryElementaryOperator op, X const& cnst) const override {
        return _heap_move(op(_cast(*this),cnst)); }
    virtual ElementaryAlgebraInterface<X>* _rapply(BinaryElementaryOperator op, X const& cnst) const override {
        return _heap_move(op(cnst,_cast(*this))); }
    virtual ElementaryAlgebraInterface<X>* _apply(GradedElementaryOperator op, Int n) const override {
        return _heap_move(op(_cast(*this),n)); }

    virtual OutputStream& _write(OutputStream& os) const override { return os << *static_cast<A const*>(this); }
};

template<class A, class X> class NormedAlgebraMixin
    : public virtual NormedAlgebraInterface<X>
    , public AlgebraMixin<A,X>
{
    using typename NormedAlgebraInterface<X>::ErrorType;

    virtual NormedAlgebraInterface<X>* _create_ball(ErrorType r) const { return new A(static_cast<const A&>(*this).A::create_ball(r)); }
    virtual NormedAlgebraInterface<X>* _create_constant(X c) const { return new A(static_cast<const A&>(*this).A::create_constant(c)); }
    virtual NormedAlgebraInterface<X>* _create_zero() const { return new A(static_cast<const A&>(*this).A::create()); }
    virtual NormedAlgebraInterface<X>* _create_copy() const { return new A(static_cast<const A&>(*this)); }
};

template<class A, class X> class GradedAlgebraMixin
    : public virtual GradedAlgebraInterface<X>
    , public AlgebraMixin<A,X>
{
    virtual GradedAlgebraMixin<A,X>* _create_zero() const { return new A(static_cast<const A&>(*this).A::create()); }
    virtual GradedAlgebraMixin<A,X>* _create_copy() const { return new A(static_cast<const A&>(*this)); }
    virtual GradedAlgebraMixin<A,X>* _apply(const Series<X>& f) const { return new A(compose(f,static_cast<const A&>(*this))); }
};

template<class A, class X> class SymbolicAlgebraMixin
    : public virtual SymbolicAlgebraInterface<X>
    , public AlgebraMixin<A,X>
{
    using SymbolicAlgebraInterface<X>::_apply;
    using AlgebraMixin<A,X>::_apply;
    virtual SymbolicAlgebraMixin<A,X>* _create_copy() const { return new A(static_cast<const A&>(*this)); }
    virtual SymbolicAlgebraMixin<A,X>* _create_zero() const { return new A(static_cast<const A&>(*this).A::create()); }
    virtual SymbolicAlgebraMixin<A,X>* _create_constant(X const& c) const { return new A(static_cast<const A&>(*this).A::create_constant(c)); }
    virtual SymbolicAlgebraMixin<A,X>* _apply(UnaryElementaryOperator op) { return new A(op,static_cast<const A&>(*this)); }
};

} // namespace Ariadne

#endif /* ARIADNE_ALGEBRA_MIXIN_HPP */
