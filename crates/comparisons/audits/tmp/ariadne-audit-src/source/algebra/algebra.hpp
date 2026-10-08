/***************************************************************************
 *            algebra/algebra.hpp
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

/*! \file algebra/algebra.hpp
 *  \brief Algebras and normed algebras.
 */
#ifndef ARIADNE_ALGEBRA_HPP
#define ARIADNE_ALGEBRA_HPP

#include <iosfwd>
#include <iostream>
#include "numeric/numeric.hpp"
#include "utility/pointer.hpp"
#include "utility/exceptions.hpp"
#include "utility/metaprogramming.hpp"
#include "numeric/operators.hpp"
#include "algebra/algebra_interface.hpp"
#include "algebra/algebra_wrapper.hpp"
#include "algebra/operations.hpp"

namespace Ariadne {

template<class A, class X> struct AlgebraOperations;

template<class X> struct AlgebraOperations<Algebra<X>,X> {
    static Algebra<X> apply(UnaryRingOperator op, Algebra<X> a) { return Algebra<X>(a.managed_pointer()->_apply(op)); }
    static Algebra<X> apply(BinaryRingOperator op, Algebra<X> a1, Algebra<X> a2) { return Algebra<X>(a1.managed_pointer()->_apply(op,*a2.managed_pointer())); }
    static Algebra<X> apply(BinaryFieldOperator op, Algebra<X> a1, X const& c2) { return Algebra<X>(a1.managed_pointer()->_apply(op,c2)); }
    static Algebra<X> apply(BinaryRingOperator op, X const& c1, Algebra<X> a2) { return Algebra<X>(a2.managed_pointer()->_rapply(op,c1)); }
    static Algebra<X> apply(GradedRingOperator op, Algebra<X> a1, Nat m2) { return Algebra<X>(a1.managed_pointer()->_apply(op,m2)); }
};

template<class X> struct AlgebraOperations<TranscendentalAlgebra<X>,X> {
    static TranscendentalAlgebraInterface<X> const* _upcast(AlgebraInterface<X> const* ap) {
        return dynamic_cast<TranscendentalAlgebraInterface<X> const*>(ap); }
    static TranscendentalAlgebraInterface<X> const* _upcast(TranscendentalAlgebraInterface<X> const* ap) {
        return ap; }
    static TranscendentalAlgebra<X> apply(BinaryFieldOperator op, TranscendentalAlgebra<X> a1, TranscendentalAlgebra<X> a2) {
        return TranscendentalAlgebra<X>(_upcast(a1.raw_pointer())->_apply(op,a2.reference())); }
    static TranscendentalAlgebra<X> apply(BinaryFieldOperator op, TranscendentalAlgebra<X> a1, X const& c2) {
        return  TranscendentalAlgebra<X>(_upcast(a1.raw_pointer())->_apply(op,c2)); }
    static TranscendentalAlgebra<X> apply(BinaryFieldOperator op, X const& c1, TranscendentalAlgebra<X> a2) {
        return TranscendentalAlgebra<X>(_upcast(a2.raw_pointer())->_rapply(op,c1)); }
    static TranscendentalAlgebra<X> apply(UnaryTranscendentalOperator op, TranscendentalAlgebra<X> a) {
        return TranscendentalAlgebra<X>(_upcast(a.raw_pointer())->_apply(op)); }
    static TranscendentalAlgebra<X> apply(Pow op, TranscendentalAlgebra<X> a1, Int n2) {
        return TranscendentalAlgebra<X>(_upcast(a1.raw_pointer())->_apply(op,n2)); }
};

template<class X> struct AlgebraOperations<ElementaryAlgebra<X>,X> {
    static ElementaryAlgebraInterface<X> const* _upcast(AlgebraInterface<X> const* ap) {
        return dynamic_cast<ElementaryAlgebraInterface<X> const*>(ap); }
    static ElementaryAlgebraInterface<X> const* _upcast(ElementaryAlgebraInterface<X> const* ap) {
        return ap; }
    static ElementaryAlgebra<X> apply(BinaryElementaryOperator op, ElementaryAlgebra<X> a1, ElementaryAlgebra<X> a2) {
        return ElementaryAlgebra<X>(_upcast(a1.raw_pointer())->_apply(op,a2.reference())); }
    static ElementaryAlgebra<X> apply(BinaryElementaryOperator op, ElementaryAlgebra<X> a1, X const& c2) {
        return  ElementaryAlgebra<X>(_upcast(a1.raw_pointer())->_apply(op,c2)); }
    static ElementaryAlgebra<X> apply(BinaryElementaryOperator op, X const& c1, ElementaryAlgebra<X> a2) {
        return ElementaryAlgebra<X>(_upcast(a2.raw_pointer())->_rapply(op,c1)); }
    static ElementaryAlgebra<X> apply(UnaryElementaryOperator op, ElementaryAlgebra<X> a) {
        return ElementaryAlgebra<X>(_upcast(a.raw_pointer())->_apply(op)); }
    static ElementaryAlgebra<X> apply(GradedElementaryOperator op, ElementaryAlgebra<X> a1, Int n2) {
        return ElementaryAlgebra<X>(_upcast(a1.raw_pointer())->_apply(op,n2)); }
};


//! \brief Generic class for elements of unital algebras.
template<class X> class Algebra
    : public Handle<const AlgebraInterface<X>>
    , public DispatchAlgebraOperations<Algebra<X>,X>
{
  public:
    typedef AlgebraInterface<X> Interface;
    typedef X ScalarType;
    typedef typename X::Paradigm Paradigm;
    typedef typename X::NumericType NumericType;
  public:
    using Handle<const Interface>::Handle;
    template<AnAlgebraOver<X> A> Algebra(A const&);
    template<class A> A extract() const;
    Algebra(const Algebra<X>& a) : Handle<const AlgebraInterface<X>>(a) { }
    Algebra<X>& operator=(const Algebra<X>& a) { (*this).Handle<const AlgebraInterface<X>>::operator=(std::shared_ptr< const AlgebraInterface<X> >(a.managed_pointer()->_create_copy())); return *this; }
    Algebra<X>& operator=(const X& c) { return *this = this->create_constant(c); }
    operator const AlgebraInterface<X>& () const { return *this->managed_pointer(); }
    Algebra<X> create() const { return Algebra<X>(this->managed_pointer()->_create_zero()); }
    Algebra<X> clone() const { return Algebra<X>(this->managed_pointer()->_create_copy()); }
    Algebra<X> create_zero() const { return Algebra<X>(this->managed_pointer()->_create_zero()); }
    Algebra<X> create_constant(X const& c) const { return Algebra<X>(this->managed_pointer()->_create_constant(c)); }
    friend OutputStream& operator<<(OutputStream& os, Algebra<X> const& a) { return a.managed_pointer()->_write(os); }
  private:
    template<class A> static AlgebraInterface<X>* make_algebra(A const& a) {
        if constexpr(BaseOf<AlgebraInterface<X>,A>) { return new A(a); }
        else { return new AlgebraWrapper<A,X>(a); }
    }
};



//! \brief Generic class for elements of unital algebras.
template<class X> class TranscendentalAlgebra
    : public Handle<const TranscendentalAlgebraInterface<X>>
    , public DispatchTranscendentalAlgebraOperations<TranscendentalAlgebra<X>,X>
{
  public:
    typedef TranscendentalAlgebraInterface<X> Interface;
    typedef X ScalarType;
    typedef typename X::Paradigm Paradigm;
    typedef typename X::NumericType NumericType;
  public:
    using Handle<const Interface>::Handle;
    TranscendentalAlgebra() : TranscendentalAlgebra(nullptr) { }
    template<ATranscendentalAlgebraOver<X> A> explicit TranscendentalAlgebra(const A& a);
    template<class A> A extract() const;
    TranscendentalAlgebra<X>& operator=(const X& c) { return *this = this->create_constant(c); }
    TranscendentalAlgebra<X> create() const { return TranscendentalAlgebra<X>(this->managed_pointer()->_create_zero()); }
    TranscendentalAlgebra<X> clone() const { return TranscendentalAlgebra<X>(this->managed_pointer()->_create_copy()); }
    TranscendentalAlgebra<X> create_zero() const { return TranscendentalAlgebra<X>(this->managed_pointer()->_create_zero()); }
    TranscendentalAlgebra<X> create_constant(X const& c) const { return TranscendentalAlgebra<X>(this->managed_pointer()->_create_constant(c)); }
    friend OutputStream& operator<<(OutputStream& os, TranscendentalAlgebra<X> const& a) { return a.managed_pointer()->_write(os); }
  private:
    template<class A> static TranscendentalAlgebraInterface<X>* make_elementary_algebra(A const& a) {
        if constexpr(BaseOf<TranscendentalAlgebraInterface<X>,A>) { return new A(a); }
        else { return new TranscendentalAlgebraWrapper<A,X>(a); }
    }
};

//! \brief Generic class for elements of unital algebras.
template<class X> class ElementaryAlgebra
    : public Handle<const ElementaryAlgebraInterface<X>>
    , public DispatchElementaryAlgebraOperations<ElementaryAlgebra<X>,X>
{
  public:
    typedef ElementaryAlgebraInterface<X> Interface;
    typedef X ScalarType;
    typedef typename X::Paradigm Paradigm;
    typedef typename X::NumericType NumericType;
  public:
    using Handle<const Interface>::Handle;
    ElementaryAlgebra() : ElementaryAlgebra(nullptr) { }
    template<AnElementaryAlgebraOver<X> A> explicit ElementaryAlgebra(const A& a);
    template<class A> A extract() const;
    ElementaryAlgebra<X>& operator=(const X& c) { return *this = this->create_constant(c); }
    ElementaryAlgebra<X> create() const { return ElementaryAlgebra<X>(this->managed_pointer()->_create_zero()); }
    ElementaryAlgebra<X> clone() const { return ElementaryAlgebra<X>(this->managed_pointer()->_create_copy()); }
    ElementaryAlgebra<X> create_zero() const { return ElementaryAlgebra<X>(this->managed_pointer()->_create_zero()); }
    ElementaryAlgebra<X> create_constant(X const& c) const { return ElementaryAlgebra<X>(this->managed_pointer()->_create_constant(c)); }
    friend OutputStream& operator<<(OutputStream& os, ElementaryAlgebra<X> const& a) { return a.managed_pointer()->_write(os); }
  private:
    template<class A> static ElementaryAlgebraInterface<X>* make_elementary_algebra(A const& a) {
        if constexpr(BaseOf<ElementaryAlgebraInterface<X>,A>) { return new A(a); }
        else { return new ElementaryAlgebraWrapper<A,X>(a); }
    }
};



template<class X> class NormedAlgebra
    : public Handle<const NormedAlgebraInterface<X>>
{
    static_assert(not Same<X,Real>);
  public:
    typedef NormedAlgebraInterface<X> Interface;
    typedef X ScalarType;
    typedef typename X::NumericType NumericType;

    using Handle<const Interface>::Handle;
    NormedAlgebra(const Algebra<X>& a) : Handle<const Interface>(std::dynamic_pointer_cast(a.managed_pointer())) { }
    operator Algebra<X> () const { return this->managed_pointer()->_create_copy(); }
    NormedAlgebra<X>& operator=(Int c) { *this = this->create(); this->iadd(c); return *this; }
    NormedAlgebra<X>& operator=(const X& c) { *this = this->create(); this->iadd(c); return *this; }
    //! \brief The norm of the element.
    typename AlgebraTraits<X>::NormType norm() const { return this->managed_pointer()->norm(); }
    //! \brief A value \a c minimising |a-c|.
    typename AlgebraTraits<X>::ValueType average() const { return this->managed_pointer()->average(); }
    //! \brief The tolerance used to determine the truncation error when applying an analytic function.
    FloatDP tolerance() const { return this->managed_pointer()->tolerance(); }
    //! \brief A value \c r such that \c |a-c1|<=r.
    typename AlgebraTraits<X>::NormType radius() const { return this->managed_pointer()->radius(); }
    NormedAlgebra<X> create_zero() const { return NormedAlgebra<X>(this->managed_pointer()->_create_zero()); }
    NormedAlgebra<X> create_constant(X c) const { return NormedAlgebra<X>(this->managed_pointer()->_create_constant(c)); }
    NormedAlgebra<X> create_ball(FloatDPError r) const { return NormedAlgebra<X>(this->managed_pointer()->_create_ball(r)); }
    NormedAlgebra<X> create_ball(FloatDPApproximation r) const {
        return create_ball(FloatDPError(r.raw())); }
    NormedAlgebra<X> create() const { return NormedAlgebra<X>(this->managed_pointer()->_create_zero()); }
    NormedAlgebra<X> clone() const { return NormedAlgebra<X>(this->managed_pointer()->_create_copy()); }
    Void clear() { this->imul(0); }
    Void iadd(const X& c) { this->managed_pointer()->_iadd(c); }
    Void imul(const X& c) { this->managed_pointer()->_imul(c); }
    Void isma(const X& c, const NormedAlgebra<X>& x) { this->managed_pointer()->_isma(c,*x.managed_pointer()); }
    Void ifma(const NormedAlgebra<X>& x1, const NormedAlgebra<X>& x2) { this->managed_pointer()->_ifma(*x1.managed_pointer(),*x2.managed_pointer()); }
    friend OutputStream& operator<<(OutputStream& os, NormedAlgebra<X> const& a) { return a.managed_pointer()->_write(os); }
};

//! \brief Generic class for elements of unital algebras.
template<class X> class GradedAlgebra
    : public Handle<const GradedAlgebraInterface<X>>
{
  public:
    typedef GradedAlgebraInterface<X> Interface;
    typedef X ScalarType;
    typedef typename X::NumericType NumericType;

    using Handle<const Interface>::Handle;
    GradedAlgebra<X>& operator=(Int c) { *this = this->create(); this->iadd(c); return *this; }
    GradedAlgebra<X>& operator=(const X& c) { *this = this->create(); this->iadd(c); return *this; }
    GradedAlgebra<X>& operator=(const Algebra<X>& a) { this->managed_pointer()=std::shared_ptr< AlgebraInterface<X> >(a.managed_pointer()->_create_copy()); return *this; }
    operator Algebra<X> () const { return this->managed_pointer()->_create_copy(); }
    GradedAlgebra<X> create() const { return GradedAlgebra<X>(this->managed_pointer()->_create_zero()); }
    GradedAlgebra<X> clone() const { return GradedAlgebra<X>(this->managed_pointer()->_create_copy()); }
    //! \brief The degree of the algebra.
    Nat degree() const { return this->managed_pointer()->degree(); }
    //! \brief The value in the null grade.
    const X& value() const { return this->managed_pointer()->value(); }
    friend OutputStream& operator<<(OutputStream& os, GradedAlgebra<X> const& a) { return a.managed_pointer()->_write(os); }
  public:
    Void iadd(const X& c) { this->managed_pointer()->_iadd(c); }
    Void imul(const X& c) { this->managed_pointer()->_imul(c); }
    Void isma(const X& c, const GradedAlgebra<X>& x) { this->managed_pointer()->_isma(c,*x.managed_pointer()); }
    Void ifma(const GradedAlgebra<X>& x1, const GradedAlgebra<X>& x2) { this->managed_pointer()->_ifma(*x1.managed_pointer(),*x2.managed_pointer()); }
};

//! \brief Generic class for elements of unital algebras.
template<class X> class SymbolicAlgebra
    : public Handle<const SymbolicAlgebraInterface<X>>
{
  public:
    typedef SymbolicAlgebraInterface<X> Interface;
    typedef X ScalarType;
    typedef typename X::NumericType NumericType;

    using Handle<const Interface>::Handle;
    //! \brief Create the representation of the operator \a op applied to \a a.
    SymbolicAlgebra(UnaryElementaryOperator op, const SymbolicAlgebra<X>& a) : Handle<const Interface>(a.managed_pointer()->_apply(op)) { }
    SymbolicAlgebra<X>& operator=(Int c) { *this = this->create(); this->iadd(c); return *this; }
    SymbolicAlgebra<X>& operator=(const X& c) { *this = this->create(); this->iadd(c); return *this; }
    SymbolicAlgebra<X>& operator=(const Algebra<X>& a) { this->managed_pointer()=std::shared_ptr< AlgebraInterface<X> >(a.managed_pointer()->_create_copy()); return *this; }
    operator Algebra<X> () const { return this->managed_pointer()->_create_copy(); }
    SymbolicAlgebra<X> create() const { return SymbolicAlgebra<X>(this->managed_pointer()->_create_zero()); }
    SymbolicAlgebra<X> create_zero() const { return SymbolicAlgebra<X>(this->managed_pointer()->_create_zero()); }
    SymbolicAlgebra<X> clone() const { return SymbolicAlgebra<X>(this->managed_pointer()->_create_copy()); }
    friend OutputStream& operator<<(OutputStream& os, SymbolicAlgebra<X> const& a) { return a.managed_pointer()->_write(os); }
  public:
    Void iadd(const X& c) { this->managed_pointer()->_iadd(c); }
    Void imul(const X& c) { this->managed_pointer()->_imul(c); }
    Void isma(const X& c, const SymbolicAlgebra<X>& x) { this->managed_pointer()->_isma(c,*x.managed_pointer()); }
    Void ifma(const SymbolicAlgebra<X>& x1, const SymbolicAlgebra<X>& x2) { this->managed_pointer()->_ifma(*x1.managed_pointer(),*x2.managed_pointer()); }
};


template<class X> template<AnAlgebraOver<X> A> Algebra<X>::Algebra(A const& a)
    : Handle<const AlgebraInterface<X>>(make_algebra(a)) { }

template<class X> template<class A> A Algebra<X>::extract() const {
    if constexpr(BaseOf<AlgebraInterface<X>,A>) { auto ap=dynamic_cast<A const*>(this->raw_pointer()); assert(ap); return *ap; }
    else { auto awp=dynamic_cast<AlgebraWrapper<A,X>const*>(this->raw_pointer()); assert(awp); return *awp; }
}


template<class X> template<ATranscendentalAlgebraOver<X> A> TranscendentalAlgebra<X>::TranscendentalAlgebra(A const& a)
    : Handle<const TranscendentalAlgebraInterface<X>>(make_elementary_algebra(a)) { }

template<class X> template<class A> A TranscendentalAlgebra<X>::extract() const {
    if constexpr(BaseOf<TranscendentalAlgebraInterface<X>,A>) { auto ap=dynamic_cast<A const*>(this->raw_pointer()); assert(ap); return *ap; }
    else { auto awp=dynamic_cast<TranscendentalAlgebraWrapper<A,X>const*>(this->raw_pointer()); assert(awp); return *awp; }
}


template<class X> template<AnElementaryAlgebraOver<X> A> ElementaryAlgebra<X>::ElementaryAlgebra(A const& a)
    : Handle<const ElementaryAlgebraInterface<X>>(make_elementary_algebra(a)) { }

template<class X> template<class A> A ElementaryAlgebra<X>::extract() const {
    if constexpr(BaseOf<ElementaryAlgebraInterface<X>,A>) { auto ap=dynamic_cast<A const*>(this->raw_pointer()); assert(ap); return *ap; }
    else { auto awp=dynamic_cast<ElementaryAlgebraWrapper<A,X>const*>(this->raw_pointer()); assert(awp); return *awp; }
}



template<class T> class PrototypeCharacteristics {
    T _prototype;
  public:
    PrototypeCharacteristics(T prototype) : _prototype(prototype.create_zero()) { }
    operator T const& () const { return _prototype; }
    friend Bool operator==(PrototypeCharacteristics<T> const& prs1, PrototypeCharacteristics<T> const& prs2) {
        ARIADNE_NOT_IMPLEMENTED; }
};

template<class X> PrototypeCharacteristics<ElementaryAlgebra<X>> characteristics(ElementaryAlgebra<X> const& a) { return PrototypeCharacteristics(a); }


} // namespace Ariadne

#endif /* ARIADNE_ALGEBRA_HPP */
