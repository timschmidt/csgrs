/***************************************************************************
 *            function/formula.hpp
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

/*! \file function/formula.hpp
 *  \brief Formulae over variables
 */

#ifndef ARIADNE_FORMULA_HPP
#define ARIADNE_FORMULA_HPP

#include <cstdarg>
#include <iostream>
#include <iomanip>
#include <string>


#include "utility/macros.hpp"
#include "utility/pointer.hpp"
#include "utility/container.hpp"
#include "helper/stlio.hpp"

#include "numeric/operators.hpp"
#include "numeric/numeric.hpp"
#include "algebra/vector.hpp"
#include "algebra/expansion.hpp"
#include "algebra/operations.hpp"

#include "symbolic/templates.hpp"
#include "symbolic/constant.hpp"

namespace Ariadne {


template<class A, class X >
struct DispatchAlgebraOperations;

template<class Y> class Formula;
typedef Formula<ApproximateNumber> ApproximateFormula;
typedef Formula<ValidatedNumber> ValidatedFormula;
typedef Formula<EffectiveNumber> EffectiveFormula;
typedef Formula<ExactNumber> ExactFormula;

struct Index {
    Nat _i;
  public:
    explicit Index(Nat i) : _i(i) { }
    operator Nat () const { return _i; }
    friend OutputStream& operator<<(OutputStream& os, const Index& ind) {
        return os << Nat(ind); }
};

template<class Y> class FormulaNode;

class FormulaOperations {
    template<class Y, class R> requires Constructible<Y,R> friend Formula<Y> operator+(Formula<Y> f, R c) { return f + Y(c); }
    template<class Y, class R> requires Constructible<Y,R> friend Formula<Y> operator-(Formula<Y> f, R c) { return f - Y(c); }
    template<class Y, class R> requires Constructible<Y,R> friend Formula<Y> operator*(Formula<Y> f, R c) { return f * Y(c); }
    template<class Y, class R> requires Constructible<Y,R> friend Formula<Y> operator/(Formula<Y> f, R c) { return f / Y(c); }
    template<class Y, class R> requires Constructible<Y,R> friend Formula<Y> operator+(R c, Formula<Y> f) { return Y(c)+f; }
    template<class Y, class R> requires Constructible<Y,R> friend Formula<Y> operator-(R c, Formula<Y> f) { return Y(c)-f; }
    template<class Y, class R> requires Constructible<Y,R> friend Formula<Y> operator*(R c, Formula<Y> f) { return Y(c)*f; }
    template<class Y, class R> requires Constructible<Y,R> friend Formula<Y> operator/(R c, Formula<Y> f) { return Y(c)/f; }

    template<class Y, class R> requires Constructible<Y,R> friend Formula<Y>& operator+=(Formula<Y>& f, const R& c) { return f+=Y(c); }
    template<class Y, class R> requires Constructible<Y,R> friend Formula<Y>& operator-=(Formula<Y>& f, const R& c) { return f-=Y(c); }
    template<class Y, class R> requires Constructible<Y,R> friend Formula<Y>& operator*=(Formula<Y>& f, const R& c) { return f*=Y(c); }
    template<class Y, class R> requires Constructible<Y,R> friend Formula<Y>& operator/=(Formula<Y>& f, const R& c) { return f/=Y(c); }
};

template<class Y> struct AlgebraOperations<Formula<Y>,Y> {
  public:
    static Formula<Y> apply(UnaryElementaryOperator op, Formula<Y> const& f) {
        return Formula<Y>::unary(op,f); }
    static Formula<Y> apply(BinaryElementaryOperator op, Formula<Y> const& f1, Formula<Y> const& f2) {
        return Formula<Y>::binary(op,f1,f2); }
    static Formula<Y> apply(BinaryElementaryOperator op, Formula<Y> const& f1, Y const& c2) {
        return Formula<Y>::binary(op,f1,Formula<Y>::constant(c2)); }
    static Formula<Y> apply(BinaryElementaryOperator op, Y const& c1, Formula<Y> const& f2) {
        return Formula<Y>::binary(op,Formula<Y>::constant(c1),f2); }
    static Formula<Y> apply(GradedElementaryOperator op, Formula<Y> const& f, Int n) {
        return Formula<Y>::graded(op,f,n); }
};

//! \ingroup FunctionModule
//! \brief A formula defining a real function.
//!
//! The Formula class is implemented as a directed acyclic graph, with
//! each node being an atomic operation.
template<class Y>
class Formula
    : public DispatchElementaryAlgebraOperations<Formula<Y>,Y>
    , public FormulaOperations
{
    typedef Index I;
  public:
    typedef typename Y::Paradigm Paradigm;
    typedef Y NumericType;
    typedef Y ConstantType;
    typedef I IndexType;
  private:
    explicit Formula(const FormulaNode<Y>* fptr, PointerTag) : _root(fptr) { }
//    explicit Formula(counted_pointer<const FormulaNode<Y>> fptr, PointerTag) : _root(fptr) { }
    explicit Formula(SharedPointer<const FormulaNode<Y>> fptr, PointerTag) : _root(fptr) { }
  public:
    //! \brief Construct the constant expression with the default value of \a Y.
    Formula();
    //! \brief Construct the constant expression with value \a c.
    Formula(Y const& c);
    //! \brief Construct the coordinate expression with index \a i.
    explicit Formula(Index const& i);
    //! \brief Set equal to a constant.
    Formula<Y>& operator=(const Y& c);
    template<class X> requires Constructible<Y,X> Formula<Y>& operator=(const X& c) { return *this=Y(c); }
  public:
    //! \brief Return the constant formula zero.
    Formula<Y> create_zero() const;
    //! \brief Return the constant formula with value \a c.
    Formula<Y> create_constant(const Y& c) const;
  public:
    static Formula<Y> zero();
    static Formula<Y> constant(const Y& c);
    static Formula<Y> coordinate(SizeType i);
    static Vector<Formula<Y>> coordinates(SizeType n);
    static Vector<Formula<Y>> identity(SizeType n);
    static Formula<Y> unary(const UnaryElementaryOperator& op, Formula<Y> const& a);
    static Formula<Y> binary(const BinaryElementaryOperator& op, Formula<Y> const& a1, Formula<Y> const& a2);
    static Formula<Y> graded(const GradedElementaryOperator& op, Formula<Y> const& a1, Int n2);
    static Formula<Y> scalar(const BinaryElementaryOperator& op, Y const& c1, Formula<Y> const& a2);
  public:
    Operator op() const;
    OperatorCode code() const;
    OperatorKind kind() const;
    const Y& val() const;
    const I& ind() const;
    const Formula<Y>& arg() const;
    const Y& cnst() const;
    const Int& num() const;
    const Formula<Y>& arg1() const;
    const Formula<Y>& arg2() const;
  public:
    friend OutputStream& operator<<(OutputStream& os, Formula<Y> const& f) { return Formula<Y>::_write_multivariate(os,f); }
  public:
    Formula<Y> _derivative(SizeType j) const;
    static OutputStream& _write_multivariate(OutputStream& os, Formula<Y> const& f);
    static OutputStream& _write_multivariate(OutputStream& os, Vector<Formula<Y>> const& f);
    static OutputStream& _write_univariate(OutputStream& os, Formula<Y> const& f);
    static OutputStream& _write_univariate(OutputStream& os, Vector<Formula<Y>> const& f);
  public:
    const FormulaNode<Y>* node_ptr() const { return _root.operator->(); }
    const FormulaNode<Y>& node_ref() const { return _root.operator*(); }
  private:
    SharedPointer<const FormulaNode<Y>> _root;
};

//! \brief Substitute all occurrences of index \a i with constant value \a c.
template<class Y> Formula<Y> substitute(const Formula<Y>& a, const Nat& i, const Y& c);
//! \brief Substitute all occurrences of indices with formulae \a as into \a a.
template<class Y> Formula<Y> substitute(const Formula<Y>& a, const List<Pair<Nat,Formula<Y>>>& as);
//! \brief Substitute all occurrences of indices with formulae \a as into \a av.
template<class Y> Vector<Formula<Y>> substitute(const Vector<Formula<Y>>& av, const List<Pair<Nat,Formula<Y>>>& as);

//! \brief Simplify the formula to reduce the number of nodes.
template<class Y> Formula<Y> simplify(const Formula<Y>& a);
//! \brief Simplify the formulae to reduce the number of nodes.
template<class Y> Vector<Formula<Y>> simplify(const Vector<Formula<Y>>& a);

//! \brief Tests whether two formulas are identical.
template<class Y> Bool identical(const Formula<Y>& a1, const Formula<Y>& a2);
//! \brief Tests whether two vector formulas are identical.
template<class Y> Bool identical(const Vector<Formula<Y>>& a1, const Vector<Formula<Y>>& a2);

//! \brief Returns \a true if the formula \a a is syntactically equal to a constant.
template<class Y> Bool is_constant(Formula<Y> const& f);

//! \brief Returns \a true if the formula \a a is syntactically equal to the constant \a c.
template<class Y> Bool is_constant(const Formula<Y>& f, const SelfType<Y>& c);

//! \brief Returns \a true if the formula \a a is syntactically constant in the indices \a is.
template<class Y> Bool is_constant_in(const Formula<Y>& a, const Set<Nat>& is);
//! \brief Returns \a true if the formula \a a is syntactically affine in the indices \a is.
template<class Y> Bool is_affine_in(const Formula<Y>& a, const Set<Nat>& is);
//! \brief Returns \a true if the vector formula \a e is syntactically affine in the indices \a is.
template<class Y> Bool is_affine_in(const Vector<Formula<Y>>& a, const Set<Nat>& is);
//! \brief Returns \a true if the vector formula \a a is syntactically additive (possibly with multipliers) in the indices \a is.
template<class Y> Bool is_additive_in(const Vector<Formula<Y>>& a, const Set<Nat>& is);

template<class Y> Formula<Y> derivative(Formula<Y> const& f, SizeType j) { return f._derivative(j); }
template<class X, class Y> X evaluate(const Formula<Y>& f, const Vector<X>& x);
template<class X, class Y> Vector<X> evaluate(const Vector<Formula<Y>>& f, const Vector<X>& x);

template<class X, class Y> X make_constant(const Y& c, X r);
template<class X, class Y> X make_constant(const Y& c, const Vector<X>& v);

template<class X, class Y> inline X make_constant(const Y& c, X r) {
    r=c; return r; }

inline Real make_constant(const EffectiveNumber& c, const Real& x) {
    return Real(c); }
inline Formula<Real> make_constant(const EffectiveNumber& c, const Formula<Real>& x) {
    return Formula<Real>::constant(Real(c)); }
template<SameAs<Real> X> Algebra<X> make_constant(const EffectiveNumber& c, const Algebra<X>& x) {
    return make_constant(Real(c),x); }
template<SameAs<Real> X> ElementaryAlgebra<X> make_constant(const EffectiveNumber& c, const ElementaryAlgebra<X>& x) {
    return make_constant(Real(c),x); }

// Make a constant of type Y with value c based on a prototype vector v
template<class X, class Y> inline X make_constant(const Y& c, const Vector<X>& v) {
    return make_constant(c,v.zero_element());
}

template<class X, class Y> X direct_evaluate(const Formula<Y>& f, const Vector<X>& x);

template<class X, class Y> X cached_evaluate(const Formula<Y>& f, const Vector<X>& v);

template<class X, class Y> Vector<X> cached_evaluate(const Vector<Formula<Y>>& f, const Vector<X>& v);

template<class X, class Y> X evaluate(const Formula<Y>& f, const Vector<X>& x);

template<class X, class Y> Vector<X> evaluate(const Vector<Formula<Y>>& f, const Vector<X>& x);

//! \ingroup FunctionModule
//! \brief Convert a power-series expansion into a formula using a version of Horner's rule.
//! See J. M. Pena and T. Sauer, "On the multivariate Horner scheme", SIAM J. Numer. Anal. 37(4) 1186-1197, 2000.
//!
template<class X> Formula<X> formula(const Expansion<MultiIndex,X>& e);

template<class Y> Formula<Y> substitute(const Formula<Y>& a, const Nat& i, const Formula<Y>& is);

template<class Y> Formula<Y> substitute(const Formula<Y>& a, const Nat& i, const Y& c) {
    return substitute(a,i,Formula<Y>::constant(c)); }

template<class Y> Formula<Y> substitute(const Formula<Y>& a, const Pair<Nat,Formula<Y>>& as) {
    return substitute(a,as.first,as.second); }

template<class Y> Formula<Y> substitute(const Formula<Y>& a, const List<Pair<Nat,Formula<Y>>>& as) {
    Formula<Y> r=a; for(SizeType i=0; i!=as.size(); ++i) { r=substitute(r,as[i]); } return r; }

template<class Y> Vector<Formula<Y>> substitute(const Vector<Formula<Y>>& av, const List<Pair<Nat,Formula<Y>>>& as) {
    return Vector<Formula<Y>>(av.size(),[&av,&as](SizeType i){return substitute(av[i],as);}); }


template<class Y> Formula<Y> simplify(const Formula<Y>& a);


template<class Y> Vector<Formula<Y>> simplify(const Vector<Formula<Y>>& a);

template<class Y> Bool identical(const Formula<Y>& a1, const Formula<Y>& a2);


template<class Y> Bool identical(const Vector<Formula<Y>>& a1, const Vector<Formula<Y>>& a2);

template<class Y> Bool is_constant_in(const Formula<Y>& a, const Set<Nat>& is);


template<class Y> Bool is_affine_in(const Formula<Y>& a, const Set<Nat>& is);


template<class Y> Bool is_affine_in(const Vector<Formula<Y>>& as, const Set<Nat>& is);


template<class Y> Bool is_additive_in(const Vector<Formula<Y>>& as, const Set<Nat>& is);

template<class Y> class Coordinates {
  public:
    operator Formula<Y> () const { return Formula<Y>::coordinate(0); }
    Formula<Y> operator[] (SizeType i) const { return Formula<Y>::coordinate(i); }
};
static const Coordinates<EffectiveNumber> _ = Coordinates<EffectiveNumber>();

} // namespace Ariadne


#endif // ARIADNE_FORMULA_HPP
