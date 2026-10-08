/***************************************************************************
 *            function/polynomial.hpp
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

/*! \file function/polynomial.hpp
 *  \brief Base class for polynomial rings.
 */

#ifndef ARIADNE_POLYNOMIAL_HPP
#define ARIADNE_POLYNOMIAL_HPP

#include <cassert>
#include <string>
#include <iostream>
#include <vector>
#include <map>
#include <algorithm>

#include "algebra/multi_index.hpp"
#include "algebra/expansion.hpp"
#include "algebra/expansion.inl.hpp"
#include "algebra/operations.hpp"
#include "algebra/differential.hpp"

namespace Ariadne {

template<class T> class Array;
template<class X> class Algebra;

template<class I, class X> class Monomial;
template<class I, class X> class Polynomial;

template<class X> using UnivariateMonomial = Monomial<UniIndex,X>;
template<class X> using UnivariatePolynomial = Polynomial<UniIndex,X>;

template<class X> using MultivariateMonomial = Monomial<MultiIndex,X>;
template<class X> using MultivariatePolynomial = Polynomial<MultiIndex,X>;

template<class I, class Y> using ArgumentOf = typename IndexTraits<I>::template Argument<Y>;

template<class X, class A> ArithmeticType<X,A> horner_evaluate(const Expansion<MultiIndex,X>& e, const Vector<A>& y);
template<class X, class A> ArithmeticType<X,A> horner_evaluate(const Expansion<UniIndex,X>& e, const A& y);

//! \brief A monomial with index \a I and coefficients of some type \a X.
template<class I, class X>
class Monomial
    : public ExpansionValue<I,X>
{
  public:
    Monomial(const I& a, const X& x) : ExpansionValue<I,X>(a,x) { }
    Monomial(const ExpansionValue<I,X>& v) : ExpansionValue<I,X>(v) { }
};

template<class I, class X> class PolynomialConstructors {
};

template<class X> class PolynomialConstructors<UniIndex,X> {
    typedef UniIndex I;
  public:
    static Polynomial<I,X> constant(X const& c) { return Polynomial<I,X>::_constant(SizeOne(),c); }
    static Polynomial<I,X> coordinate(X const& z) { return Polynomial<I,X>::_coordinate(SizeOne(),IndexZero(),z); }
    template<class... PRS> requires Constructible<X,Nat,PRS...> static Polynomial<I,X> coordinate(PRS... prs) { return Polynomial<I,X>::_coordinate(SizeOne(),IndexZero(),X(0u,prs...)); }
//    static Polynomial<I,X> variable() { return Polynomial<I,X>::_coordinate(SizeOne(),IndexZero()); }
    static Polynomial<I,X> constant(SizeOne as, X const& c) { return Polynomial<I,X>::_constant(as,c); }
    static Polynomial<I,X> coordinate(SizeOne as, IndexZero j, X const& z) { return Polynomial<I,X>::_coordinate(as,j,z); }
    template<class... PRS> requires Constructible<X,Nat,PRS...> static Polynomial<I,X> coordinate(SizeOne as, IndexZero j, PRS... prs) { return Polynomial<I,X>::_coordinate(as,j,X(0u,prs...)); }
//    static Polynomial<I,X> variable(SizeOne as, IndexZero j) { return Polynomial<I,X>::_coordinate(as,j); }
};

template<class X> class PolynomialConstructors<MultiIndex,X> {
    typedef MultiIndex I;
  public:
    //! \brief Create a constant polynomial in \a as variables with value \a c.
    static Polynomial<I,X> constant(SizeType as, X const& c) { return Polynomial<I,X>::_constant(as,c); }
    //! \brief Create a polynomial in \a as variables which returns the value of the \a j<sup>th</sup> variable.
    static Polynomial<I,X> coordinate(SizeType as, SizeType j, X const& z) { return
        Polynomial<I,X>::_coordinate(as,j,z); }
    template<class... PRS> requires Constructible<X,Nat,PRS...> static Polynomial<I,X> coordinate(SizeType as, SizeType j, PRS... prs) { return
        Polynomial<I,X>::_coordinate(as,j,X(0u,prs...)); }
    template<class... PRS> static Polynomial<I,X> variable(SizeType as, SizeType j, PRS... prs) { return Polynomial<I,X>::_coordinate(as,j,X(0u,prs...)); }
};


//! \ingroup FunctionModule
//! \brief A polynomial with coefficients of some type \a X.
template<class I, class X>
class Polynomial
    : public PolynomialConstructors<I,X>
    , public DispatchAlgebraOperations<Polynomial<I,X>,X>
{
    template<class II, class XX> friend class Polynomial;
    friend struct AlgebraOperations<Polynomial<I,X>,X>;
  public:
    typedef typename Expansion<I,X>::ValueType ValueType;
    typedef typename Expansion<I,X>::Reference Reference;
    typedef typename Expansion<I,X>::ConstReference ConstReference;
    typedef typename Expansion<I,X>::Iterator Iterator;
    typedef typename Expansion<I,X>::ConstIterator ConstIterator;

    typedef typename Expansion<I,X>::IndexReference IndexReference;
    typedef typename Expansion<I,X>::IndexConstReference IndexConstReference;
    typedef typename Expansion<I,X>::CoefficientReference CoefficientReference;
    typedef typename Expansion<I,X>::CoefficientConstReference CoefficientConstReference;

    typedef typename Expansion<I,X>::IndexInitializerType IndexInitializerType;

    template<class Y> using Argument = typename IndexTraits<I>::template Argument<Y>;

    typedef typename Expansion<I,X>::ArgumentSizeType ArgumentSizeType;
    typedef typename Expansion<I,X>::VariableIndexType VariableIndexType;
    typedef I IndexType;

    typedef typename X::Paradigm Paradigm;
    typedef typename X::NumericType NumericType;
    typedef Polynomial<I,X> SelfType;
    typedef ReverseLexicographicIndexLess ComparisonType;
    typedef ReverseLexicographicLess IndexComparisonType;
    typedef Ariadne::CharacteristicsType<X> CoefficientCharacteristicsType;
    typedef Tuple<ArgumentSizeType, CoefficientCharacteristicsType> CharacteristicsType;
  public:
    //!@{
    //! \name Constructors

    //! \brief The zero polynomial in \a as variables, with characteristics from \a z.
    explicit Polynomial(ArgumentSizeType as, CoefficientCharacteristicsType pr);
    //! \brief The zero polynomial in \a as variables, with zero taking characteristics from \a z.
    explicit Polynomial(ArgumentSizeType as, X const& z);
    //! \brief The zero polynomial in \a as variables.
    template<class... PRS> requires Constructible<X,PRS...>
        explicit Polynomial(ArgumentSizeType as, PRS... prs);
    //! \brief Copy/conversion constructor.
    template<ConvertibleTo<X> XX> Polynomial(const Polynomial<I,XX>& p);
    //! \brief Copy/conversion constructor.
    template<ConvertibleTo<X> XX> explicit Polynomial(const Expansion<I,XX>& e);
    //! \brief A sparse polynomial with coefficients given by an initializer list of indices and coefficients.
    Polynomial(InitializerList<Pair<IndexInitializerType,X>> lst);
    //! \brief Construct a differential of degree \a deg from an initializer list of (index,coefficient) pairs.
    template<class... PRS> requires Constructible<X,ExactDouble,PRS...>
        explicit Polynomial(InitializerList<Pair<IndexInitializerType,ExactDouble>> lst, PRS... prs);
    //!@}

    //! \brief Characteristics needed to construct a compatible null polynomial.
    CharacteristicsType characteristics() const;
    //! \brief Create the null polynomial in the same number of variables.
    Polynomial<I,X> create_zero() const;

    using PolynomialConstructors<I,X>::constant;
    using PolynomialConstructors<I,X>::coordinate;

    //! \brief Create an Array of polynomials in \a as variables,
    //! the i<sup>th</sup> of  which returns the value of the i<sup>th</sup> variable.
    static Argument<Polynomial<I,X>> coordinates(ArgumentSizeType as, X const& z);
    static Argument<Polynomial<I,X>> variables(ArgumentSizeType as, X const& z);
    template<class... PRS> requires Constructible<X,Nat,PRS...> Argument<Polynomial<I,X>> static coordinates(ArgumentSizeType as, PRS... prs) { return coordinates(as,X(0u,prs...)); }
    template<class... PRS> requires Constructible<X,Nat,PRS...> Argument<Polynomial<I,X>> static variables(ArgumentSizeType as, PRS... prs) { return variables(as,X(0u,prs...)); }

    //! \brief %Set equal to a constant.
    Polynomial<I,X>& operator=(const X& x);
    //!@{
    //! \name Comparisons

    //! \brief Equality operator.
    template<class XX> EqualityType<X,XX> operator==(const Polynomial<I,XX>& p) const;
    //! \brief Inequality operator.
    template<class XX> InequalityType<X,XX> operator!=(const Polynomial<I,XX>& p) const;
    //!@}

    //!@{
    //! \name Data access

    //! \brief The number of variables in the argument of the polynomial.
    ArgumentSizeType argument_size() const;
    //! \brief The number of structural nonzero terms.
    SizeType number_of_terms() const;
    //! \brief The order of the highest term.
    DegreeType degree() const;
    //! \brief The value of the polynomial at zero.
    const X& value() const;
    //! \brief A reference to the coefficient of the term in \f$x^{a_1}\cdots x^{a_n}\f$.
    X& operator[](const IndexType& a);
    //! \brief A constant referent to the coefficient of the term in \f$x^{a_1}\cdots x^{a_n}\f$.
    const X& operator[](const IndexType& a) const;
    //! \brief A constant reference to the raw data expansion.
    const Expansion<I,X>& expansion() const;
    //! \brief A reference to the raw data expansion.
    Expansion<I,X>& expansion();
    //! \brief A zero value usable as a coefficient.
    X const& zero_coefficient() const;
    //!@}

    //!@{
    //! \name Iterators

    //! \brief An Iterator to the beginning of the list of terms.
    Iterator begin();
    //! \brief An Iterator to the end of the list of terms..
    Iterator end();
    //! \brief An Iterator to the term in \f$x^a\f$. Returns \c end() if there is no term in \a a.
    Iterator find(const I& a);
    //! \brief A constant Iterator to the beginning of the list of terms.
    ConstIterator begin() const;
    //! \brief A constant Iterator to the end of the list of terms.
    ConstIterator end() const;
    //! \brief An Iterator to the term in \f$x^a\f$. Returns \c end() if there is no term in \a a.
    ConstIterator find(const I& a) const;
    //!@}


    //!@{
    //! \name Modifying operations

    //! \brief Insert the term \f$c x^{a_1}\f$ into a sorted list of terms.
    Void insert(const IndexType& a, const X& c);
    //! \brief Reserve space for a total of \a n terms.
    Void reserve(SizeType n);
    //! \brief Remove the term pointed to by \a iter. May be expensive if the term is near the beginning of the list of terms.
    Void erase(Iterator iter);
    //! \brief %Set the polynomial to zero.
    Void clear();
    //! \brief Remove all zero terms from the expansion, and order the expansion reverse lexicographically by term.
    Void cleanup();
    //!@}

    //!@{
    //! \name Evaluation

    //! Evaluate on a vector of algebra elements.
    template<class A> A operator() (Vector<A> const&) const;
    //!@}

    //!@{
    //! \name Modifying operators

    //! \brief Truncate to degree \a d.
    Polynomial<I,X>& truncate(DegreeType d);
    //! \brief Differentiate with respect to the \a j<sup>th</sup> variable.
    Polynomial<I,X>& differentiate(VariableIndexType j);
    //! \brief Antidifferentiate (integrate) with respect to the \a j<sup>th</sup> variable.
    Polynomial<I,X>& antidifferentiate(VariableIndexType j);
    //!@}

    //!@{
    //! \name Related operations
    friend Polynomial<I,X>& operator*=(Polynomial<I,X>& p, const Monomial<I,X>& m) { return Polynomial<I,X>::_imul(p,m); }

    template<class XX, class A> friend A evaluate(const UnivariateMonomial<XX>& p, const A& v);
    template<class XX, class A> friend A evaluate(const MultivariatePolynomial<XX>& p, const Vector<A>& v);
    template<class II, class XX> friend Polynomial<II,XX> compose(const UnivariatePolynomial<XX>& p, const Scalar<Polynomial<II,XX>>& q);
    template<class II, class XX> friend Polynomial<II,XX> compose(const MultivariatePolynomial<XX>& p, const Vector<Polynomial<II,XX>>& q);
    //!@}

    Void check() const;
    static Polynomial<I,X> _constant(ArgumentSizeType as, const X& c);
    static Polynomial<I,X> _coordinate(ArgumentSizeType as, VariableIndexType j, X const& z);
    static Polynomial<UniIndex,X> _compose(const Polynomial<I,X>& p, const ArgumentOf<I,Polynomial<UniIndex,X>>& q);
    static Polynomial<MultiIndex,X> _compose(const Polynomial<I,X>& p, const ArgumentOf<I,Polynomial<MultiIndex,X>>& q);
    static X _evaluate(const Polynomial<I,X>& p, const ArgumentOf<I,X>& vx);
    static Algebra<X> _evaluate(const Polynomial<I,X>& p, const ArgumentOf<I,Algebra<X>>& va);
    static Polynomial<I,X> _partial_evaluate(const Polynomial<I,X>& p, SizeType k, const X& c);
    OutputStream& _write(OutputStream& os) const;
    OutputStream& _write(OutputStream& os, typename IndexTraits<I>::NameType const& names) const;
  private:
    Void _append(const IndexType& a, const X& c);
    Iterator _unique_key();
  private:
    SortedExpansion<I,X,ReverseLexicographicIndexLess> _expansion;
  private: // FIXME: Put these concrete-generic operations in proper place
    template<AssignableTo<X> Y>
        friend Polynomial<I,X> operator+(Polynomial<I,X> p, const Y& c) {
            X xc=p.value(); xc=c; return p+xc; }
    template<AssignableTo<X> Y>
        friend Polynomial<I,X> operator-(Polynomial<I,X> p, const Y& c) {
            X xc=p.value(); xc=c; return p-xc; }
    template<AssignableTo<X> Y>
        friend Polynomial<I,X> operator*(const Y& c, Polynomial<I,X> p) {
            X xc=p.value(); xc=c; return xc*p; }
    template<AssignableTo<X> Y>
        friend Polynomial<I,X> operator*(Polynomial<I,X> p, const Y& c) {
            X xc=p.value(); xc=c; return p*xc; }
    template<AssignableTo<X> Y>
        friend Polynomial<I,X> operator/(Polynomial<I,X> p, const Y& c) {
            X xc=p.value(); xc=c; return p/xc; }

};

template<class I, class X> struct AlgebraOperations<Polynomial<I,X>> {
    typedef I IndexType;
  public:
    static Polynomial<I,X> apply(Nul, const Polynomial<I,X>& p);
    static Polynomial<I,X> apply(Pos, const Polynomial<I,X>& p);
    static Polynomial<I,X> apply(Neg, const Polynomial<I,X>& p);
    static Polynomial<I,X> apply(Add, const Polynomial<I,X>& p1, const Polynomial<I,X>& p2);
    static Polynomial<I,X> apply(Sub, const Polynomial<I,X>& p1, const Polynomial<I,X>& p2);
    static Polynomial<I,X> apply(Mul, const Polynomial<I,X>& p1, const Polynomial<I,X>& p2);
    static Polynomial<I,X> apply(Add, Polynomial<I,X> p, const X& c);
    static Polynomial<I,X> apply(Mul, Polynomial<I,X> p, const X& c);
    static Polynomial<I,X> apply(Mul, Polynomial<I,X> p, const Monomial<I,X>& m);
    static Polynomial<I,X>& iapply(Add, Polynomial<I,X>& p, const X& c);
    static Polynomial<I,X>& iapply(Mul, Polynomial<I,X>& p, const X& c);
    static Polynomial<I,X>& iapply(Mul, Polynomial<I,X>& p, const Monomial<I,X>& m);

};


template<class I, class X> template<ConvertibleTo<X> XX> Polynomial<I,X>::Polynomial(const Polynomial<I,XX>& p)
    : _expansion(p._expansion) { }

template<class I, class X> template<ConvertibleTo<X> XX> Polynomial<I,X>::Polynomial(const Expansion<I,XX>& e)
    : _expansion(e) { this->cleanup(); }

template<class I, class X> template<class... PRS> requires Constructible<X,PRS...>
Polynomial<I,X>::Polynomial(ArgumentSizeType as, PRS... prs) : Polynomial(as,X(prs...)) { }

template<class I, class X> template<class... PRS> requires Constructible<X,ExactDouble,PRS...>
Polynomial<I,X>::Polynomial(InitializerList<Pair<IndexInitializerType,ExactDouble>> lst, PRS... prs)
    : _expansion(lst,prs...) { this->cleanup(); }

template<class I, class X> template<class XX> EqualityType<X,XX> Polynomial<I,X>::operator==(const Polynomial<I,XX>& p) const {
    const_cast<Polynomial<I,X>*>(this)->cleanup();
    const_cast<Polynomial<I,XX>&>(p).cleanup();
    return this->_expansion==p._expansion;
}

template<class I, class X> template<class XX> InequalityType<X,XX> Polynomial<I,X>::operator!=(const Polynomial<I,XX>& p) const {
    return !(*this==p);
}

template<class I, class X> inline Polynomial<I,X> partial_evaluate(const Polynomial<I,X>& p, SizeType k, const X& c) {
    return Polynomial<I,X>::_partial_evaluate(p,k,c); }

template<class X> inline X evaluate(const UnivariatePolynomial<X>& p, const Scalar<X>& v) {
    return UnivariatePolynomial<X>::_evaluate(p,v); }

template<class X> inline X evaluate(const MultivariatePolynomial<X>& p, const Vector<X>& v) {
    return MultivariatePolynomial<X>::_evaluate(p,v); }

template<class X> inline Vector<X> evaluate(Vector<MultivariatePolynomial<X>> const& p, Vector<X> const& v) {
    Vector<X> r(p.size(),v.zero_element());
    for (SizeType i=0; i!= p.size(); ++i) r[i] = evaluate(p[i],v);
    return r;
}

template<class I, class X> inline Polynomial<I,X> compose(const UnivariatePolynomial<X>& p, const Polynomial<I,X>& q) {
    return UnivariatePolynomial<X>::_compose(p,q); }

template<class I, class X> inline Polynomial<I,X> compose(const MultivariatePolynomial<X>& p, const Vector<Polynomial<I,X>>& q) {
    return MultivariatePolynomial<X>::_compose(p,q); }

template<class I, class X> inline Vector<Polynomial<I,X>> compose(const Vector<MultivariatePolynomial<X>>& p, const Vector<Polynomial<I,X>>& q) {
    const SizeType rs = p.size();
    Vector<Polynomial<I,X>> r(rs,[&p,&q](SizeType i){return compose(p[i],q);}, q.element_characteristics());
    return r;
}

template<class X, class A> inline A evaluate(const UnivariatePolynomial<X>& p, const Scalar<A>& v) {
    return horner_evaluate(p.expansion(),v); }

template<class X, class A> inline A evaluate(const MultivariatePolynomial<X>& p, const Vector<A>& v) {
    return horner_evaluate(p.expansion(),v); }

template<class I, class X> inline Polynomial<I,X> derivative(Polynomial<I,X> p, SizeType k) {
    p.differentiate(k); return p; }

template<class I, class X> inline Polynomial<I,X> antiderivative(Polynomial<I,X> p, SizeType k) {
    p.antidifferentiate(k); return p; }

template<class I, class X> inline Polynomial<I,X> truncate(Polynomial<I,X> p, DegreeType deg) {
    p.truncate(deg); return p; }

template<class I, class X> inline OutputStream& operator<<(OutputStream& os, const Polynomial<I,X>& p) {
    return p._write(os); }

template<class I, class X> inline Bool compatible(const Polynomial<I,X>& x1, const Polynomial<I,X>& x2) {
    return x1.argument_size()==x2.argument_size(); }

template<class F> struct NamedArgumentRepresentation {
    const F& function; const List<String>& argument_names;
};

template<class F> inline NamedArgumentRepresentation<F> named_argument_repr(const F& function, const List<String>& argument_names) {
    NamedArgumentRepresentation<F> r={function,argument_names}; return r; }

template<class I, class X> inline OutputStream& operator<<(OutputStream& os, const NamedArgumentRepresentation<Polynomial<I,X>>& repr) {
    return repr.function._write(os,repr.argument_names); }

template<class I, class X> inline MultivariatePolynomial<MidpointType<X>> midpoint(const Polynomial<I,X>& p) {
    return MultivariatePolynomial<MidpointType<X>>(midpoint(p.expansion())); }


// Vectorised operations
template<class I, class X, class A> Vector<A> evaluate(const Vector<Polynomial<I,X>>& p, const Vector<A>& v) {
    return Vector<A>(p.size(),[&p,v](SizeType i){return evaluate(p[i],v);});
}

template<class I, class X> Vector<Polynomial<I,X>> derivative(const Vector<Polynomial<I,X>>& p, SizeType j) {
    return Vector<Polynomial<I,X>>(p.size(),[&p,j](SizeType i){return derivative(p[i],j);});
}

template<class I, class X> Vector<Polynomial<I,X>> antiderivative(const Vector<Polynomial<I,X>>& p, SizeType j) {
    return Vector<Polynomial<I,X>>(p.size(),[&p,j](SizeType i){return antiderivative(p[i],j);},p.element_characteristics());
}

template<class I, class X> Vector<Polynomial<I,X>> truncate(const Vector<Polynomial<I,X>>& p, DegreeType d) {
    return Vector<Polynomial<I,X>>(p.size(),[&p,d](SizeType i){return truncate(p[i],d);},characteristics(truncate(p.zero_element(),d)));
 }

template<class I, class X> Vector<MultivariatePolynomial<MidpointType<X>>> midpoint(const Vector<Polynomial<I,X>>& p) {
    return Vector<Polynomial<I,X>>(p.size(),[&p](SizeType i){return midpoint(p[i]);});
}

//! \brief Compute the Lie derivative of \a g with respect to the vector field \a f
template<class I, class X> Vector<Polynomial<I,X>> lie_derivative(Vector<Polynomial<I,X>> const& f, Vector<Polynomial<I,X>> const& g) {
    const SizeType rs = f.size();
    Vector<Polynomial<I,X>> r(rs,f[0].create_zero());
    for (SizeType d=0; d!=rs; ++d) {
        for (SizeType i=0; i!=rs; ++i) {
            r[d] += derivative(g[d],i)*f[i];
        }
        r[d] += derivative(g[d],rs);
    }
    return r;
}



//! \brief Compute the flow polynomial from the vector field \a f, with order \a d, using Picard iteration.
template<class I, class X> Vector<Polynomial<I,X>> flow_polynomial_picard_iteration(Vector<Polynomial<I,X>> const& f, DegreeType d);

//! \brief Compute the flow polynomial from the vector field \a f, with order \a d, using Lie derivative.
template<class I, class X> Vector<Polynomial<I,X>> flow_polynomial_lie_derivative(Vector<Polynomial<I,X>> const& f, DegreeType d);

//! \brief Compute the flow polynomial from the vector field \a f, with order \a d.
//! \details For a polynomial \f$p(x,t)\f$, the flow \f$\phi(x,t)\f$ is the solution of the differential equation \f$\dot{\phi}(x,t) = p(\phi(x,t),t)\f$ with \f$\phi(x,0)=x\f$.
template<class I, class X> Vector<Polynomial<I,X>> flow_polynomial(Vector<Polynomial<I,X>> const& f, DegreeType d);

} // namespace Ariadne

#endif /* ARIADNE_POLYNOMIAL_HPP */
