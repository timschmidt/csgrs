/***************************************************************************
 *            algebra/univariate_differential.hpp
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

/*! \file algebra/univariate_differential.hpp
 *  \brief Differentials with respect to a single variable.
 */

#ifndef ARIADNE_UNIVARIATE_DIFFERENTIAL_HPP
#define ARIADNE_UNIVARIATE_DIFFERENTIAL_HPP

#include "utility/macros.hpp"
#include "utility/array.hpp"
#include "algebra/series.hpp"
#include "numeric/numeric.hpp"

namespace Ariadne {


template<class X> class Series;
template<class X> class UnivariateDifferential;

//! \ingroup DifferentiationSubModule
//! \brief Arbitrary-order derivatives with respect to a single argument.
template<class X> class UnivariateDifferential
    : public DispatchTranscendentalAlgebraOperations<UnivariateDifferential<X>,X>
    , public DispatchLatticeAlgebraOperations<UnivariateDifferential<X>,X>
    , public ProvideConcreteGenericArithmeticOperations<UnivariateDifferential<X>>
{
    Array<X> _ary;
  public:
    typedef X NumericType;
    typedef X ValueType;
    typedef typename X::Paradigm Paradigm;
    typedef UnivariateDifferential<X> SelfType;
    typedef typename Array<X>::Iterator Iterator;
    typedef typename Array<X>::ConstIterator ConstIterator;

    UnivariateDifferential() = delete;
    explicit UnivariateDifferential(DegreeType d, const NumericType& c);
    template<class... PRS> requires Constructible<X,Nat,PRS...>
        explicit UnivariateDifferential(DegreeType d, PRS... prs) : UnivariateDifferential(d,X(0u,prs...)) { }
    UnivariateDifferential(DegreeType d, InitializerList<X> e);
    UnivariateDifferential(Array<X> ary) : _ary(ary) { }
    template<class... PRS> requires Constructible<X,ExactDouble,PRS...>
        explicit UnivariateDifferential(DegreeType deg, InitializerList<ExactDouble> lst, PRS... prs);
    UnivariateDifferential(DegreeType d, Series<X> const& s); // explicit

    template<class OP> UnivariateDifferential(OP op, DegreeType d, X const& c);

    static SelfType constant(DegreeType d, const NumericType& c);
    static SelfType variable(DegreeType d, const NumericType& c);

    template<class Y, class... PRS> requires Constructible<X,Y,PRS...>
    static SelfType constant(DegreeType deg, const Y& c, const PRS&... prs) {
        return constant(deg,X(c,prs...)); }
    template<class Y, class... PRS> requires Constructible<X,Y,PRS...>
    static SelfType variable(DegreeType deg, const Y& v, const PRS&... prs) {
        return variable(deg,X(v,prs...)); }

    SelfType create_zero() const;
    SelfType create_constant(const NumericType& c) const;
    SelfType create_variable(const NumericType& c) const;

    SizeType argument_size() const;
    DegreeType degree() const;
    X zero_coefficient() const;
    const Array<X>& array() const;
    Array<X>& array();
    const X& operator[](SizeType k) const;
    X& operator[](SizeType k);

    SelfType& operator=(const NumericType& c);
    template<AssignableTo<X> W>
        SelfType& operator=(const W& c) { X xc=nul(this->value()); xc=c; return (*this)=xc; }

    SelfType& operator+=(const SelfType& x);
    SelfType& operator-=(const SelfType& x);
    SelfType& operator*=(const SelfType& x);
    SelfType& operator+=(const NumericType& c);
    SelfType& operator*=(const NumericType& c);

    SelfType apply(Rec) const;

    const X& value() const;
    const X& gradient() const;
    const X hessian() const;
    const X& half_hessian() const;

    Void clear();
    OutputStream& _write(OutputStream& os) const;

    friend UnivariateDifferential<X> compose(Series<X> const& f, UnivariateDifferential<X> const& dx) {
        return UnivariateDifferential<X>::_compose(f,dx); }
    friend UnivariateDifferential<X> compose(UnivariateDifferential<X> const& f, UnivariateDifferential<X> const& dx) {
        return UnivariateDifferential<X>::_compose(f,dx); }
    friend UnivariateDifferential<X> derivative(UnivariateDifferential<X> const& dx) {
        return UnivariateDifferential<X>::_derivative(dx); }
    friend UnivariateDifferential<X> antiderivative(UnivariateDifferential<X> const& dx) {
        return UnivariateDifferential<X>::_antiderivative(dx); }
    friend UnivariateDifferential<X> antiderivative(UnivariateDifferential<X> const& dx, X const& c) {
        return UnivariateDifferential<X>::_antiderivative(dx,c); }

    friend OutputStream& operator<<(OutputStream& os, UnivariateDifferential<X> const& dx) {
        return dx._write(os); }
  public:
    static Differential<X> _compose(Series<X> const& f, Differential<X> const& dx);
  private:
    static UnivariateDifferential<X> _compose(Series<X> const& f, UnivariateDifferential<X> const& dx);
    static UnivariateDifferential<X> _compose(UnivariateDifferential<X> const& f, UnivariateDifferential<X> const& dx);
    static UnivariateDifferential<X> _derivative(UnivariateDifferential<X> const& dx);
    static UnivariateDifferential<X> _antiderivative(UnivariateDifferential<X> const& dx);
    static UnivariateDifferential<X> _antiderivative(UnivariateDifferential<X> const& dx, X const& c);
};

template<class X> template<class OP> UnivariateDifferential<X>::UnivariateDifferential(OP op, DegreeType d, X const& c)
    : UnivariateDifferential(d,Series<X>(op,c)) { }

template<class X> inline const X& UnivariateDifferential<X>::value() const { return _ary[0]; }
template<class X> inline const X& UnivariateDifferential<X>::gradient() const { assert(this->degree()>=1); return _ary[1]; }
template<class X> inline const X& UnivariateDifferential<X>::half_hessian() const { assert(this->degree()>=2); return _ary[2]; }
template<class X> inline const X UnivariateDifferential<X>::hessian() const { assert(this->degree()>=2); return _ary[2]*2; }

template<class X> template<class... PRS> requires Constructible<X,ExactDouble,PRS...>
UnivariateDifferential<X>::UnivariateDifferential(DegreeType d, InitializerList<ExactDouble> lst, PRS... prs)
    : _ary(d+1u,X(0,prs...))
{
    auto iter=lst.begin();
    for(SizeType i=0; iter!=lst.end(); ++i, ++iter) { _ary[i]=X(*iter,prs...); }
}



template<class X> UnivariateDifferential<X> create_zero(const UnivariateDifferential<X>& c) {
    return UnivariateDifferential<X>(c.degree(),create_zero(c[0])); }

template<class X> struct AlgebraOperations<UnivariateDifferential<X>,X> {
    static UnivariateDifferential<X> apply(Pos, UnivariateDifferential<X> x) {
        return x; }

    static UnivariateDifferential<X> apply(Neg, UnivariateDifferential<X> x) {
        for(DegreeType i=0; i<=x.degree(); ++i) { x[i]=-x[i]; } return x; }

    static UnivariateDifferential<X> apply(Sqr, const UnivariateDifferential<X>& dx) {
        return apply(Mul(),dx,dx); }

    static UnivariateDifferential<X> apply(Add, const UnivariateDifferential<X>& x1, const UnivariateDifferential<X>& x2) {
        UnivariateDifferential<X> r(std::min(x1.degree(),x2.degree()),nul(x1[0]+x2[0]));
        for(DegreeType i=0; i<=r.degree(); ++i) { r[i]=x1[i]+x2[i]; } return r; }

    static UnivariateDifferential<X> apply(Sub, const UnivariateDifferential<X>& x1, const UnivariateDifferential<X>& x2) {
        UnivariateDifferential<X> r(std::min(x1.degree(),x2.degree()),nul(x1[0]-x2[0]));
        for(DegreeType i=0; i<=r.degree(); ++i) { r[i]=x1[i]-x2[i]; } return r; }

    static UnivariateDifferential<X> apply(Mul, const UnivariateDifferential<X>& x1, const UnivariateDifferential<X>& x2) {
        UnivariateDifferential<X> r(std::min(x1.degree(),x2.degree()),nul(x1[0]*x2[0]));
        for(DegreeType i1=0; i1<=r.degree(); ++i1) {
            for(DegreeType i2=0; i2<=r.degree()-i1; ++i2) {
                r[static_cast<DegreeType>(i1+i2)]+=x1[i1]*x2[i2];
            }
        }
        return r;
    }

    static UnivariateDifferential<X> apply(Div, const UnivariateDifferential<X>& x1, const UnivariateDifferential<X>& x2) {
        return x1*rec(x2); }

    static UnivariateDifferential<X> apply(Add, UnivariateDifferential<X> x, X const& c) {
        x[0]+=c; return x; }

    static UnivariateDifferential<X> apply(Mul, UnivariateDifferential<X> x, X const& c) {
        for(DegreeType i=0; i<=x.degree(); ++i) { x[i]*=c; } return x; }

    template<class OP> static UnivariateDifferential<X> apply(OP op, const UnivariateDifferential<X>& dx) {
        return compose(UnivariateDifferential<X>(op,dx.degree(),dx[0]),dx); }

    static UnivariateDifferential<X> apply(Pow, UnivariateDifferential<X> x, Int n) {
        return generic_pow(x,n); }

};


} // namespace Ariadne


#endif // ARIADNE_UNIVARIATE_DIFFERENTIAL_HPP

