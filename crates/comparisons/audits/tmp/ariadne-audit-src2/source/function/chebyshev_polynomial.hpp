/***************************************************************************
 *            function/chebyshev_polynomial.hpp
 *
 *  Copyright  2008-20  Pieter Collins
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

/*! \file function/chebyshev_polynomial.hpp
 *  \brief Chebyshev functions on a bounded domain with a sparse representation.
 */

#ifndef ARIADNE_CHEBYSHEV_POLYNOMIAL_HPP
#define ARIADNE_CHEBYSHEV_POLYNOMIAL_HPP

#include <map>
#include <functional>

#include "utility/macros.hpp"
#include "utility/typedefs.hpp"
#include "utility/array.hpp"
#include "utility/pointer.hpp"
#include "algebra/vector.hpp"
#include "algebra/covector.hpp"
#include "algebra/multi_index.hpp"
#include "algebra/expansion.hpp"
#include "algebra/operations.hpp"

namespace Ariadne {

struct IndexLess;

template<class X> class UnivariateChebyshevPolynomial;
template<class X> class MultivariateChebyshevPolynomial;

template<class X> class UnivariateChebyshevPolynomial
    : public DispatchAlgebraOperations<UnivariateChebyshevPolynomial<X>,X>
{
    typedef typename X::Paradigm P;
    typedef typename X::PrecisionType PR;
    SortedExpansion<UniIndex,X,IndexLess> _terms;
  public:
    template<class XX> using ChebyshevPolynomial = UnivariateChebyshevPolynomial<XX>;

    typedef P Paradigm;
    typedef PR PrecisionType;

    typedef X NumericType;

    UnivariateChebyshevPolynomial(PR pr) : _terms(SizeOne(),pr) { }
    UnivariateChebyshevPolynomial(DegreeType d, std::function<X(DegreeType)> const& g) : _terms(SizeOne(),get_characteristics(g(0))) {
        for(SizeType i=0; i<=d; ++i) { _terms.append(i,g(i)); } }

    static ChebyshevPolynomial<X> constant(Number<P> y, PR pr);
    static ChebyshevPolynomial<X> constant(X const& c);
    static ChebyshevPolynomial<X> coordinate(PR pr);
    static ChebyshevPolynomial<X> basis(SizeType k, PR pr);

    UnivariateChebyshevPolynomial<X> create_constant(Number<P> y);

    DegreeType degree() const { return _terms.back().index(); }
    PR precision() const { return _terms.zero_coefficient().precision(); }
    X const& zero_coefficient() const { return _terms.zero_coefficient(); }
    X const& operator[] (SizeType i) const { return _terms[i]; }

    MagType<X> sup_norm() const;
    friend MagType<X> sup_norm(ChebyshevPolynomial<X> cm) { return cm.sup_norm(); }

    friend decltype(auto) operator==(ChebyshevPolynomial<X> const& cm1, ChebyshevPolynomial<X> const& cm2) {
        return cm1._terms==cm2._terms; }

  private: // FIXME: Put these concrete-generic operations in proper place
    template<AssignableTo<X> Y>
        friend ChebyshevPolynomial<X> operator+(ChebyshevPolynomial<X> p, const Y& c) {
            X xc=p.zero_coefficient(); xc=c; return p+xc; }
    template<AssignableTo<X> Y>
        friend ChebyshevPolynomial<X> operator-(ChebyshevPolynomial<X> p, const Y& c) {
            X xc=p.zero_coefficient(); xc=c; return p-xc; }
    template<AssignableTo<X> Y>
        friend ChebyshevPolynomial<X> operator*(const Y& c, ChebyshevPolynomial<X> p) {
            X xc=p.zero_coefficient(); xc=c; return xc*p; }
    template<AssignableTo<X> Y>
        friend ChebyshevPolynomial<X> operator*(ChebyshevPolynomial<X> p, const Y& c) {
            X xc=p.zero_coefficient(); xc=c; return p*xc; }
    template<AssignableTo<X> Y>
        friend ChebyshevPolynomial<X> operator/(ChebyshevPolynomial<X> p, const Y& c) {
            X xc=p.zero_coefficient(); xc=c; return p/xc; }

  public:
    friend OutputStream& operator<<(OutputStream& os, ChebyshevPolynomial<X> const& cm) { return cm._write(os); }

    X operator() (X const& x) const;
    template<class Y> friend decltype(auto) evaluate(ChebyshevPolynomial<X> const& p, Y const& y) { return _evaluate(p,y); }
  private: public:
    static ChebyshevPolynomial<X> apply(Neg, ChebyshevPolynomial<X> cm);
    static ChebyshevPolynomial<X> apply(Add, ChebyshevPolynomial<X> const& cm1, ChebyshevPolynomial<X> const& cm2);
    static ChebyshevPolynomial<X> apply(Sub, ChebyshevPolynomial<X> const& cm1, ChebyshevPolynomial<X> const& cm2);
    static ChebyshevPolynomial<X> apply(Mul, ChebyshevPolynomial<X> const& cm1, ChebyshevPolynomial<X> const& cm2);
    static ChebyshevPolynomial<X> apply(Add, ChebyshevPolynomial<X> cm1, X const& s2);
    static ChebyshevPolynomial<X> apply(Mul, ChebyshevPolynomial<X> cm1, X const& s2);
    static ChebyshevPolynomial<X> apply(Sqr, ChebyshevPolynomial<X> cm);

    OutputStream& _write(OutputStream& os) const;

  public:
    template<class Y> static auto _evaluate(ChebyshevPolynomial<X> const& f, Y const& v) -> ArithmeticType<X,Y>;
};

template<class X> struct AlgebraOperations<UnivariateChebyshevPolynomial<X>,X> : public UnivariateChebyshevPolynomial<X> { };


template<class X> template<class Y> auto
UnivariateChebyshevPolynomial<X>::_evaluate(UnivariateChebyshevPolynomial<X> const& f, Y const& x) -> ArithmeticType<X,Y> {
    typedef ArithmeticType<X,Y> R;
    R r=x*0;
    if (f._terms.empty()) { return r; }
    R p=x*0+1;
    R c(x);
    R n(x);
    Y tx=x*2;
    SizeType k=1;
    auto iter=f._terms.begin();
    if(iter!=f._terms.end() && iter->index()==0) {
        r=iter->coefficient();
        ++iter;
    }
    while(iter!=f._terms.end()) {
        while (k<iter->index()) {
            n=tx*c-p;
            p=c; c=n;
            ++k;
        }
        r+=c*iter->coefficient();
        ++iter;
    }
    return r;
}



template<class X> class MultivariateChebyshevPolynomial
    : public DispatchAlgebraOperations<MultivariateChebyshevPolynomial<X>,X>
{
    typedef typename X::Paradigm P;
    typedef typename X::PrecisionType PR;
    Expansion<MultiIndex,X> _terms;
  public:
    template<class XX> using ChebyshevPolynomial = MultivariateChebyshevPolynomial<XX>;

    typedef P Paradigm;
    typedef PR PrecisionType;
    typedef X NumericType;
    typedef typename Expansion<MultiIndex,X>::ConstIterator ConstIterator;

    MultivariateChebyshevPolynomial(SizeType as, PR pr) : _terms(as,pr) { }

    static ChebyshevPolynomial<X> constant(SizeType as, Number<P> y, PR pr);
    static ChebyshevPolynomial<X> constant(SizeType as, X const& c);
    static ChebyshevPolynomial<X> coordinate(SizeType as, SizeType i, PR pr);
    static ChebyshevPolynomial<X> basis(SizeType as, SizeType i, SizeType k, PR pr);

    ChebyshevPolynomial<X> create_constant(Number<P> y);

    SizeType argument_size() const { return this->_terms.argument_size(); }
    PR precision() const { return this->_terms.zero_coefficient().precision(); }
    X const& zero_coefficient() const { return _terms.zero_coefficient(); }

    MagType<X> sup_norm() const;
    friend MagType<X> sup_norm(ChebyshevPolynomial<X> const& cm) { return cm.sup_norm(); }

    friend ChebyshevPolynomial<X> operator*(Number<P> const& s1, ChebyshevPolynomial<X> cm2) { return X(s1,cm2.precision()) * cm2; }
    friend OutputStream& operator<<(OutputStream& os, ChebyshevPolynomial<X> const& cm) { return cm._write(os); }

    X operator() (Vector<X> const& x) const;
    friend X evaluate(ChebyshevPolynomial<X> const& f, Vector<X> const& x) { return f(x); }
  private: public:
    static ChebyshevPolynomial<X> apply(Neg, ChebyshevPolynomial<X> cm);
    static ChebyshevPolynomial<X> apply(Sqr, ChebyshevPolynomial<X> cm);
    static ChebyshevPolynomial<X> apply(Add, ChebyshevPolynomial<X> const& cm1, ChebyshevPolynomial<X> const& cm2);
    static ChebyshevPolynomial<X> apply(Sub, ChebyshevPolynomial<X> const& cm1, ChebyshevPolynomial<X> const& cm2);
    static ChebyshevPolynomial<X> apply(Mul, ChebyshevPolynomial<X> const& cm1, ChebyshevPolynomial<X> const& cm2);
    static ChebyshevPolynomial<X> apply(Add, ChebyshevPolynomial<X> cm1, Scalar<X> const& s2);
    static ChebyshevPolynomial<X> apply(Mul, ChebyshevPolynomial<X> cm1, Scalar<X> const& s2);
    OutputStream& _write(OutputStream& os) const;

    static X _eval(ConstIterator& from, ConstIterator end, Vector<X> const& s, SizeType k);
    static ChebyshevPolynomial<X> _mul_from(ConstIterator& from1, ConstIterator end1, ConstIterator& from2, ConstIterator end2, SizeType k);
};

template<class X> struct AlgebraOperations<MultivariateChebyshevPolynomial<X>,X> : public MultivariateChebyshevPolynomial<X> { };

} // namespace Ariadne

#endif // ARIADNE_CHEBYSHEV_POLYNOMIAL_HPP
