/***************************************************************************
 *            algebra/covector.hpp
 *
 *  Copyright  2013-20  Pieter Collins
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

/*! \file algebra/covector.hpp
 *  \brief
 */



#ifndef ARIADNE_COVECTOR_HPP
#define ARIADNE_COVECTOR_HPP

#include "utility/metaprogramming.hpp"
#include "utility/container.hpp"
#include "algebra/vector.hpp"

namespace Ariadne {

template<class X> class Vector;
template<class X> class Covector;

struct DeclareCovectorOperations {
    template<class X> friend Covector<X> operator+(Covector<X> const& v);
    template<class X> friend Covector<NegationType<X>> operator-(Covector<X> const& v);
    template<class X1, class X2> friend Covector<SumType<X1,X2>> operator+(Covector<X1> const& v1, Covector<X2> const& v2);
    template<class X1, class X2> friend Covector<DifferenceType<X1,X2>> operator-(Covector<X1> const& v1, Covector<X2> const& v2);
    template<class X1, class X2> friend Covector<ProductType<Scalar<X1>,X2>> operator*(X1 const& x1, Covector<X2> const& v2);
    template<class X1, class X2> friend Covector<ProductType<X1,Scalar<X2>>> operator*(Covector<X1> const& v1, X2 const& x2);
    template<class X1, class X2> friend Covector<QuotientType<X1,Scalar<X2>>> operator/(Covector<X1> const& v1, X2 const& x2);
    template<class X1, class X2> friend Covector<InplaceSumType<X1,X2>>& operator+=(Covector<X1>& u1, const Covector<X2>& u2);
    template<class X1, class X2> friend Covector<InplaceDifferenceType<X1,X2>>& operator-=(Covector<X1>& u1, const Covector<X2>& u2);
    template<class X1, class X2> friend Covector<InplaceProductType<X1,X2>>& operator*=(Covector<X1>& u1, const X2& s2);
    template<class X1, class X2> friend Covector<InplaceQuotientType<X1,X2>>& operator/=(Covector<X1>& u1, const X2& s2);
    template<class X1, class X2> friend ArithmeticType<X1,X2> operator*(const Covector<X1>& u1, const Vector<X2>& v2);
    template<class X1, class X2> friend EqualsType<X1,X2> operator==(const Covector<X1>& u1, const Covector<X2>& u2);
    template<class X> friend OutputStream& operator<<(OutputStream& os, const Covector<X>& u);
};

template<class U> struct CovectorExpression : public DeclareCovectorOperations { const U& operator()() const { return static_cast<const U&>(*this); } };
template<class U> struct CovectorContainer : public CovectorExpression<U> { };


//! \ingroup LinearAlgebraSubModule
//! \brief A covector (dual to a vector, or a row vector) over some type \a X.
template<class X> class Covector
    : public CovectorContainer<Covector<X>>
{
  private: public:
    UniformArray<X> _ary;
  public:
    typedef X ScalarType;
    template<ConvertibleTo<X> XX>
        Covector(InitializerList<XX> const& lst) : _ary(lst) { }
    template<ExplicitlyConvertibleTo<X> XX>
        explicit Covector(InitializerList<XX> const& lst) : _ary(lst._ary) { }
    template<ConvertibleTo<X> XX>
        Covector(Covector<XX> const& u) : _ary(u._ary) { }
    template<ExplicitlyConvertibleTo<X> XX>
        explicit Covector(Covector<XX> const& u) : _ary(u._ary) { }
    template<class... PRS> requires Constructible<X,Nat,PRS...>
        explicit Covector(SizeType n, PRS... prs) : _ary(n,X(0u,prs...)) { }
    template<class XX, class... PRS> requires Constructible<X,XX,PRS...>
        explicit Covector(Covector<XX> const& u, PRS... prs) : _ary(u.array(),prs...) { }
    explicit Covector() : _ary() { }
    explicit Covector(SizeType n) : _ary(n) { }
    explicit Covector(SizeType n, const X& x) : _ary(n,x) { }
    explicit Covector(Array<X> ary) : _ary(std::move(ary)) { }
    template<class... PRS> requires Constructible<X,Nat,PRS...>
        static Covector<X> unit(SizeType n, SizeType j, PRS... prs) { Covector<X> r(n,prs...); r[j]=1; return r; }
    template<class G, class... PRS> requires InvocableReturning<X,G,SizeType> and Constructible<X,PRS...>
        explicit Covector(SizeType n, G const& g, PRS... prs) : _ary(n,g,prs...) { }
    template<class G, class... PRS> requires InvocableReturning<X,G,SizeType> and Constructible<X,PRS...>
        explicit Covector(SizeType n, G const& g, Tuple<PRS...> prs) : _ary(n,g,prs) { }

    SizeType size() const { return _ary.size(); }
    Void resize(SizeType n) { _ary.resize(n); }
    const X& operator[](SizeType j) const { return _ary[j]; }
    ElementReference<X> operator[](SizeType j) { return _ary[j]; }
    const X& at(SizeType j) const { ARIADNE_PRECONDITION_MSG(j<this->size(),*this<<"["<<j<<"]"); return _ary[j]; }
    ElementReference<X>& at(SizeType j) { ARIADNE_PRECONDITION_MSG(j<this->size(),*this<<"["<<j<<"]"); return _ary[j]; }
    X zero_element() const { ARIADNE_DEBUG_ASSERT(not _ary.empty()); return create_zero(_ary[0]); }
    UniformArray<X> const& array() const { return _ary; }

    template<class CVE> requires Convertible<typename CVE::ScalarType,X>
    Covector(CovectorExpression<CVE> const& cve) : _ary(cve().size(),cve().zero_element()) {
        for(SizeType i=0; i!=this->size(); ++i) { this->_ary[i]=cve()[i]; } }
    template<class CVE> requires Assignable<typename CVE::ScalarType,X>
    Covector<X> const& operator=(CovectorExpression<CVE> const& cve) {
        this->resize(cve.size()); for(SizeType i=0; i!=this->size(); ++i) { this->_ary[i]=cve()[i]; } }
};

template<class X> inline Covector<X> const& transpose(Vector<X> const& v) {
    return reinterpret_cast<Covector<X> const&>(v); }

template<class X> inline Vector<X> const& transpose(Covector<X> const& u) {
    return reinterpret_cast<Vector<X> const&>(u); }


template<class U> struct CovectorRange
    : public CovectorContainer< CovectorRange<U> >
{
    typedef typename U::ScalarType ScalarType;
    const U& _u; Range _rng;
    CovectorRange(const U& u, Range rng) : _u(u), _rng(rng) { }
    SizeType size() const { return _rng.size(); }
    ScalarType zero_element() const { return _u.zero_element(); }
    ScalarType operator[](SizeType i) const { return _u[i+_rng.start()]; }
};

template<class V> inline CovectorRange<V> project(const CovectorExpression<V>& v, Range rng) {
    return CovectorRange<V>(v(),rng); }

class ProvideCovectorOperations {

    template<class X> friend Covector<X> operator+(Covector<X> const& u) {
        Covector<X> r(u.size(),+u.zero_element());
        for(SizeType i=0; i!=r.size(); ++i) { r[i]=+u[i]; } return r; }

    template<class X> friend Covector<NegationType<X>> operator-(Covector<X> const& u) {
        Covector<NegationType<X>> r(u.size(),-u.zero_element());
        for(SizeType i=0; i!=r.size(); ++i) { r[i]=-u[i]; } return r; }

    template<class X1,class X2> friend Covector<SumType<X1,X2>> operator+(Covector<X1> const& u1, Covector<X2> const& u2) {
        ARIADNE_PRECONDITION(u1.size()==u2.size()); Covector<SumType<X1,X2>> r(u1.size(),u1.zero_element()+u2.zero_element());
        for(SizeType i=0; i!=r.size(); ++i) { r[i]=u1[i]+u2[i]; } return r; }

    template<class X1,class X2> friend Covector<DifferenceType<X1,X2>> operator-(Covector<X1> const& u1, Covector<X2> const& u2) {
        ARIADNE_PRECONDITION(u1.size()==u2.size()); Covector<DifferenceType<X1,X2>> r(u1.size(),u1.zero_element()-u2.zero_element());
        for(SizeType i=0; i!=r.size(); ++i) { r[i]=u1[i]-u2[i]; } return r; }

    template<class X1,class X2> friend Covector<ProductType<Scalar<X1>,X2>> operator*(X1 const& s1, Covector<X2> const& u2) {
        Covector<ProductType<X1,X2>> r(u2.size(),s1*u2.zero_element());
        for(SizeType i=0; i!=r.size(); ++i) { r[i]=s1*u2[i]; } return r; }

    template<class X1,class X2> friend Covector<ProductType<X1,Scalar<X2>>> operator*(Covector<X1> const& u1, X2 const& s2) {
        Covector<ProductType<X1,X2>> r(u1.size(),u1.zero_element()*s2);
        for(SizeType i=0; i!=r.size(); ++i) { r[i]=u1[i]*s2; } return r; }

    template<class X1,class X2> friend Covector<QuotientType<X1,Scalar<X2>>> operator/(Covector<X1> const& u1, X2 const& s2) {
        Covector<QuotientType<X1,X2>> r(u1.size(),u1.zero_element()*s2);
        for(SizeType i=0; i!=r.size(); ++i) { r[i]=u1[i]/s2; } return r; }

    template<class X1,class X2> friend Covector<InplaceSumType<X1,X2>>& operator+=(Covector<X1>& u1, Covector<X2> const& u2) {
        ARIADNE_PRECONDITION(u1.size()==u2.size()); for(SizeType i=0; i!=u1.size(); ++i) { u1[i]+=u2[i]; } return u1; }

    template<class X1,class X2> friend Covector<InplaceDifferenceType<X1,X2>>& operator-=(Covector<X1>& u1, Covector<X2> const& u2) {
        ARIADNE_PRECONDITION(u1.size()==u2.size()); for(SizeType i=0; i!=u1.size(); ++i) { u1[i]-=u2[i]; } return u1; }

    template<class X1,class X2> friend Covector<InplaceProductType<X1,X2>>& operator*=(Covector<X1>& u1, X2 const& s2) {
        for(SizeType i=0; i!=u1.size(); ++i) { u1[i]*=s2; } return u1; }

    template<class X1,class X2> friend Covector<InplaceQuotientType<X1,X2>>& operator/=(Covector<X1>& u1, X2 const& s2) {
        for(SizeType i=0; i!=u1.size(); ++i) { u1[i]/=s2; } return u1; }

    template<class X1, class X2> friend ArithmeticType<X1,X2> operator*(const Covector<X1>& u1, const Vector<X2>& v2) {
        ARIADNE_PRECONDITION(u1.size()==v2.size()); ArithmeticType<X1,X2> r=u1.zero_element()*v2.zero_element();
        for(SizeType i=0; i!=v2.size(); ++i) { r+=u1[i]*v2[i]; } return r; }

    template<class X1, class X2> friend EqualsType<X1,X2> operator==(const Covector<X1>& u1, const Covector<X2>& u2) {
        if(u1.size()!=u2.size()) { return false; } decltype(u1[0]==u2[0]) r=true;
        for(SizeType i=0; i!=u1.size(); ++i) { r = r && (u1[i]==u2[i]); } return r; }

    template<class X> friend OutputStream& operator<<(OutputStream& os, const Covector<X>& u) {
        if(u.size()==0) { os << "{"; } for(SizeType i=0; i!=u.size(); ++i) { os << (i==0u?"{":",") << u[i]; } return os << "}"; }

};



template<class X>
Covector<X> cojoin(const Covector<X>& u1, const Covector<X>& u2)
{
    Array<X> ra(u1.size()+u2.size(),Uninitialised());
    X* rp=ra.begin();
    for(X const* u1p=u1.array().begin(); u1p!=u1.array().end(); ++rp, ++u1p) { new (rp) X(*u1p); }
    for(X const* u2p=u2.array().begin(); u2p!=u2.array().end(); ++rp, ++u2p) { new (rp) X(*u2p); }
    assert(rp==ra.end());
    return Covector<X>(std::move(ra));
}

template<class X>
Covector<X> cojoin(const Covector<X>& u1, const typename Covector<X>::ScalarType& s2)
{
    Array<X> ra(u1.size()+1u,Uninitialised());
    X* rp=ra.begin();
    for(X const* u1p=u1.array().begin(); u1p!=u1.array().end(); ++rp, ++u1p) { new (rp) X(*u1p); }
    new (rp) X(s2);
    return Covector<X>(std::move(ra));
}

template<class X>
Covector<X> cojoin(const typename Covector<X>::ScalarType& s1, const Covector<X>& u2)
{
    Array<X> ra(1u+u2.size(),Uninitialised());
    X* rp=ra.begin();
    new (rp) X(s1); ++rp;
    for(X const* u2p=u2.array().begin(); u2p!=u2.array().end(); ++rp, ++u2p) { new (rp) X(*u2p); }
    return Covector<X>(std::move(ra));
}


} // namespace Ariadne

#endif
