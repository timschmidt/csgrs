/***************************************************************************
 *            univariate_differential.tpl.hpp
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

/*! \file univariate_differential.tpl.hpp
 *  \brief Differentials with respect to a single variable.
 */

#include "algebra/univariate_differential.hpp"

namespace Ariadne {

template<class X> UnivariateDifferential<X>::UnivariateDifferential(DegreeType d, X const& c)
    : _ary(d+1u,nul(c)) { _ary[0]=c; }

template<class X> UnivariateDifferential<X>::UnivariateDifferential(DegreeType d, InitializerList<X> lst)
    : _ary(d+1u,nul(*lst.begin()))
{
    ARIADNE_PRECONDITION(lst.size()==d+1u);
    std::copy(lst.begin(),lst.end(),_ary.begin());
}

template<class X> UnivariateDifferential<X>::UnivariateDifferential(DegreeType d, Series<X> const& s)
    : _ary(d+1u,nul(s[0])) { for(SizeType i=0; i<=d; ++i) { this->_ary[i]=s[i]; }
}

template<class X> UnivariateDifferential<X> UnivariateDifferential<X>::constant(DegreeType d, X const& c) {
    return UnivariateDifferential(d,c);
}

template<class X> UnivariateDifferential<X> UnivariateDifferential<X>::variable(DegreeType d, X const& c) {
    UnivariateDifferential r(d,c);
    if(d>=1) { r[1]=1; }
    return r;
}

template<class X> X UnivariateDifferential<X>::zero_coefficient() const {
    return nul(this->_ary[0]);
}

template<class X> DegreeType UnivariateDifferential<X>::degree() const {
    return this->_ary.size()-1u;
}

template<class X> Array<X> const& UnivariateDifferential<X>::array() const {
    return this->_ary;
}

template<class X> Array<X>& UnivariateDifferential<X>::array() {
    return this->_ary;
}

template<class X> X const& UnivariateDifferential<X>::operator[](SizeType k) const {
    return this->_ary[k];
}

template<class X> X& UnivariateDifferential<X>::operator[](SizeType k) {
    return this->_ary[k];
}

template<class X> UnivariateDifferential<X>& UnivariateDifferential<X>::operator+=(X const& c) {
    this->_ary[0]+=c;
    return *this;
}

template<class X> UnivariateDifferential<X>& UnivariateDifferential<X>::operator*=(X const& c) {
    for(DegreeType i=0; i<=this->degree(); ++i) {
        this->_ary[i]*=c;
    }
    return *this;
}


template<class X>
UnivariateDifferential<X> UnivariateDifferential<X>::_compose(const Series<X>& x, const UnivariateDifferential<X>& y)
{
    return compose(UnivariateDifferential(y.degree(),x),y);
}

template<class X>
UnivariateDifferential<X> UnivariateDifferential<X>::_compose(const UnivariateDifferential<X>& x, const UnivariateDifferential<X>& y)
{
    DegreeType d=std::min(x.degree(),y.degree());
    X y0 = y[0];
    X z = Ariadne::create_zero(y0);
    const_cast<X&>(y[0]) = z;
    UnivariateDifferential<X> r(d, z);
    r[0]=x[d];
    for(DegreeType n=1; n<=d; ++n) {
        r=r*y;
        r[0]+=x[static_cast<SizeType>(d-n)];
    }
    const_cast<X&>(y[0])=y0;
    return r;
}

template<class X>
UnivariateDifferential<X>
UnivariateDifferential<X>::_derivative(const UnivariateDifferential<X>& x)
{
    DegreeType n=x.degree(); DegreeType one=1u;
    UnivariateDifferential<X> r(std::min(n,one)-one,x.zero_coefficient());
    if(n==0) { r[0]=x[0]*0; }
    for(DegreeType i=0; i<n; ++i) {
        r[i]=(i+1u)*x[i+1u];
    }
    return r;
}

template<class X>
UnivariateDifferential<X>
UnivariateDifferential<X>::_antiderivative(const UnivariateDifferential<X>& x)
{
    DegreeType n=x.degree();
    UnivariateDifferential<X> r(n+1u,nul(x[0]));
    for(DegreeType i=0; i<=n; ++i) {
        r[i+1u]=x[i]/(i+1u);
    }
    return r;
}

template<class X>
UnivariateDifferential<X>
UnivariateDifferential<X>::_antiderivative(const UnivariateDifferential<X>& x, const X& c)
{
    DegreeType n=x.degree();
    UnivariateDifferential<X> r(n+1u,Ariadne::create_zero(x[0]));
    r[0]=c;
    for(DegreeType i=0; i<=n; ++i) {
        r[i+1u]=x[i]/(i+1u);
    }
    return r;
}



template<class X>
OutputStream& UnivariateDifferential<X>::_write(OutputStream& os) const
{
    UnivariateDifferential<X> const& x=*this;
    os << "D<"<<x.degree()<<">";
    for(DegreeType i=0; i<=x.degree(); ++i) {
        os << (i==0 ? '[' : ',') << x[i];
    }
    return os << "]";
}

/*
template<class X> OutputStream& UnivariateDifferential<X>::_write(OutputStream& os) const {
    os << this->_ary[0];
    for(DegreeType i=1; i<=this->degree(); ++i) {
        os <<" ";
        if(decide(this->_ary[i]>=X(0))) { os << "+"; }
        os << this->_ary[i] << "*dx^" << (uint)i;
    }
    return os;
}
*/

} // namespace Ariadne
