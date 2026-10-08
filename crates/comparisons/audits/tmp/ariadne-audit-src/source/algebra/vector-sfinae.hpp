/***************************************************************************
 *            algebra/vector-sfinae.hpp
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

/*! \file algebra/vector-sfinae.hpp
 *  \brief
 */



#ifndef ARIADNE_VECTOR_HPP
#define ARIADNE_VECTOR_HPP

#include "utility/module.hpp"
#include "utility/container.hpp"

namespace Ariadne {

/************ Vector *********************************************************/

template<class V> struct IsVector : False { };
template<class V> struct IsCovector : False { };
template<class M> struct IsMatrix : False { };

template<class S> struct IsScalar { static const Bool value=not (IsVector<S>::value or IsCovector<S>::value or IsMatrix<S>::value); };

template<class X> struct Vector {
    X _zero;
    Array<X> _ary;
  public:
    Vector() : _zero(), _ary() { }
    Vector(SizeType n) : _zero(), _ary(n) { }
    Vector(SizeType n, const X& x) : _zero(x*0), _ary(n,x) { }
    Vector(Array<X> ary) : _zero(*ary.begin()*0), _ary(std::move(ary)) { }
    Vector(InitializerList<X> lst) : _zero(*lst.begin()*0), _ary(lst.begin(),lst.end()) { }
    SizeType size() const { return _ary.size(); }
    const X& operator[](SizeType i) const { return _ary.at(i); }
    X& operator[](SizeType i) { return _ary.at(i); }
    X zero_element() const { return _zero; }
    OutputStream& _write(OutputStream& os) const;
};
template<class X> OutputStream& Vector<X>::_write(OutputStream& os) const {
    if(size()==0) { os << "{"; }
    for(SizeType i=0; i!=size(); ++i) { os << (i==0u?"{":";") << this->_ary[i]; }
    return os << "}";
}
template<class X> OutputStream& operator<<(OutputStream& os, const Vector<X>& v) {
    return v._write(os);
}
template<class X> struct IsVector<Vector<X>> : True { };


template<class V1, class V2> struct VectorSum {
    const V1& _v1; const V2& _v2;
    VectorSum(const V1& v1, const V2& v2) : _v1(v1), _v2(v2) { }
    SizeType size() const { return _v1.size(); }
    auto operator[](SizeType i) const -> decltype(_v1[i]+_v2[i]) { return _v1[i]+_v2[i]; }
};
template<class V1, class V2> struct IsVector<VectorSum<V1,V2>> : True { };

template<AVector V1, AVector V2>
inline VectorSum<V1,V2> operator+(const V1& v1, const V2& v2) { return VectorSum<V1,V2>(v1,v2); }

template<class X1, class V2> struct ScalarVectorProduct {
    const X1& _x1; const V2& _v2;
    ScalarVectorProduct(const X1& x1, const V2& v2) : _x1(x1), _v2(v2) { }
    SizeType size() const { return _v2.size(); }
    auto operator[](SizeType i) const -> decltype(_x1*_v2[i]) { return _x1*_v2[i]; }
};
template<class X1, class V2> struct IsVector<ScalarVectorProduct<X1,V2>> : True { };

template<AScaler X1, AVector V2>
inline ScalarVectorProduct<X1,V2> operator*(const X1& x1, const V2& v2) { return ScalarVectorProduct<X1,V2>(x1,v2); }


template<class X1, class X2> auto operator==(const Vector<X1>& v1, const Vector<X2>& v2)
    -> decltype(v1[0]==v2[0])
{
    if(v1.size()!=v2.size()) { return false; }
    decltype(v1[0]==v2[0]) r=true;
    for(SizeType i=0; i!=v1.size(); ++i) {
        r = r && (v1[i]==v2[i]);
    }
    return r;
}


template<class X> struct Covector {
    X _zero;
    Array<X> _ary;
  public:
    Covector(SizeType n) : _zero(), _ary(n) { }
    Covector(SizeType n, const X& x) : _zero(x*0), _ary(n,x) { }
    SizeType size() const { return _ary.size(); }
    const X& operator[](SizeType i) const { return _ary.at(i); }
    X& operator[](SizeType i) { return _ary.at(i); }
    X zero_element() const { return _zero; }
    OutputStream& _write(OutputStream& os) const;
};
template<class X> OutputStream& Covector<X>::_write(OutputStream& os) const {
    if(size()==0) { os << "{"; }
    for(SizeType i=0; i!=size(); ++i) { os << (i==0u?"{":",") << this->_ary[i]; }
    return os << "}";
}
template<class X> OutputStream& operator<<(OutputStream& os, const Covector<X>& v) {
    return v._write(os);
}
template<class X> struct IsCovector<Covector<X>> : True { };



} // namespace Ariadne

#endif
