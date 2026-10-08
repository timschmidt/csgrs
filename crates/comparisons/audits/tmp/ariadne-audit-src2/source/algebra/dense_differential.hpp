/***************************************************************************
 *            algebra/dense_differential.hpp
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

/*! \file algebra/dense_differential.hpp
 *  \brief Differential algebra variables with a dense representation.
 */
#ifndef ARIADNE_DENSE_DIFFERENTIAL_HPP
#define ARIADNE_DENSE_DIFFERENTIAL_HPP

#include <cmath>
#include <limits>

#include "utility/macros.hpp"
#include "utility/array.hpp"
#include "numeric/numeric.hpp"
#include "algebra/vector.hpp"
#include "algebra/matrix.hpp"
#include "algebra/series.hpp"
#include "algebra/multi_index.hpp"

namespace Ariadne {

class MultiIndex;
template<class X> class Array;
template<class X> class Vector;
template<class X> class Matrix;
template<class X> class Series;
template<class X> class DenseDifferential;


//! The partial derivatives of a variable with respect to other variables.
template<class X>
class DenseDifferential
{
  public:
    typedef X ScalarType;
    typedef X ValueType;
    typedef MultiIndex IndexType;

    /// Default constructor constructs a constant of degree zero.
    DenseDifferential();
    /// The constant zero of degree \a d in \a a arguments.
    DenseDifferential(SizeType a, DegreeType d);
    /// A taylor variable of degree \a d in \a arguments, with values given by the Array based at \a ptr.
    template<class XX> DenseDifferential(SizeType a, DegreeType d, const XX* ptr);
    /// A taylor variable of degree \a d in \a arguments, with values given by the Array based at \a ptr.
    template<class XX> DenseDifferential(const DenseDifferential<XX>& x);

    /// Assign from a constant.
    DenseDifferential<X>& operator=(const X& c);

    /// Equality operator.
    Bool operator==(const DenseDifferential<X>& other) const;
    /// Inequality operator.
    Bool operator!=(const DenseDifferential<X>& other) const;

    /// The number of variables of the argument.
    SizeType argument_size() const;
    /// The degree (number of derivatives computed).
    DegreeType degree() const;
    /// The value of the quantity.
    const X& value() const;
    /// A reference to the value of the quantity.
    X& value();
    /// Set the value of the quantity.
    Void set_value(const X&);
    /// The variation of the quantity with respect to the \a j<sup>th</sup> argument.
    const X& gradient(SizeType j) const;
    Void set_gradient(SizeType j, const X&);
    /// The Array of derivative values.
    const Vector<X>& data() const;
    /// A reference to the Array of derivative values.
    Vector<X>& data();
    /// A reference to the \a i<sup>th</sup> derivative \f$D^af=d^{|a|}f/dx_1^{a_1}\cdots dx_n^{a_n}\f$.
    X& operator[](const MultiIndex& a);
    X& operator[](const SizeType& j);
    /// The \a i<sup>th</sup> derivative \f$D^af=d^{|a|}f/dx_1^{a_1}\cdots dx_n^{a_n}\f$.
    const X& operator[](const MultiIndex& a) const;
    const X& operator[](const SizeType& j) const;

    /// Assign all elements of degree less than the degree of \a x to those of \a x .
    DenseDifferential<X>& assign(const DenseDifferential<X>& x);

    /// Remove any unnecessary entries. (Only needed for compatibility with other Differential classes.)
    Void cleanup();

    /// Add another variable.
    DenseDifferential<X>& operator+=(const DenseDifferential<X>& x);
    /// Subtract another variable.
    DenseDifferential<X>& operator-=(const DenseDifferential<X>& x);
    /// Add a constant.
    DenseDifferential<X>& operator+=(const X& c);
    /// Subtract a constant.
    DenseDifferential<X>& operator-=(const X& c);
    /// Multiply by a constant.
    DenseDifferential<X>& operator*=(const X& c);
    /// Divide by a constant.
    DenseDifferential<X>& operator/=(const X& c);

    /// A constant differential in \a as variables at degree \a d with value \a c.
    static DenseDifferential<X> constant(SizeType as, DegreeType d, const X& c);
    /// The \a i<sup>th</sup> variable of \a as at degree \a d with value \a x.
    static DenseDifferential<X> variable(SizeType as, DegreeType d, const X& x, SizeType i);
    /// A vector of constant differentials of size \a rs in \a as variables at degree \a d and value \a c.
    static Vector< DenseDifferential<X> > constants(SizeType rs, SizeType as, DegreeType d, const Vector<X>& c);
    /// A vector of differential variables of size \a rs in \a as variables (with \a rs equal to \a as) at degree \a d and value \a x.
    static Vector< DenseDifferential<X> > variables(SizeType rs, SizeType as, DegreeType d, const Vector<X>& x);

  public:
#ifdef DOXYGEN
    ///
    friend Bool operator<(const DenseDifferential<X>& x1, const DenseDifferential<X>& x2);

    ///
    friend DenseDifferential<X> operator+(const DenseDifferential<X>& x);
    ///
    friend DenseDifferential<X> operator-(const DenseDifferential<X>& x);
    ///
    friend DenseDifferential<X> operator+(const DenseDifferential<X>& x, const DenseDifferential<X>& y);
    ///
    friend DenseDifferential<X> operator-(const DenseDifferential<X>& x, const DenseDifferential<X>& y);
    ///
    friend DenseDifferential<X> operator*(const DenseDifferential<X>& x, const DenseDifferential<X>& y);
    ///
    friend DenseDifferential<X> operator/(const DenseDifferential<X>& x, const DenseDifferential<X>& y);
    ///
    friend DenseDifferential<X> min(const DenseDifferential<X>& x1, const DenseDifferential<X>& x2);
    ///
    friend DenseDifferential<X> max(const DenseDifferential<X>& x1, const DenseDifferential<X>& x2);
    ///
    friend DenseDifferential<X> abs(const DenseDifferential<X>& x);

    /// Reciprocal
    friend DenseDifferential<X> rec(const DenseDifferential<X>& x);
    /// Power
    friend DenseDifferential<X> pow(const DenseDifferential<X>& x, Int k);
    /// Square root
    friend DenseDifferential<X> sqrt(const DenseDifferential<X>& x);
    /// Exponential
    friend DenseDifferential<X> exp(const DenseDifferential<X>& x);
    /// Natural logarithm
    friend DenseDifferential<X> log(const DenseDifferential<X>& x);
#endif
  public:
    SizeType _argument_size;
    DegreeType _degree;
    Vector<X> _data;
};

template<class X> DenseDifferential<X> scalar_constant(SizeType as, DegreeType d, const X& c);
template<class X> DenseDifferential<X> scalar_variable(SizeType as, DegreeType d, const X& x, SizeType i);



template<class X, class Y> Y evaluate(const DenseDifferential<X>& y, const Vector<Y>& z);
template<class X> DenseDifferential<X> compose(const Series<X>& y, const DenseDifferential<X>& x);
template<class X> DenseDifferential<X> derivative(const DenseDifferential<X>& x, SizeType i);
template<class X> DenseDifferential<X> antiderivative(const DenseDifferential<X>& x, SizeType i);

template<class X> Bool operator<(const DenseDifferential<X>& x1, const DenseDifferential<X>& x2);
template<class X> DenseDifferential<X> operator+(const DenseDifferential<X>& x);
template<class X> DenseDifferential<X> operator-(const DenseDifferential<X>& x);
template<class X> DenseDifferential<X> operator+(const DenseDifferential<X>& x, const DenseDifferential<X>& y);
template<class X> DenseDifferential<X> operator-(const DenseDifferential<X>& x, const DenseDifferential<X>& y);
template<class X> DenseDifferential<X> operator*(const DenseDifferential<X>& x, const DenseDifferential<X>& y);
template<class X> DenseDifferential<X> operator/(const DenseDifferential<X>& x, const DenseDifferential<X>& y);

template<class X> DenseDifferential<X> min(const DenseDifferential<X>& x1, const DenseDifferential<X>& x2);
template<class X> DenseDifferential<X> max(const DenseDifferential<X>& x1, const DenseDifferential<X>& x2);
template<class X> DenseDifferential<X> abs(const DenseDifferential<X>& x);

template<class X> DenseDifferential<X> rec(const DenseDifferential<X>& x);
template<class X> DenseDifferential<X> pow(const DenseDifferential<X>& x, Int k);
template<class X> DenseDifferential<X> sqrt(const DenseDifferential<X>& x);
template<class X> DenseDifferential<X> exp(const DenseDifferential<X>& x);
template<class X> DenseDifferential<X> log(const DenseDifferential<X>& x);

template<class X, class XX> DenseDifferential<X> operator+(const XX& c, const DenseDifferential<X>& x) {
    return DenseDifferential<X>(x)+=X(c); }
template<class X, class XX> DenseDifferential<X> operator+(const DenseDifferential<X>& x, const XX& c) {
    return DenseDifferential<X>(x)+=X(c); }
template<class X, class XX> DenseDifferential<X> operator-(const XX& c, const DenseDifferential<X>& x) {
    return DenseDifferential<X>(-x)+=X(c); }
template<class X, class XX> DenseDifferential<X> operator-(const DenseDifferential<X>& x, const XX& c) {
    return DenseDifferential<X>(x)-=X(c); }
template<class X, class XX> DenseDifferential<X> operator*(const DenseDifferential<X>& x, const XX& c) {
    return DenseDifferential<X>(x)*=c; }
template<class X, class XX> DenseDifferential<X> operator*(const XX& c, const DenseDifferential<X>& x) {
    return DenseDifferential<X>(x)*=X(c); }
template<class X, class XX> DenseDifferential<X> operator/(const DenseDifferential<X>& x, const XX& c) {
    return DenseDifferential<X>(x)*=(1.0/c); }

template<class X> OutputStream& operator<<(OutputStream& os, const DenseDifferential<X>& x);

//template<class X> DenseDifferential<X>& acc(DenseDifferential<X>& r, const DenseDifferential<X>& x, const DenseDifferential<X>& y);
//template<class X> DenseDifferential<X>& acc(DenseDifferential<X>& r, const X& c, const DenseDifferential<X>& x);




inline SizeType compute_polynomial_data_size(SizeType rs, SizeType as, DegreeType d) { return rs*Ariadne::bin(d+as,as); }

template<class X> template<class XX>
DenseDifferential<X>::DenseDifferential(SizeType a, DegreeType d, const XX* ptr)
    : _argument_size(a), _degree(d), _data(compute_polynomial_data_size(1,a,d))
{
    for(SizeType i=0; i!=this->_data.size(); ++i) {
        this->_data[i]=ptr[i];
    }
}

template<class X> template<class XX>
DenseDifferential<X>::DenseDifferential(const DenseDifferential<XX>& x)
    : _argument_size(x.argument_size()), _degree(x.degree()), _data(x.data())
{
}




template<class X>
DenseDifferential<X>&
DenseDifferential<X>::operator=(const X& c)
{
    this->_data[0]=c;
    for(SizeType i=1; i!=this->_data.size(); ++i) {
        this->_data[i]=0;
    }
    return *this;
}


template<class X>
Bool
DenseDifferential<X>::operator==(const DenseDifferential<X>& other) const
{
    return this->_argument_size==other._argument_size
        && this->_degree==other._degree
        && this->_data==other._data;
}



template<class X>
Bool
DenseDifferential<X>::operator!=(const DenseDifferential<X>& other) const
{
    return !(*this==other);
}



template<class X>
X
evaluate(const DenseDifferential<X>& y, const Vector<X>& x)
{
    //std::cerr<<ARIADNE_PRETTY_FUNCTION<<std::endl;
    ARIADNE_ASSERT(y.argument_size()==x.size());
    //std::cerr << "y=" << y << std::endl;
    //std::cerr << "x=" << x << std::endl;
    DegreeType d=y.degree();
    SizeType ms=x.size();
    ARIADNE_ASSERT(d>=1);

    X zero = x.zero_element();
    X one = zero; one+=1;

    // Use inefficient brute-force approach with lots of storage...
    Array< Array< X > > val(ms, Array< X >(d+1));
    for(SizeType j=0; j!=ms; ++j) {
        val[j][0]=one;
        val[j][1]=x[j];
        for(SizeType k=2; k<=d; ++k) {
            val[j][k]=val[j][k-1]*x[j];
        }
    }

    X r(zero);
    for(MultiIndex j(ms); j.degree()<=d; ++j) {
        X t=one;
        for(SizeType k=0; k!=ms; ++k) {
            t=t*val[k][j[k]];
        }
        t*=y[j];
        r+=t;
    }
    return r;
}


template<class X, class Y>
Y
evaluate(const DenseDifferential<X>& y, const Vector<Y>& x)
{
    //std::cerr<<ARIADNE_PRETTY_FUNCTION<<std::endl;
    ARIADNE_ASSERT(y.argument_size()==x.size());
    //std::cerr << "y=" << y << std::endl;
    //std::cerr << "x=" << x << std::endl;
    DegreeType d=y.degree();
    SizeType ms=x.size();
    ARIADNE_ASSERT(d>=1);

    Y zero = x.zero_element();
    Y one = zero; one+=1;

    // Use inefficient brute-force approach with lots of storage...
    Array< Array< Y > > val(ms, Array< Y >(d+1));
    for(SizeType j=0; j!=ms; ++j) {
        val[j][0]=one;
        val[j][1]=x[j];
        for(SizeType k=2; k<=d; ++k) {
            val[j][k]=val[j][k-1]*x[j];
        }
    }

    Y r(zero);
    for(MultiIndex j(ms); j.degree()<=d; ++j) {
        Y sf=fac(j);
        Y t=one;
        for(SizeType k=0; k!=ms; ++k) {
            t=t*val[k][j[k]];
        }
        t*=y[j];
        r+=t;
    }
    return r;
}





template<class X>
DenseDifferential<X>
min(const DenseDifferential<X>& x1, const DenseDifferential<X>& x2)
{
    if(x1.value()==x2.value()) {
        ARIADNE_THROW(std::runtime_error,"min(DenseDifferential<X> x1, DenseDifferential<X> x2)","x1[0]==x2[0]");
    }
    return x1.value()<x2.value() ? x1 : x2;
}


template<class X>
DenseDifferential<X>
max(const DenseDifferential<X>& x1,const DenseDifferential<X>& x2)
{
    if(x1.value()==x2.value()) {
        ARIADNE_THROW(std::runtime_error,"max(DenseDifferential<X> x1, DenseDifferential<X> x2)","x1[0]==x2[0]");
    }
    return x1.value()>x2.value() ? x1 : x2;
}


template<class X>
DenseDifferential<X>
pos(const DenseDifferential<X>& x)
{
    return x;
}


template<class X>
DenseDifferential<X>
neg(const DenseDifferential<X>& x)
{
    DenseDifferential<X> y(x.argument_size(),x.degree());
    for(SizeType n=0; n<y.data().size(); ++n) {
        y.data()[n] = -x.data()[n];
    }
    return y;
}


template<class X>
DenseDifferential<X>
abs(const DenseDifferential<X>& x)
{
    if(x.value()==0) {
        ARIADNE_THROW(std::runtime_error,"abs(DenseDifferential<X> x)","x[0]==0");
    }
    return x.value()>0 ? pos(x) : neg(x);
}


template<class X>
DenseDifferential<X>
add(const DenseDifferential<X>& x, const DenseDifferential<X>& y)
{
    assert(x.argument_size()==y.argument_size());
    DenseDifferential<X> z(x.argument_size(),std::min(x.degree(),y.degree()));
    for(SizeType n=0; n<z.data().size(); ++n) {
        z.data()[n] = x.data()[n]+y.data()[n];
    }
    return z;
}


template<class X>
DenseDifferential<X>
sub(const DenseDifferential<X>& x, const DenseDifferential<X>& y)
{
    assert(x.argument_size()==y.argument_size());
    DenseDifferential<X> z(x.argument_size(),std::min(x.degree(),y.degree()));
    for(SizeType n=0; n<z.data().size(); ++n) {
        z.data()[n] = x.data()[n]-y.data()[n];
    }
    return z;
}


template<class X>
DenseDifferential<X>
mul(const DenseDifferential<X>& x, const DenseDifferential<X>& y)
{
    assert(x.argument_size()==y.argument_size());
    DenseDifferential<X> z(x.argument_size(),std::min(x.degree(),y.degree()));
    acc(z,x,y);
    return z;
}


template<class X>
DenseDifferential<X>
div(const DenseDifferential<X>& x, const DenseDifferential<X>& y)
{
    return mul(x,rec(y));
}

template<class X>
DenseDifferential<X>
pow(const DenseDifferential<X>& x, Int k)
{
    return compose(Series<X>::pow(x.degree(),x.value(),k),x);
}





template<class X>
DenseDifferential<X>
rec(const DenseDifferential<X>& x)
{
    return compose(Series<X>::rec(x.degree(),x.value()),x);
}


template<class X>
DenseDifferential<X>
sqrt(const DenseDifferential<X>& x)
{
    return compose(Series<X>::sqrt(x.degree(),x.value()),x);
}


template<class X>
DenseDifferential<X>
exp(const DenseDifferential<X>& x)
{
    return compose(Series<X>::exp(x.degree(),x.value()),x);
}


template<class X>
DenseDifferential<X>
log(const DenseDifferential<X>& x)
{
    return compose(Series<X>::log(x.degree(),x.value()),x);
}


template<class X>
DenseDifferential<X>
operator+(const DenseDifferential<X>& x)
{
    return pos(x);
}


template<class X>
DenseDifferential<X>
operator-(const DenseDifferential<X>& x)
{
    return neg(x);
}


template<class X>
DenseDifferential<X>
operator+(const DenseDifferential<X>& x, const DenseDifferential<X>& y)
{
    return add(x,y);
}


template<class X>
DenseDifferential<X>
operator-(const DenseDifferential<X>& x, const DenseDifferential<X>& y)
{
    return sub(x,y);
}


template<class X>
DenseDifferential<X>
operator*(const DenseDifferential<X>& x, const DenseDifferential<X>& y)
{
    return mul(x,y);
}


template<class X>
DenseDifferential<X>
operator/(const DenseDifferential<X>& x, const DenseDifferential<X>& y)
{
    return div(x,y);
}







template<class X>
DenseDifferential<X>&
DenseDifferential<X>::operator+=(const DenseDifferential<X>& x)
{
    this->_data+=x._data;
    return *this;
}

template<class X>
DenseDifferential<X>&
DenseDifferential<X>::operator-=(const DenseDifferential<X>& x)
{
    this->_data-=x._data;
    return *this;
}


template<class X>
DenseDifferential<X>&
DenseDifferential<X>::operator+=(const X& c)
{
    this->_data[0]+=c;
    return *this;
}

template<class X>
DenseDifferential<X>&
DenseDifferential<X>::operator-=(const X& c)
{
    this->_data[0]-=c;
    return *this;
}

template<class X>
DenseDifferential<X>&
DenseDifferential<X>::operator*=(const X& c)
{
    this->_data*=c;
    return *this;
}

template<class X>
DenseDifferential<X>&
DenseDifferential<X>::operator/=(const X& c)
{
    this->_data/=c;
    return *this;
}

template<class X>
DenseDifferential<X>::DenseDifferential()
    : _argument_size(1), _degree(0), _data(1u,X(0))
{
}


template<class X>
DenseDifferential<X>::DenseDifferential(SizeType a, DegreeType d)
    : _argument_size(a), _degree(d), _data(compute_polynomial_data_size(1,a,d),X(0))
{
}


template<class X>
DenseDifferential<X>
DenseDifferential<X>::constant(SizeType as, DegreeType d, const X& c)
{
    DenseDifferential<X> r(as,d); r._data[0]=c; return r;
}


template<class X>
DenseDifferential<X>
DenseDifferential<X>::variable(SizeType as, DegreeType d, const X& x, SizeType i)
{
    DenseDifferential<X> r(as,d); r._data[0]=x; r._data[1+i]=1; return r;
}

template<class X>
Vector< DenseDifferential<X> >
DenseDifferential<X>::constants(SizeType rs, SizeType as, DegreeType d, const Vector<X>& c)
{
    ARIADNE_ASSERT(c.size()==rs);
    Vector< DenseDifferential<X> > result(rs,DenseDifferential(as,d));
    for(SizeType i=0; i!=rs; ++i) { result[i]=c[i]; }
    return result;
}

template<class X>
Vector< DenseDifferential<X> >
DenseDifferential<X>::variables(SizeType rs, SizeType as, DegreeType d, const Vector<X>& x)
{
    ARIADNE_ASSERT(rs==x.size()); ARIADNE_ASSERT(as==x.size());
    Vector< DenseDifferential<X> > r(rs,DenseDifferential(as,d));
    for(SizeType i=0; i!=rs; ++i) { r[i]=x[i]; r[i][i]=X(1.0); }
    return r;
}


template<class X>
SizeType
DenseDifferential<X>::argument_size() const
{
    return this->_argument_size;
}


template<class X>
DegreeType
DenseDifferential<X>::degree() const
{
    return this->_degree;
}


template<class X>
const X&
DenseDifferential<X>::value() const
{
    return this->_data[0];
}


template<class X>
X&
DenseDifferential<X>::value()
{
    return this->_data[0];
}

template<class X>
Void
DenseDifferential<X>::set_value(const X& x)
{
    this->_data[0]=x;
}

template<class X>
const X&
DenseDifferential<X>::gradient(SizeType j) const
{
    return this->_data[j+1u];
}

template<class X>
Void
DenseDifferential<X>::set_gradient(SizeType j, const X& x)
{
    this->_data[j+1u]=x;
}


template<class X>
Vector<X>&
DenseDifferential<X>::data()
{
    return this->_data;
}


template<class X>
const Vector<X>&
DenseDifferential<X>::data() const
{
    return this->_data;
}



template<class X>
X&
DenseDifferential<X>::operator[](const MultiIndex& a)
{
    return this->_data[a.position()];
}


template<class X>
const X&
DenseDifferential<X>::operator[](const MultiIndex& a) const
{
    return this->_data[a.position()];
}

template<class X>
X&
DenseDifferential<X>::operator[](const SizeType& i)
{
    ARIADNE_ASSERT(i<this->argument_size());
    return this->_data[i+1];
}

template<class X>
const X&
DenseDifferential<X>::operator[](const SizeType& i) const
{
    ARIADNE_ASSERT(i<this->argument_size());
    return this->_data[i+1];
}


template<class X>
Void
DenseDifferential<X>::cleanup()
{
}


template<class X>
Bool
operator<(const DenseDifferential<X>& x1, const DenseDifferential<X>& x2)
{
    return x1.value() < x2.value();
}


template<class X>
DenseDifferential<X>&
acc(DenseDifferential<X>& r, const DenseDifferential<X>& x1, const DenseDifferential<X>& x2)
{
    ARIADNE_ASSERT(r.argument_size()==x1.argument_size());
    ARIADNE_ASSERT(r.argument_size()==x2.argument_size());
    for(MultiIndex i1(x1.argument_size()); i1.degree() <= std::min(r.degree(),x1.degree()); ++i1) {
        for(MultiIndex i2(x2.argument_size()); i2.degree() <= std::min(x2.degree(),DegreeType(r.degree()-i1.degree())); ++i2) {
            MultiIndex i0=i1+i2;
            r[i0]+=x1[i1]*x2[i2];
        }
    }
    return r;
}


template<class X>
DenseDifferential<X>&
acc(DenseDifferential<X>& r, const X& c, const DenseDifferential<X>& x)
{
    ARIADNE_ASSERT(r.argument_size()==x.argument_size());
    SizeType n=std::max(r.data().size(),x.data().size());
    for(SizeType i=0; i!=n; ++i) {
        r.data()[i]+=c*x.data()[i];
    }
    return r;
}



template<class X>
DenseDifferential<X>&
DenseDifferential<X>::assign(const DenseDifferential<X>& x)
{
    ARIADNE_ASSERT(this->argument_size()==x.argument_size());
    ARIADNE_ASSERT(this->degree()>=x.degree());
    for(SizeType i=0; i!=x.data().size(); ++i) {
        this->_data[i]=x.data()[i];
    }
    return *this;
}



template<class X>
DenseDifferential<X>&
operator+=(DenseDifferential<X>& r, const DenseDifferential<X>& x)
{
    ARIADNE_ASSERT(r.argument_size()==x.argument_size());
    ARIADNE_ASSERT(r.degree()==x.degree());
    for(SizeType i=0; i!=r.data().size(); ++i) {
        r.data()[i]+=x.data()[i];
    }
    //reinterpret_cast<LinearAlgebra::Vector&>(r._data)
    //  += reinterpret_cast<LinearAlgebra::Vectorconst&>(x._data);
    return r;
}


template<class X>
Void
compute_composition(DenseDifferential<X>& z,
                    const Series<X>& y,
                    const DenseDifferential<X>& x)
{
    SizeType as=x.argument_size();
    DegreeType d=z.degree();

    DenseDifferential<X> w=x;
    w.value()=0;
    DenseDifferential<X> t(as,d);
    t.value()=y[d];
    for(SizeType n=1; n<=d; ++n) {
        DenseDifferential<X> u(as,d);
        acc(u,t,w);
        t=u; t+=y[d-n];
    }
    z=t;
    return;
}


template<class X>
DenseDifferential<X>
compose(const Series<X>& y, const DenseDifferential<X>& x)
{
    DenseDifferential<X> z(x.argument_size(),std::min(x.degree(),y.degree()));
    compute_composition(z,y,x);
    return z;
}


template<class X>
DenseDifferential<X>
reduce(const DenseDifferential<X>& x, const DegreeType& d)
{
    assert(x.degree()>=d);
    DenseDifferential<X> r(x.argument_size(),d);
    for(MultiIndex i(x.argument_size()); i.degree() <= x.degree(); ++i) {
        r[i]=x[i];
    }
    return r;
}


template<class X>
DenseDifferential<X> scalar_constant(SizeType as, DegreeType d,const X& x)
{
    return DenseDifferential<X>::constant(as,d,x);
}

template<class X>
DenseDifferential<X> scalar_variable(SizeType as, DegreeType d,const X& x, SizeType i)
{
    return DenseDifferential<X>::variable(as,d,x,i);
}



template<class X>
DenseDifferential<X> derivative(const DenseDifferential<X>& x, SizeType i)
{
    if(x.degree()==0) { return DenseDifferential<X>(x.argument_size(),0u); }

    DenseDifferential<X> r(x.argument_size(), x.degree()-1);
    DegreeType d=r.degree();
    MultiIndex da=MultiIndex::zero(x.argument_size());
    MultiIndex ai=MultiIndex::unit(x.argument_size(),i);
    MultiIndex a=MultiIndex::zero(x.argument_size());
    while(a.degree()<=d) {
        da=a+ai;
        const X& xj=x[da];
        DegreeType dai=da[i]; r[a]=xj*dai;
        ++a;
    }
    return r;
}

template<class X>
DenseDifferential<X> antiderivative(const DenseDifferential<X>& x, SizeType i)
{
    DenseDifferential<X> r(x.argument_size(), x.degree()+1);
    DegreeType d=x.degree();
    MultiIndex da=MultiIndex::zero(x.argument_size());
    MultiIndex ai=MultiIndex::unit(x.argument_size(),i);
    MultiIndex a=MultiIndex::zero(x.argument_size());
    while(a.degree()<=d) {
        const X& xj=x[a];
        da=a+ai;
        DegreeType dai=da[i]; r[da]=xj/dai;
        ++a;
    }
    return r;
}





template<class X>
OutputStream&
operator<<(OutputStream& os, const DenseDifferential<X>& x) {
    //  return os << "DenseDifferential<X>( argument_size=" << x.argument_size() << ", degree=" << x.degree() << ", data=" << x.data() << ")";
    //os << "DenseDifferential<X>(";
    os << "D("<<x.argument_size()<<","<<x.degree()<<")";
    DegreeType degree=0;
    for(MultiIndex i(x.argument_size()); i.degree()<=x.degree(); ++i) {
        if(i.degree()==0) {
            os << '[';
        } else if(i.degree()==degree) {
            os << ',';
        } else {
            degree=i.degree();
            os << ';';
        }
        os << x[i];
    }
    os << ']';
    //os << ")";
    return os;

}










} // namespace Ariadne




#endif // ARIADNE_DENSE_DIFFERENTIAL_HPP

