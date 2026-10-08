/***************************************************************************
 *            algebra/series.hpp
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

/*! \file algebra/series.hpp
 *  \brief Taylor series in a single variable.
 */

#ifndef ARIADNE_SERIES_HPP
#define ARIADNE_SERIES_HPP

#include "utility/array.hpp"
#include "utility/container.hpp"
#include "numeric/operators.hpp"

namespace Ariadne {

template<class X> class Series;

//! \brief Algorithm for computing a power series.
template<class X> class SeriesGeneratorInterface {
  public:
    virtual ~SeriesGeneratorInterface() = default;
    //! \brief Compute the \a d-th term of a power series, given centering at \a x and previous terms \a s.
    virtual X _next(DegreeType d, X const& x, List<X>& s) const = 0;
};

template<class OP, class X> class SeriesGenerator : public SeriesGeneratorInterface<X> {
    virtual X _next(DegreeType d, X const& c, List<X>& y) const final {
        OP op; const X* yp=y.begin().operator->(); return next_series_coefficient(op, d, c, yp); }
};

template<class X> inline X next_series_coefficient(Pow, DegreeType d, X const& c, DegreeType n, X const* y) {
    return (d==0) ? pow(c,n) : (d>n) ? nul(c) : y[d-1]/c*(n-d+1)/d; }

template<class X> inline X next_series_coefficient(Pos, DegreeType d, X const& c, X const* y) {
    return (d==0) ? c : (d==1) ? nul(c)+1 : nul(c); }

template<class X> inline X next_series_coefficient(Nul, DegreeType d, X const& c, X const* y) {
    return nul(c); }

template<class X> inline X next_series_coefficient(Neg, DegreeType d, X const& c, X const* y) {
    return (d==0) ? -c : (d==1) ? nul(c)-1 : nul(c); }

template<class X> inline X next_series_coefficient(Hlf, DegreeType d, X const& c, X const* y) {
    return (d==0) ? hlf(c) : (d==1) ? hlf(nul(c)+1) : nul(c); }

template<class X> inline X next_series_coefficient(Sqr, DegreeType d, X const& c, X const* y) {
    switch(d) { case 0: return sqr(c); case 1: return 2*c; case 2: return nul(c)+1; default: return nul(c); } }

template<class X> inline X next_series_coefficient(Rec, DegreeType d, X const& c, X const* y) {
    return (d==0) ? rec(c) : y[d-1]*(-y[0]); }

template<class X> inline X next_series_coefficient(Sqrt, DegreeType d, X const& c, X const* y) {
    return (d==0) ? sqrt(c) : ((2*(Int)d-3)*(-1/c)/2)/d*y[d-1]; }

template<class X> inline X next_series_coefficient(Exp, DegreeType d, X const& c, X const* y) {
    return (d==0) ? exp(c) : y[d-1]/d; }

template<class X> inline X next_series_coefficient(Log, DegreeType d, X const& c, X const* y) {
    return (d==0) ? log(c) : (d==1) ? 1/c : -y[d-1]*y[1]*(d-1)/d; }

template<class X> inline X next_series_coefficient(Sin, DegreeType d, X const& c, X const* y) {
    return (d==0) ? sin(c) : (d==1) ? cos(c) : -y[d-2]/(d*(d-1)); }

template<class X> inline X next_series_coefficient(Cos, DegreeType d, X const& c, X const* y) {
    return (d==0) ? cos(c) : (d==1) ? -sin(c) : -y[d-2]/(d*(d-1)); }

template<class X> inline X next_series_coefficient(Tan, DegreeType d, X const& c, X const* y) {
    switch(d) { case 0: return tan(c); case 1: return 1+sqr(y[0]); case 2: return y[0]*y[1];
        default: { X r=y[0]*y[d-1]; for(DegreeType i=1; i!=d; ++i) { r+=y[i]*y[d-1-i]; } return r/d; } } }

template<class X> inline X next_series_coefficient(Asin, DegreeType d, X const& c, X const* y) {
    ARIADNE_NOT_IMPLEMENTED; }

template<class X> inline X next_series_coefficient(Acos, DegreeType d, X const& c, X const* y) {
    ARIADNE_NOT_IMPLEMENTED; }

template<class X> inline X next_series_coefficient(Atan, DegreeType d, X const& c, X const* y) {
    switch(d) { case 0: return atan(c); case 1: return rec(1+sqr(c)); case 2: return -c*sqr(y[1]);
        default: return -y[1]*((d-2)*y[d-2]+2*(d-1)*c*y[d-1])/d; } }

template<class X> inline X next_series_coefficient(Abs, DegreeType d, X const& c, X const* y) {
    ARIADNE_NOT_IMPLEMENTED; }


//! \brief A power series, centred on a value, with unlimited many coefficients.
template<class X>
class Series
{
    std::shared_ptr<const SeriesGeneratorInterface<X>> _ptr;
    X _centre;
    mutable List<X> _data;
  private:
    friend class AnalyticFunction;
    Series(std::shared_ptr<const SeriesGeneratorInterface<X>> p, X const& c, std::nullptr_t dummy);
    Void _compute(DegreeType n) const;
    OutputStream& _write(OutputStream& os) const;
  public:
    template<class OP> Series(OP op, X const& x);
    template<class OP, class XX> requires Constructible<X,XX> Series(OP op, XX const& x) : Series(op,X(x)) { }
    const X& operator[](DegreeType n) const;
    const List<X>& coefficients(DegreeType n) const;
    friend OutputStream& operator<<(OutputStream& os, Series<X> const& s) { return s._write(os); }
};

template<class X, DegreeType D> class FiniteSeries {
    X _data[D+1u];
  private:
    OutputStream& _write(OutputStream& os) const;
  public:
    template<class OP> FiniteSeries(OP op, X const& c) {
        for(DegreeType d=0; d<=D; ++d) { next_series_coefficient(op,d,c,_data); } }
    const X& operator[](DegreeType n) const { return _data[n]; }
    friend OutputStream& operator<<(OutputStream& os, FiniteSeries<X,D> const& s) { return s._write(os); }
};

template<class X> template<class OP> Series<X>::Series(OP op, X const& x)
    : Series(std::make_shared<SeriesGenerator<OP,X>>(),x, nullptr) {
}

template<class X> inline List<X> const& Series<X>::coefficients(DegreeType n) const {
    this->_compute(n); return this->_data;
}

template<class X> inline X const& Series<X>::operator[] (DegreeType n) const {
    this->_compute(n); return this->_data[n];
}


template<class X> inline Series<X>::Series(std::shared_ptr<const SeriesGeneratorInterface<X>> p, X const& c, std::nullptr_t)
    : _ptr(p), _centre(c), _data() {
}

template<class X> inline Void Series<X>::_compute(DegreeType n) const {
    while(_data.size()<=n) { _data.append(_ptr->_next(_data.size(),_centre,_data)); }
}

template<class X> inline OutputStream& Series<X>::_write(OutputStream& os) const {
    this->_compute(2); return os << "Series( centre=" << this->_centre << ", terms=" << this->_data << " )"; }

template<class OP, class X> inline Series<X> make_series(OP op, DegreeType d, const X& x) { return Series<X>(op,x); }
template<class OP, class X> inline Series<X> make_series(OP op, const X& x) { return Series<X>(op,x); }


class AnalyticFunction {
    typedef FloatDPApproximation ApproximateNumericType; // [[deprecated]]
    typedef FloatDPBounds ValidatedNumericType; // [[deprecated]]
    typedef FloatDP ExactNumericType; // [[deprecated]]
  private:
    UnaryTranscendentalOperator _op;
    std::shared_ptr<const SeriesGeneratorInterface<ApproximateNumericType>> _asg_ptr;
    std::shared_ptr<const SeriesGeneratorInterface<ValidatedNumericType>> _vsg_ptr;
  public:
    template<class OP> explicit AnalyticFunction(OP op)
        : _op(op), _asg_ptr(new SeriesGenerator<OP,ApproximateNumericType>()), _vsg_ptr(new SeriesGenerator<OP,ValidatedNumericType>()) { }
    Series<ApproximateNumericType> series(ApproximateNumericType c) const {
        return Series<ApproximateNumericType>(_asg_ptr,c,nullptr); }
    Series<ValidatedNumericType> series(ValidatedNumericType c) const {
        return Series<ValidatedNumericType>(_vsg_ptr,c,nullptr); }
    Series<ValidatedNumericType> series(ExactNumericType c) const {
        return Series<ValidatedNumericType>(_vsg_ptr,c,nullptr); }
    friend OutputStream& operator<<(OutputStream& os, AnalyticFunction const& af) {
        return os << "AnalyticFunction( " << af._op << " )"; }
};


// Code below is for missing functions
/*

template<class X> Series<X> Series<X>::asin(Nat d, const X& c) {
    if(d==0) { Series<X> y(d,c*0); y[0]=Ariadne::asin(c); return y; }
    Series<X> y(d-1,c*0); y[0]=c; y[1]=X(1);
    y = Ariadne::rec(Ariadne::sqrt(X(1)-Ariadne::sqr(y)));
    return antiderivative(y,Ariadne::asin(c));
}

template<class X> Series<X> Series<X>::acos(Nat d, const X& c) {
    if(d==0) { Series<X> y(d,c*0); y[0]=Ariadne::acos(c); return y; }
    Series<X> y(d-1,c*0); y[0]=c; y[1]=X(1);
    y = Ariadne::neg(Ariadne::rec(Ariadne::sqrt(X(1)-Ariadne::sqr(y))));
    return antiderivative(y,Ariadne::acos(c));
}

template<class X> Series<X> antiderivative(const Series<X>& x, const X& c) {
    Nat n=x.degree(); Series<X> r(n+1,x[0]*0);
    r[0]=c; for(Nat i=0; i<=n; ++i) { r[i+1]=x[i]/(i+1); }
    return r;
}

*/


} // namespace Ariadne


#endif // ARIADNE_SERIES_HPP

