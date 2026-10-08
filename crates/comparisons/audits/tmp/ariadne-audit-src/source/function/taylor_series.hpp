/***************************************************************************
 *            function/taylor_series.hpp
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

/*! \file function/taylor_series.hpp
 *  \brief ApproximateTag power series in one variable on a is_bounded domain with a dense
 *  representation as a floating-point polynomial plus an error bound.
 */

#ifndef ARIADNE_TAYLOR_SERIES_HPP
#define ARIADNE_TAYLOR_SERIES_HPP

#include "numeric/numeric.hpp"
#include "algebra/series.hpp"
#include "function/domain.hpp"
#include "utility/container.hpp"

namespace Ariadne {

class AnalyticFunction;

// A univariate Taylor function for computing power series expansions
// The difference between this and the Series class is that a TaylorSeries
// uses floating-point coefficients and an interval remainder, and is only
// valid on a subdomain, while a Series class uses arbitrary coefficients
// and is valid within the entire radius of convergence
template<class X> class TaylorSeries;

template<> class TaylorSeries<FloatDPBounds> {
    IntervalDomainType _domain;
    Array<FloatDP> _expansion;
    FloatDPError _error;
  public:
    TaylorSeries(const IntervalDomainType& dom, DegreeType deg) : _domain(dom), _expansion(deg+1u,FloatDP(dp)), _error(0u,dp) { }

    TaylorSeries(const IntervalDomainType& domain, const FloatDP& centre, DegreeType degree,
                 AnalyticFunction const& function);

    template<class OP> TaylorSeries(OP unary_operator, const IntervalDomainType& domain, const FloatDP& centre, DegreeType degree);

    DegreeType degree() const { return _expansion.size()-1u; }
    FloatDP const& operator[](DegreeType i) const { return _expansion[i]; }
    Array<FloatDP> expansion() const { return _expansion; }
    FloatDPError error() const { return _error; }
    Void sweep(FloatDP threshold);

    friend OutputStream& operator<<(OutputStream&, TaylorSeries<FloatDPBounds> const&);
};


template<class OP> inline
TaylorSeries<FloatDPBounds>::TaylorSeries(OP unary_operator, const IntervalDomainType& domain, const FloatDP& centre, DegreeType degree)
    : _domain(domain), _expansion(degree+1u,FloatDP(dp)), _error(0u,dp)
{
    Series<FloatDPBounds> centre_series=Series<FloatDPBounds>(unary_operator,FloatDPBounds(centre));
    Series<FloatDPBounds> range_series=Series<FloatDPBounds>(unary_operator,FloatDPBounds(cast_singleton(domain)));
    for(DegreeType i=0; i!=degree; ++i) {
        this->_expansion[i]=centre_series[i].value();
        this->_error+=mag(centre_series[i]-this->_expansion[i]);
    }
    DegreeType d=degree;
    this->_expansion[d]=range_series[d].value();
    this->_error+=mag(range_series[d]-this->_expansion[d]);
}


inline
Void TaylorSeries<FloatDPBounds>::sweep(FloatDP threshold) {
    for(DegreeType i=0; i<=degree(); ++i) {
        if(definitely(mag(_expansion[i])<=threshold)) {
            _error+=mag(_expansion[i]);
            _expansion[i]=0;
        }
    }
}

inline
OutputStream& operator<<(OutputStream& os, const TaylorSeries<FloatDPBounds>& ts) {
    return os<<"TS("<<ts._expansion<<"+/-"<<ts._error<<")";
}


} // namespace Ariadne

#endif // ARIADNE_TAYLOR_SERIES_HPP
