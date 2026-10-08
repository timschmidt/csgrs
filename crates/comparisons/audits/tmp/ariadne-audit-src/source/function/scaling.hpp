/***************************************************************************
 *            function/scaling.hpp
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

/*! \file function/scaling.hpp
 *  \brief Scaling functions.
 */

#ifndef ARIADNE_SCALING_HPP
#define ARIADNE_SCALING_HPP

#include "numeric/dyadic.hpp"
#include "function/domain.hpp"

namespace Ariadne {

struct UnscalingException : std::runtime_error {
    IntervalDomainType domain;
    inline UnscalingException(String msg, IntervalDomainType dom) : std::runtime_error(msg+to_str(dom)), domain(dom) { }
};

template<class F> inline Approximation<F> med_apprx(Interval<F> const& ivl) {
    return Approximation<F>(hlf(add(approx,ivl.lower_bound().raw(),ivl.upper_bound().raw())));
}

template<class F> inline Approximation<F> rad_apprx(Interval<F> const& ivl) {
    return Approximation<F>(hlf(sub(approx,ivl.upper_bound().raw(),ivl.lower_bound().raw())));
}

template<class F> inline Bounds<F> med_val(Interval<F> const& ivl) {
    return hlf(ivl.lower_bound()+ivl.upper_bound());
}

template<class F> inline Bounds<F> rad_val(Interval<F> const& ivl) {
    return hlf(ivl.upper_bound()-ivl.lower_bound());
}

inline Dyadic med(IntervalDomainType const& ivl) {
    return hlf(add( Dyadic(ivl.lower_bound()), Dyadic(ivl.upper_bound()) ));
}

inline Dyadic rad(IntervalDomainType const& ivl) {
    return hlf(sub( Dyadic(ivl.upper_bound()), Dyadic(ivl.lower_bound()) ));
}

template<class T> inline
T scale(T x, const IntervalDomainType& cd) {
    return std::move(x)*rad(cd)+med(cd);
}

template<class T> inline
T unscale(T x, const IntervalDomainType& d) {
    if(d.lower_bound()==d.upper_bound()) {
        return std::move(x)*0;
        // throw UnscalingException{"Cannot unscale "+to_str(x)+" over empty interval ",d};
    } else {
        return (std::move(x)-med(d))/rad(d);
    }
}

template<class X> inline
Vector<X> scale(const Vector<X>& v, const BoxDomainType& cd) {
    return Vector<X>(v.size(),[&](SizeType i){return scale(v[i],cd[i]);});
}

template<class F, class T> struct MyConvertible {
    template<class TT> static Void convert(TT const&);
    template<class FF, class TT, class=decltype(convert<TT>(std::declval<FF>()))>
        static std::true_type test(int);
    template<class FF, class TT>
        static std::false_type test(...);
    static const bool value = decltype(test<F,T>(1))::value;
};

template<class X> inline
Vector<X> unscale(const Vector<X>& v, const BoxDomainType& d) {
    return Vector<X>(v.size(),[&](SizeType i){return unscale(v[i],d[i]);},v.element_characteristics());
}


class Scaling {
    IntervalDomainType _codom;
  public:
    Scaling(IntervalDomainType codom) : _codom(codom) { }
    UnitInterval domain() const { return UnitInterval(); }
    IntervalDomainType codomain() const { return _codom; }
    template<class X> X operator() (X x) const { return scale(std::move(x),_codom); }
};

class VectorScaling {
    Box<IntervalDomainType> _codom;
  public:
    VectorScaling(Box<IntervalDomainType> codom) : _codom(codom) { }
    SizeType size() const { return _codom.dimension(); }
    Scaling operator[] (SizeType i) const { return Scaling(_codom[i]); }
    Box<IntervalDomainType> const& codomain() const { return _codom; }
    template<class X> Vector<X> operator() (Vector<X> const& v) const { return scale(v,_codom); }
};

class Unscaling {
    IntervalDomainType _dom;
  public:
    Unscaling(IntervalDomainType dom) : _dom(dom) { }
    IntervalDomainType domain() const { return _dom; }
    UnitInterval codomain() const { return UnitInterval(); }
    template<class X> X operator() (X x) const { return unscale(std::move(x),_dom); }
};

class VectorUnscaling {
    Box<IntervalDomainType> _dom;
  public:
    VectorUnscaling(Box<IntervalDomainType> dom) : _dom(dom) { }
    SizeType size() const { return _dom.dimension(); }
    Unscaling operator[] (SizeType i) const { return Unscaling(_dom[i]); }
    Box<IntervalDomainType> const& domain() const { return _dom; }
    template<class X> Vector<X> operator() (Vector<X> const& v) const { return unscale(v,_dom); }
};


} // namespace Ariadne

#endif // ARIADNE_SCALING_HPP
