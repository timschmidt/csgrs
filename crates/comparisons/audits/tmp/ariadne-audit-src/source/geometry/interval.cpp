/***************************************************************************
 *            geometry/interval.cpp
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

#include "interval.hpp"
#include "numeric/dyadic.hpp"

namespace Ariadne {

static const uint GEOMETRY_OUTPUT_PLACES = 4;

template<> OutputStream& operator<<(OutputStream& os, Interval<FloatDP> const& ivl) {
    auto places = FloatDP::output_places;
    FloatDP::output_places=GEOMETRY_OUTPUT_PLACES;
    os << "{" << ivl.lower_bound() << ":" << ivl.upper_bound() << "}";
    FloatDP::output_places=places;
    return os;
}

Interval<FloatDP> widen_domain(Interval<FloatDPUpperBound> const& ivl) {
    auto rnd=FloatDP::get_rounding_mode();
    FloatDP::set_rounding_mode(upward);
    volatile float min=std::numeric_limits<float>::min();
    volatile double neg_l=(-ivl.lower_bound()).get_d();
    volatile double l=-neg_l;
    volatile double u=ivl.upper_bound().get_d();
    volatile float neg_rl=neg_l;
    volatile float ru=u;
    if(l==u) { neg_rl=neg_rl+min; ru=ru+min; }
    if(neg_rl<neg_l) { neg_rl=neg_rl+min; }
    if(ru<u) { ru=ru+min; }
    volatile float rl=-neg_l;
    Interval<FloatDP> res(ExactDouble(rl),ExactDouble(ru),dp);
    FloatDP::set_rounding_mode(rnd);
    return res;
}

Interval<FloatDP> approximate_domain(Interval<FloatDPUpperBound> const& ivl) {
    auto rnd=FloatDP::get_rounding_mode();
    FloatDP::set_rounding_mode(to_nearest);
    volatile float eps=std::numeric_limits<float>::epsilon();
    volatile double l=ivl.lower_bound().get_d();
    volatile double u=ivl.upper_bound().get_d();
    volatile float rl=l;
    volatile float ru=u;
    if(rl==ru) { rl=rl-(rl*eps); ru=ru+(ru*eps); }
    Interval<FloatDP> res(ExactDouble(rl),ExactDouble(ru),dp);
    FloatDP::set_rounding_mode(rnd);
    return res;
}

InputStream&
operator>>(InputStream& is, Interval<FloatDP>& ivl)
{
    FloatDP l(dp),u(dp);
    char cl,cm,cr;
    is >> cl >> l >> cm >> u >> cr;
    ARIADNE_ASSERT(not is.fail());
    ARIADNE_ASSERT(cl=='[' || cl=='(');
    ARIADNE_ASSERT(cm==':' || cm==',' || cm==';');
    ARIADNE_ASSERT(cr==']' || cr==')');
    ivl.set_bounds(FloatDP(l),FloatDP(u));
    return is;
}


template<> FloatDPUpperInterval UpperIntervalFactory<FloatDP>::create(ValidatedNumber const& y) const {
    return FloatDPUpperInterval(FloatDPBounds(y,this->_precision)); }

FloatDPBounds cast_singleton(FloatDPUpperInterval const& ivl) {
    return FloatDPBounds(ivl.lower_bound(),ivl.upper_bound()); }
FloatDPUpperInterval make_interval(FloatDPBounds const& x) {
    return FloatDPUpperInterval(x.lower(),x.upper()); }

FloatDPUpperInterval max(FloatDPUpperInterval const& ivl1, FloatDPUpperInterval const& ivl2) {
    return make_interval(max(cast_singleton(ivl1),cast_singleton(ivl2))); }
FloatDPUpperInterval min(FloatDPUpperInterval const& ivl1, FloatDPUpperInterval const& ivl2) {
    return make_interval(min(cast_singleton(ivl1),cast_singleton(ivl2))); }
FloatDPUpperInterval abs(FloatDPUpperInterval const& ivl) {
    return make_interval(abs(cast_singleton(ivl))); }
FloatDPUpperInterval nul(FloatDPUpperInterval const& ivl) {
    return make_interval(nul(cast_singleton(ivl))); }
FloatDPUpperInterval pos(FloatDPUpperInterval const& ivl) {
    return make_interval(pos(cast_singleton(ivl))); }
FloatDPUpperInterval neg(FloatDPUpperInterval const& ivl) {
    return make_interval(neg(cast_singleton(ivl))); }
FloatDPUpperInterval sqr(FloatDPUpperInterval const& ivl) {
    return make_interval(sqr(cast_singleton(ivl))); }
FloatDPUpperInterval hlf(FloatDPUpperInterval const& ivl) {
    return make_interval(hlf(cast_singleton(ivl))); }
FloatDPUpperInterval rec(FloatDPUpperInterval const& ivl) {
    return make_interval(rec(cast_singleton(ivl))); }

FloatDPUpperInterval add(FloatDPUpperInterval const& ivl1, FloatDPUpperInterval const& ivl2) {
    return make_interval(add(cast_singleton(ivl1),cast_singleton(ivl2))); }
FloatDPUpperInterval sub(FloatDPUpperInterval const& ivl1, FloatDPUpperInterval const& ivl2) {
    return make_interval(sub(cast_singleton(ivl1),cast_singleton(ivl2))); }
FloatDPUpperInterval mul(FloatDPUpperInterval const& ivl1, FloatDPUpperInterval const& ivl2) {
    return make_interval(mul(cast_singleton(ivl1),cast_singleton(ivl2))); }
FloatDPUpperInterval div(FloatDPUpperInterval const& ivl1, FloatDPUpperInterval const& ivl2) {
    return make_interval(div(cast_singleton(ivl1),cast_singleton(ivl2))); }

FloatDPUpperInterval fma(FloatDPUpperInterval const& ivl1, FloatDPUpperInterval const& ivl2, FloatDPUpperInterval ivl3) {
    return make_interval(fma(cast_singleton(ivl1),cast_singleton(ivl2),cast_singleton(ivl3))); }

FloatDPUpperInterval pow(FloatDPUpperInterval const& ivl, Nat m) {
    return make_interval(pow(cast_singleton(ivl),m)); }
FloatDPUpperInterval pow(FloatDPUpperInterval const& ivl, Int n) {
    return make_interval(pow(cast_singleton(ivl),n)); }

FloatDPUpperInterval sqrt(FloatDPUpperInterval const& ivl) {
    // Defining sqrt(I)=hull{x:x^2 in I}, if I=[a,b] with b>=0, then sqrt(I)=[-sqrt(b),+sqrt(b)]
    if(ivl.upper_bound().raw()<0) { return FloatDPUpperInterval::empty_interval(); }
    else { FloatDP u=sqrt(up,ivl.upper_bound().raw()); return FloatDPUpperInterval(-u,+u); } }
FloatDPUpperInterval exp(FloatDPUpperInterval const& ivl) {
    return make_interval(exp(cast_singleton(ivl))); }
FloatDPUpperInterval log(FloatDPUpperInterval const& ivl) {
    if(ivl.upper_bound().raw()<=0) { return FloatDPUpperInterval::empty_interval(); }
    else if(ivl.lower_bound().raw()<=0) { return FloatDPUpperInterval(FloatDP(-inf,dp),log(up,ivl.upper_bound().raw())); }
    else { return make_interval(log(cast_singleton(ivl))); } }
FloatDPUpperInterval sin(FloatDPUpperInterval const& ivl) {
    return make_interval(sin(cast_singleton(ivl))); }
FloatDPUpperInterval cos(FloatDPUpperInterval const& ivl) {
    return make_interval(cos(cast_singleton(ivl))); }
FloatDPUpperInterval tan(FloatDPUpperInterval const& ivl) {
    return make_interval(tan(cast_singleton(ivl))); }
FloatDPUpperInterval asin(FloatDPUpperInterval const& ivl) {
    return make_interval(asin(cast_singleton(ivl))); }
FloatDPUpperInterval acos(FloatDPUpperInterval const& ivl) {
    return make_interval(acos(cast_singleton(ivl))); }
FloatDPUpperInterval atan(FloatDPUpperInterval const& ivl) {
    return make_interval(atan(cast_singleton(ivl))); }

FloatDPError mag(FloatDPUpperInterval const& ivl) {
    return mag(cast_singleton(ivl)); }

ValidatedKleenean eq(FloatDPUpperInterval const& ivl1, FloatDPUpperInterval const& ivl2) {
    return eq(cast_singleton(ivl1),cast_singleton(ivl2)); }
ValidatedKleenean lt(FloatDPUpperInterval const& ivl1, FloatDPUpperInterval const& ivl2) {
    return lt(cast_singleton(ivl1),cast_singleton(ivl2)); }

template<> Interval<FloatMPUpperBound>::Interval() : Interval(EmptyInterval()) { }
template<> Interval<FloatMPUpperBound>::Interval(EmptyInterval const&)
    : Interval(FloatMP::inf(Sign::POSITIVE,FloatMP::get_default_precision()),
               FloatMP::inf(Sign::NEGATIVE,FloatMP::get_default_precision())) { }
template<> Interval<FloatMPUpperBound>::Interval(UnitInterval const&)
    : Interval(FloatMP(-1,FloatMP::get_default_precision()),FloatMP(+1,FloatMP::get_default_precision())) { }
template<> Interval<FloatMPUpperBound>::Interval(EntireInterval const&)
    : Interval(FloatMP(-inf,FloatMP::get_default_precision()),FloatMP(+inf,FloatMP::get_default_precision())) { }


FloatMPBounds cast_singleton(FloatMPUpperInterval const& ivl) {
    return FloatMPBounds(ivl.lower_bound(),ivl.upper_bound()); }
FloatMPUpperInterval make_interval(FloatMPBounds const& x) {
    return FloatMPUpperInterval(x.lower(),x.upper()); }

template<> FloatMPUpperInterval UpperIntervalFactory<FloatMP>::create(ValidatedNumber const& y) const {
    return FloatMPUpperInterval(FloatMPBounds(y,this->_precision)); }

FloatMPUpperInterval max(FloatMPUpperInterval const& ivl1, FloatMPUpperInterval const& ivl2) {
    return make_interval(max(cast_singleton(ivl1),cast_singleton(ivl2))); }
FloatMPUpperInterval min(FloatMPUpperInterval const& ivl1, FloatMPUpperInterval const& ivl2) {
    return make_interval(min(cast_singleton(ivl1),cast_singleton(ivl2))); }
FloatMPUpperInterval abs(FloatMPUpperInterval const& ivl) {
    return make_interval(abs(cast_singleton(ivl))); }
FloatMPUpperInterval nul(FloatMPUpperInterval const& ivl) {
    return make_interval(nul(cast_singleton(ivl))); }
FloatMPUpperInterval pos(FloatMPUpperInterval const& ivl) {
    return make_interval(pos(cast_singleton(ivl))); }
FloatMPUpperInterval neg(FloatMPUpperInterval const& ivl) {
    return make_interval(neg(cast_singleton(ivl))); }
FloatMPUpperInterval sqr(FloatMPUpperInterval const& ivl) {
    return make_interval(sqr(cast_singleton(ivl))); }
FloatMPUpperInterval hlf(FloatMPUpperInterval const& ivl) {
    return make_interval(hlf(cast_singleton(ivl))); }
FloatMPUpperInterval rec(FloatMPUpperInterval const& ivl) {
    return make_interval(rec(cast_singleton(ivl))); }

FloatMPUpperInterval add(FloatMPUpperInterval const& ivl1, FloatMPUpperInterval const& ivl2) {
    return make_interval(add(cast_singleton(ivl1),cast_singleton(ivl2))); }
FloatMPUpperInterval sub(FloatMPUpperInterval const& ivl1, FloatMPUpperInterval const& ivl2) {
    return make_interval(sub(cast_singleton(ivl1),cast_singleton(ivl2))); }
FloatMPUpperInterval mul(FloatMPUpperInterval const& ivl1, FloatMPUpperInterval const& ivl2) {
    return make_interval(mul(cast_singleton(ivl1),cast_singleton(ivl2))); }
FloatMPUpperInterval div(FloatMPUpperInterval const& ivl1, FloatMPUpperInterval const& ivl2) {
    return make_interval(div(cast_singleton(ivl1),cast_singleton(ivl2))); }

FloatMPUpperInterval fma(FloatMPUpperInterval const& ivl1, FloatMPUpperInterval const& ivl2, FloatMPUpperInterval ivl3) {
    return make_interval(fma(cast_singleton(ivl1),cast_singleton(ivl2),cast_singleton(ivl3))); }

FloatMPUpperInterval pow(FloatMPUpperInterval const& ivl, Nat m) {
    return make_interval(pow(cast_singleton(ivl),m)); }
FloatMPUpperInterval pow(FloatMPUpperInterval const& ivl, Int n) {
    return make_interval(pow(cast_singleton(ivl),n)); }

FloatMPUpperInterval sqrt(FloatMPUpperInterval const& ivl) {
    // Defining sqrt(I)=hull{x:x^2 in I}, if I=[a,b] with b>=0, then sqrt(I)=[-sqrt(b),+sqrt(b)]
    auto pr=ivl.upper_bound().precision(); auto inf_=FloatMP::inf(pr);
    if(ivl.upper_bound().raw()<0) { return FloatMPUpperInterval(+inf_,-inf_); }
    else { FloatMP u=sqrt(up,ivl.upper_bound().raw()); return FloatMPUpperInterval(-u,+u); } }
FloatMPUpperInterval exp(FloatMPUpperInterval const& ivl) {
    return make_interval(exp(cast_singleton(ivl))); }
FloatMPUpperInterval log(FloatMPUpperInterval const& ivl) {
    auto pr=ivl.upper_bound().precision(); auto inf_=FloatMP::inf(pr);
    if(ivl.upper_bound().raw()<=0) { return FloatMPUpperInterval(+inf_,-inf_); }
    else if(ivl.lower_bound().raw()<=0) { return FloatMPUpperInterval(-inf_,log(up,ivl.upper_bound().raw())); }
    else { return make_interval(log(cast_singleton(ivl))); } }
FloatMPUpperInterval sin(FloatMPUpperInterval const& ivl) {
    return make_interval(sin(cast_singleton(ivl))); }
FloatMPUpperInterval cos(FloatMPUpperInterval const& ivl) {
    return make_interval(cos(cast_singleton(ivl))); }
FloatMPUpperInterval tan(FloatMPUpperInterval const& ivl) {
    return make_interval(tan(cast_singleton(ivl))); }
FloatMPUpperInterval asin(FloatMPUpperInterval const& ivl) {
    return make_interval(asin(cast_singleton(ivl))); }
FloatMPUpperInterval acos(FloatMPUpperInterval const& ivl) {
    return make_interval(acos(cast_singleton(ivl))); }
FloatMPUpperInterval atan(FloatMPUpperInterval const& ivl) {
    return make_interval(atan(cast_singleton(ivl))); }

FloatMPError mag(FloatMPUpperInterval const& ivl) {
    return mag(cast_singleton(ivl)); }

ValidatedKleenean eq(FloatMPUpperInterval const& ivl1, FloatMPUpperInterval const& ivl2) {
    return eq(cast_singleton(ivl1),cast_singleton(ivl2)); }
ValidatedKleenean lt(FloatMPUpperInterval const& ivl1, FloatMPUpperInterval const& ivl2) {
    return lt(cast_singleton(ivl1),cast_singleton(ivl2)); }


ApproximateKleenean eq(FloatDPApproximateInterval const& ivl1, FloatDPApproximateInterval const& ivl2) {
    return ivl1.lower_bound()==ivl2.lower_bound() && ivl1.upper_bound()==ivl2.upper_bound(); }
ApproximateKleenean eq(FloatMPApproximateInterval const& ivl1, FloatMPApproximateInterval const& ivl2) {
    return ivl1.lower_bound()==ivl2.lower_bound() && ivl1.upper_bound()==ivl2.upper_bound(); }

template<> String class_name<FloatDPUpperInterval>() { return "FloatDPUpperInterval"; }
template<> String class_name<FloatMPUpperInterval>() { return "FloatMPUpperInterval"; }
template<> String class_name<Interval<FloatDP>>() { return "FloatDPInterval"; }
template<> String class_name<Interval<FloatMP>>() { return "FloatDPInterval"; }
template<> String class_name<Interval<Real>>() { return "RealInterval"; }

} // namespace Ariadne
