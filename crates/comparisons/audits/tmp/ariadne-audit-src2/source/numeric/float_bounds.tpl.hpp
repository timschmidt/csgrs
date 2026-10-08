/***************************************************************************
 *            float_bounds.tpl.hpp
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


#ifndef ARIADNE_FLOAT_BOUNDS_TPL_HPP
#define ARIADNE_FLOAT_BOUNDS_TPL_HPP

#include "float_bounds.hpp"

#include "float_error.hpp"
#include "float_ball.hpp"
#include "float_upper_bound.hpp"
#include "float_lower_bound.hpp"

#include "number.hpp"
#include "lower_number.hpp"
#include "upper_number.hpp"
#include "number_wrapper.hpp"

namespace Ariadne {

int abslog10floor(double x);

template<class F> Nat Bounds<F>::output_places=8;

template<class F> Bounds<F>::Bounds(LowerBound<F> const& lower, UpperBound<F> const& upper) : Bounds<F>(lower.raw(),upper.raw()) { }

template<class F> Bounds<F>::Bounds(Real const& x, PR pr) : Bounds(x.get(pr)) {}
template<class F> Bounds<F>::Bounds(LowerBound<F> const& lower, ValidatedUpperNumber const& upper) : Bounds<F>(lower,lower.create(upper)) { }
template<class F> Bounds<F>::Bounds(ValidatedLowerNumber const& lower, UpperBound<F> const& upper) : Bounds<F>(upper.create(lower),upper) { }
template<class F> Bounds<F>::Bounds(ValidatedLowerNumber const& lower, ValidatedUpperNumber const& upper, PR pr) : Bounds<F>(lower.get(pr),upper.get(pr)) { }
template<class F> Bounds<F>::Bounds(ValidatedNumber const& y, PR pr) : Bounds(y.get(pr)) {}
template<class F> Bounds<F>::operator ValidatedNumber() const { return ValidatedNumber(new NumberWrapper<Bounds<F>>(*this));}


template<class F> LowerBound<F> const Bounds<F>::lower() const {
    return LowerBound<F>(lower_raw()); }
template<class F> UpperBound<F> const Bounds<F>::upper() const {
    return UpperBound<F>(upper_raw()); }
template<class F> const F Bounds<F>::value() const {
    return F(med(near,this->_l,this->_u)); }
template<class F> const Error<F> Bounds<F>::error() const {
    RawFloat<PR> _v=med(near,this->_l,this->_u); return Error<F>(max(sub(up,this->_u,_v),sub(up,_v,this->_l))); }

template<class F> Bounds<F> Bounds<F>::pm(Error<F> const& e) const {
    return Bounds<F>(sub(down,this->_l,e._e),add(up,this->_u,e._e)); }


template<class F> auto Operations<Bounds<F>>::_pi(PR pr) -> Bounds<F> {
    return Bounds<F>(F::pi(down,pr),F::pi(up,pr));
}

template<class F> auto Operations<Bounds<F>>::_sin(Bounds<F> const& x) -> Bounds<F> {
    return cos(x-hlf(_pi(x.precision())));
}

template<class F> auto Operations<Bounds<F>>::_cos(Bounds<F> const& x) -> Bounds<F> {
    ARIADNE_ASSERT(x.lower_raw()<=x.upper_raw());
    typename F::RoundingModeType rnd = F::get_rounding_mode();
    PR prec=x.precision();

    const F one(1,prec);
    const Bounds<F> pi_val=_pi(prec);
    const Bounds<F> two_pi_val=2*pi_val;
    if(x.error().raw()>two_pi_val.lower().raw()) { return Bounds<F>(-one,+one); }

    F n(round(div(near,x.value_raw(),(two_pi_val.value_raw()))));
    Bounds<F> y=x-2*(n*pi_val);

    ARIADNE_ASSERT(y.lower_raw()<=pi_val.upper_raw());
    ARIADNE_ASSERT(y.upper_raw()>=-pi_val.upper_raw());

    F rl(prec),ru(prec);
    if(y.lower_raw()<=-pi_val.lower_raw()) {
        if(y.upper_raw()<=0.0_x) { rl=-one; ru=cos(up,y.upper_raw()); }
        else { rl=-one; ru=+one; }
    } else if(y.lower_raw()<=0.0_x) {
        if(y.upper_raw()<=0.0_x) { rl=cos(down,y.lower_raw()); ru=cos(up,y.upper_raw()); }
        else if(y.upper_raw()<=pi_val.lower_raw()) { rl=cos(down,max(-y.lower_raw(),y.upper_raw())); ru=+one; }
        else { rl=-one; ru=+one; }
    } else if(y.lower_raw()<=pi_val.upper_raw()) {
        if(y.upper_raw()<=pi_val.lower_raw()) { rl=cos(down,y.upper_raw()); ru=cos(up,y.lower_raw()); }
        else if(y.upper_raw()<=two_pi_val.lower_raw()) { rl=-one; ru=cos(up,min(y.lower_raw(),sub(down,two_pi_val.lower_raw(),y.upper_raw()))); }
        else { rl=-one; ru=+one; }
    } else {
        assert(false);
    }

    F::set_rounding_mode(rnd);
    return Bounds<F>(rl,ru);
}

template<class F> auto Operations<Bounds<F>>::_tan(Bounds<F> const& x) -> Bounds<F> {
    ARIADNE_ASSERT(x.lower_raw()<=x.upper_raw());
    PR prec=x.precision();
    const Bounds<F> pi_bnds=_pi(prec);
    const F pi_val=pi_bnds.value_raw();
    typename F::RoundingModeType rnd = F::get_rounding_mode();
    F n(round(div(near,x.value_raw(),pi_val)));
    F::set_rounding_mode(rnd);
    Bounds<F> y=x-n*pi_bnds;
    assert(y.lower_raw()>=-hlf(pi_val));
    assert(y.upper_raw()<=+hlf(pi_val));
    return Bounds<F>(tan(down,y._l),tan(up,y._u));
}

template<class F> auto Operations<Bounds<F>>::_trunc(Bounds<F> const& x) -> Bounds<F> {
    typename F::RoundingModeType rm=F::get_rounding_mode();
    const double& xl=x.lower_raw().get_d();
    const double& xu=x.upper_raw().get_d();
    // Use machine epsilon instead of minimum to move away from zero
    const float fm=std::numeric_limits<float>::epsilon();
    volatile float tu=xu;
    if(tu<xu) { F::set_rounding_upward(); tu=tu+fm; }
    volatile float tl=xl;
    if(tl>xl) { F::set_rounding_downward(); tl=tl-fm; }
    F::set_rounding_mode(rm);
    assert(tl<=xl); assert(tu>=xu);
    return Bounds<F>(ExactDouble(tl),ExactDouble(tu),x.precision());
}

template<class F> auto Operations<Bounds<F>>::_trunc(Bounds<F> const& x, Nat n) -> Bounds<F> {
    Bounds<F> _e=Bounds<F>(ExactDouble(std::pow(2.0,52-(Int)n)),x.precision());
    Bounds<F> y=x+_e; return y-_e;
}

template<class F> auto Operations<Bounds<F>>::_cast_integer(Bounds<F> const& x) -> Integer {
    Integer z=round(static_cast<Dyadic>(x.value()));
    ARIADNE_ASSERT_MSG(x.lower_raw()<=z && z<=x.upper_raw(),"cast_integer(Bounds<"<<class_name<F>()<<"> const& x): "
                            "x="<<x<<" does model any integer values.");
    return z;
}

template<class F> auto Operations<Bounds<F>>::_write(OutputStream& os, Bounds<F> const& x) -> OutputStream& {
    // Display using a number of fractional places so that the larger number in absolute value is displayed to the given precision.
    F amax=max(-x._l,x._u);
    if(amax==0) { return os << "{0.:0.}"; }

    Int zdgts=abslog10floor(amax.get_d())+1;

    Nat fdgts=Nat(std::max(int(Bounds<F>::output_places)-zdgts,0));
    os << '{';
    write(os,x.lower().raw(),DecimalPlaces{fdgts},downward);
    os << ':';
    write(os,x.upper().raw(),DecimalPlaces{fdgts},upward);
    os << '}';
    return os;
}

template<class F> auto Operations<Bounds<F>>::_read(InputStream& is, Bounds<F>& x) -> InputStream& {
    char cl,cm,cr;
    F _l(x.precision()),_u(x.precision());
    auto rnd=F::get_rounding_mode();
    is >> cl;
    F::set_rounding_downward();
    is >> _l;
    is >> cm;
    F::set_rounding_upward();
    is >> _u;
    is >> cr;
    F::set_rounding_mode(rnd);
    ARIADNE_ASSERT(not is.fail());
    ARIADNE_ASSERT(cl=='[' || cl=='(');
    ARIADNE_ASSERT(cm==':' || cm==',' || cm==';');
    ARIADNE_ASSERT(cr==']' || cr==')');
    x._l=_l; x._u=_u;
    return is;
}



} // namespace Ariadne

#endif
