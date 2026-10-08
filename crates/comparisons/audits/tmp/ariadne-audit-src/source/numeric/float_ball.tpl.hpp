/***************************************************************************
 *            float_ball.tpl.hpp
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


#ifndef ARIADNE_FLOAT_BALL_TPL_HPP
#define ARIADNE_FLOAT_BALL_TPL_HPP

#include "float_ball.hpp"

#include "foundations/logical.hpp"

#include "integer.hpp"
#include "dyadic.hpp"
#include "decimal.hpp"
#include "rational.hpp"

#include "float_error.hpp"
#include "float_bounds.hpp"

#include "number_wrapper.hpp"

namespace Ariadne {

namespace {
template<class FE, class FLT> requires (not SameAs<FE,FLT>) inline FE _make_error(FLT const& x) {
    typename FE::PrecisionType pre; return FE(Dyadic(x),upward,pre); }
template<class FE, class FLT> requires SameAs<FE,FLT> inline FE _make_error(FLT const& x) {
    return x; }
template<class FE, class FLT, class PRE> inline FE _make_error(FLT const& x, PRE pre) {
    return FE(Dyadic(x),upward,pre); }
} // namespace


template<class F, class FE> Ball<F,FE>::Ball(F const& v, Error<FE> const& e)
    : _v(v), _e(e.raw()) { }

template<class F, class FE> Ball<F,FE>::Ball(Bounds<F> const& x)
    : _v(x.value_raw()), _e(_make_error<FE>(x.error_raw())) { }
template<class F, class FE> Ball<F,FE>::Ball(Bounds<F> const& x, PRE pre)
    : _v(x.value_raw()) , _e(_make_error<FE>(x.error_raw(),pre)) { }

template<class F, class FE> Ball<F,FE>::Ball(Ball<F,FE> const& x, PR pr)
    : _v(x._v,near,pr), _e(x._e,up,_error_precision<PRE>(pr))
{
    F d = (this->_v>=x._v) ? sub(up,this->_v,x._v) : sub(up,x._v,this->_v); _e=add(up,_e,_make_error<FE>(d));
}

template<class F, class FE> Ball<F,FE>::Ball(ExactDouble const& d, PR pr) : _v(d,pr), _e(0,_error_precision<PRE>(pr)) {}
template<class F, class FE> Ball<F,FE>::Ball(TwoExp const& t, PR pr) : _v(t,pr), _e(0u,_error_precision<PRE>(pr)) {}
template<class F, class FE> Ball<F,FE>::Ball(Integer const& z, PR pr) : _v(z,near,pr), _e(abs(Dyadic(_v)-z),up,_error_precision<PRE>(pr)) {}
template<class F, class FE> Ball<F,FE>::Ball(Dyadic const& w, PR pr) : _v(w,near,pr), _e(abs(Dyadic(_v)-w),up,_error_precision<PRE>(pr)) {}
template<class F, class FE> Ball<F,FE>::Ball(Decimal const& d, PR pr) : Ball(Rational(d),pr) {}
template<class F, class FE> Ball<F,FE>::Ball(Rational const& q, PR pr) : _v(q,near,pr), _e(abs(Rational(_v)-q),up,_error_precision<PRE>(pr)) {}
template<class F, class FE> Ball<F,FE>::Ball(Integer const& z, PR pr, PRE pre) : _v(F(z,near,pr)), _e(abs(Dyadic(_v)-z),up,pre) {}
template<class F, class FE> Ball<F,FE>::Ball(Dyadic const& w, PR pr, PRE pre) : _v(F(w,near,pr)), _e(abs(Dyadic(_v)-w),up,pre) {}
template<class F, class FE> Ball<F,FE>::Ball(Rational const& q, PR pr, PRE pre) : _v(F(q,near,pr)), _e(abs(Rational(_v)-q),up,pre) {}

template<class F, class FE> Ball<F,FE>::Ball(Decimal const& dv, Decimal const& de, PrecisionType pr)
    : _v(dv,near,pr), _e(de,up,_error_precision<PRE>(pr)) { }

template<class F, class FE> Ball<F,FE>::Ball(Real const& r, PR pr) : Ball(r.get(pr)) {}
template<class F, class FE> Ball<F,FE>::Ball(ValidatedNumber const& y, PR pr) : Ball(y.get(pr,_error_precision<PRE>(pr))) {}
template<class F, class FE> Ball<F,FE>::Ball(Real const& r, PR pr, PRE pre) : Ball(r.get(pr),pre) {}
template<class F, class FE> Ball<F,FE>::Ball(ValidatedNumber const& y, PR pr, PRE pre) : Ball(y.get(pr,pre)) {}
template<class F, class FE> Ball<F,FE>::operator ValidatedNumber() const { return ValidatedNumber(new NumberWrapper<Ball<F,FE>>(*this));}

template<class F, class FE> LowerBound<F> const Ball<F,FE>::lower() const { return LowerBound<F>(sub(down,this->_v,this->_e)); }
template<class F, class FE> UpperBound<F> const Ball<F,FE>::upper() const { return UpperBound<F>(add(up,this->_v,this->_e)); }

template<class F, class FE> F const Ball<F,FE>::value() const { return F(this->_v); }
template<class F, class FE> Error<FE> const Ball<F,FE>::error() const { return Error<FE>(this->_e); }


template<class F, class FE> Integer Operations<Ball<F,FE>>::_cast_integer(Ball<F,FE> const& x) {
    Dyadic w=static_cast<Dyadic>(x.value_raw());
    Integer r=round(w);
    ARIADNE_ASSERT_MSG(abs(w-r)<=x.error_raw(),"cast_integer(Ball<"<<class_name<F>()<<","<<class_name<FE>()<<"> const& x): "
                                                    "x="<<x<<"does not model an integer.")
    return r;
}

template<class F, class FE> InputStream& Operations<Ball<F,FE>>::_read(InputStream& is, Ball<F,FE>& x) {
    static const char pmstr[] = "\u00b1";
    char cpm[3];
    F _v(x.precision()); FE _e(x.error_precision());
    auto rnd=F::get_rounding_mode();
    F::set_rounding_to_nearest();
    is >> _v;
    is >> cpm[0] >> cpm[1];
    F::set_rounding_mode(rnd);
    auto rnde=FE::get_rounding_mode();
    FE::set_rounding_upward();
    is >> _e;
    FE::set_rounding_mode(rnde);
    ARIADNE_ASSERT(not is.fail());
    ARIADNE_ASSERT(std::strcmp(cpm,pmstr));
    x._v=_v; x._e=_e;
    return is;
}


template<class F, class FE> OutputStream& Operations<Ball<F,FE>>::_write(OutputStream& os, Ball<F,FE> const& x) {
    return os << x.value() << "\u00b1" << x.error();
}

} // namespace Ariadne

#endif
