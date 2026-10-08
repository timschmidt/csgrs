/***************************************************************************
 *            numeric/float64.cpp
 *
 *  Copyright  2008-22  Pieter Collins
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

#include "utility/standard.hpp"

#include <iostream>
#include <iomanip>
#include <cassert>
#include <limits>



#include "config.hpp"

#include "utility/macros.hpp"
#include "numeric/builtin.hpp"
#include "numeric/twoexp.hpp"
#include "numeric/dyadic.hpp"
#include "numeric/decimal.hpp"
#include "numeric/rational.hpp"
#include "numeric/rounding.hpp"
#include "numeric/floatdp.hpp"
#include "numeric/floatmp.hpp"

#include "numeric/concepts.hpp"

namespace Ariadne {

/************  Publicly-accessible rounding-mode changing *******************/

static const double _pi_down=3.1415926535897931;
static const double _pi_near=3.1415926535897931;
static const double _pi_up  =3.1415926535897936;

int abslog10floor(double x);


static_assert(not GenericNumber<FloatDP>);

FloatDP::Float(ExactDouble const& d, PrecisionType)
    : FloatDP(d.get_d())
{
}

FloatDP::Float(TwoExp const& t, PrecisionType)
    : FloatDP(std::ldexp(1.0,t.exponent()))
{
    ARIADNE_ASSERT(Dyadic(*this)==Dyadic(t));
}

FloatDP::Float(Dyadic const& w, PrecisionType)
    : FloatDP(w.get_d())
{
    if (Dyadic(*this)==w || is_nan(w)) {
    } else {
        ARIADNE_THROW(std::runtime_error,"Float(Dyadic)","Dyadic \""<<w<<"\" is not an exact double-precision floating-point number.");
    }
}

FloatDP::Float(String const& str, PrecisionType pr)
    : FloatDP(Dyadic(str),pr)
{
}

FloatDP::Float(Integer const& x, RoundingModeType rnd, PrecisionType pr)
    : FloatDP(Dyadic(x),rnd,pr)
{
}

FloatDP::Float(Dyadic const& w, RoundingModeType rnd, PrecisionType pr)
    : FloatDP(w.get_d())
{
    if (is_finite(w)) {
         RoundingModeType old_rnd=get_rounding_mode();
         if(rnd==ROUND_UPWARD) {
             set_rounding_upward();
             while (Dyadic(dbl)<w) { dbl=dbl+std::numeric_limits<double>::min(); }
             set_rounding_mode(old_rnd);
         }
         if(rnd==ROUND_DOWNWARD) {
             set_rounding_downward();
             while (Dyadic(dbl)>w) { dbl=dbl-std::numeric_limits<double>::min(); }
             set_rounding_mode(old_rnd);
         }
     }
}

FloatDP::Float(Decimal const& dec, RoundingModeType rnd, PrecisionType pr)
    : FloatDP(Rational(dec),rnd,pr)
{
}

inline Rational cast_rational(double d) { return Rational(ExactDouble(d)); }

FloatDP::Float(Rational const& q, RoundingModeType rnd, PrecisionType pr)
    : FloatDP(q.get_d())
{
    if (is_finite(q)) {
        RoundingModeType old_rnd=get_rounding_mode();
        if(rnd==ROUND_UPWARD) {
            set_rounding_upward();
            while (cast_rational(dbl)<q) { dbl=dbl+std::numeric_limits<double>::min(); }
            set_rounding_mode(old_rnd);
        }
        if(rnd==ROUND_DOWNWARD) {
            set_rounding_downward();
            while (cast_rational(dbl)>q) { dbl=dbl-std::numeric_limits<double>::min(); }
            set_rounding_mode(old_rnd);
        }
    }
}

FloatDP::Float(FloatDP const& x, RoundingModeType rnd, PrecisionType)
    : FloatDP(x)
{
}

FloatDP& FloatDP::operator=(ExactDouble const& x) {
    this->dbl=x.get_d();
    return *this;
}

FloatDP& FloatDP::operator=(Dyadic const& w) {
    if (is_finite(w)) {
        this->dbl=mpf_get_d(w.get_mpf());
        assert(*this==w);
    } else if (is_nan(w)) {
        *this=FloatDP::nan(this->precision());
    } else {
        assert(is_inf(w));
        *this=FloatDP::inf(sgn(w),this->precision());
    }
    return *this;
}

FloatDP::operator Dyadic () const {
    return Dyadic(this->dbl);
}

FloatDP::operator Rational () const {
    return Rational(ExactDouble(this->dbl));
}

FloatDP pow_rnd(FloatDP x, Int n)
{
    return FloatDP(pow_rnd(x.dbl,n));
}

FloatDP sqrt_rnd(FloatDP x)
{
    return FloatDP(sqrt_rnd(x.dbl));
}

FloatDP exp_rnd(FloatDP x)
{
    return FloatDP(exp_rnd(x.dbl));
}

FloatDP log_rnd(FloatDP x)
{
    return FloatDP(log_rnd(x.dbl));
}

FloatDP sin_rnd(FloatDP x)
{
    return FloatDP(sin_rnd(x.dbl));
}

FloatDP cos_rnd(FloatDP x)
{
    return FloatDP(cos_rnd(x.dbl));
}

FloatDP tan_rnd(FloatDP x)
{
    return FloatDP(tan_rnd(x.dbl));
}

FloatDP atan_rnd(FloatDP x)
{
    return FloatDP(atan_rnd(x.dbl));
}

FloatDP FloatDP::pi(BuiltinRoundingModeType rnd, DoublePrecision pr) {
    switch(rnd) {
        case FloatDP::ROUND_UPWARD: return FloatDP(_pi_up);
        case FloatDP::ROUND_DOWNWARD: return FloatDP(_pi_down);
        case FloatDP::ROUND_TO_NEAREST: return FloatDP(_pi_near);
        default: assert(false); return FloatDP(_pi_near);
    }
}

Int abslog10floor(FloatDP const& x)
{
    return abslog10floor(x.get_d());
}

FloatDP::RoundingModeType FloatDP::get_rounding_mode() { return Ariadne::get_builtin_rounding_mode(); }
Void FloatDP::set_rounding_mode(RoundingModeType rnd) { Ariadne::set_builtin_rounding_mode(rnd); }
Void FloatDP::set_rounding_downward() { Ariadne::set_builtin_rounding_downward(); }
Void FloatDP::set_rounding_upward() { Ariadne::set_builtin_rounding_upward(); }
Void FloatDP::set_rounding_to_nearest() { Ariadne::set_builtin_rounding_to_nearest(); }
Void FloatDP::set_rounding_toward_zero() { Ariadne::set_builtin_rounding_toward_zero(); }

FloatDP::PrecisionType FloatDP::get_default_precision() { return FloatDP::PrecisionType(); }
FloatDP::PrecisionType FloatDP::precision() const { return FloatDP::PrecisionType(); }
Void FloatDP::set_precision(FloatDP::PrecisionType) { }

FloatDP FloatDP::min(PrecisionType) { return FloatDP(std::numeric_limits<double>::min()); }
FloatDP FloatDP::max(PrecisionType) { return FloatDP(std::numeric_limits<double>::max()); }
FloatDP FloatDP::eps(PrecisionType) { return FloatDP(std::numeric_limits<double>::epsilon()); }

FloatDP FloatDP::inf(Sign sgn, PrecisionType pr) {
    switch (sgn) {
    case Sign::POSITIVE: return FloatDP(std::numeric_limits<double>::infinity());
    case Sign::NEGATIVE: return FloatDP(-std::numeric_limits<double>::infinity());
    default: return FloatDP(std::numeric_limits<double>::quiet_NaN());
    }
}
FloatDP FloatDP::inf(PrecisionType pr) { return FloatDP::inf(Sign::POSITIVE,pr); }
FloatDP FloatDP::nan(PrecisionType pr) { return FloatDP::inf(Sign::ZERO,pr); }

Integer cast_integer(FloatDP x) {
    Dyadic w(x);
    Integer z=round(w);
    ARIADNE_ASSERT_MSG(z==w,"Cannot cast non-integral value "<<z<<" to an Integer");
    return z;
}

template<class R, class A> R integer_cast(A const&);
template<> Nat integer_cast<Nat,FloatDP>(FloatDP const& x) { return static_cast<Nat>(x.dbl); }
template<> Int integer_cast<Int,FloatDP>(FloatDP const& x) { return static_cast<Int>(x.dbl); }


Comparison cmp(FloatDP x1, Rational const& q2) {
    if(std::isfinite(x1.get_d())) { return cmp(Rational(x1),q2); }
    else { return x1.get_d()>0.0 ? Comparison::GREATER : Comparison::LESS; }
}
Comparison cmp(Rational const& q1, FloatDP x2) {
    if(std::isfinite(x2.get_d())) { return cmp(q1,Rational(x2)); }
    else { return x2.get_d()>0.0 ? Comparison::LESS : Comparison::GREATER; }
}

/*
OutputStream& write(OutputStream& os, FloatDP const& x, Nat bits, BuiltinRoundingModeType rnd) {
    Nat dgts = std::ceil(bits*std::log(2))/std::log(10);
    FloatDP::RoundingModeType old_rnd=FloatDP::get_rounding_mode();
    FloatDP::set_rounding_mode(rnd);
    os << std::setprecision(dgts) << x.get_d();
    FloatDP::set_rounding_mode(old_rnd);
    return os;
}
*/

OutputStream& write(OutputStream& os, FloatMP const& x, DecimalPlaces plcs, MPFRRoundingModeType rnd);
OutputStream& write(OutputStream& os, FloatMP const& x, DecimalPrecision figs, MPFRRoundingModeType rnd);

MPFRRoundingModeType to_mpfr_rounding_mode(BuiltinRoundingModeType rnd) {
    assert(rnd==ROUND_TO_NEAREST || rnd==ROUND_UPWARD || rnd==ROUND_DOWNWARD);
    return (rnd==FloatDP::ROUND_TO_NEAREST) ? FloatMP::ROUND_TO_NEAREST
               : (rnd==FloatDP::ROUND_DOWNWARD) ? FloatMP::ROUND_DOWNWARD : FloatMP::ROUND_UPWARD;
}

String FloatDP::literal() const {
    StringStream ss;
    write(ss,*this,DecimalPrecision(53),near);
    return ss.str();
}

String FloatDP::literal(RoundingModeType rnd) const {
    StringStream ss;
    write(ss,*this,DecimalPrecision(18),rnd);
    return ss.str();
}

OutputStream& write(OutputStream& os, FloatDP const& x, DecimalPlaces plcs, BuiltinRoundingModeType rnd) {
    MultiplePrecision pr_mp(53);
    MPFRRoundingModeType rnd_mp=to_mpfr_rounding_mode(rnd);
    return write(os,FloatMP(x,pr_mp),plcs,rnd_mp);
}

OutputStream& write(OutputStream& os, FloatDP const& x, DecimalPrecision figs, BuiltinRoundingModeType rnd) {
    MultiplePrecision pr_mp(53);
    MPFRRoundingModeType rnd_mp=to_mpfr_rounding_mode(rnd);
    return write(os,FloatMP(x,pr_mp),figs,rnd_mp);
}

OutputStream& repr(OutputStream& os, FloatDP const& x) {
    return write(os,x,DecimalPrecision(17),near);
}
OutputStream& repr(OutputStream& os, FloatDP const& x, BuiltinRoundingModeType rnd) {
    static const DecimalPrecision dgts(18);
    return write(os,x,dgts,rnd);
}

OutputStream& operator<<(OutputStream& os, FloatDP const& x) {
    return repr(os,x);
}

OutputStream& operator<<=(OutputStream& os, FloatDP const& x) {
    return os << "FloatDP(" << x << ",near," << x.precision() << ")";
}

InputStream& operator>>(InputStream& is, FloatDP& x) {
    double r; is >> r; x.dbl=r; return is;
}

Nat FloatDP::output_places=16;
Void FloatDP::set_output_places(Nat pl) { output_places=pl; }

template<class PR> PR make_default_precision();
template<> DP make_default_precision<DP>() { return dp; }

Float32::operator FloatDP() const { return FloatDP(cast_exact((double)this->flt),dp); }

template<> String class_name<double>() { return "double"; }

template<> String class_name<ApproximateDouble>() { return "ApproximateDouble"; }
template<> String class_name<ExactDouble>() { return "ExactDouble"; }


template<> String class_name<DoublePrecision>() { return "DoublePrecision"; }
template<> String class_name<FloatDP>() { return "FloatDP"; }

template<class X> class Rounded;
template<> class Rounded<FloatDP> { public: double dbl; Rounded(Approximation<FloatDP> const&); };
template<> String class_name<Rounded<FloatDP>>() { return "Rounded<FloatDP>"; }


template<class X> class UpperBound { X _u; public: UpperBound(X const& u) : _u(u) { } X const& raw() const { return this->_u; } };
template<class X> class LowerBound { X _l; public: LowerBound(X const& l) : _l(l) { } X const& raw() const { return this->_l; } };
template<class X> class Approximation { X _a; public: X const& raw() const { return this->_a; } };

Rounded<FloatDP>::Rounded(Approximation<FloatDP> const& x) : dbl(x.raw().dbl) { }

template<class X> class Positive : public X { public: Positive(X const& x) : X(x) { } };
Positive<FloatDP> abs(FloatDP x) { return Positive<FloatDP>(FloatDP(std::fabs(x.dbl))); }
Positive<UpperBound<FloatDP>> mag(FloatDP x) { return Positive<UpperBound<FloatDP>>(abs(x)); }
Positive<LowerBound<FloatDP>> mig(FloatDP x) { return Positive<LowerBound<FloatDP>>(abs(x)); }

static_assert(RoundedTranscendentalField<FloatDP>);
static_assert(OrderedLattice<FloatDP>);

} // namespace Ariadne
