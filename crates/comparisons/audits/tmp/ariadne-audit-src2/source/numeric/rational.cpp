/***************************************************************************
 *            numeric/rational.cpp
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

/*! \file numeric/rational.cpp
 *  \brief
 */



#include "utility/stdlib.hpp"
#include "utility/macros.hpp"
#include "utility/typedefs.hpp"

#include "rational.hpp"
#include "foundations/logical.hpp"
#include "number.hpp"
#include "builtin.hpp"
#include "integer.hpp"
#include "dyadic.hpp"
#include "sign.hpp"
#include "extended.hpp"

#include "concepts.hpp"

#include <limits>
#include <iostream>
#include <cmath>
#include <stdexcept>

namespace Ariadne {

class InvalidRationalLiteralException {
  public:
    InvalidRationalLiteralException(StringType what) { }
};

// Shortened version of raw float classes sufficient for comparison operator
template<> class Float<DP> { volatile double _dbl; public: double get_d() const { return _dbl; } };



template<> class ExtensionOperations<Rational> {
    friend class Rational;
    friend class ExtendedOperations<Rational>;

    static Bool is_nan(Rational const& q) { return mpz_cmp_si(mpq_denref(q._mpq),0)==0 && mpz_cmp_si(mpq_numref(q._mpq),0)==0; }
    static Bool is_inf(Rational const& q) { return mpz_cmp_si(mpq_denref(q._mpq),0)==0 && mpz_cmp_si(mpq_numref(q._mpq),0)!=0; }
    static Bool is_finite(Rational const& q) { return mpz_cmp_si(mpq_denref(q._mpq),0)!=0; }
    static Bool is_zero(Rational const& q) { return mpz_cmp_si(mpq_numref(q._mpq),0)==0 && mpz_cmp_si(mpq_denref(q._mpq),0)!=0; }

    static Sign sgn(Rational const& q) {
        if (is_finite(q)) { return static_cast<Sign>(mpq_cmp_si(q._mpq,0,1)); }
        else { return static_cast<Sign>(mpz_cmp_si(mpq_numref(q._mpq),0)); } }

    static Void set_nan(Rational& q) { mpz_set_si(mpq_denref(q._mpq),0); mpz_set_si(mpq_numref(q._mpq),0); }
    static Void set_inf(Rational& q, Sign s) { mpz_set_si(mpq_denref(q._mpq),0); mpz_set_si(mpq_numref(q._mpq),static_cast<Int>(s)); }
    static Void set_zero(Rational& q) { mpq_set_si(q._mpq,0,1); }
};

template<> class FiniteOperations<Rational> {
    friend class ExtendedOperations<Rational>;

    static Void set(Rational& r, Rational const& q) { mpq_set(r._mpq,q._mpq); }

    static Void add(Rational& r, Rational const& q1, Rational const& q2) { return mpq_add(r._mpq, q1._mpq, q2._mpq); }
    static Void sub(Rational& r, Rational const& q1, Rational const& q2) { return mpq_sub(r._mpq, q1._mpq, q2._mpq); }
    static Void mul(Rational& r, Rational const& q1, Rational const& q2) { return mpq_mul(r._mpq, q1._mpq, q2._mpq); }
    static Void div(Rational& r, Rational const& q1, Rational const& q2) { return mpq_div(r._mpq, q1._mpq, q2._mpq); }

    static Void pos(Rational& r, Rational const& q) { mpq_set(r._mpq,q._mpq); }
    static Void neg(Rational& r, Rational const& q) { mpq_neg(r._mpq,q._mpq); }
    static Void hlf(Rational& r, Rational const& q) { mpq_div_2exp(r._mpq,q._mpq,1u); }
    static Void sqr(Rational& r, Rational const& q) { mpq_mul(r._mpq,q._mpq,q._mpq); }
    static Void rec(Rational& r, Rational const& q) { mpq_inv(r._mpq,q._mpq); }

    static Void max(Rational& r, Rational const& q1, Rational const& q2) {
        if(mpq_cmp(q1._mpq,q2._mpq)>=0) { mpq_set(r._mpq,q1._mpq); } else { mpq_set(r._mpq,q2._mpq); } }
    static Void min(Rational& r, Rational const& q1, Rational const& q2) {
        if(mpq_cmp(q1._mpq,q2._mpq)<=0) { mpq_set(r._mpq,q1._mpq); } else { mpq_set(r._mpq,q2._mpq); } }
    static Void abs(Rational& r, Rational const& q) { mpq_abs(r._mpq,q._mpq); }

    static Comparison cmp(Rational const& q1, Rational const& q2) { return static_cast<Comparison>(mpq_cmp(q1._mpq,q2._mpq)); }
    static Boolean eq(Rational const& q1, Rational const& q2) { return mpq_equal(q1._mpq,q2._mpq); }
};


Rational rec(Integer const& z) {
    return Rational(1,z);
}

Rational pow(Integer const& z, Int n) {
    if(n>=0) { return Rational(pow(z,Nat(n))); } else { return Rational(1,pow(z,Nat(-n))); }
}

Rational::~Rational() {
    mpq_clear(_mpq);
}

Rational::Rational() {
    mpq_init(_mpq);
    mpq_set_si(_mpq,0,1);
}

/*
Rational::Rational(Nat m) {
    mpq_init(_mpq);
    mpq_set_ui(_mpq,m,1u);
}

Rational::Rational(Int n) {
    mpq_init(_mpq);
    mpq_set_si(_mpq,n,1);
}
*/

Rational::Rational(Integer const& z) {
    mpq_init(_mpq);
    mpq_set_z(_mpq,z._mpz);
}

Rational::Rational(Dyadic const& f) {
    mpq_init(this->_mpq);
    Rational& q=*this;
    if (is_finite(f)) {
        mpq_set_f(q._mpq,f._mpf);
    } else if (is_nan(f)) {
        ExtensionOperations<Rational>::set_nan(q);
    } else {
        ExtensionOperations<Rational>::set_inf(q, f>0 ? Sign::POSITIVE : Sign::NEGATIVE);
    }
}

Rational::Rational(Integer const& znum, Integer const& zden) {
    mpq_init(_mpq);
    mpq_set_den(_mpq,zden._mpz);
    if(zden==0) {
        Integer num = (znum==0) ? 0 : (znum>0)?1:-1;
        mpq_set_num(_mpq,num._mpz);
    } else {
        mpq_set_num(_mpq,znum._mpz);
        mpq_canonicalize(_mpq);
    }
}

Rational::Rational(Int64 n) : Rational(Integer(n)) {
}

Rational::Rational(ExactDouble const& x) {
    mpq_init(this->_mpq);
    Rational& q=*this;
    Dbl d=x.get_d();
    if(std::isfinite(d)) {
        mpq_set_d(q._mpq,d);
        mpq_canonicalize(q._mpq);
    } else if (std::isnan(d)) {
        ExtensionOperations<Rational>::set_nan(q);
    } else {
        ExtensionOperations<Rational>::set_inf(q, d>0 ? Sign::POSITIVE : Sign::NEGATIVE);
    }
}

Rational::Rational(FloatDP const& x) : Rational(ExactDouble(x.get_d())) {
}

template<> class Float<MP> { mpf_t _mpf; public: operator Dyadic() const; };
Rational::Rational(FloatMP const& x) : Rational(x.operator Dyadic()) {
}

Rational::Rational(const String& str) {
    Int base=10;
    mpq_init(_mpq);
    int fail = mpq_set_str(_mpq,str.c_str(),base);
    if (fail!=0) {
        ARIADNE_THROW(std::runtime_error,"Rational(string)","String \""<<str<<"\" does not have a valid rational number format.");
    }
    mpq_canonicalize(_mpq);
}

Rational::Rational(const mpq_t q) {
    mpq_init(_mpq);
    mpq_set(_mpq,q);
    mpq_canonicalize(_mpq);
}

Rational::Rational(const Rational& q) {
    mpq_init(_mpq);
    mpq_set(_mpq,q._mpq);
}

Rational::Rational(Rational&& q) {
    mpq_init(_mpq);
    mpq_swap(_mpq,q._mpq);
}

Rational& Rational::operator=(const Rational& q) {
    mpq_set(_mpq,q._mpq);
    return *this;
}

Rational& Rational::operator=(Rational&& q) {
    mpq_swap(_mpq,q._mpq);
    return *this;
}


mpq_t const& Rational::get_mpq() const {
    return this->_mpq;
}

double Rational::get_d() const {
    if (is_finite(*this))
        return mpq_get_d(this->_mpq);
    else if (is_nan(*this)) {
        return std::numeric_limits<double>::quiet_NaN();
    } else {
        return (sgn(*this) == Sign::POSITIVE) ? std::numeric_limits<double>::infinity() : -std::numeric_limits<double>::infinity();
    }
}

Integer Rational::get_num() const {
    Integer z;
    mpq_get_num(z._mpz,this->_mpq);
    return z;
}

Integer Rational::get_den() const {
    Integer z;
    mpq_get_den(z._mpz,this->_mpq);
    return z;
}

Integer Rational::numerator() const {
    Integer z;
    mpq_get_num(z._mpz,this->_mpq);
    return z;
}

Natural Rational::denominator() const {
    Natural n;
    mpq_get_den(n._mpz,this->_mpq);
    return n;
}

Rational operator/(Integer const& z1, Integer const& z2) {
    return Rational(z1,z2);
}


Rational operator+(Rational& q1, Rational const& q2) {
    Rational r; ExtendedOperations<Rational>::add(r,q1,q2); return r;
}

Rational operator-(Rational& q1, Rational const& q2) {
    Rational r; ExtendedOperations<Rational>::sub(r,q1,q2); return r;
}

Rational operator*(Rational& q1, Rational const& q2) {
    Rational r; ExtendedOperations<Rational>::mul(r,q1,q2); return r;
}

Rational operator/(Rational& q1, Rational const& q2) {
    Rational r; ExtendedOperations<Rational>::div(r,q1,q2); return r;
}


namespace { Rational const& unsign(PositiveRational const& q) { return q; } }

Rational max(Rational const& q1, Rational const& q2) {
    Rational r; ExtendedOperations<Rational>::max(r,q1,q2); return r;
}

PositiveRational max(Rational const& q1, PositiveRational const& q2) {
    return cast_positive(max(q1,unsign(q2)));
}
PositiveRational max(PositiveRational const& q1, Rational const& q2) {
    return cast_positive(max(unsign(q1),q2));
}
PositiveRational max(PositiveRational const& q1, PositiveRational const& q2) {
    return cast_positive(max(unsign(q1),unsign(q2)));
}

Rational min(Rational const& q1, Rational const& q2) {
    Rational r; ExtendedOperations<Rational>::min(r,q1,q2); return r;
}
PositiveRational min(PositiveRational const& q1, PositiveRational const& q2) {
    return cast_positive(min(unsign(q1),unsign(q2)));
}

Positive<Rational> abs(Rational const& q) {
    Positive<Rational> r; ExtendedOperations<Rational>::abs(r,q); return r;
}

Rational mag(Rational const& q) {
    Rational r; ExtendedOperations<Rational>::abs(r,q); return r;
}

Rational mig(Rational const& q) {
    Rational r; ExtendedOperations<Rational>::abs(r,q); return r;
}

Rational nul(Rational const& q) {
    return Rational(0);
}

Rational pos(Rational const& q) {
    return q;
}

Rational neg(Rational const& q) {
    Rational r; ExtendedOperations<Rational>::neg(r,q); return r;
}

Positive<Rational> sqr(Rational const& q) {
    Positive<Rational> r; ExtendedOperations<Rational>::sqr(r,q); return r;
}

Rational hlf(Rational const& q) {
    Rational r; ExtendedOperations<Rational>::hlf(r,q); return r;
}

Rational rec(Rational const& q) {
    Rational r; ExtendedOperations<Rational>::rec(r,q); return r;
}

Rational add(Rational const& q1, Rational const& q2) {
    Rational r; ExtendedOperations<Rational>::add(r,q1,q2); return r;
}

Rational sub(Rational const& q1, Rational const& q2) {
    Rational r; ExtendedOperations<Rational>::sub(r,q1,q2); return r;
}

Rational mul(Rational const& q1, Rational const& q2) {
    Rational r; ExtendedOperations<Rational>::mul(r,q1,q2); return r;
}

Rational div(Rational const& q1, Rational const& q2) {
    Rational r; ExtendedOperations<Rational>::div(r,q1,q2); return r;
}

Rational div(Integer const& z1, Integer const& z2) {
    return Rational(z1,z2);
}

Rational pow(Rational const& q, Nat m) {
    Rational r=1; Rational p=q;
    while(m!=0) { if(m%2==1) { r=r*p; } p=p*p; m/=2; }
    return r;
}

Rational pow(Rational const& q, Int n) {
    if(n<0) { return rec(pow(q,Nat(-n))); }
    else { return pow(q,Nat(n)); }
}

Boolean eq(Rational const& q1, Rational const& q2) {
    return ExtendedOperations<Rational>::eq(q1,q2);
}

Boolean lt(Rational const& q1, Rational const& q2) {
    return cmp(q1,q2)<Comparison::EQUAL;
}

Rational Rational::inf(Sign sgn) {
    Rational q(sgn==Sign::ZERO ? 0 : sgn==Sign::POSITIVE ? +1 : -1);
    mpz_set_si(mpq_denref(q._mpq),0);
    return q;
}

Rational Rational::inf() {
    return Rational::inf(Sign::POSITIVE);
}

Rational Rational::nan() {
    return Rational::inf(Sign::ZERO);
}

Bool is_nan(Rational const& q) {
    return is_zero(q.get_den()) and is_zero(q.get_num());
}

Bool is_inf(Rational const& q) {
    return is_zero(q.get_den()) and not is_zero(q.get_num());
}

Bool is_finite(Rational const& q) {
    return not is_zero(q.get_den());
}

Bool is_zero(Rational const& q) {
    return is_zero(q.get_num()) and not is_zero(q.get_den());
}

Sign sgn(Rational const& q) {
    return sgn(q.get_num());
}

Integer round(Rational const& q) {
    assert(is_finite(q));

    Integer num=q.numerator();
    Integer den=q.denominator();
    mpz_mul_2exp(num._mpz,num._mpz,1u);
    if (num>=0) { mpz_add(num._mpz,num._mpz,den._mpz); } else { mpz_sub(num._mpz,num._mpz,den._mpz); }
    mpz_mul_2exp(den._mpz,den._mpz,1u);
    Integer& res=num;
    mpz_tdiv_q(res._mpz,num._mpz,den._mpz);
    return res;
}

Integer ceil(Rational const& q) {
    assert(is_finite(q));
    Integer res;
    mpz_cdiv_q(res._mpz,mpq_numref(q._mpq),mpq_denref(q._mpq));
    return res;
}

Integer floor(Rational const& q) {
    assert(is_finite(q));
    Integer res;
    mpz_fdiv_q(res._mpz,mpq_numref(q._mpq),mpq_denref(q._mpq));
    return res;
}


Comparison cmp(Rational const& q1, Rational const& q2) {
    return ExtendedOperations<Rational>::cmp(q1,q2);
}

Comparison cmp(Rational const& q1, ExactDouble const& x2) {
    double d2=x2.get_d();
    assert(not std::isnan(d2));
    if(std::isfinite(d2)) {
        return cmp(q1,Rational(x2));
    } else {
        return d2 > 0.0 ? Comparison::LESS : Comparison::GREATER;
    }
}

Comparison cmp(ExactDouble const& x1, Rational const& q2) {
    return Comparison(-(int)cmp(q2,x1));
}

Comparison cmp(Rational const& q1, Int const& n2) {
    return Comparison(mpq_cmp_si(q1._mpq,n2,1));
}

Comparison cmp(Int const& n1, Rational const& q2) {
    return Comparison(-(int)cmp(q2,n1));
}

Rational operator"" _q(unsigned long long int n) {
    return Rational(operator""_z(n));
}

Rational operator"" _q(long double x) {
    static const uint32_t max_cf_coef = std::numeric_limits<uint32_t>::max();
    static const std::size_t N=11;

    // See if the long double value is an exact single-precision value
    volatile float sx=static_cast<float>(x);
    if(static_cast<long double>(sx)==x) {
        return Rational(ExactDouble(sx));
    }
    // Compute the continued fraction expansion of x, storing coefficients in cf
    // Stop if a coefficient is larger than max_cf_coef, since then the result
    //   is accurate to within 2^{-32}
    long double t=x;
    Int s=1;
    if(x<0) { s=-1; t=-x; }
    long double cf[N+1];
    cf[N]=0;
    std::size_t i;
    for(i=0; i!=N; ++i) {
        cf[i]=std::floor(t);
        t=1/(t-cf[i]);
        if(t>max_cf_coef) { break; }
    }
    if(i==N) {
        ARIADNE_THROW(InvalidRationalLiteralException,"Rational operator"" _q(long double)",
                      "x="<<x<<" is not a sufficiently close approximation to a simple rational number.");
    }
    // Compute the result from the continued fraction coefficients
    Rational q = Rational(ExactDouble(cf[i]));
    while(i!=0) {
        --i;
        q=Rational(ExactDouble(cf[i]))+rec(q);
    }
    if(s==-1) { q=-q; }
    double xd=static_cast<double>(x);
    Rational xq=Rational(ExactDouble(xd));
    //volatile double qd=q.get_d();
    double ae=std::abs((q-xq).get_d());
    double re=ae/std::max(1.0,std::abs(xd));
    if(re>std::numeric_limits<double>::epsilon()) {
        ARIADNE_THROW(InvalidRationalLiteralException,"Rational operator"" _q(long double)",
                      "Rational approximation q="<<q<<" to x="<<x<<"="<<xd<<" has error "<<ae<<" and relative error "<<re<<" while is larger than machine epsilon");
    }
    mpq_canonicalize(q._mpq);
    return q;
}

OutputStream& write(OutputStream& os, mpz_t const z) {
    char str[512];
    str[511]='\0';
    mpz_get_str (str, 10, z);
    assert(str[511]=='\0');
    return os << str;
}

InputStream& operator>>(InputStream& is, Rational& q1) {
    ARIADNE_NOT_IMPLEMENTED;
}

//   mpq_get_str (char *str, mpq_eq1p_t *eq1pptr, Int b, SizeType n, mpq_t op, mpq_rnd_t rnd)
// If str is not a null pointer, it should point to a block of storage large enough for the significand,
// i.e., at least maq1(n + 2, 7). The eq1tra two bytes are for a possible minus sign,
// and for the terminating null character, and the value 7 accounts for -@Inf@ plus the terminating null character.
OutputStream& operator<<(OutputStream& os, Rational const& q1) {
    mpz_t num; mpz_t den;
    mpz_init(num); mpz_init(den);
    mpq_get_num(num,q1._mpq);
    mpq_get_den(den,q1._mpq);
    write(os,num);
    if(mpz_cmp_si(den,1)!=0) { os << "/"; write(os,den); }
    mpz_clear(num); mpz_clear(den);
    return os;
}

struct RoundExact { };
inline RoundExact opposite(RoundExact) { return RoundExact(); }
template<class Y> inline decltype(auto) sqr(RoundExact, Y const& y) { return sqr(y); }
template<class Y> inline decltype(auto) rec(RoundExact, Y const& y) { return rec(y); }
template<class Y1, class Y2> inline decltype(auto) mul(RoundExact, Y1 const& y1, Y2 const& y2) { return y1*y2; }
template<class Y1, class Y2> inline decltype(auto) div(RoundExact, Y1 const& y1, Y2 const& y2) { return y1/y2; }

template<class RNDUP, class Y> inline auto _mul(RNDUP up, Bounds<Y> const& y1, Bounds<Y> const& y2) -> Bounds<Y>
{
    auto down=opposite(up);
    const Y& y1l=y1.lower_raw(); const Y& y1u=y1.upper_raw();
    const Y& y2l=y2.lower_raw(); const Y& y2u=y2.upper_raw();
    if(y1l>=0) {
        if(y2l>=0) {
            return Bounds<Y>(mul(down,y1l,y2l),mul(up,y1u,y2u));
        } else if(y2u<=0) {
            return Bounds<Y>(mul(down,y1u,y2l),mul(up,y1l,y2u));
        } else {
            return Bounds<Y>(mul(down,y1u,y2l),mul(up,y1u,y2u));
        }
    }
    else if(y1u<=0) {
        if(y2l>=0) {
            return Bounds<Y>(mul(down,y1l,y2u),mul(up,y1u,y2l));
        } else if(y2u<=0) {
            return Bounds<Y>(mul(down,y1u,y2u),mul(up,y1l,y2l));
        } else {
            return Bounds<Y>(mul(down,y1l,y2u),mul(up,y1l,y2l));
        }
    } else {
        if(y2l>=0) {
            return Bounds<Y>(mul(down,y1l,y2u),mul(up,y1u,y2u));
        } else if(y2u<=0) {
            return Bounds<Y>(mul(down,y1u,y2l),mul(up,y1l,y2l));
        } else {
            return Bounds<Y>(min(mul(down,y1u,y2l),mul(down,y1l,y2u)),max(mul(up,y1l,y2l),mul(up,y1u,y2u)));
        }
    }
}

template<class RNDUP, class Y> inline auto _div(RNDUP up, Bounds<Y> const& y1, Bounds<Y> const& y2) -> Bounds<Y>
{
    auto down=opposite(up);
    const Y& y1l=y1.lower_raw(); const Y& y1u=y1.upper_raw();
    const Y& y2l=y2.lower_raw(); const Y& y2u=y2.upper_raw();
    // IMPORTANT: Need to be careful when one of the bounds is 0, since if y2l=-0.0 and y1u>0, then y2l>=0 but y1u/y2l=-inf
    if(y2l>0) {
        if(y1l>=0) {
            return Bounds<Y>(div(down,y1l,y2u),div(up,y1u,y2l));
        } else if(y1u<=0) {
            return Bounds<Y>(div(down,y1l,y2l),div(up,y1u,y2u));
        } else {
            return Bounds<Y>(div(down,y1l,y2l),div(up,y1u,y2l));
        }
    }
    else if(y2u<0) {
        if(y1l>=0) {
            return Bounds<Y>(div(down,y1u,y2u),div(up,y1l,y2l));
        } else if(y1u<=0) {
            return Bounds<Y>(div(down,y1u,y2l),div(up,y1l,y2u));
        } else {
            return Bounds<Y>(div(down,y1u,y2u),div(up,y1l,y2u));
        }
    }
    else {
        //ARIADNE_THROW(DivideByZeroException,"FloatBounds div(FloatBounds y1, FloatBounds y2)","y1="<<y1<<", y2="<<y2);
        if constexpr(HasPrecisionType<Y>) {
            auto pr=max(y1.precision(),y2.precision());
            return Bounds<Y>(-Y::inf(pr),+Y::inf(pr));
        } else {
            return Bounds<Y>(-Y::inf(),+Y::inf());
        }
    }
}

template<class RNDUP, class Y> Bounds<Y> _sqr(RNDUP up, Bounds<Y> const& y) {
    auto down=opposite(up);
    const Y& yl=y.lower_raw(); const Y& yu=y.upper_raw();
    if(yl>0) {
        return Bounds<Y>(sqr(down,yl),sqr(up,yu));
    } else if(yu<0) {
        return Bounds<Y>(sqr(down,yu),sqr(up,yl));
    } else {
        return Bounds<Y>(nul(yl),max(sqr(up,yl),sqr(up,yu)));
    }
}

template<class RNDUP, class Y> Bounds<Y> _rec(RNDUP up, Bounds<Y> const& y) {
    auto down=opposite(up);
    const Y& yl=y.lower_raw(); const Y& yu=y.upper_raw();
    if(yl>0 || yu<0) {
        return Bounds<Y>(rec(down,yu),rec(up,yl));
    } else {
        if constexpr(HasPrecisionType<Y>) {
            auto pr=y.precision();
            return Bounds<Y>(-Y::inf(pr),+Y::inf(pr));
        } else {
            return Bounds<Y>(-Y::inf(),+Y::inf());
        }
    }
}

RationalBounds operator*(RationalBounds const& q1, RationalBounds const& q2) {
    return _mul(RoundExact(),q1,q2); }
RationalBounds operator/(RationalBounds const& q1, RationalBounds const& q2) {
    return _div(RoundExact(),q1,q2); }


RationalBounds nul(RationalBounds const& q) {
    return RationalBounds(nul(q._l),nul(q._u)); }
RationalBounds pos(RationalBounds const& q) {
    return RationalBounds(pos(q._l),pos(q._u)); }
RationalBounds neg(RationalBounds const& q) {
    return RationalBounds(neg(q._u),neg(q._l)); }
RationalBounds hlf(RationalBounds const& q) {
    return RationalBounds(hlf(q._u),hlf(q._l)); }
RationalBounds sqr(RationalBounds const& q) {
    return _sqr(RoundExact(),q); }
RationalBounds rec(RationalBounds const& q) {
    return _rec(RoundExact(),q); }

RationalBounds add(RationalBounds const& q1, RationalBounds const& q2) {
    return RationalBounds(q1._l+q2._l,q1._u+q2._u); }
RationalBounds sub(RationalBounds const& q1, RationalBounds const& q2) {
    return RationalBounds(q1._l-q2._u,q1._u-q2._l); }
RationalBounds mul(RationalBounds const& q1, RationalBounds const& q2) {
    return _mul(RoundExact(),q1,q2); }
RationalBounds div(RationalBounds const& q1, RationalBounds const& q2) {
    return _div(RoundExact(),q1,q2); }
RationalBounds pow(RationalBounds const& q, Nat m) {
    RationalBounds r = (m%2==0) ? abs(q) : q;  Int n=static_cast<Int>(m); return RationalBounds(pow(q._l,n),pow(q._u,n)); }
RationalBounds pow(RationalBounds const& q, Int n) {
    if(n<0) { return pow(rec(q),Nat(-n)); } else return pow(q,Nat(n));}

static_assert(not Convertible<Int,PositiveRational>);
RationalBounds abs(RationalBounds const& q) {
    return RationalBounds(max(min(q._l,-q._u),0),max(-q._l,q._u)); }
RationalBounds max(RationalBounds const& q1, RationalBounds const& q2) {
    return RationalBounds(max(q1._l,q2._l),max(q1._u,q2._u)); }
RationalBounds min(RationalBounds const& q1, RationalBounds const& q2) {
    return RationalBounds(min(q1._l,q2._l),min(q1._u,q2._u)); }

ValidatedKleenean operator==(RationalBounds const& q1, RationalBounds const& q2) {
    if (q1._l >= q2._u && q1._u <= q2._l) { return true; } else if (q1._l > q2._u || q1._u < q2._l) { return false; } else { return indeterminate; } }
ValidatedKleenean operator!=(RationalBounds const& q1, RationalBounds const& q2) {
    if (q1._l > q2._u || q1._u < q2._l) { return true; } else if (q1._l >= q2._u && q1._u <= q2._l) { return false; } else { return indeterminate; } }
ValidatedKleenean operator<=(RationalBounds const& q1, RationalBounds const& q2) {
    if (q1._u <= q2._l) { return true; } else if (q1._l > q2._u) { return false; } else { return indeterminate; } }
ValidatedKleenean operator>=(RationalBounds const& q1, RationalBounds const& q2) {
    if (q1._l >= q2._u) { return true; } else if (q1._u < q2._l) { return false; } else { return indeterminate; } }
ValidatedKleenean operator< (RationalBounds const& q1, RationalBounds const& q2) {
    if (q1._u < q2._l) { return true; } else if (q1._l >= q2._u) { return false; } else { return indeterminate; } }
ValidatedKleenean operator> (RationalBounds const& q1, RationalBounds const& q2) {
    if (q1._l > q2._u) { return true; } else if (q1._u <= q2._l) { return false; } else { return indeterminate; } }

template<> String class_name<Rational>() { return "Rational"; }
template<> String class_name<RationalBounds>() { return "RationalBounds"; }


// TODO: Move to dyadic.cpp
DyadicBounds operator*(DyadicBounds const& w1, DyadicBounds const& w2) {
    return _mul(RoundExact(),w1,w2); }

DyadicBounds mul(DyadicBounds const& w1, DyadicBounds const& w2) {
    return _mul(RoundExact(),w1,w2); }

    DyadicBounds pow(DyadicBounds const& w, Int n) {
    assert(n>=0);
    return pow(w,static_cast<Nat>(n)); }

static_assert(Field<Rational>);
static_assert(Lattice<Rational>);

} // namespace Ariadne
