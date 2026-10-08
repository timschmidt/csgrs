/***************************************************************************
 *            numeric/dyadic.cpp
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

/*! \file numeric/dyadic.cpp
 *  \brief
 */


#include "utility/stdlib.hpp"

#include "dyadic.hpp"

#include "utility/macros.hpp"
#include "utility/string.hpp"
#include "foundations/logical.hpp"
#include "numeric/twoexp.hpp"
#include "numeric/builtin.hpp"
#include "numeric/decimal.hpp"
#include "numeric/rational.hpp"
#include "numeric/extended.hpp"

#include "numeric/concepts.hpp"

#include <limits>

namespace Ariadne {

template<class X> class FiniteOperations;
template<class X> class ExtensionOperations;

template<> class ExtensionOperations<Dyadic> {
    static mp_exp_t const nan_flag = std::numeric_limits<mp_exp_t>::min();

  public:
    static Bool is_nan(Dyadic const& x) { return x._mpf[0]._mp_size==0 and x._mpf[0]._mp_exp==nan_flag; }
    static Bool is_inf(Dyadic const& x) { return x._mpf[0]._mp_size==0 and std::abs(x._mpf[0]._mp_exp)==1; }
    static Bool is_finite(Dyadic const& x) { return x._mpf[0]._mp_size!=0 || x._mpf[0]._mp_exp==0; }
    static Bool is_zero(Dyadic const& x) { return x._mpf[0]._mp_size==0 && x._mpf[0]._mp_exp==0; }

    static Sign sgn(Dyadic const& x) {
        if (is_finite(x)) { return static_cast<Sign>(mpf_cmp_si(x._mpf,0)); }
        else { return (x._mpf[0]._mp_exp==nan_flag) ? Sign::ZERO : (x._mpf[0]._mp_exp>=0) ? Sign::POSITIVE : Sign::NEGATIVE; } }

    static Void set_nan(Dyadic& x) { x._mpf[0]._mp_size=0; x._mpf[0]._mp_exp=nan_flag; }
    static Void set_inf(Dyadic& x, Sign s) { x._mpf[0]._mp_size=0;
        x._mpf[0]._mp_exp = (s==Sign::ZERO ? nan_flag : s==Sign::POSITIVE ? +1 : -1); }
    static Void set_zero(Dyadic& x) { mpf_set_si(x._mpf,0); }
};

template<> class FiniteOperations<Dyadic> {
    friend class ExtendedOperations<Dyadic>;

    static Void set(Dyadic& r, Dyadic const& x) { mpf_set(r._mpf,x._mpf); }

    static Void add(Dyadic& r, Dyadic const& x1, Dyadic const& x2) { return mpf_add(r._mpf, x1._mpf, x2._mpf); }
    static Void sub(Dyadic& r, Dyadic const& x1, Dyadic const& x2) { return mpf_sub(r._mpf, x1._mpf, x2._mpf); }
    static Void mul(Dyadic& r, Dyadic const& x1, Dyadic const& x2) { return mpf_mul(r._mpf, x1._mpf, x2._mpf); }
    static Void div(Dyadic& r, Dyadic const& x1, Dyadic const& x2) { return mpf_div(r._mpf, x1._mpf, x2._mpf); }

    static Void pos(Dyadic& r, Dyadic const& x) { mpf_set(r._mpf,x._mpf); }
    static Void neg(Dyadic& r, Dyadic const& x) { mpf_neg(r._mpf,x._mpf); }
    static Void hlf(Dyadic& r, Dyadic const& x) { mpf_div_2exp(r._mpf,x._mpf,1u); }
    static Void rec(Dyadic& r, Dyadic const& x) { assert(false); }
    static Void pow(Dyadic& r, Dyadic const& x, Nat m) { return mpf_pow_ui(r._mpf, x._mpf, m); }

    static Void max(Dyadic& r, Dyadic const& x1, Dyadic const& x2) {
        if(mpf_cmp(x1._mpf,x2._mpf)>=0) { mpf_set(r._mpf,x1._mpf); } else { mpf_set(r._mpf,x2._mpf); } }
    static Void min(Dyadic& r, Dyadic const& x1, Dyadic const& x2) {
        if(mpf_cmp(x1._mpf,x2._mpf)<=0) { mpf_set(r._mpf,x1._mpf); } else { mpf_set(r._mpf,x2._mpf); } }
    static Void abs(Dyadic& r, Dyadic const& x) { mpf_abs(r._mpf,x._mpf); }

    static Comparison cmp(Dyadic const& x1, Dyadic const& x2) { return static_cast<Comparison>(mpf_cmp(x1._mpf,x2._mpf)); }
};


static const mp_bitcnt_t maximum_precision = 65535;

const Dyadic infty = Dyadic::inf();

Dyadic::~Dyadic() {
    mpf_clear(_mpf);
}

Dyadic::Dyadic() {
    mpf_init2(_mpf,maximum_precision);
}

Dyadic::Dyadic(mpf_t mpf) {
    mpf_init2(_mpf,maximum_precision);
    mpf_set(_mpf,_mpf);
}

Dyadic::Dyadic(Integer const& p, Natural q) {
    ARIADNE_ASSERT(q.get_si()==q);
    mpf_init2(_mpf,maximum_precision);
    mpf_set_z(_mpf,p._mpz);
    mpf_div_2exp(_mpf,_mpf,static_cast<mp_bitcnt_t>(q.get_si()));
    //if(q>=0) { mpf_div_2exp(_mpf,_mpf,q); } else { mpf_mul_2exp(_mpf,_mpf,-q); }
}

Dyadic::Dyadic(Integer const& p, Nat q) {
    mpf_init2(_mpf,maximum_precision);
    mpf_set_z(_mpf,p._mpz);
    mpf_div_2exp(_mpf,_mpf,q);
    //if(q>=0) { mpf_div_2exp(_mpf,_mpf,q); } else { mpf_mul_2exp(_mpf,_mpf,-q); }
}

Dyadic::Dyadic(Dbl d) : Dyadic(ExactDouble(d)) {
}

Dyadic::Dyadic(ExactDouble const& x) {
    Dbl d=x.get_d();
    mpf_init2(_mpf,maximum_precision);
    if(std::isfinite(d)) {
        mpf_set_d(_mpf,d);
    } else if(std::isnan(d)) {
        *this = Dyadic::nan();
    } else {
        *this = (d>0) ? Dyadic::inf() : -Dyadic::inf();
    }
}

Dyadic::Dyadic(Integer const& z) {
    mpf_init2(_mpf,maximum_precision);
    mpf_set_z(_mpf,z._mpz);
}

Dyadic::Dyadic(TwoExp const& w) : Dyadic(1u) {
    const int q=w.exponent();
    if(q>=0) { mpf_mul_2exp(_mpf,_mpf,static_cast<mp_bitcnt_t>(q)); }
    else { mpf_div_2exp(_mpf,_mpf,static_cast<mp_bitcnt_t>(-q)); }
}

Dyadic::Dyadic(String const& str)
{
    mpf_t _mpf_tmp;
    mp_bitcnt_t prec=std::ceil(3+str.size()*3.322265625);
    mpf_init2(_mpf_tmp,prec);
    int fail = mpf_set_str(_mpf_tmp,str.c_str(),10);
    if (fail!=0) {
        ARIADNE_THROW(std::runtime_error,"Dyadic(string)","String \""<<str<<"\" does not have a valid dyadic number format.");
    }
    mpf_init2(_mpf,maximum_precision);
    mpf_set(_mpf,_mpf_tmp);

    if (Decimal(*this)!=Decimal(str)) {
        ARIADNE_THROW(std::runtime_error,"Dyadic(string)","String \""<<str<<"\" does not represent an exact dyadic number.");
    }
}

Dyadic::Dyadic(const Dyadic& x) {
    mpf_init2(_mpf,maximum_precision);
    mpf_set(_mpf,x._mpf);
}

Dyadic::Dyadic(Dyadic&& x) {
    mpf_init(_mpf);
    mpf_swap(_mpf,x._mpf);
}

Dyadic& Dyadic::operator=(const Dyadic& x) {
    mpf_set(_mpf,x._mpf);
    return *this;
}

Dyadic Dyadic::inf(Sign sgn) {
    Dyadic x; ExtensionOperations<Dyadic>::set_inf(x,sgn); return x;
}

Dyadic Dyadic::inf() {
    Dyadic x; ExtensionOperations<Dyadic>::set_inf(x,Sign::POSITIVE); return x;
}

Dyadic Dyadic::nan() {
    Dyadic x; ExtensionOperations<Dyadic>::set_nan(x); return x;
}

Bool is_nan(const Dyadic& x) {
    return ExtensionOperations<Dyadic>::is_nan(x);
}

Bool is_inf(const Dyadic& x) {
    return ExtensionOperations<Dyadic>::is_inf(x);
}

Bool is_finite(const Dyadic& x) {
    return ExtensionOperations<Dyadic>::is_finite(x);
}

Bool is_zero(const Dyadic& x) {
    return ExtensionOperations<Dyadic>::is_zero(x);
}

Dyadic& Dyadic::operator=(Dyadic&& x) {
    mpf_swap(_mpf,x._mpf);
    return *this;
}

Integer Dyadic::mantissa() const {
    return Rational(*this).numerator();
}

Int Dyadic::exponent() const {
    return log2floor(Rational(*this).denominator());
}

mpf_t const& Dyadic::get_mpf() const {
    return this->_mpf;
}

double Dyadic::get_d() const {
    if (is_finite(*this))
        return mpf_get_d(this->_mpf);
    else if (is_nan(*this)) {
        return std::numeric_limits<double>::quiet_NaN();
    } else {
        return (sgn(*this) == Sign::POSITIVE) ? std::numeric_limits<double>::infinity() : -std::numeric_limits<double>::infinity();
    }
}

Dyadic operator+(TwoExp y) {
    return +Dyadic(y);
}

Dyadic operator-(TwoExp y) {
    return -Dyadic(y);
}

Dyadic operator*(Dyadic x, TwoExp y) {
    const int q=y.exponent();
    if(q>=0) { mpf_mul_2exp(x._mpf,x._mpf,static_cast<mp_bitcnt_t>(q)); }
    else { mpf_div_2exp(x._mpf,x._mpf,static_cast<mp_bitcnt_t>(-q)); }
    return x;
}

Dyadic operator/(Dyadic x, TwoExp y) {
    return x*rec(y);
}

Dyadic operator/(Int n, Two) {
    return Dyadic(n,1u);
}

Dyadic operator/(Integer z, Two) {
    return Dyadic(z,1u);
}

Dyadic operator/(ExactDouble d, Two) {
    return Dyadic(d)/two;
}

Dyadic operator/(Dyadic x, Two) {
    return x*(two^(-1));
}

OutputStream& operator<<(OutputStream& os, TwoExp w) {
    return os << "2^" <<  w.exponent();
}

Dyadic operator+(Dyadic& x1, Dyadic const& x2) {
    Dyadic r; ExtendedOperations<Dyadic>::add(r,x1,x2); return r;
}

Dyadic operator-(Dyadic& x1, Dyadic const& x2) {
    Dyadic r; ExtendedOperations<Dyadic>::sub(r,x1,x2); return r;
}

Dyadic operator*(Dyadic& x1, Dyadic const& x2) {
    Dyadic r; ExtendedOperations<Dyadic>::mul(r,x1,x2); return r;
}

Integer round(Dyadic const& x) {
    assert(is_finite(x));
    Integer z;
    Dyadic y=x;
    mpf_mul_2exp(y._mpf,y._mpf,1u);
    if(y>=0) {
        mpf_add_ui(y._mpf,y._mpf,1u);
    } else {
        mpf_sub_ui(y._mpf,y._mpf,1u);
    }
    mpf_div_2exp(y._mpf,y._mpf,1u);
    // mpf_trunc(y._mpf,y._mpf); // Not needed since mpf_set_f truncates
    mpz_set_f(z._mpz,y._mpf);
    return z;
}

Integer floor(Dyadic const& x) {
    assert(is_finite(x));
    Integer z;
    Dyadic y(x);
    mpf_floor(y._mpf,y._mpf);
    mpz_set_f(z._mpz,y._mpf);
    return z;
}

Integer ceil(Dyadic const& x) {
    assert(is_finite(x));
    Integer z;
    Dyadic y(x);
    mpf_ceil(y._mpf,y._mpf);
    mpz_set_f(z._mpz,y._mpf);
    return z;
}


Dyadic nul(Dyadic const& x) {
    Dyadic r; mpf_set_si(r._mpf,0); return r;
}

Dyadic pos(Dyadic const& x) {
    return x;
}

Dyadic neg(Dyadic const& x) {
    Dyadic r; ExtendedOperations<Dyadic>::neg(r,x); return r;
}

Positive<Dyadic> sqr(Dyadic const& x) {
    return cast_positive(x*x);
}

Dyadic add(Dyadic const& x1, Dyadic const& x2) {
    Dyadic r; ExtendedOperations<Dyadic>::add(r,x1,x2); return r;
}

Dyadic sub(Dyadic const& x1, Dyadic const& x2) {
    Dyadic r; ExtendedOperations<Dyadic>::sub(r,x1,x2); return r;
}

Dyadic mul(Dyadic const& x1, Dyadic const& x2) {
    Dyadic r; ExtendedOperations<Dyadic>::mul(r,x1,x2); return r;
}

Dyadic hlf(Dyadic const& x) {
    Dyadic r; ExtendedOperations<Dyadic>::hlf(r,x); return r;
}


/*
Dyadic pow(Dyadic const& x, Int n) {
    assert(n >= 0);
    return pow(x,static_cast<Nat>(n));
}
*/

Dyadic pow(Dyadic const& x, Nat m) {
    Dyadic r; ExtendedOperations<Dyadic>::pow(r,x,m); return r;
}

Rational pow(Dyadic const& x, Int n) {
    return n>=0 ? pow(x,Nat(n)) : rec(pow(x,Nat(-n)));
}


Positive<Dyadic> abs(Dyadic const& x) {
    Positive<Dyadic> r; ExtendedOperations<Dyadic>::abs(r,x); return r;
}

Dyadic min(Dyadic const& x1,Dyadic const& x2) {
    Dyadic r; ExtendedOperations<Dyadic>::min(r,x1,x2); return r;
}

Dyadic max(Dyadic const& x1,Dyadic const& x2) {
    Dyadic r; ExtendedOperations<Dyadic>::max(r,x1,x2); return r;
}


Comparison cmp(Dyadic const& x1, Dyadic const& x2) {
    return ExtendedOperations<Dyadic>::cmp(x1,x2);
}

Comparison cmp(Dyadic const& x1, Int const& n2) {
    return ExtendedOperations<Dyadic>::cmp(x1,n2);
}

Comparison cmp(Integer const& z1, Int const& n2);

Sign sgn(Dyadic const& x) {
    return ExtensionOperations<Dyadic>::sgn(x);
}

Boolean eq(Dyadic const& x1, Dyadic const& x2) {
    return cmp(x1,x2)==Comparison::EQUAL;
}

Boolean lt(Dyadic const& x1, Dyadic const& x2) {
    return cmp(x1,x2)==Comparison::LESS;
}

Writer<Dyadic> Dyadic::_default_writer(new DecimalWriter());

OutputStream& operator<<(OutputStream& os, Dyadic const& x) {
    return os << Dyadic::_default_writer(x);
}

String Dyadic::literal() const {
    DecimalWriter writer;
    StringStream ss;
    ss << writer(*this);
    return ss.str();
}

template<class X> inline OutputStream& write_infinite(OutputStream& os, X const& x) {
    if(is_nan(x)) {
        os << "NaN";
    } else {
        os << (sgn(x)==Sign::POSITIVE ? "" : "-") << "inf";
    }
    return os;
}

auto DecimalWriter::_write(OutputStream& os, Dyadic const& x) const -> OutputStream& {
    if(is_finite(x)) {
        Dyadic w=x;
        if(w<0) { os << "-"; w=-w; }
        Integer z=floor(w);
        os << z << ".";
        w-=z;
        while (w!=0) {
            w*=10;
            z=floor(w);
            w-=z;
            os << z;
        }
    } else {
        write_infinite(os,x);
    }
    return os;
}

auto ScientificWriter::_write(OutputStream& os, Dyadic const& x) const -> OutputStream& {
    if(is_finite(x)) {
        Dyadic w=x;
        if(w<0) { os << "-"; w=-w; }
        int e=0;
        while (w>=2) { w=hlf(w); e=e+1; }
        while (w<1) { w=w*2; e=e-1; }
        os << DecimalWriter()(w);
        os << "*2^" << e;
    } else {
        write_infinite(os,x);
    }
    return os;
}


auto FractionWriter::_write(OutputStream& os, Dyadic const& x) const -> OutputStream& {
    if(is_finite(x)) {
        Rational q;
        mpq_set_f (q._mpq,x._mpf);
        os << q.numerator();
        Int exp = log2floor(q.denominator());
        if (exp!=0) { if(exp==1) { os << "/2"; } else { os << "/2^" << exp; } }
    } else {
        write_infinite(os,x);
    }
    return os;
}

auto RepresentationWriter<Dyadic>::_write(OutputStream& os, Dyadic const& x) const -> OutputStream& {
    return os << "Dyadic(" << x.mantissa() << "," << x.exponent() << "u)";
}

Dyadic make_dyadic(unsigned long long int n) {
    static const unsigned int max=std::numeric_limits<Int>::max();
    static const unsigned long long int m=max;
    unsigned long long int q = n / m;
    unsigned long long int r = n % m;
    unsigned int rem = static_cast<unsigned int>(r);
    assert(n==q*m+r);
    if(q==0) {
        return Dyadic(rem);
    } else {
        return make_dyadic(q)*Dyadic(max)+Dyadic(rem);
    }
}

template<> String class_name<Dyadic>() { return "Dyadic"; }
template<> String class_name<DyadicBounds>() { return "DyadicBounds"; }


DyadicBounds pow(DyadicBounds const& w, Nat m) {
    DyadicBounds y = (m%2==0) ? abs(w) : w;  return DyadicBounds(pow(y._l,m),pow(y._u,m)); }

Dyadic hlf(Integer const& n) {
    return hlf(Dyadic(n));
}

struct RoundExact { };
inline RoundExact opposite(RoundExact) { return RoundExact(); }
template<class Y1, class Y2> inline decltype(auto) mul(RoundExact, Y1 const& y1, Y2 const& y2) { return y1*y2; }

template<class RNDUP, class Y> auto _mul(RNDUP up, Bounds<Y> const& y1, Bounds<Y> const& y2) -> Bounds<Y>;
template<class RNDUP, class Y> auto _div(RNDUP up, Bounds<Y> const& y1, Bounds<Y> const& y2) -> Bounds<Y>;


static_assert(DyadicRing<Dyadic>);
static_assert(OrderedLattice<Dyadic>);

} // namespace Ariadne
