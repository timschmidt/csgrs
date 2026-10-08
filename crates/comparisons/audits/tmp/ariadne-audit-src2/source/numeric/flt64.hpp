/***************************************************************************
 *            numeric/flt64.hpp
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

/*! \file numeric/flt64.hpp
 *  \brief
 */

#ifndef ARIADNE_FLT64_HPP
#define ARIADNE_FLT64_HPP

#include <cmath>
#include <iostream>

#include "utility/metaprogramming.hpp"
#include "utility/typedefs.hpp"
#include "foundations/paradigm.hpp"

#include "numeric/sign.hpp"
#include "numeric/is_number.hpp"

#include "foundations/logical.decl.hpp"
#include "numeric/float.decl.hpp"
#include "numeric/number.hpp"
#include "numeric/real.hpp"
#include "foundations/logical.hpp"

namespace Ariadne {

/************ Flt64 ********************************************************/

Void set_default_rounding();

Void FloatDP::set_rounding_to_nearest();
Void FloatDP::set_rounding_downward();
Void FloatDP::set_rounding_upward();
Void FloatDP::set_rounding_toward_zero();

typedef unsigned short RoundingModeType;
RoundingModeType FloatDP::get_rounding_mode();
Void FloatDP::set_rounding_mode(RoundingModeType);

extern const RoundingModeType ROUND_NEAR;
extern const RoundingModeType ROUND_DOWN;
extern const RoundingModeType ROUND_UP;
extern const RoundingModeType ROUND_ZERO;

// ----------------- RawTag FloatDP class ---------------------------------------------------------

template<BuiltinIntegral Z> Z integer_cast(Flt64);

// ----------------- RawTag FloatDP class ---------------------------------------------------------

//! \ingroup FloatDPSubModule
//! \brief Wrapper class for double-precision floating-point numbers.
//! \details
//! The main purpose of this class is as a base implementation for the \em safe floating-point number types
//! FloatDPApproximation, FloatDPLowerBound, FloatDPUpperBound,FloatDPBounds, FloatDPBall, FloatDPError and FloatDP.
//! These classes
//!
//! Default arithmetic operations are approximate, and comparisons are exact, so this class is \em unsafe.
//! %Rounded operations on \b FloatDP classes are provided using the \c _down
//! and \c _up suffixes. %ApproximateTag arithmetic without control of the
//! rounding mode can be specified by the \c _approx rounding suffix, or \c _near if round-to-nearest is available.
//! %ExactTag arithmetic can be specified explicitly using the \c _exact rounding suffix.
//!
//! Since the raw \b FloatDP classes represent unsafe values, they cannot be converted to other numbers.
//! The exception is conversion to \b FloatDPApproximation classes, since these represent approximate values with no control of the error.
//! Since the raw \b FloatDP classes are used as the implementation of the safe \c FloatDP classes,
//! they are valid arguments to constructors, but these constructors are all declared \c explicit.
//! Comparison of raw \b %FloatDP data and other numbers is performed as if the %FloatDP object were an \b %FloatDPApproximation.
//!
//! When testing, it is often useful to perform comparisons with \c %Flt64 or \c double values.
//! Although care must be taken, since compiler rounding may change the truth of certain comparisons.
//! comparisons with \c double are performed as if the value were \c FloatDP.


class Flt64 {
  public:
    volatile double d;
  public:
    typedef RawTag Paradigm;
    typedef Flt64 NumericType;
    Flt64() : d(0.0) { }
    Flt64(double dbl) : d(dbl) { }
    explicit operator double() const { return d; }
    Flt64 raw() const { return *this; }
    Flt64 get_flt() const { return *this; }
    double get_d() const { return d; }

    static Flt64 eps();
    static Flt64 inf();

    static Void FloatDP::set_rounding_mode(RoundingModeType);
    static RoundingModeType FloatDP::get_rounding_mode();

    static Void FloatDP::set_rounding_to_nearest();
    static Void FloatDP::set_rounding_downward();
    static Void FloatDP::set_rounding_upward();
    static Void FloatDP::set_rounding_toward_zero();

    friend inline Flt64 operator+(Flt64 x) { return Flt64{+x.d}; }
    friend inline Flt64 operator-(Flt64 x) { return Flt64{-x.d}; }
    friend inline Flt64 operator+(Flt64 x1, Flt64 x2) { return Flt64{x1.d+x2.d}; }
    friend inline Flt64 operator-(Flt64 x1, Flt64 x2) { return Flt64{x1.d-x2.d}; }
    friend inline Flt64 operator*(Flt64 x1, Flt64 x2) { return Flt64{x1.d*x2.d}; }
    friend inline Flt64 operator/(Flt64 x1, Flt64 x2) { return Flt64{x1.d/x2.d}; }
    friend inline Flt64 operator/(Flt64 x1, Int n2) { return Flt64{x1.d/n2}; }
    friend inline Flt64& operator+=(Flt64& x1, Flt64 x2) { x1.d+=x2.d; return x1; }
    friend inline Flt64& operator-=(Flt64& x1, Flt64 x2) { x1.d-=x2.d; return x1; }
    friend inline Flt64& operator*=(Flt64& x1, Flt64 x2) { x1.d*=x2.d; return x1; }
    friend inline Flt64& operator/=(Flt64& x1, Flt64 x2) { x1.d/=x2.d; return x1; }

    friend inline Flt64 min(Flt64 x1, Flt64 x2) { return Flt64{x2.d<x1.d?x2.d:x1.d}; }
    friend inline Flt64 max(Flt64 x1, Flt64 x2) { return Flt64{x2.d>x1.d?x2.d:x1.d}; }
    friend inline Flt64 abs(Flt64 x) { return Flt64{std::fabs(x.d)}; }
    friend inline Flt64 mig(Flt64 x) { return abs(x); }
    friend inline Flt64 mag(Flt64 x) { return abs(x); }

    friend inline Flt64 add(Flt64 x1, Flt64 x2) { return Flt64{x1.d+x2.d}; }
    friend inline Flt64 sub(Flt64 x1, Flt64 x2) { return Flt64{x1.d-x2.d}; }
    friend inline Flt64 mul(Flt64 x1, Flt64 x2) { return Flt64{x1.d*x2.d}; }
    friend inline Flt64 div(Flt64 x1, Flt64 x2) { return Flt64{x1.d/x2.d}; }

    friend inline Flt64 pos(Flt64 x) { return Flt64{x.d}; }
    friend inline Flt64 hlf(Flt64 x) { return Flt64{x.d/2}; }
    friend inline Flt64 sqr(Flt64 x) { return Flt64{x.d*x.d}; }

    friend inline Flt64 neg(Flt64 x) { return Flt64{-x.d}; }
    friend inline Flt64 rec(Flt64 x) { return Flt64{1.0/x.d}; }

    friend inline Flt64 pow(Flt64 x, Nat m) { return std::pow(x.d,m); }
    friend inline Flt64 pow(Flt64 x, Int n) { return std::pow(x.d,n); }
    friend inline Flt64 sqrt(Flt64 x) { return std::sqrt(x.d); }
    friend inline Flt64 exp(Flt64 x) { return std::exp(x.d); }
    friend inline Flt64 log(Flt64 x) { return std::log(x.d); }
    friend inline Flt64 sin(Flt64 x) { return std::sin(x.d); }
    friend inline Flt64 cos(Flt64 x) { return std::cos(x.d); }
    friend inline Flt64 tan(Flt64 x) { return std::tan(x.d); }
    friend inline Flt64 asin(Flt64 x) { return std::asin(x.d); }
    friend inline Flt64 acos(Flt64 x) { return std::acos(x.d); }
    friend inline Flt64 atan(Flt64 x) { return std::atan(x.d); }

    friend inline Int32Type floor(Flt64 x) { return int32_t(std::floor(x.d)); }
    friend inline Int32Type ceil(Flt64 x) { return int32_t(std::ceil(x.d)); }

    friend Bool isnan(Flt64 x);

    friend inline OutputStream& operator<<(OutputStream& os, Flt64 x) { return os << x.get_d(); }
    friend inline InputStream& operator>>(InputStream& is, Flt64& x) { return is >> reinterpret_cast<double&>(x); }

    friend inline Comparison cmp(Flt64 x1, Flt64 x2) {
        return (x1.d==x2.d)?Comparison::EQUAL:(x1.d>x2.d)?Comparison::GREATER:Comparison::LESS; }
    friend inline Bool operator==(Flt64 x1, Flt64 x2) { return x1.d==x2.d; }
    friend inline Bool operator!=(Flt64 x1, Flt64 x2) { return !(x1==x2); }
    friend inline Bool operator<=(Flt64 x1, Flt64 x2) { return x1.d<=x2.d; }
    friend inline Bool operator>=(Flt64 x1, Flt64 x2) { return (x2<=x1); }
    friend inline Bool operator< (Flt64 x1, Flt64 x2) { return !(x2<=x1); }
    friend inline Bool operator> (Flt64 x1, Flt64 x2) { return !(x1<=x2); }

    friend inline Bool operator==(Flt64 x1, double x2) { return x1.d==x2; }
    friend inline Bool operator!=(Flt64 x1, double x2) { return !(x1==x2); }
    friend inline Bool operator<=(Flt64 x1, double x2) { return x1.d<=x2; }
    friend inline Bool operator>=(Flt64 x1, double x2) { return (x2<=x1); }
    friend inline Bool operator< (Flt64 x1, double x2) { return !(x2<=x1); }
    friend inline Bool operator> (Flt64 x1, double x2) { return !(x1<=x2); }

};

inline Flt64 Flt64::inf() { return Flt64{1.0/0.0}; }

/************ Rounded arithmetic of Flt64 ***************************************************/

Flt64 hlf(exact,Flt64);

Flt64 next(down,Flt64);
Flt64 next(up,Flt64);

Flt64 add(near,Flt64, Flt64);
Flt64 add(down,Flt64, Flt64);
Flt64 add(up,Flt64, Flt64);

Flt64 sub(near,Flt64, Flt64);
Flt64 sub(down,Flt64, Flt64);
Flt64 sub(up,Flt64, Flt64);

Flt64 mul(near,Flt64, Flt64);
Flt64 mul(down,Flt64, Flt64);
Flt64 mul(up,Flt64, Flt64);

Flt64 div(near,Flt64, Flt64);
Flt64 div(down,Flt64, Flt64);
Flt64 div(up,Flt64, Flt64);

Flt64 rad(up,Flt64, Flt64);
Flt64 med(near,Flt64, Flt64);

Flt64 pow(approx,Flt64, Int);
Flt64 pow(down,Flt64, Int);
Flt64 pow(up,Flt64, Int);

Flt64 sqrt(approx,Flt64);
Flt64 sqrt(down,Flt64);
Flt64 sqrt(up,Flt64);
Flt64 exp(approx,Flt64);
Flt64 exp(down,Flt64);
Flt64 exp(up,Flt64);
Flt64 log(approx,Flt64);
Flt64 log(down,Flt64);
Flt64 log(up,Flt64);
Flt64 sin(approx,Flt64);
Flt64 sin(down,Flt64);
Flt64 sin(up,Flt64);
Flt64 cos(approx,Flt64);
Flt64 cos(down,Flt64);
Flt64 cos(up,Flt64);
Flt64 tan(approx,Flt64);
Flt64 tan(down,Flt64);
Flt64 tan(up,Flt64);
Flt64 atan(approx,Flt64);
Flt64 atan(down,Flt64);
Flt64 atan(up,Flt64);

// Arithmetic respecting the rounding mode
Flt64 add_rnd(Flt64 x, Flt64 y);
Flt64 sub_rnd(Flt64 x, Flt64 y);
Flt64 mul_rnd(Flt64 x, Flt64 y);
Flt64 div_rnd(Flt64 x, Flt64 y);
Flt64 add_opp(Flt64 x, Flt64 y);
Flt64 sub_opp(Flt64 x, Flt64 y);
Flt64 mul_opp(Flt64 x, Flt64 y);
Flt64 div_opp(Flt64 x, Flt64 y);

} // namespace Ariadne

#endif
