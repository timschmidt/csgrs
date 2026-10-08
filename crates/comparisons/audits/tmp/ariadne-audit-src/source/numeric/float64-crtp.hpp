/***************************************************************************
 *            numeric/float64-crtp.hpp
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

/*! \file numeric/float64-crtp.hpp
 *  \brief
 */



#ifndef ARIADNE_FLOAT64_HPP
#define ARIADNE_FLOAT64_HPP

#include "utility/module.hpp"
#include "utility/metaprogramming.hpp"
#include "foundations/paradigm.hpp"
#include "numeric/sign.hpp"

namespace Ariadne {

/************ FloatDP ********************************************************/

struct ExactTag { };

class Boolean;
class ValidatedKleenean;
class ValidatedSierpinskian;
class Fuzzy;

class FloatDPApproximation;
class FloatDPLowerBound;
class FloatDPUpperBound;
class ValidFloatDP;
class FloatDPBounds;
class FloatDPError;
class FloatDP;

class Rational;

FloatDP operator"" _x(long double);
FloatDPError operator"" _e(long double);
FloatDPLowerBound operator"" _l(long double);
FloatDPUpperBound operator"" _u(long double);
FloatDPApproximation operator"" _a(long double);

Void FloatDP::set_rounding_to_nearest();
Void FloatDP::set_rounding_downward();
Void FloatDP::set_rounding_upward();
Void FloatDP::set_rounding_toward_zero();

template<class N> inline N& operator+=(NumberObject<N>& n1, const NumberObject<N>& n2) {
    n1.upcast()=n1.upcast()+n2.upcast(); return n1.upcast(); }
template<class N> inline N& operator*=(NumberObject<N>& n1, const NumberObject<N>& n2) {
    return n1.upcast()=n1.upcast()*n2.upcast(); return n1.upcast(); }

class FloatDP : public ScalarObject<FloatDP> {
    volatile double d;
  public:
    typedef Void Paradigm;
    FloatDP() : d(0.0) { }
    FloatDP(double dbl) : d(dbl) { }
    explicit operator double() const { return d; }
    FloatDP get_flt() const { return *this; }
    double get_d() const { return d; }
    friend FloatDP operator+(FloatDP x);
    friend FloatDP operator-(FloatDP x);
    friend FloatDP operator+(FloatDP x1, FloatDP x2);
    friend FloatDP operator-(FloatDP x1, FloatDP x2);
    friend FloatDP operator*(FloatDP x1, FloatDP x2);
    friend FloatDP operator/(FloatDP x1, FloatDP x2);
    friend FloatDP operator/(FloatDP x1, Int n2);
    friend FloatDP& operator+=(FloatDP& x1, FloatDP x2);
    friend FloatDP& operator-=(FloatDP& x1, FloatDP x2);
    friend FloatDP& operator*=(FloatDP& x1, FloatDP x2);
    friend FloatDP& operator/=(FloatDP& x1, FloatDP x2);
    friend FloatDP max(FloatDP x1, FloatDP x2);
    friend FloatDP neg(FloatDP x);
    friend FloatDP abs(FloatDP x);
    friend Int32 floor(FloatDP x);
    friend Int32 ceil(FloatDP x);
    friend Sign sng(FloatDP x);
    friend Comparison cmp(FloatDP x1, FloatDP x2);
    friend Bool operator==(FloatDP x1, FloatDP x2);
    friend Bool operator<=(FloatDP x1, FloatDP x2);
    friend OutputStream& operator<<(OutputStream& os, FloatDP x);
};
inline FloatDP operator+(FloatDP x) { return FloatDP{+x.d}; }
inline FloatDP operator-(FloatDP x) { return FloatDP{-x.d}; }
inline FloatDP operator+(FloatDP x1, FloatDP x2) { return FloatDP{x1.d+x2.d}; }
inline FloatDP operator-(FloatDP x1, FloatDP x2) { return FloatDP{x1.d-x2.d}; }
inline FloatDP operator*(FloatDP x1, FloatDP x2) { return FloatDP{x1.d*x2.d}; }
inline FloatDP operator/(FloatDP x1, FloatDP x2) { return FloatDP{x1.d/x2.d}; }
inline FloatDP operator/(FloatDP x1, Int n2) { return FloatDP{x1.d/n2}; }
inline FloatDP& operator+=(FloatDP& x1, FloatDP x2) { x1.d+=x2.d; return x1; }
inline FloatDP& operator-=(FloatDP& x1, FloatDP x2) { x1.d-=x2.d; return x1; }
inline FloatDP& operator*=(FloatDP& x1, FloatDP x2) { x1.d*=x2.d; return x1; }
inline FloatDP& operator/=(FloatDP& x1, FloatDP x2) { x1.d/=x2.d; return x1; }
inline Comparison cmp(FloatDP x1, FloatDP x2) { return (x1.d==x2.d)?Comparison::EQUAL:(x1.d>x2.d)?Comparison::GREATER:Comparison::LESS; }
Bool isnan(FloatDP x);

FloatDP abs(FloatDP x);
inline FloatDP mig(FloatDP x) { return abs(x); }
inline FloatDP mag(FloatDP x) { return abs(x); }

inline FloatDP neg(FloatDP x) { return FloatDP{-x.d}; }

inline Bool operator==(FloatDP x1, FloatDP x2) { return x1.d==x2.d; }
inline Bool operator!=(FloatDP x1, FloatDP x2) { return !(x1==x2); }
inline Bool operator<=(FloatDP x1, FloatDP x2) { return x1.d<=x2.d; }
inline Bool operator>=(FloatDP x1, FloatDP x2) { return (x2<=x1); }
inline Bool operator< (FloatDP x1, FloatDP x2) { return !(x2<=x1); }
inline Bool operator> (FloatDP x1, FloatDP x2) { return !(x1<=x2); }

/************ FloatDP ***************************************************/

extern const FloatDP inf;

class FloatDP
    : public NumberObject<FloatDP>
{
    friend class FloatDPBounds;
    friend class FloatDPApproximation;
    volatile double v;
    static Int output_places;
  public:
    static Void set_output_places(Int);
  public:
    typedef ExactTag Paradigm;
    FloatDP();
    template<BuiltinIntegral N> FloatDP(N);
    FloatDP(Nat);
    FloatDP(Int);
    explicit FloatDP(double);
    explicit FloatDP(FloatDP);
    operator FloatDPBounds () const;
    operator ValidFloatDP () const;
    operator FloatDPLowerBound () const;
    operator FloatDPUpperBound () const;
    operator FloatDPApproximation () const;
    FloatDP get_flt() const;
    double get_d() const;
    friend FloatDP operator+(FloatDP);
    friend FloatDP operator-(FloatDP);
    friend FloatDPBounds operator+(FloatDP x1, FloatDP x2);
    friend FloatDPBounds operator-(FloatDP x1, FloatDP x2);
    friend FloatDPBounds operator*(FloatDP x1, FloatDP x2);
    friend FloatDPBounds operator/(FloatDP x1, FloatDP x2);
    friend OutputStream& operator<<(OutputStream& os, FloatDP const&);
  private:
    FloatDP(long long int n, std::nullptr_t);
};
Boolean operator==(FloatDP x1, FloatDP x2);
Boolean operator!=(FloatDP x1, FloatDP x2);
Boolean operator<=(FloatDP x1, FloatDP x2);
Boolean operator>=(FloatDP x1, FloatDP x2);
Boolean operator< (FloatDP x1, FloatDP x2);
Boolean operator> (FloatDP x1, FloatDP x2);
FloatDP min(FloatDP x1, FloatDP x2);
FloatDP max(FloatDP x1, FloatDP x2);

class FloatDPError : public NumberObject<FloatDPUpperBound> {
    volatile double e;
    static Int output_places;
  public:
    static Void set_output_places(Int);
  public:
    FloatDPError();
    template<BuiltinUnsignedIntegral M> = dummy> FloatDPError(M m);
    template<BuiltinFloatingPoint X> explicit FloatDPError(X x);
    explicit FloatDPError(double);
    explicit FloatDPError(FloatDP);
    explicit operator FloatDPUpperBound () const;
    FloatDP get_flt() const;
    FloatDP& get_flt();
    double get_d() const;
    explicit operator double() const;
    friend FloatDPError operator+(FloatDPError);
    friend FloatDPError operator+(FloatDPError,FloatDPError);
    friend FloatDPError operator*(FloatDPError,FloatDPError);
    friend FloatDPError pow(FloatDPError,Nat);
    friend FloatDPUpperBound operator/(FloatDPError x1, FloatDP x2);
    friend OutputStream& operator<<(OutputStream& os, FloatDPError const&);
    friend Fuzzy operator==(FloatDPError, FloatDP);
};

FloatDPError max(FloatDPError x1, FloatDPError x2);

template<BuiltinUnsignedIntegral M> inline FloatDPError operator/(FloatDPError x1, M m2) {
    return FloatDPError(x1.get_d()/m2); }

class ValidFloatDP : public NumberObject<ValidFloatDP> {
    volatile double v; volatile double e;
  private:
    ValidFloatDP(double d, ExactTag);
  public:
    typedef ValidatedTag Paradigm;
    ValidFloatDP();
    template<BuiltinIntegral N> ValidFloatDP(N n);
    explicit ValidFloatDP(double);
    explicit ValidFloatDP(double,double);
    ValidFloatDP(FloatDP, FloatDPError);
    operator FloatDPApproximation () const;
    FloatDP value() const;
    FloatDPError error() const;
    FloatDPUpperBound upper() const;
    FloatDPLowerBound lower() const;
    friend ValidFloatDP operator+(ValidFloatDP);
    friend ValidFloatDP operator-(ValidFloatDP);
    friend ValidFloatDP operator+(ValidFloatDP,ValidFloatDP);
    friend ValidFloatDP operator-(ValidFloatDP,ValidFloatDP);
    friend ValidFloatDP operator*(ValidFloatDP,ValidFloatDP);
    friend ValidFloatDP operator/(ValidFloatDP,ValidFloatDP);
    friend ValidFloatDP sqr(ValidFloatDP);
    friend ValidFloatDP rec(ValidFloatDP x);
    friend Bool operator==(ValidFloatDP,Int);
    friend OutputStream& operator<<(OutputStream& os, ValidFloatDP const&);
};
ValidFloatDP min(ValidFloatDP x1, ValidFloatDP x2);
ValidFloatDP max(ValidFloatDP x1, ValidFloatDP x2);


Bool same(ValidFloatDP x1, ValidFloatDP x2);

template<BuiltinIntegral N> inline
ValidFloatDP::ValidFloatDP(N n) : ValidFloatDP(double(n),ExactTag())
{ }

class FloatDPBounds : public NumberObject<FloatDPBounds> {
    friend class FloatDPApproximation; friend class ValidFloatDP;
    volatile double l; volatile double u;
    static Int output_places;
  public:
    static Void set_output_places(Int);
  public:
    typedef ValidatedTag Paradigm;
    FloatDPBounds();
    FloatDPBounds(Int);
    FloatDPBounds(const Rational&);
    explicit FloatDPBounds(double);
    explicit FloatDPBounds(double,double);
    FloatDPBounds(FloatDPLowerBound, FloatDPUpperBound);
    operator FloatDPApproximation () const;
    operator FloatDPUpperBound () const;
    operator FloatDPLowerBound () const;
    operator ValidFloatDP () const;
    friend FloatDPBounds operator+(FloatDPBounds);
    friend FloatDPBounds operator-(FloatDPBounds);
    friend FloatDPBounds operator+(FloatDPBounds,FloatDPBounds);
    friend FloatDPBounds operator-(FloatDPBounds,FloatDPBounds);
    friend FloatDPBounds operator*(FloatDPBounds,FloatDPBounds);
    friend FloatDPBounds operator/(FloatDPBounds,FloatDPBounds);
    friend FloatDPBounds sqr(FloatDPBounds);
    friend FloatDPBounds max(FloatDPBounds,FloatDPBounds);
    friend FloatDPBounds min(FloatDPBounds,FloatDPBounds);
    friend Bool operator==(FloatDPBounds,Int);
    friend ValidatedKleenean operator> (FloatDPBounds,FloatDPBounds);
    friend OutputStream& operator<<(OutputStream& os, FloatDPBounds const&);

    FloatDPLowerBound lower() const;
    FloatDPUpperBound upper() const;
    FloatDPError error() const;
    FloatDPError width() const;
    FloatDPError radius() const;

};
FloatDPBounds min(FloatDPBounds x1, FloatDPBounds x2);
FloatDPBounds max(FloatDPBounds x1, FloatDPBounds x2);
FloatDPError mag(FloatDPBounds x);
FloatDPBounds sqr(FloatDPBounds);

Bool same(FloatDPBounds x1, FloatDPBounds x2);

ValidatedKleenean operator==(FloatDPBounds x1, FloatDPBounds x2);
ValidatedKleenean operator!=(FloatDPBounds x1, FloatDPBounds x2);
ValidatedKleenean operator<=(FloatDPBounds x1, FloatDPBounds x2);
ValidatedKleenean operator>=(FloatDPBounds x1, FloatDPBounds x2);
ValidatedKleenean operator< (FloatDPBounds x1, FloatDPBounds x2);
ValidatedKleenean operator> (FloatDPBounds x1, FloatDPBounds x2);

class FloatDPLowerBound : public NumberObject<FloatDPLowerBound> {
    volatile double l;
  public:
    FloatDPLowerBound();
    FloatDPLowerBound(Int);
    explicit FloatDPLowerBound(double);
    operator FloatDPApproximation () const;
    FloatDP get_flt() const;
    friend FloatDPLowerBound operator+(FloatDPLowerBound);
    friend FloatDPUpperBound operator-(FloatDPLowerBound);
    friend FloatDPLowerBound operator-(FloatDPUpperBound);
    friend FloatDPLowerBound operator+(FloatDPLowerBound,FloatDPLowerBound);
    friend FloatDPLowerBound operator-(FloatDPLowerBound,FloatDPUpperBound);
    friend FloatDPUpperBound operator-(FloatDPUpperBound,FloatDPLowerBound);
    friend FloatDPUpperBound min(FloatDPUpperBound,FloatDPUpperBound);
    friend FloatDPUpperBound max(FloatDPUpperBound,FloatDPUpperBound);
    friend OutputStream& operator<<(OutputStream& os, FloatDPLowerBound const&);
};
FloatDPLowerBound min(FloatDPLowerBound,FloatDPLowerBound);
FloatDPLowerBound max(FloatDPLowerBound,FloatDPLowerBound);

FloatDPUpperBound rec(FloatDPLowerBound);

class FloatDPUpperBound : public NumberObject<FloatDPUpperBound> {
    volatile double u;
  public:
    FloatDPUpperBound();
    FloatDPUpperBound(Int);
    explicit FloatDPUpperBound(double);
    operator FloatDPApproximation () const;
    FloatDP get_flt() const;
    friend FloatDPUpperBound operator+(FloatDPUpperBound);
    friend FloatDPUpperBound operator-(FloatDPLowerBound);
    friend FloatDPLowerBound operator-(FloatDPUpperBound);
    friend FloatDPUpperBound operator+(FloatDPUpperBound,FloatDPUpperBound);
    friend FloatDPUpperBound operator-(FloatDPUpperBound,FloatDPLowerBound);
    friend FloatDPLowerBound operator-(FloatDPLowerBound,FloatDPUpperBound);
    friend FloatDPUpperBound operator*(FloatDPUpperBound,FloatDPUpperBound);
    friend FloatDPLowerBound min(FloatDPLowerBound,FloatDPLowerBound);
    friend FloatDPLowerBound max(FloatDPLowerBound,FloatDPLowerBound);
    friend OutputStream& operator<<(OutputStream& os, FloatDPUpperBound const&);
};
FloatDPUpperBound min(FloatDPUpperBound,FloatDPUpperBound);
FloatDPUpperBound max(FloatDPUpperBound,FloatDPUpperBound);
FloatDPLowerBound rec(FloatDPUpperBound);

FloatDPUpperBound operator+(FloatDPUpperBound,FloatDPError);
FloatDPLowerBound operator-(FloatDPLowerBound,FloatDPError);

class FloatDPApproximation : public NumberObject<FloatDPApproximation> {
    volatile double a;
  public:
    typedef ApproximateTag Paradigm;
    FloatDPApproximation();
    FloatDPApproximation(double);
    FloatDPApproximation(const Rational&);
    FloatDP get_flt() const;
    double get_d() const;
    friend FloatDPApproximation operator+(FloatDPApproximation);
    friend FloatDPApproximation operator-(FloatDPApproximation);
    friend FloatDPApproximation operator+(FloatDPApproximation,FloatDPApproximation);
    friend FloatDPApproximation operator-(FloatDPApproximation,FloatDPApproximation);
    friend FloatDPApproximation operator*(FloatDPApproximation,FloatDPApproximation);
    friend FloatDPApproximation operator/(FloatDPApproximation,FloatDPApproximation);
    friend FloatDPApproximation& operator+=(FloatDPApproximation&,FloatDPApproximation);
    friend FloatDPApproximation neg(FloatDPApproximation);
    friend FloatDPApproximation sqr(FloatDPApproximation);
    friend FloatDPApproximation min(FloatDPApproximation,FloatDPApproximation);
    friend FloatDPApproximation max(FloatDPApproximation,FloatDPApproximation);
    friend Fuzzy operator==(FloatDPApproximation,FloatDPApproximation);
    friend OutputStream& operator<<(OutputStream& os, FloatDPApproximation const&);
};

Fuzzy operator==(FloatDPApproximation,FloatDPApproximation);
Fuzzy operator!=(FloatDPApproximation,FloatDPApproximation);
Fuzzy operator<=(FloatDPApproximation,FloatDPApproximation);
Fuzzy operator>=(FloatDPApproximation,FloatDPApproximation);
Fuzzy operator< (FloatDPApproximation,FloatDPApproximation);
Fuzzy operator> (FloatDPApproximation,FloatDPApproximation);
FloatDPApproximation neg(FloatDPApproximation);
FloatDPApproximation abs(FloatDPApproximation);
FloatDPApproximation sqr(FloatDPApproximation);


class TwoExp {
    Int _n;
  public:
    TwoExp(Int n) : _n(n) { }
    double get_d() const { if(_n>=0) { return 1<<_n; } else { return 1.0/(1<<(-_n)); } }
    operator FloatDP () const { return FloatDP(this->get_d()); }
    operator FloatDPError () const { return FloatDPError(this->get_d()); }
    operator ValidFloatDP () const { return ValidFloatDP(this->get_d()); }
    operator FloatDPBounds () const { return FloatDPBounds(this->get_d()); }
};
inline TwoExp two_exp(Int n) { return TwoExp(n); }


FloatDP cast_exact(const FloatDP&);
FloatDPError make_error(const FloatDP&);
ValidFloatDP make_valid(const FloatDP&);
FloatDP cast_raw(const FloatDPApproximation&);

} // namespace Ariadne

#endif
