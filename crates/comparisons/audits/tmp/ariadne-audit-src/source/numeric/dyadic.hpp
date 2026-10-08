/***************************************************************************
 *            numeric/dyadic.hpp
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

/*! \file numeric/dyadic.hpp
 *  \brief Dyadic numbers.
 */

#ifndef ARIADNE_DYADIC_HPP
#define ARIADNE_DYADIC_HPP

#include "numeric/gmp.hpp"

#include <iostream> // For std::floor std::ceil etc
#include <iomanip> // For std::setprecision
#include <cmath> // For std::floor std::ceil etc
#include <algorithm> // For std::max, std::min
#include <limits> // For std::numeric_limits<double>

#include "utility/handle.hpp"
#include "utility/writable.hpp"

#include "foundations/logical.hpp"
#include "numeric/integer.hpp"
#include "numeric/twoexp.hpp"
#include "numeric/arithmetic.hpp"

namespace Ariadne {

class ExactDouble;
class Dyadic;
extern const Dyadic infty;

class InfinityException : public std::runtime_error {
    using std::runtime_error::runtime_error;
};

//! \ingroup NumericModule
//! \brief A dyadic number of the form \f$p/2^q\f$ for integers \f$p\f$, \f$q\f$; equivalently, a number with a finite binary expansion.
//! \sa Integer, Decimal, Rational, Float<DP>, Float<MP>
//! \details The number \f$1.375\f$ is an example of a dyadic number, since it is equal to \f$1.375=11/8=11/2^3=1.101_2\f$. The number \f$0.1375\f$ is not a dyadic number, since \f$0.1375=11/80=11/(2^4\times 5)=0.0010\overline{0011}_2\f$, so the denominator is not a power of \f$2\f$, and its binary expansion is recurring.
class Dyadic
{
    static Writer<Dyadic> _default_writer;
  public:
//    template<class W> requires BaseOf<WriterInterface<Dyadic>,W> static Void set_default_writer(W w) {
//        _default_writer=std::make_shared<W>(std::move(w)); }
//    static Void set_default_writer(Writer<Dyadic> w) { _default_writer=w.managed_pointer(); }
    static Void set_default_writer(Writer<Dyadic> w) { _default_writer=w; }
    static Writer<Dyadic> default_writer() { return _default_writer; }
  public:
    mpf_t _mpf;
  public:
    typedef ExactTag Paradigm;

    //! \brief Construct a Dyadic number from a GNU mpf object.
    explicit Dyadic (mpf_t mpf);
    //! \brief Construct the Dyadic number \a p/2<sup>q</sup>.
    Dyadic (Integer const& p, Nat q);
    Dyadic (Integer const& p, Int q) = delete; // Can only construct a dyadic p/2^q for a positive (unsigned) value q, such at uint or Nat.
    Dyadic (Integer const& p, Natural q);
    //! \brief Destructor.
    ~Dyadic();
    //! \brief Default constructor creates the number 0 (zero).
    Dyadic();
    //! \brief Copy constructor.
    Dyadic(Dyadic const& n);
    Dyadic(Dyadic&& n);
    //! \brief Assignment constructor.
    Dyadic& operator=(Dyadic const& n);
    Dyadic& operator=(Dyadic&& n);
    //! \brief Convert from a built-in integer.
    template<BuiltinIntegral N> Dyadic(N n);
    //! \brief Convert from an exact double-precision number.
    Dyadic(const ExactDouble& d);
    //! \brief Convert from an integer.
    Dyadic(const Integer& z);
    //! \brief Convert from a power of two.
    Dyadic(const TwoExp& t);
    //! \brief Explicit construction from a built-in double-precision value.
    //! \details Tests to ensure that the number is not 'accidentally' created from a rounded version of a string literal,
    //! by comparing the input with it's single-precision approximation.
    explicit Dyadic(double x);
    //! \brief Construct from a string representation.
    explicit Dyadic(String const&);
    //! \brief Convert to a generic number.
    operator ExactNumber () const;
    //! \brief A representation of ±∞ or NaN.
    static Dyadic inf(Sign sgn);
    //! \brief A representation of +∞.
    static Dyadic inf();
    //! \brief A representation of NaN (not-a-number).
    static Dyadic nan();

    //! \brief The smallest integer \a p such that \a x=p/2<sup>q</sup>
    Integer mantissa() const;
    //! \brief The (negative) integer \a -q such that \a x=p/2<sup>q</sup>
    Int exponent() const;
    //! \brief A double-precision approximation.
    double get_d() const;
    mpf_t const& get_mpf() const;
    //! \brief A string literal, comprising the exact value in decimal format.
    String literal() const;
    //! \brief Convert a floating-point literal to Dyadic i.e. long binary format.
    friend Dyadic operator"" _bin(long double x);
    //! \brief Convert a floating-point literal to Dyadic.
    friend Dyadic operator"" _dyadic(long double x);
    //! \brief Shorthand for operator""_dyadic.
    friend Dyadic operator"" _dy(long double x);
    //! \brief Alternative for operator""_dyadic for use in Python interface.
    friend Dyadic dy_(long double x);

    //!@{
    //! \name Arithmetic operators
    friend Dyadic operator+(Dyadic const& w) { return pos(w); } //!< <p/>
    friend Dyadic operator-(Dyadic const& w) { return neg(w); } //!< <p/>
    friend Dyadic operator+(Dyadic const& w1, Dyadic const& w2) { return add(w1,w2); } //!< <p/>
    friend Dyadic operator-(Dyadic const& w1, Dyadic const& w2) { return sub(w1,w2); } //!< <p/>
    friend Dyadic operator*(Dyadic const& w1, Dyadic const& w2) { return mul(w1,w2); } //!< <p/>
    friend Dyadic& operator+=(Dyadic& w1, Dyadic const& w2) { return w1=add(w1,w2); } //!< <p/>
    friend Dyadic& operator-=(Dyadic& w1, Dyadic const& w2) { return w1=sub(w1,w2); } //!< <p/>
    friend Dyadic& operator*=(Dyadic& w1, Dyadic const& w2) { return w1=mul(w1,w2); } //!< <p/>
    friend Rational operator/(Rational const&, Rational const&);
    //!@}

    //!@{
    //! \name Comparison operators
    friend Boolean operator==(Dyadic const& w1, Dyadic const& w2) { return eq(w1,w2); } //!< <p/>
    friend Boolean operator!=(Dyadic const& w1, Dyadic const& w2) { return !eq(w1,w2); } //!< <p/>
    friend Boolean operator<=(Dyadic const& w1, Dyadic const& w2) { return !lt(w2,w1); } //!< <p/>
    friend Boolean operator>=(Dyadic const& w1, Dyadic const& w2) { return !lt(w1,w2); } //!< <p/>
    friend Boolean operator< (Dyadic const& w1, Dyadic const& w2) { return lt(w1,w2); } //!< <p/>
    friend Boolean operator> (Dyadic const& w1, Dyadic const& w2) { return lt(w2,w1); } //!< <p/>
    //!@}

    //!@{
    //! \name Arithmetic operations
    friend Dyadic nul(Dyadic const& w); //!< Zero \a 0.
    friend Dyadic pos(Dyadic const& w); //!< %Positive \a +w.
    friend Dyadic neg(Dyadic const& w); //!< Negative \a -w.
    friend Positive<Dyadic> sqr(Dyadic const& w); //!< Square \a w<sup>2</sup>.
    friend Dyadic hlf(Dyadic const& w); //!< Half \a w/2.

    friend Dyadic add(Dyadic const& w1, Dyadic const& w2); //!< Add \a w1+w2.
    friend Dyadic sub(Dyadic const& w1, Dyadic const& w2); //!< Subtract \a w1-w2.
    friend Dyadic mul(Dyadic const& w1, Dyadic const& w2); //!< Multiply \a w1×w2.
    friend Dyadic pow(Dyadic const& w, Nat m); //!< Power \a w<sup>m</sup> where \a  m is positive.
    friend Rational pow(Dyadic const& w, Int n); //!< Power \a w<sup>n</sup>.
    friend Dyadic fma(Dyadic const& w1, Dyadic const& w2, Dyadic const& w3); //!< Fused multiply-and-add \a w1×w2+w3.

    friend Rational rec(Rational const&);
    friend Rational div(Rational const&, Rational const&);
    //!@}

    friend Real sqrt(Real const&);
    friend Real exp(Real const&);
    friend Real log(Real const&);
    friend Real sin(Real const&);
    friend Real cos(Real const&);
    friend Real tan(Real const&);
    friend Real asin(Real const&);
    friend Real acos(Real const&);
    friend Real atan(Real const&);

    //!@{
    //! \name Lattice operations
    friend Positive<Dyadic> abs(Dyadic const& w); //!< Absolute value \a |w|.
    friend Dyadic min(Dyadic const& w1, Dyadic const& w2); //!< Minimum \a w1∧w2.
//    friend Positive<Dyadic> min(Positive<Dyadic> const& w1, Positive<Dyadic> const& w2);
    friend Dyadic max(Dyadic const& w1, Dyadic const& w2); //!< Maximum \a w1∨w2.
//    friend Positive<Dyadic> max(Dyadic const& w1, Positive<Dyadic> const& w2);
//    friend Positive<Dyadic> max(Positive<Dyadic> const& w1, Dyadic const& w2);
//    friend Positive<Dyadic> max(Positive<Dyadic> const& w1, Positive<Dyadic> const& w2);
    //!@}

    //!@{
    //! \name Rounding operations
    friend Integer floor(Dyadic const& w); //!< Round down to the nearest lower integer.
    friend Integer round(Dyadic const& w); //!< Round to the nearest integer. %Rounding of halves is implementation-dependent.
    friend Integer ceil(Dyadic const& w); //!< Round up to the nearest higher integer.
    //!@}

    //!@{
    //! \name Comparison operations
    friend Sign sgn(Dyadic const& w); //!< The sign of \a w.
    friend Comparison cmp(Dyadic const& w1, Dyadic const& w2); //!< Compares which of \a w1 and \a w2 is larger.
    friend Boolean eq(Dyadic const& w1, Dyadic const& w2); //!< Tests if \a w1 is equal to \a w2.
    friend Boolean lt(Dyadic const& w1, Dyadic const& w2); //!< Tests if \a w1 is less than \a w2.
    //!@}

    //!@{
    //! \name Special value tests
    friend Bool is_nan(Dyadic const& w); //!< Tests whether \a w is NaN (not-a-number).
    friend Bool is_inf(Dyadic const& w); //!< Tests whether \a w is ±∞.
    friend Bool is_finite(Dyadic const& w); //!< Tests whether \a w is finite.
    friend Bool is_zero(Dyadic const& w); //!< Tests whether \a w is zero.
    //!@}

    //!@{
    //! \name Input/output operations
    friend OutputStream& operator<<(OutputStream& os, Dyadic const& w); //!< Write to an output stream.
    //!@}
};

class DecimalWriter : public WriterInterface<Dyadic> {
    virtual OutputStream& _write(OutputStream& os, Dyadic const& w) const final override;
};
class ScientificWriter : public WriterInterface<Dyadic> {
    virtual OutputStream& _write(OutputStream& os, Dyadic const& w) const final override;
};
class FractionWriter : public WriterInterface<Dyadic> {
    virtual OutputStream& _write(OutputStream& os, Dyadic const& w) const final override;
};
template<> class RepresentationWriter<Dyadic> : public WriterInterface<Dyadic> {
    virtual OutputStream& _write(OutputStream& os, Dyadic const& w) const final override;
};


template<BuiltinIntegral N> inline Dyadic::Dyadic(N n) : Dyadic(Integer(n)) { }


template<> class Positive<Dyadic> : public Dyadic {
  public:
    Positive() : Dyadic() { }
    template<BuiltinUnsignedIntegral M> Positive(M m) : Dyadic(m) { }
    Positive(int n) = delete;
    explicit Positive(Dyadic const& w) : Dyadic(w) { ARIADNE_ASSERT(w>=0); }
};
inline Positive<Dyadic> cast_positive(Dyadic const& w) { return Positive<Dyadic>(w); }

using PositiveDyadic = Positive<Dyadic>;

//! \relates Dyadic
//! \name Type synonyms
//!@{
//! \ingroup NumericModule
using DyadicBounds = Bounds<Dyadic>; //!< Alias for dyadic bounds on a number.
using DyadicBall = Ball<Dyadic,Dyadic>; //!< Alias for ball about a number with dyadic value and error.
using DyadicUpperBound = UpperBound<Dyadic>; //!< Alias for dyadic upper bound for a number.
using DyadicLowerBound = LowerBound<Dyadic>; //!< Alias for dyadic lower bound for a number.
using DyadicApproximation = Approximation<Dyadic>; //!< Alias for dyadic approximateion to a number.

using PositiveDyadicApproximation = Positive<Approximation<Dyadic>>; //!< <p/>
using PositiveDyadicLowerBound = Positive<LowerBound<Dyadic>>; //!< <p/>
using PositiveDyadicUpperBound = Positive<UpperBound<Dyadic>>; //!< <p/>
using PositiveDyadicBounds = Positive<Bounds<Dyadic>>; //!< <p/>
//!@}

template<> class Bounds<Dyadic> {
    Dyadic _l, _u;
  public:
    typedef ValidatedTag Paradigm;
    Bounds(Dyadic w) : _l(w), _u(w) { }
    template<ConvertibleTo<Dyadic> X> Bounds(X const& x)
        : Bounds(Dyadic(x)) { }
    Bounds(Dyadic l, Dyadic u) : _l(l), _u(u) { }
    template<class X> requires Constructible<Dyadic,X> Bounds(Bounds<X> const& x)
        : Bounds(Dyadic(x.lower_raw()),Dyadic(x.upper_raw())) { }
    operator ValidatedNumber() const;
    Bounds<Dyadic> pm(Dyadic e) { return Bounds<Dyadic>(_l-e,_u+e); }
    Dyadic lower() const { return _l; }
    Dyadic upper() const { return _u; }
    Dyadic lower_raw() const { return _l; }
    Dyadic upper_raw() const { return _u; }
    Bounds<FloatDP> get(DoublePrecision pr) const;
    Bounds<FloatMP> get(MultiplePrecision pr) const;
    friend Bounds<Dyadic> operator+(Bounds<Dyadic> const& w) { return Bounds<Dyadic>(+w._l,w._u); }
    friend Bounds<Dyadic> operator-(Bounds<Dyadic> const& w) { return Bounds<Dyadic>(-w._u,-w._l); }
    friend Bounds<Dyadic> operator+(Bounds<Dyadic> const& w1, Bounds<Dyadic> const& w2) {
        return Bounds<Dyadic>(w1._l+w2._l,w1._u+w2._u); }
    friend Bounds<Dyadic> operator-(Bounds<Dyadic> const& w1, Bounds<Dyadic> const& w2) {
        return Bounds<Dyadic>(w1._l-w2._u,w1._u-w2._l); }
    friend Bounds<Dyadic> operator*(Bounds<Dyadic> const& w1, Bounds<Dyadic> const& w2);
    friend Bounds<Rational> operator/(Bounds<Rational> const& w1, Bounds<Rational> const& w2);
    friend Bounds<Dyadic> add(Bounds<Dyadic> const& w1, Bounds<Dyadic> const& w2) {
        return Bounds<Dyadic>(w1._l+w2._l,w1._u+w2._u); }
    friend Bounds<Dyadic> sub(Bounds<Dyadic> const& w1, Bounds<Dyadic> const& w2) {
        return Bounds<Dyadic>(w1._l-w2._u,w1._u-w2._l); }
    friend Bounds<Dyadic> mul(Bounds<Dyadic> const& w1, Bounds<Dyadic> const& w2);
    friend Bounds<Rational> div(Bounds<Rational> const& w1, Bounds<Rational> const& w2);
    friend Bounds<Dyadic> pow(Bounds<Dyadic> const& w, Nat m);
    friend Bounds<Dyadic> pow(Bounds<Dyadic> const& w, Int m); // DEPRECATED m always positive
    friend Bounds<Dyadic> nul(Bounds<Dyadic> const& w) {
        return Bounds<Dyadic>(nul(w._l),nul(w._u)); }
    friend Bounds<Dyadic> pos(Bounds<Dyadic> const& w) {
        return Bounds<Dyadic>(pos(w._l),pos(w._u)); }
    friend Bounds<Dyadic> neg(Bounds<Dyadic> const& w) {
        return Bounds<Dyadic>(neg(w._l),neg(w._u)); }
    friend Bounds<Dyadic> sqr(Bounds<Dyadic> const& w) {
        if(w._l>0) { return Bounds<Dyadic>(sqr(w._l),sqr(w._u)); }
        else if(w._u<0) { return Bounds<Dyadic>(sqr(w._u),sqr(w._l)); }
        else { return Bounds<Dyadic>(nul(w._l),max(sqr(w._l),sqr(w._u))); } }
    friend Bounds<Dyadic> hlf(Bounds<Dyadic> const& w) { return Bounds<Dyadic>(hlf(w._l),hlf(w._u)); }
    friend Bounds<Rational> rec(Bounds<Rational> const& w);
    friend Bounds<Dyadic> abs(Bounds<Dyadic> const& w) { return Bounds<Dyadic>(max(min(w._l,-w._u),0),max(-w._l,w._u)); }
    friend Bounds<Dyadic> max(Bounds<Dyadic> const& w1, Bounds<Dyadic> const& w2) { return Bounds<Dyadic>(max(w1._l,w2._l),max(w1._u,w2._u)); }
    friend Bounds<Dyadic> min(Bounds<Dyadic> const& w1, Bounds<Dyadic> const& w2) { return Bounds<Dyadic>(min(w1._l,w2._l),min(w1._u,w2._u)); }

    friend ValidatedKleenean sgn(Bounds<Dyadic> const& w) {
        if (w._l>0) { return true; } else if (w._u<0) { return false; } else { return indeterminate; } }
    friend ValidatedKleenean operator==(Bounds<Dyadic> const& w1, Bounds<Dyadic> const& w2) {
        if (w1._l>=w2._u && w1._u<=w2._l) { return true; } else if (w1._u< w2._l || w1._l> w2._u) { return false; } else { return indeterminate; } }
    friend ValidatedKleenean operator!=(Bounds<Dyadic> const& w1, Bounds<Dyadic> const& w2) {
        if (w1._u< w2._l || w1._l> w2._u) { return true; } else if (w1._l>=w2._u && w1._u<=w2._l) { return false; } else { return indeterminate; } }
    friend ValidatedKleenean operator<=(Bounds<Dyadic> const& w1, Bounds<Dyadic> const& w2) {
        if (w1._u<=w2._l) { return true; } else if (w1._l> w2._u) { return false; } else { return indeterminate; } }
    friend ValidatedKleenean operator>=(Bounds<Dyadic> const& w1, Bounds<Dyadic> const& w2) {
        if (w1._l>=w2._u) { return true; } else if (w1._u< w2._l) { return false; } else { return indeterminate; } }
    friend ValidatedKleenean operator< (Bounds<Dyadic> const& w1, Bounds<Dyadic> const& w2) {
        if (w1._u< w2._l) { return true; } else if (w1._l>=w2._u) { return false; } else { return indeterminate; } }
    friend ValidatedKleenean operator> (Bounds<Dyadic> const& w1, Bounds<Dyadic> const& w2) {
        if (w1._l> w2._u) { return true; } else if (w1._u<=w2._l) { return false; } else { return indeterminate; } }
    friend Bool consistent(Bounds<Dyadic> const& w1, Bounds<Dyadic> const& w2) {
        return w1._l<=w2._u && w1._u >= w2._l; }
    friend Boolean inconsistent(Bounds<Dyadic> const& w1, Bounds<Dyadic> const& w2) {
        return w1._l>w2._u || w1._u < w2._l; }
    friend Boolean refines(Bounds<Dyadic> const& w1, Bounds<Dyadic> const& w2) {
        return w1._l>=w2._l and w1._u<=w2._u; }
    friend Bounds<Dyadic> refinement(Bounds<Dyadic> const& w1, Bounds<Dyadic> const& w2) {
        return Bounds<Dyadic>(max(w1._l,w2._l),min(w1._u,w2._u)); }
    friend Bounds<Dyadic> coarsening(Bounds<Dyadic> const& w1, Bounds<Dyadic> const& w2) {
        return Bounds<Dyadic>(min(w1._l,w2._l),max(w1._u,w2._u)); }
    friend OutputStream& operator<<(OutputStream& os, Bounds<Dyadic> y) { return os << "[" << y._l << ":" << y._u << "]"; }
};


template<> class Ball<Dyadic,Dyadic> {
    Dyadic _v, _e;
  public:
    Ball(Dyadic w) : _v(w), _e(0) { }
    Ball(Dyadic v, Dyadic e) : _v(v), _e(e) { }
    template<class X, class XE> requires Constructible<Dyadic,X> and Constructible<Dyadic,XE>
    Ball(Ball<X,XE> const& x)
        : Ball(Dyadic(x.value_raw()),Dyadic(x.error_raw())) { }
    explicit Ball(Bounds<Dyadic> const& w) : _v(hlf(w.lower()+w.upper())), _e(hlf(w.upper()-w.lower())) { }
    explicit operator Bounds<Dyadic>() const { return Bounds<Dyadic>(_v-_e,_v+_e); }
    operator ValidatedNumber() const;
    Dyadic value() const { return _v; }
    Dyadic error() const { return _e; }
    Dyadic value_raw() const { return _v; }
    Dyadic error_raw() const { return _e; }
    friend DyadicBall operator+(DyadicBall const& w1, DyadicBall const& w2) { return DyadicBall(w1._v+w2._v,w1._e+w2._e); }
    friend DyadicBall operator-(DyadicBall const& w1, DyadicBall const& w2) { return DyadicBall(w1._v-w2._v,w1._e+w2._e); }
    friend DyadicBall operator*(DyadicBall const& w1, DyadicBall const& w2) { return DyadicBall(w1._v*w2._v,abs(w1._v)*w2._e+w1._e*abs(w2._v)+w1._e*w2._e); }
    friend DyadicBall abs(DyadicBall const& w) {
        if (abs(w._v)>=w._e) { return DyadicBall(abs(w._v),w._e); } else { Dyadic av=hlf(max(w._e-w._v,w._v+w._e)); return DyadicBall(av,av); } }
    friend ValidatedKleenean operator<(DyadicBall const& w1, DyadicBall const& w2) {
        if (w1._v+w1._e<w2._v-w2._e) { return true; } else if (w1._v-w1._e >= w2._v+w2._e) { return false; } else { return indeterminate; } }
    friend Boolean inconsistent(DyadicBall const& w1, DyadicBall const& w2) { return abs(w1._v-w2._v) > w1._e+w2._e; }
    friend Boolean refines(DyadicBall const& w1, DyadicBall const& w2) { return abs(w1._v-w2._v)+w1._e <= w2._e; }
    friend DyadicBall refinement(DyadicBall const& w1, DyadicBall const& w2) { return DyadicBall(refinement(DyadicBounds(w1),DyadicBounds(w2))); }
    friend DyadicBall coarsening(DyadicBall const& w1, DyadicBall const& w2) { return DyadicBall(coarsening(DyadicBounds(w1),DyadicBounds(w2))); }
    friend OutputStream& operator<<(OutputStream& os, DyadicBall y) { return os << "[" << y._v << ":" << y._e << "]"; }
};

template<> class LowerBound<Dyadic> {
    Dyadic _l;
  public:
    LowerBound(Dyadic l) : _l(l) { }
    LowerBound(Bounds<Dyadic> lu) : _l(lu.lower_raw()) { }
    LowerBound(LowerBound<FloatDP> const& x);
    LowerBound(LowerBound<FloatMP> const& x);
    Dyadic raw() const { return _l; }
    LowerBound<FloatDP> get(DoublePrecision pr) const;
    LowerBound<FloatMP> get(MultiplePrecision pr) const;
    friend OutputStream& operator<<(OutputStream& os, LowerBound<Dyadic> const& y) { return os << y._l << ":"; }
};

template<> class UpperBound<Dyadic> {
    Dyadic _u;
  public:
    explicit UpperBound(Dyadic u) : _u(u) { }
    UpperBound(Bounds<Dyadic> lu) : _u(lu.upper_raw()) { }
    UpperBound(UpperBound<FloatDP> const& x);
    UpperBound(UpperBound<FloatMP> const& x);
    Dyadic raw() const { return _u; }
    UpperBound<FloatDP> get(DoublePrecision pr) const;
    UpperBound<FloatMP> get(MultiplePrecision pr) const;
    friend OutputStream& operator<<(OutputStream& os, UpperBound<Dyadic> const& y) { return os << ":" << y._u; }
};

template<> class Approximation<Dyadic> {
    Dyadic _a;
  public:
    Approximation(Dyadic a) : _a(a) { }
    Approximation(Approximation<FloatDP> const& x);
    Approximation(Approximation<FloatMP> const& x);
    Dyadic raw() const { return _a; }
    Approximation<FloatDP> get(DoublePrecision pr) const;
    Approximation<FloatMP> get(MultiplePrecision pr) const;
    friend OutputStream& operator<<(OutputStream& os, Approximation<Dyadic> const& y);
};

template<> class Positive<Bounds<Dyadic>> : public Bounds<Dyadic> { public: Positive(Bounds<Dyadic> w) : Bounds<Dyadic>(w) { }; };
template<> class Positive<LowerBound<Dyadic>> : public LowerBound<Dyadic> { public: Positive(LowerBound<Dyadic> w) : LowerBound<Dyadic>(w) { }; };
template<> class Positive<UpperBound<Dyadic>> : public UpperBound<Dyadic> { public: Positive(UpperBound<Dyadic> w) : UpperBound<Dyadic>(w) { } ; };
template<> class Positive<Approximation<Dyadic>> : public Approximation<Dyadic> { public: Positive(Approximation<Dyadic> w) : Approximation<Dyadic>(w) { } };

inline Dyadic operator"" _dyadic(long double x) { return Dyadic(static_cast<double>(x)); }
inline Dyadic operator"" _dy(long double x) { return operator"" _dyadic(x); }
inline Dyadic operator"" _q2(long double x) { return operator"" _dyadic(x); }
inline Dyadic operator"" _bin(long double x) { return operator"" _dyadic(x); }

Comparison cmp(Dyadic const& x1, Dyadic const& x2);
Dyadic make_dyadic(unsigned long long int n);

} // namespace Ariadne

#endif
