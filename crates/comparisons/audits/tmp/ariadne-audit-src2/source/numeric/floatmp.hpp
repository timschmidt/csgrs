/***************************************************************************
 *            numeric/floatmp.hpp
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

/*! \file numeric/floatmp.hpp
 *  \brief RawTag floating-point numbers based on MPFT floats.
 */



#ifndef ARIADNE_FLOATMP_HPP
#define ARIADNE_FLOATMP_HPP

#include "foundations/paradigm.hpp"
#include "number.hpp"
#include "bits.hpp"
#include "rounding.hpp"
#include "sign.hpp"
#include "numeric/float_operations.hpp"
#include <mpfr.h>

namespace Ariadne {

/************ FloatMP ********************************************************/

struct NoInit;
struct RawPtr { };
struct DefaultTag;

struct RoundUpward; struct RoundDownward;

typedef mpfr_rnd_t RoundingModeMP;

typedef std::make_unsigned<mpfr_prec_t>::type unsigned_mpfr_prec_t;

//! \ingroup NumericModule
//! \brief The precision of a FloatMP object.
//! \relates FloatMP
class MultiplePrecision {
    mpfr_prec_t prec;
  public:
    typedef unsigned_mpfr_prec_t Type;
    //! \brief
//    explicit MultiplePrecision(DefaultTag const&);
    //! \brief
    explicit MultiplePrecision(mpfr_prec_t pr) : prec(pr) { }
    //! \brief
    explicit MultiplePrecision(Bits pr) : prec(static_cast<mpfr_prec_t>(pr)) { }
    //! \brief
    explicit MultiplePrecision(DoublePrecision const& pr) : prec(53u) { }
    //! \brief The number of binary digits of precision requested.
    unsigned_mpfr_prec_t bits() const { return static_cast<unsigned_mpfr_prec_t>(prec); }
    operator mpfr_prec_t () const { return prec; }
    //! \brief <p/>
    friend MultiplePrecision max(MultiplePrecision mp1, MultiplePrecision mp2) { return MultiplePrecision(static_cast<mpfr_prec_t>(std::max(mp1.bits(),mp2.bits()))); }
    //! \brief <p/>
    friend MultiplePrecision min(MultiplePrecision mp1, MultiplePrecision mp2) { return MultiplePrecision(static_cast<mpfr_prec_t>(std::min(mp1.bits(),mp2.bits()))); }
    //! \brief <p/>
    friend Bool operator==(MultiplePrecision mp1, MultiplePrecision mp2) { return mp1.bits()==mp2.bits(); }
    //! \brief <p/>
    friend Bool operator<=(MultiplePrecision mp1, MultiplePrecision mp2) { return mp1.bits()<=mp2.bits(); }
    //! \brief Write to an output stream.
    friend OutputStream& operator<<(OutputStream& os, MultiplePrecision mp) { return os << "mp("<<mp.bits()<<")"; }
    //! \brief Write a full representation to an output stream.
    friend OutputStream& repr(OutputStream& os, MultiplePrecision mp) { return os << "MultiplePrecision("<<mp.bits()<<")"; }
};
//! \brief Shorthand for MultiplePrecision.
//! \relates MultiplePrecision
using MP = MultiplePrecision;
inline MultiplePrecision multiple_precision(mpfr_prec_t pr) { return MultiplePrecision(pr); }
inline MultiplePrecision multiple_precision(Bits pr) { return MultiplePrecision(pr); }
inline MultiplePrecision precision(mpfr_prec_t pr) { return MultiplePrecision(pr); }
inline MultiplePrecision precision(Bits pr) { return MultiplePrecision(pr); }
inline MP mp(mpfr_prec_t pr) { return MP(pr); }

typedef Float<MP> FloatMP; //!< Shorthand for \ref Float<MP> "Float<MP>".

//! \ingroup NumericModule
//! \brief Multiple-precision floating-point numbers. Abbreviation \ref Ariadne::FloatMP "FloatMP".
//! \details
//! Abbreviated form FloatMP. See Float<DP>.
//! Currently defined as a wrapper around \c mpfr_t from the MPFR library.
//! Default arithmetic operations are approximate, and comparisons are exact, so this class is \em unsafe.
//! \see Float<DP>, Real, FloatBall, FloatBounds, FloatUpperBound, FloatLowerBound, FloatApproximation, FloatError
template<> class Float<MP>
    : public DeclareRoundedOperations<Float<MP>>
{
  private:
    mpfr_t _mpfr;
  public:
    typedef ExactTag Paradigm;
    typedef FloatMP NumericType;
    typedef mpfr_exp_t ExponentType;
    //! \brief The abstract number class providing the same kind of information.
    typedef ExactNumber GenericType;
    //! \brief The type representing the precision of the number.
    typedef MultiplePrecision PrecisionType;
    //! \brief The type representing the rounding modes available for arithmetic.
    typedef MPFRRoundingModeType RoundingModeType;
  public:
    static const RoundingModeType ROUND_TO_NEAREST;
    static const RoundingModeType ROUND_DOWNWARD;
    static const RoundingModeType ROUND_UPWARD;
    static const RoundingModeType ROUND_TOWARD_ZERO;

    static RoundingModeType get_rounding_mode(); //!< <p/>
    static Void set_rounding_mode(RoundingModeType); //!< <p/>
    static Void set_rounding_downward(); //!< <p/>
    static Void set_rounding_upward(); //!< <p/>
    static Void set_rounding_to_nearest(); //!< <p/>
    static Void set_rounding_toward_zero(); //!< <p/>
  public:
    static Void set_default_precision(PrecisionType prec); //!< <p/>
    static PrecisionType get_default_precision(); //!< <p/>
  public:
    static FloatMP inf(Sign, PrecisionType); //!< <p/>
    static FloatMP inf(PrecisionType); //!< <p/>
    static FloatMP nan(PrecisionType); //!< <p/>

    static FloatMP max(PrecisionType); //!< <p/>
    static FloatMP eps(PrecisionType); //!< <p/>
    static FloatMP min(PrecisionType); //!< <p/>
  public:
    ~Float(); //!< <p/>
    explicit Float(NoInit const&); //!< <p/>
    explicit Float(PrecisionType, NoInit const&); //!< <p/>
    explicit Float(const mpfr_t, RawPtr const&); //!< Construct from a raw MPFR number. The RawPtr tag is required to prevent converting from 0 as a null-pointer.

  private:
    Float(); //!< <p/>
    explicit Float(double); //!< <p/>
  public:
    explicit Float(PrecisionType); //!< <p/>
    template<BuiltinIntegral N> explicit Float(N n, PrecisionType pr)
        : Float(ExactDouble(n),pr) { } //!< <p/>
    explicit Float(FloatDP const& x, PrecisionType pr); //!< <p/>
    explicit Float(ExactDouble const& x, PrecisionType pr); //!< <p/>
    explicit Float(TwoExp const& x, PrecisionType pr); //!< <p/>
    explicit Float(Dyadic const& w, PrecisionType pr); //!< <p/>
    //! \brief Construct from an exact string literal.
    explicit Float(String const& str, PrecisionType pr); //!< <p/>

    Float(const FloatMP& x); //!< <p/>
    Float(FloatMP&& x); //!< <p/>

    template<BuiltinIntegral N> FloatMP& operator=(N n) {
        return this->operator=(ExactDouble(n)); } //!< <p/>
    FloatMP& operator=(const ExactDouble& x); //!< <p/>
    FloatMP& operator=(const Dyadic& w); //!< <p/>
    FloatMP& operator=(const FloatMP& x); //!< <p/>
    FloatMP& operator=(FloatMP&& x); //!< <p/>

    template<BuiltinIntegral N> Float(N n, RoundingModeType rnd, PrecisionType pr)
        : FloatMP(ExactDouble(n),rnd,pr) { } //!< <p/>
    Float(ExactDouble, RoundingModeType, PrecisionType); //!< <p/>
    Float(FloatDP const&, RoundingModeType, PrecisionType); //!< <p/>

    Float(Integer const&, RoundingModeType, PrecisionType); //!< <p/>
    Float(Dyadic const&, RoundingModeType, PrecisionType); //!< <p/>
    Float(Decimal const&, RoundingModeType, PrecisionType); //!< <p/>
    Float(Rational const&, RoundingModeType, PrecisionType); //!< <p/>
    Float(FloatMP const&, RoundingModeType, PrecisionType); //!< <p/>
    explicit operator Dyadic() const; //!< <p/>
    explicit operator Rational() const; //!< <p/>

    //! \brief Convert to a generic number.
    operator ExactNumber () const;

    //! \brief Construct a ball with radius \a e.
    Ball<FloatMP,FloatDP> pm(Error<FloatDP> const&) const;
    //! \brief Construct a ball with radius \a e.
    Ball<FloatMP,FloatMP> pm(Error<FloatMP> const&) const;

    //! \brief The exponent, being a number \a e such that \a x=±m×2<sup>e</sup> with \a 1≤m<2.
    ExponentType exponent() const;
    MultiplePrecision precision() const; //!< <p/>
    Void set_precision(MultiplePrecision); //!< <p/>
  public:
    FloatMP const& raw() const;
    FloatMP& raw();
    mpfr_t const& get_mpfr() const;
    mpfr_t& get_mpfr();
    double get_d() const;
    //! \brief The exact value as a decimal string.
    String literal() const;
    //! \brief An approximate value as a decimal string, rounded by \a rnd.
    //! The exact value is guaranteed to be the nearest value to the value's precision.
    String literal(RoundingModeType rnd) const;
  public:
    //! \name Special value tests
    //!@{
    friend Bool is_nan(FloatMP const& x); //!< <p/>
    friend Bool is_inf(FloatMP const& x); //!< <p/>
    friend Bool is_finite(FloatMP const& x); //!< <p/>
    friend Bool is_zero(FloatMP const& x); //!< <p/>
    //!@}

    //! \name Rounding operations
    //!@{
    friend FloatMP next(RoundUpward rnd, FloatMP const& x); //!< Round \a x up by 1 ulp.
    friend FloatMP next(RoundDownward rnd, FloatMP const& x); //!< Round \a x up by 1 ulp.

    friend FloatMP floor(FloatMP const& x); //!< Round \a x down to the nearest lower integer.
    friend FloatMP ceil(FloatMP const& x); //!< Round \a x to the nearest integer. %Rounding of halves is implementation-dependent.
    friend FloatMP round(FloatMP const& x); //!< Round \a x up to the nearest higher integer.

    friend Integer cast_integer(FloatMP const& x); //!< Round \a x to the nearest integer
    //!@}
  public:
    //! \name Arithmetic operators
    //!@{
    friend FloatMP operator+(FloatMP const& x); //!< <p/>
    friend FloatMP operator-(FloatMP const& x); //!< <p/>
    friend Bounds<FloatMP> operator+(FloatMP const& x1, FloatMP const& x2); //!< <p/>
    friend Bounds<FloatMP> operator-(FloatMP const& x1, FloatMP const& x2); //!< <p/>
    friend Bounds<FloatMP> operator*(FloatMP const& x1, FloatMP const& x2); //!< <p/>
    friend Bounds<FloatMP> operator/(FloatMP const& x1, FloatMP const& x2); //!< <p/>
    friend FloatMP operator*(TwoExp e1, FloatMP const& x2) { return shft(x2,e1.exponent()); } //!< <p/>
    friend FloatMP operator*(FloatMP const& x1, TwoExp e2) { return shft(x1,e2.exponent()); } //!< <p/>
    friend FloatMP operator/(FloatMP const& x1, TwoExp e2) { return shft(x1,-e2.exponent()); } //!< <p/>
    friend FloatMP& operator*=(FloatMP& v1, TwoExp e2) { return v1=v1*e2; } //!< <p/>
    friend FloatMP& operator/=(FloatMP& v1, TwoExp e2) { return v1=v1/e2; } //!< <p/>
    //!@}

    //!@{
    //! \name Comparison operators
    friend Boolean operator==(FloatMP const& x1, FloatMP const& x2); //!< <p/>
    friend Boolean operator!=(FloatMP const& x1, FloatMP const& x2); //!< <p/>
    friend Boolean operator< (FloatMP const& x1, FloatMP const& x2); //!< <p/>
    friend Boolean operator> (FloatMP const& x1, FloatMP const& x2); //!< <p/>
    friend Boolean operator<=(FloatMP const& x1, FloatMP const& x2); //!< <p/>
    friend Boolean operator>=(FloatMP const& x1, FloatMP const& x2); //!< <p/>
    //!@}

    //! \name Exact arithmetical functions
    //!@{
    friend FloatMP nul(FloatMP const& x); //!< Zero \a 0.
    friend FloatMP pos(FloatMP const& x); //!< Identity \a +x.
    friend FloatMP neg(FloatMP const& x); //!< Negation \a -x.
    friend FloatMP hlf(FloatMP const& x); //!< Half \a x÷2.
    friend FloatMP shft(FloatMP const& x, Int n); //!< Bit-shift \a x×2<sup>n</sup>.
    friend FloatMP mul(FloatMP const& x1, TwoExp e2) { return shft(x1,e2.exponent()); } //!< Multiplication by a power of two.
    friend FloatMP div(FloatMP const& x1, TwoExp e2) { return shft(x1,-e2.exponent()); } //!< Division by a power of two.
    //!@}

    //! \name Outward-rounded arithmetical functions
    //!@{
    friend Bounds<FloatMP> sqr(FloatMP const& x); //!< Square \a x<sup>2</sup>.
    friend Bounds<FloatMP> rec(FloatMP const& x); //!< Reciprocal \a 1/x.
    friend Bounds<FloatMP> add(FloatMP const& x1, FloatMP const& x2); //!< \brief Add \a x1+x2.
    friend Bounds<FloatMP> sub(FloatMP const& x1, FloatMP const& x2); //!< \brief Subtract \a x1-x2.
    friend Bounds<FloatMP> mul(FloatMP const& x1, FloatMP const& x2); //!< \brief Multiply \a x1×x2.
    friend Bounds<FloatMP> div(FloatMP const& x1, FloatMP const& x2); //!< \brief Divide \a x1÷x2.
    friend Bounds<FloatMP> fma(FloatMP const& x1, FloatMP const& x2, FloatMP const& x3); //!< \brief Fused multiply-and-add \a x1×x2+x3.
    friend Bounds<FloatMP> pow(FloatMP const& x, Nat m); //!< \brief Power \a x<sup>m</sup>.
    friend Bounds<FloatMP> pow(FloatMP const& x, Int n); //!< \brief Power \a x<sup>n</sup>.
    //!@}

    //! \name Algebraic and transcendental functions
    //!@{
    friend Bounds<FloatMP> sqrt(FloatMP const& x); //!< The square root of \a x, √\a x. Requires \a x ≥ 0.
    friend Bounds<FloatMP> exp(FloatMP const& x); //!< The natural exponent of \a x, \em e<sup>x</sup>.
    friend Bounds<FloatMP> log(FloatMP const& x); //!< The natural logarithm of \a x, log<em><sub>e</sub> x</em> or ln <em>x</em>. Requires \a x ≥ 0.
    friend Bounds<FloatMP> sin(FloatMP const& x); //!< The sine of \a x.
    friend Bounds<FloatMP> cos(FloatMP const& x); //!< The cosine of \a x.
    friend Bounds<FloatMP> tan(FloatMP const& x); //!< The tangent of \a x, sin(\a x)/cos(\a x).
    friend Bounds<FloatMP> asin(FloatMP const& x); //!< The arc-sine of \a x.
    friend Bounds<FloatMP> acos(FloatMP const& x); //!< The arc-cosine of \a x.
    friend Bounds<FloatMP> atan(FloatMP const& x); //!< The arc-tangent of \a x.
    //!@}

    //! \name Lattice operations
    //!@{
    friend Positive<FloatMP> abs(FloatMP const& x); //!< Absolute value \a |x|.
    friend FloatMP max(FloatMP const& x1, FloatMP const& x2); //!< Maximum \a x1∨x2.
    friend FloatMP min(FloatMP const& x1, FloatMP const& x2); //!< Mimimum \a x1∧x2.

    friend PositiveUpperBound<FloatMP> mag(FloatMP const& x); //!< An upper bound for the absolute value \a |x|.
    friend PositiveLowerBound<FloatMP> mig(FloatMP const& x); //!< A lower bound for the absolute value \a |x|.
    //!@}

    //! \name Comparison operations
    friend Boolean eq(FloatMP const& x1, FloatMP const& x2) { return x1 == x2; } //!< <p/>
    friend Boolean lt(FloatMP const& x1, FloatMP const& x2) { return x1 <  x2; } //!< <p/>

    friend Comparison cmp(FloatMP const& x1, FloatMP const& x2); //!< <p/>
    template<BuiltinIntegral N> friend Comparison cmp(FloatMP const& x1, N n2);
    template<BuiltinIntegral N> friend Comparison cmp(N n1, FloatMP const& x2);
    friend Comparison cmp(FloatMP const& x1, Nat m2); //!< <p/>
    friend Comparison cmp(Nat m1, FloatMP const& x2); //!< <p/>
    friend Comparison cmp(FloatMP const& x1, Int n2); //!< <p/>
    friend Comparison cmp(Int n1, FloatMP const& x2); //!< <p/>
    friend Comparison cmp(FloatMP const& x1, ExactDouble x2); //!< <p/>
    friend Comparison cmp(ExactDouble x1, FloatMP const& x2); //!< <p/>
    friend Comparison cmp(FloatMP const& x1, Rational const& x2); //!< <p/>
    friend Comparison cmp(Rational const& x1, FloatMP const& x2); //!< <p/>
    friend Comparison cmp(FloatMP const& x1, FloatDP const& x2); //!< <p/>
    friend Comparison cmp(FloatDP const& x1, FloatMP const& x2); //!< <p/>
    //!@}
  public:
    //! \name Correctly rounded operations
    //!@{
    friend FloatMP sqr(CurrentRoundingMode, FloatMP const& x); //!< <p/>
    friend FloatMP rec(CurrentRoundingMode, FloatMP const& x); //!< <p/>
    friend FloatMP add(CurrentRoundingMode, FloatMP const& x1, FloatMP const& x2); //!< <p/>
    friend FloatMP sub(CurrentRoundingMode, FloatMP const& x1, FloatMP const& x2); //!< <p/>
    friend FloatMP mul(CurrentRoundingMode, FloatMP const& x1, FloatMP const& x2); //!< <p/>
    friend FloatMP div(CurrentRoundingMode, FloatMP const& x1, FloatMP const& x2); //!< <p/>
    friend FloatMP fma(CurrentRoundingMode, FloatMP const& x1, FloatMP const& x2, FloatMP const& x3); //!< <p/>
    friend FloatMP pow(CurrentRoundingMode, FloatMP const& x, Nat m); //!< <p/>
    friend FloatMP pow(CurrentRoundingMode, FloatMP const& x, Int n); //!< <p/>
    friend FloatMP sqrt(CurrentRoundingMode, FloatMP const& x); //!< <p/>
    friend FloatMP exp(CurrentRoundingMode, FloatMP const& x); //!< <p/>
    friend FloatMP log(CurrentRoundingMode, FloatMP const& x); //!< <p/>
    friend FloatMP sin(CurrentRoundingMode, FloatMP const& x); //!< <p/>
    friend FloatMP cos(CurrentRoundingMode, FloatMP const& x); //!< <p/>
    friend FloatMP tan(CurrentRoundingMode, FloatMP const& x); //!< <p/>
    friend FloatMP asin(CurrentRoundingMode, FloatMP const& x); //!< <p/>
    friend FloatMP acos(CurrentRoundingMode, FloatMP const& x); //!< <p/>
    friend FloatMP atan(CurrentRoundingMode, FloatMP const& x); //!< <p/>
    static FloatMP pi(CurrentRoundingMode, MultiplePrecision pr); //!< <p/>

    friend FloatMP add(CurrentRoundingMode rnd, FloatMP const& x1, FloatDP const& x2);
    friend FloatMP sub(CurrentRoundingMode rnd, FloatMP const& x1, FloatDP const& x2);
    friend FloatMP mul(CurrentRoundingMode rnd, FloatMP const& x1, FloatDP const& x2);
    friend FloatMP div(CurrentRoundingMode rnd, FloatMP const& x1, FloatDP const& x2);
    friend FloatMP add(CurrentRoundingMode rnd, FloatDP const& x1, FloatMP const& x2);
    friend FloatMP sub(CurrentRoundingMode rnd, FloatDP const& x1, FloatMP const& x2);
    friend FloatMP mul(CurrentRoundingMode rnd, FloatDP const& x1, FloatMP const& x2);
    friend FloatMP div(CurrentRoundingMode rnd, FloatDP const& x1, FloatMP const& x2);

    friend FloatMP add(CurrentRoundingMode rnd, FloatMP const& x1, ExactDouble x2);
    friend FloatMP sub(CurrentRoundingMode rnd, FloatMP const& x1, ExactDouble x2);
    friend FloatMP mul(CurrentRoundingMode rnd, FloatMP const& x1, ExactDouble x2);
    friend FloatMP div(CurrentRoundingMode rnd, FloatMP const& x1, ExactDouble x2);
    friend FloatMP add(CurrentRoundingMode rnd, ExactDouble x1, FloatMP const& x2);
    friend FloatMP sub(CurrentRoundingMode rnd, ExactDouble x1, FloatMP const& x2);
    friend FloatMP mul(CurrentRoundingMode rnd, ExactDouble x1, FloatMP const& x2);
    friend FloatMP div(CurrentRoundingMode rnd, ExactDouble x1, FloatMP const& x2);
    //!@}
  public:
    //! \name Explicitly rounded operations
    //!@{
    friend FloatMP max(RoundingModeType rnd, FloatMP const& x1, FloatMP const& x2); //!< <p/>
    friend FloatMP min(RoundingModeType rnd, FloatMP const& x1, FloatMP const& x2); //!< <p/>
    friend FloatMP abs(RoundingModeType rnd, FloatMP const& x); //!< <p/>
    friend FloatMP mag(RoundingModeType rnd, FloatMP const& x); //!< <p/>

    // Explcitly rounded inplace operations
    friend FloatMP& iadd(RoundingModeType rnd, FloatMP& x1, FloatMP const& x2); //!< <p/>
    friend FloatMP& isub(RoundingModeType rnd, FloatMP& x1, FloatMP const& x2); //!< <p/>
    friend FloatMP& imul(RoundingModeType rnd, FloatMP& x1, FloatMP const& x2); //!< <p/>
    friend FloatMP& idiv(RoundingModeType rnd, FloatMP& x1, FloatMP const& x2); //!< <p/>

    // Explcitly rounded operations
    friend FloatMP nul(RoundingModeType rnd, FloatMP const& x); //!< <p/>
    friend FloatMP pos(RoundingModeType rnd, FloatMP const& x); //!< <p/>
    friend FloatMP neg(RoundingModeType rnd, FloatMP const& x); //!< <p/>
    friend FloatMP hlf(RoundingModeType rnd, FloatMP const& x); //!< <p/>
    friend FloatMP shft(RoundingModeType rnd, FloatMP const& x, Int n); //!< <p/>
    friend FloatMP add(RoundingModeType rnd, FloatMP const& x1, FloatMP const& x2); //!< <p/>
    friend FloatMP sub(RoundingModeType rnd, FloatMP const& x1, FloatMP const& x2); //!< <p/>
    friend FloatMP mul(RoundingModeType rnd, FloatMP const& x1, FloatMP const& x2); //!< <p/>
    friend FloatMP div(RoundingModeType rnd, FloatMP const& x1, FloatMP const& x2); //!< <p/>
    friend FloatMP fma(RoundingModeType rnd, FloatMP const& x1, FloatMP const& x2, FloatMP const& x3); // x1*x2+x3
    friend FloatMP pow(RoundingModeType rnd, FloatMP const& x, Nat m); //!< <p/>
    friend FloatMP pow(RoundingModeType rnd, FloatMP const& x, Int n); //!< <p/>
    friend FloatMP sqr(RoundingModeType rnd, FloatMP const& x); //!< <p/>
    friend FloatMP rec(RoundingModeType rnd, FloatMP const& x); //!< <p/>
    friend FloatMP sqrt(RoundingModeType rnd, FloatMP const& x); //!< <p/>
    friend FloatMP exp(RoundingModeType rnd, FloatMP const& x); //!< <p/>
    friend FloatMP log(RoundingModeType rnd, FloatMP const& x); //!< <p/>
    friend FloatMP sin(RoundingModeType rnd, FloatMP const& x); //!< <p/>
    friend FloatMP cos(RoundingModeType rnd, FloatMP const& x); //!< <p/>
    friend FloatMP tan(RoundingModeType rnd, FloatMP const& x); //!< <p/>
    friend FloatMP asin(RoundingModeType rnd, FloatMP const& x); //!< <p/>
    friend FloatMP acos(RoundingModeType rnd, FloatMP const& x); //!< <p/>
    friend FloatMP atan(RoundingModeType rnd, FloatMP const& x); //!< <p/>
    static FloatMP pi(RoundingModeType rnd, MultiplePrecision pr); //!< <p/>

    friend FloatMP med(RoundingModeType rnd, FloatMP x1, FloatMP x2) { return hlf(add(rnd,x1,x2)); } //!< <p/>
    friend FloatMP rad(RoundingModeType rnd, FloatMP x1, FloatMP x2) { return hlf(sub(rnd,x2,x1)); } //!< <p/>
    //!@}

    // Mixed operations
    friend FloatMP add(RoundingModeType rnd, FloatMP const& x1, ExactDouble x2);
    friend FloatMP sub(RoundingModeType rnd, FloatMP const& x1, ExactDouble x2);
    friend FloatMP mul(RoundingModeType rnd, FloatMP const& x1, ExactDouble x2);
    friend FloatMP div(RoundingModeType rnd, FloatMP const& x1, ExactDouble x2);
    friend FloatMP add(RoundingModeType rnd, ExactDouble x1, FloatMP const& x2);
    friend FloatMP sub(RoundingModeType rnd, ExactDouble x1, FloatMP const& x2);
    friend FloatMP mul(RoundingModeType rnd, ExactDouble x1, FloatMP const& x2);
    friend FloatMP div(RoundingModeType rnd, ExactDouble x1, FloatMP const& x2);

    friend FloatMP add(RoundingModeType rnd, FloatMP const& x1, FloatDP const& x2);
    friend FloatMP sub(RoundingModeType rnd, FloatMP const& x1, FloatDP const& x2);
    friend FloatMP mul(RoundingModeType rnd, FloatMP const& x1, FloatDP const& x2);
    friend FloatMP div(RoundingModeType rnd, FloatMP const& x1, FloatDP const& x2);
    friend FloatMP add(RoundingModeType rnd, FloatDP const& x1, FloatMP const& x2);
    friend FloatMP sub(RoundingModeType rnd, FloatDP const& x1, FloatMP const& x2);
    friend FloatMP mul(RoundingModeType rnd, FloatDP const& x1, FloatMP const& x2);
    friend FloatMP div(RoundingModeType rnd, FloatDP const& x1, FloatMP const& x2);
  private:
    template<class OP, class X1, class X2> static Boolean _cmp(OP op, X1 const& x1, X2 const& x2) {
        return op(cmp(x1,x2),Comparison::EQUAL); }
  public:
    //!@{
    //! \name Exact information tests
    friend Bool same(FloatMP const& x1, FloatMP const& x2) { return x1 == x2; } //!< Test equality of \a x1 and x2.
    //!@}
  public:
    //!@{
    //! \name Input/output operations
    friend OutputStream& operator<<(OutputStream& os, FloatMP const& x); //!< Write to an output stream.
    friend InputStream& operator>>(InputStream& os, FloatMP& x); //!< Read from an input stream.
    //!@}
  private:
    // Mixed operations
    friend Bounds<FloatMP> add(FloatMP const& x1, ExactDouble x2);
    friend Bounds<FloatMP> sub(FloatMP const& x1, ExactDouble x2);
    friend Bounds<FloatMP> mul(FloatMP const& x1, ExactDouble x2);
    friend Bounds<FloatMP> div(FloatMP const& x1, ExactDouble x2);
    friend Bounds<FloatMP> add(ExactDouble x1, FloatMP const& x2);
    friend Bounds<FloatMP> sub(ExactDouble x1, FloatMP const& x2);
    friend Bounds<FloatMP> mul(ExactDouble x1, FloatMP const& x2);
    friend Bounds<FloatMP> div(ExactDouble x1, FloatMP const& x2);

    // Correctly rounded operations
    friend FloatMP sqr_rnd(FloatMP const& x);
    friend FloatMP add_rnd(FloatMP const& x1, FloatMP const& x2);
    friend FloatMP sub_rnd(FloatMP const& x1, FloatMP const& x2);
    friend FloatMP mul_rnd(FloatMP const& x1, FloatMP const& x2);
    friend FloatMP div_rnd(FloatMP const& x1, FloatMP const& x2);
    friend FloatMP pow_rnd(FloatMP const& x, Int n);
    friend FloatMP sqrt_rnd(FloatMP const& x);
    friend FloatMP exp_rnd(FloatMP const& x);
    friend FloatMP log_rnd(FloatMP const& x);
    friend FloatMP sin_rnd(FloatMP const& x);
    friend FloatMP cos_rnd(FloatMP const& x);
    friend FloatMP tan_rnd(FloatMP const& x);
    friend FloatMP atan_rnd(FloatMP const& x);
  public:
  public:
    friend OutputStream& write(OutputStream& os, FloatMP const& x, DecimalPlaces dgts, RoundingModeMP rnd);
    friend OutputStream& write(OutputStream& os, FloatMP const& x, DecimalPrecision dgts, RoundingModeMP rnd);
    friend OutputStream& repr(OutputStream& os, FloatMP const& x);
    friend OutputStream& repr(OutputStream& os, FloatMP const& x, RoundingModeMP rnd);
    friend String print(const mpfr_t x, int zdgts, int fdgts, mpfr_rnd_t rnd);
    friend String print(FloatMP const& x, DecimalPrecision figs, RoundingModeMP rnd);
    friend String print(FloatMP const& x, DecimalPlaces plcs, RoundingModeMP rnd);
  public:
    static Nat output_places;
    static Void set_output_places(Nat pl);
};

template<class R, class A> R integer_cast(const A& a);
template<> inline Int integer_cast(const FloatMP& x) { return static_cast<Int>(x.get_d()); }
template<> inline Nat integer_cast(const FloatMP& x) { return static_cast<Nat>(x.get_d()); }

//inline MultiplePrecision::MultiplePrecision(DefaultTag const&) : MultiplePrecision(FloatMP::get_default_precision()) { }


} // namespace Ariadne

#endif
