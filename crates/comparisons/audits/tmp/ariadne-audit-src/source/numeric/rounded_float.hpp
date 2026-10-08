/***************************************************************************
 *            numeric/rounded_float.hpp
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

/*! \file numeric/rounded_float.hpp
 *  \brief Floating-point numbers supporting rounding-mode changes and rounded arithmetic.
 */

#ifndef ARIADNE_ROUNDED_FLOAT_HPP
#define ARIADNE_ROUNDED_FLOAT_HPP

#include <iosfwd> // For std::floor std::ceil etc
#include <cmath> // For std::floor std::ceil etc
#include <algorithm> // For std::max, std::min
#include <limits> // For std::numeric_limits<double>

#include "utility/declarations.hpp"
#include "numeric/operators.hpp"
#include "numeric/rounding.hpp"
#include "numeric/builtin.hpp"
#include "numeric/sign.hpp"
#include "numeric/number.decl.hpp"
#include "numeric/float.decl.hpp"

#include "numeric/floatdp.hpp"

namespace Ariadne {

class Rational;
enum class Comparison : ComparableEnumerationType;

//! \ingroup NumericModule
//! \brief The precision of a FloatDP object. Since this is fixed, the class is only a tag; all objects are equal.
//! \relates FloatDP
class DoublePrecision;
using DP = DoublePrecision;

template<class FLT> class Rounded;

using RoundedFloatDP = Rounded<FloatDP>;
using RoundedFloatMP = Rounded<FloatMP>;

template<> class Rounded<FloatDP>
{
    FloatDP _flt;
  public:
    typedef RawTag Paradigm;
    typedef FloatDP FloatType;
    typedef Rounded<FloatDP> NumericType;
    typedef DoublePrecision PrecisionType;
    typedef DoublePrecision CharacteristicsType;
    typedef BuiltinRoundingModeType RoundingModeType;
  public:
    static RoundingModeType get_rounding_mode() { return FloatDP::get_rounding_mode(); }
    static Void set_rounding_mode(RoundingModeType rnd) { FloatDP::set_rounding_mode(rnd); }
    static Void set_rounding_downward() { FloatDP::set_rounding_downward(); }
    static Void set_rounding_upward() { FloatDP::set_rounding_upward(); }
    static Void set_rounding_to_nearest() { FloatDP::set_rounding_to_nearest(); }
    static Void set_rounding_toward_zero() { FloatDP::set_rounding_toward_zero(); }
  private:
    explicit Rounded(double d) : _flt(d) { }
  public:
    explicit Rounded(PrecisionType pr) : _flt() { }
    Rounded(FloatType x) : _flt(x.dbl) { }

    template<BuiltinIntegral N>
        Rounded(N n, PrecisionType pr) : _flt(n) { }
    Rounded(ExactDouble d, PrecisionType pr) : _flt(d.get_d()) { }
    Rounded(Dyadic const& w, PrecisionType pr) : Rounded(FloatType(w,pr)) { }
    Rounded(FloatType x, PrecisionType) : _flt(x.dbl) { }
    Rounded(Rounded<FloatType> x, PrecisionType) : _flt(x._flt) { }
    template<class Y> requires Constructible<FloatType,Y,RoundingModeType,PrecisionType> and (not BuiltinIntegral<Y>)
        Rounded(Y const& y, PrecisionType pr) : Rounded(FloatType(y,FloatType::get_rounding_mode(),pr)) { }

    template<BuiltinIntegral N>
        Rounded<FloatType>& operator=(N n) { this->_flt=n; return *this; }
    Rounded<FloatType>& operator=(FloatType x) { this->_flt=x; return *this; }
    Rounded<FloatType>& operator=(ExactDouble x) { this->_flt=x; return *this; }
    template<class Y> requires Constructible<FloatType,Y,RoundingModeType,PrecisionType> and (not BuiltinIntegral<Y>)
        Rounded<FloatType> operator=(Y const& y) { return (*this)=FloatType(y,FloatType::get_rounding_mode(),this->precision()); }

    Rounded(Approximation<FloatDP> const& x) : Rounded(reinterpret_cast<FloatDP const&>(x)) { }
    operator Approximation<FloatDP> () const;

    double data() const { return this->_flt.dbl; }
    FloatType const& raw() const { return this->_flt; }
    explicit operator FloatType() const { return this->_flt; }
    PrecisionType precision() const { return DoublePrecision(); }
    CharacteristicsType characteristics() const { return DoublePrecision();  }

    // Non-finiteness tests
    friend Bool is_nan(Rounded<FloatType> x) { return std::isnan(x._flt.dbl); }

        // Integer conversions
    friend Rounded<FloatDP> floor(Rounded<FloatDP> x) { return Rounded<FloatDP>(std::floor(x._flt.dbl)); }
    friend Rounded<FloatDP> ceil(Rounded<FloatDP> x) { return Rounded<FloatDP>(std::ceil(x._flt.dbl)); }
    friend Rounded<FloatDP> round(Rounded<FloatDP> x) { return Rounded<FloatDP>(std::round(x._flt.dbl)); }

    // Correctly rounded arithmetic
    friend Rounded<FloatDP> nul(Rounded<FloatDP> x) { return Rounded<FloatDP>(nul_rnd(x._flt.dbl)); }
    friend Rounded<FloatDP> pos(Rounded<FloatDP> x) { return Rounded<FloatDP>(pos_rnd(x._flt.dbl)); }
    friend Rounded<FloatDP> neg(Rounded<FloatDP> x) { return Rounded<FloatDP>(neg_rnd(x._flt.dbl)); }
    friend Rounded<FloatDP> sqr(Rounded<FloatDP> x) { return Rounded<FloatDP>(sqr_rnd(x._flt.dbl)); }
    friend Rounded<FloatDP> hlf(Rounded<FloatDP> x) { return Rounded<FloatDP>(hlf_rnd(x._flt.dbl)); }
    friend Rounded<FloatDP> rec(Rounded<FloatDP> x) { return Rounded<FloatDP>(rec_rnd(x._flt.dbl)); }
    friend Rounded<FloatDP> add(Rounded<FloatDP> x1, Rounded<FloatDP> x2) { return Rounded<FloatDP>(add_rnd(x1._flt.dbl,x2._flt.dbl)); }
    friend Rounded<FloatDP> sub(Rounded<FloatDP> x1, Rounded<FloatDP> x2) { return Rounded<FloatDP>(sub_rnd(x1._flt.dbl,x2._flt.dbl)); }
    friend Rounded<FloatDP> mul(Rounded<FloatDP> x1, Rounded<FloatDP> x2) { return Rounded<FloatDP>(mul_rnd(x1._flt.dbl,x2._flt.dbl)); }
    friend Rounded<FloatDP> div(Rounded<FloatDP> x1, Rounded<FloatDP> x2) { return Rounded<FloatDP>(div_rnd(x1._flt.dbl,x2._flt.dbl)); }
    friend Rounded<FloatDP> fma(Rounded<FloatDP> x1, Rounded<FloatDP> x2, Rounded<FloatDP> x3) { return Rounded<FloatDP>(fma_rnd(x1._flt.dbl,x2._flt.dbl,x3._flt.dbl)); }
    friend Rounded<FloatDP> pow(Rounded<FloatDP> x, Int n) { return Rounded<FloatDP>(pow_rnd(x._flt.dbl,n)); }
    friend Rounded<FloatDP> sqrt(Rounded<FloatDP> x) { return Rounded<FloatDP>(sqrt_rnd(x._flt.dbl)); }
    friend Rounded<FloatDP> exp(Rounded<FloatDP> x) { return Rounded<FloatDP>(exp_rnd(x._flt.dbl)); }
    friend Rounded<FloatDP> log(Rounded<FloatDP> x) { return Rounded<FloatDP>(log_rnd(x._flt.dbl)); }
    friend Rounded<FloatDP> sin(Rounded<FloatDP> x) { return Rounded<FloatDP>(sin_rnd(x._flt.dbl)); }
    friend Rounded<FloatDP> cos(Rounded<FloatDP> x) { return Rounded<FloatDP>(cos_rnd(x._flt.dbl)); }
    friend Rounded<FloatDP> tan(Rounded<FloatDP> x) { return Rounded<FloatDP>(tan_rnd(x._flt.dbl)); }
    friend Rounded<FloatDP> asin(Rounded<FloatDP> x) { return Rounded<FloatDP>(asin_rnd(x._flt.dbl)); }
    friend Rounded<FloatDP> acos(Rounded<FloatDP> x) { return Rounded<FloatDP>(acos_rnd(x._flt.dbl)); }
    friend Rounded<FloatDP> atan(Rounded<FloatDP> x) { return Rounded<FloatDP>(atan_rnd(x._flt.dbl)); }
    static Rounded<FloatDP> pi(PrecisionType pr) { return Rounded<FloatDP>(pi_rnd()); }

    friend Rounded<FloatDP> abs(Rounded<FloatDP> x) { return Rounded<FloatDP>(abs_rnd(x._flt.dbl)); }
    friend Rounded<FloatDP> max(Rounded<FloatDP> x1, Rounded<FloatDP> x2) { return Rounded<FloatDP>(max_rnd(x1._flt.dbl,x2._flt.dbl)); }
    friend Rounded<FloatDP> min(Rounded<FloatDP> x1, Rounded<FloatDP> x2) { return Rounded<FloatDP>(min_rnd(x1._flt.dbl,x2._flt.dbl)); }
    friend Rounded<FloatDP> mag(Rounded<FloatDP> x) { return Rounded<FloatDP>(abs_rnd(x._flt.dbl)); }
    friend Rounded<FloatDP> mig(Rounded<FloatDP> x) { return Rounded<FloatDP>(abs_rnd(x._flt.dbl)); }

    friend Rounded<FloatDP> med(Rounded<FloatDP> x1, Rounded<FloatDP> x2) { return Rounded<FloatDP>(hlf_rnd(add_rnd(x1._flt.dbl,x2._flt.dbl))); }
    friend Rounded<FloatDP> rad(Rounded<FloatDP> x1, Rounded<FloatDP> x2) { return Rounded<FloatDP>(hlf_rnd(sub_rnd(x2._flt.dbl,x1._flt.dbl))); }

    friend Rounded<FloatDP> operator+(Rounded<FloatDP> x) { return Rounded<FloatDP>(pos_rnd(x._flt.dbl)); }
    friend Rounded<FloatDP> operator-(Rounded<FloatDP> x) { return Rounded<FloatDP>(neg_rnd(x._flt.dbl)); }
    friend Rounded<FloatDP> operator+(Rounded<FloatDP> x1, Rounded<FloatDP> x2) { return Rounded<FloatDP>(add_rnd(x1._flt.dbl,x2._flt.dbl)); }
    friend Rounded<FloatDP> operator-(Rounded<FloatDP> x1, Rounded<FloatDP> x2) { return Rounded<FloatDP>(sub_rnd(x1._flt.dbl,x2._flt.dbl)); }
    friend Rounded<FloatDP> operator*(Rounded<FloatDP> x1, Rounded<FloatDP> x2) { return Rounded<FloatDP>(mul_rnd(x1._flt.dbl,x2._flt.dbl)); }
    friend Rounded<FloatDP> operator/(Rounded<FloatDP> x1, Rounded<FloatDP> x2) { return Rounded<FloatDP>(div_rnd(x1._flt.dbl,x2._flt.dbl)); }
    friend Rounded<FloatDP>& operator+=(Rounded<FloatDP>& x1, Rounded<FloatDP> x2) { x1._flt.dbl=add_rnd(x1._flt.dbl,x2._flt.dbl); return x1; }
    friend Rounded<FloatDP>& operator-=(Rounded<FloatDP>& x1, Rounded<FloatDP> x2) { x1._flt.dbl=sub_rnd(x1._flt.dbl,x2._flt.dbl); return x1; }
    friend Rounded<FloatDP>& operator*=(Rounded<FloatDP>& x1, Rounded<FloatDP> x2) { x1._flt.dbl=mul_rnd(x1._flt.dbl,x2._flt.dbl); return x1; }
    friend Rounded<FloatDP>& operator/=(Rounded<FloatDP>& x1, Rounded<FloatDP> x2) { x1._flt.dbl=div_rnd(x1._flt.dbl,x2._flt.dbl); return x1; }

    friend Rounded<FloatDP> operator+(Rounded<FloatDP> x1, ExactDouble x2) { return Rounded<FloatDP>(add_rnd(x1._flt.dbl,x2.get_d())); }
    friend Rounded<FloatDP> operator-(Rounded<FloatDP> x1, ExactDouble x2) { return Rounded<FloatDP>(sub_rnd(x1._flt.dbl,x2.get_d())); }
    friend Rounded<FloatDP> operator*(Rounded<FloatDP> x1, ExactDouble x2) { return Rounded<FloatDP>(mul_rnd(x1._flt.dbl,x2.get_d())); }
    friend Rounded<FloatDP> operator/(Rounded<FloatDP> x1, ExactDouble x2) { return Rounded<FloatDP>(div_rnd(x1._flt.dbl,x2.get_d())); }
    friend Rounded<FloatDP> operator+(ExactDouble x1, Rounded<FloatDP> x2) { return Rounded<FloatDP>(add_rnd(x1.get_d(),x2._flt.dbl)); }
    friend Rounded<FloatDP> operator-(ExactDouble x1, Rounded<FloatDP> x2) { return Rounded<FloatDP>(sub_rnd(x1.get_d(),x2._flt.dbl)); }
    friend Rounded<FloatDP> operator*(ExactDouble x1, Rounded<FloatDP> x2) { return Rounded<FloatDP>(mul_rnd(x1.get_d(),x2._flt.dbl)); }
    friend Rounded<FloatDP> operator/(ExactDouble x1, Rounded<FloatDP> x2) { return Rounded<FloatDP>(div_rnd(x1.get_d(),x2._flt.dbl)); }
    friend Rounded<FloatDP>& operator+=(Rounded<FloatDP>& x1, ExactDouble x2) { return x1=x1+x2; }
    friend Rounded<FloatDP>& operator-=(Rounded<FloatDP>& x1, ExactDouble x2) { return x1=x1-x2; }
    friend Rounded<FloatDP>& operator*=(Rounded<FloatDP>& x1, ExactDouble x2) { return x1=x1*x2; }
    friend Rounded<FloatDP>& operator/=(Rounded<FloatDP>& x1, ExactDouble x2) { return x1=x1/x2; }

    friend Rounded<FloatDP> max(Rounded<FloatDP> x1, ExactDouble x2) { return Rounded<FloatDP>(max_rnd(x1._flt.dbl,x2.get_d())); }
    friend Rounded<FloatDP> min(Rounded<FloatDP> x1, ExactDouble x2) { return Rounded<FloatDP>(min_rnd(x1._flt.dbl,x2.get_d())); }
    friend Rounded<FloatDP> max(ExactDouble x1, Rounded<FloatDP> x2) { return Rounded<FloatDP>(max_rnd(x1.get_d(),x2._flt.dbl)); }
    friend Rounded<FloatDP> min(ExactDouble x1, Rounded<FloatDP> x2) { return Rounded<FloatDP>(min_rnd(x1.get_d(),x2._flt.dbl)); }

    friend ApproximateKleenean operator==(Rounded<FloatDP> x1, Rounded<FloatDP> x2) { return x1._flt.dbl == x2._flt.dbl; }
    friend ApproximateKleenean operator!=(Rounded<FloatDP> x1, Rounded<FloatDP> x2) { return x1._flt.dbl != x2._flt.dbl; }
    friend ApproximateKleenean operator<=(Rounded<FloatDP> x1, Rounded<FloatDP> x2) { return x1._flt.dbl <= x2._flt.dbl; }
    friend ApproximateKleenean operator>=(Rounded<FloatDP> x1, Rounded<FloatDP> x2) { return x1._flt.dbl >= x2._flt.dbl; }
    friend ApproximateKleenean operator< (Rounded<FloatDP> x1, Rounded<FloatDP> x2) { return x1._flt.dbl <  x2._flt.dbl; }
    friend ApproximateKleenean operator> (Rounded<FloatDP> x1, Rounded<FloatDP> x2) { return x1._flt.dbl >  x2._flt.dbl; }

    friend ApproximateKleenean operator==(Rounded<FloatDP> x1, ExactDouble x2) { return x1._flt.dbl == x2.get_d(); }
    friend ApproximateKleenean operator!=(Rounded<FloatDP> x1, ExactDouble x2) { return x1._flt.dbl != x2.get_d(); }
    friend ApproximateKleenean operator<=(Rounded<FloatDP> x1, ExactDouble x2) { return x1._flt.dbl <= x2.get_d(); }
    friend ApproximateKleenean operator>=(Rounded<FloatDP> x1, ExactDouble x2) { return x1._flt.dbl >= x2.get_d(); }
    friend ApproximateKleenean operator< (Rounded<FloatDP> x1, ExactDouble x2) { return x1._flt.dbl <  x2.get_d(); }
    friend ApproximateKleenean operator> (Rounded<FloatDP> x1, ExactDouble x2) { return x1._flt.dbl >  x2.get_d(); }

    friend ApproximateKleenean operator==(ExactDouble x1, Rounded<FloatDP> x2) { return x1.get_d() == x2._flt.dbl; }
    friend ApproximateKleenean operator!=(ExactDouble x1, Rounded<FloatDP> x2) { return x1.get_d() != x2._flt.dbl; }
    friend ApproximateKleenean operator<=(ExactDouble x1, Rounded<FloatDP> x2) { return x1.get_d() <= x2._flt.dbl; }
    friend ApproximateKleenean operator>=(ExactDouble x1, Rounded<FloatDP> x2) { return x1.get_d() >= x2._flt.dbl; }
    friend ApproximateKleenean operator< (ExactDouble x1, Rounded<FloatDP> x2) { return x1.get_d() <  x2._flt.dbl; }
    friend ApproximateKleenean operator> (ExactDouble x1, Rounded<FloatDP> x2) { return x1.get_d() >  x2._flt.dbl; }

    friend Bool same(Rounded<FloatDP> x1, Rounded<FloatDP> x2) { return x1._flt == x2._flt; }

    friend OutputStream& operator<<(OutputStream& os, Rounded<FloatDP> const& x) { return os << x.data(); }
};

template<class FLT> class Rounded
{
    static const CurrentRoundingMode _rnd;
    FLT _flt;
  public:
    typedef RawTag Paradigm;
    typedef FLT FloatType;
    typedef Rounded<FloatType> NumericType;
    typedef typename FloatType::PrecisionType PrecisionType;
    typedef typename FloatType::PrecisionType CharacteristicsType;
    typedef typename FloatType::RoundingModeType RoundingModeType;
  public:
    static RoundingModeType get_rounding_mode() { return FloatType::get_rounding_mode(); }
    static Void set_rounding_mode(RoundingModeType rnd) { FloatType::set_rounding_mode(rnd); }
    static Void set_rounding_downward() { FloatType::set_rounding_downward(); }
    static Void set_rounding_upward() { FloatType::set_rounding_upward(); }
    static Void set_rounding_to_nearest() { FloatType::set_rounding_to_nearest(); }
    static Void set_rounding_toward_zero() { FloatType::set_rounding_toward_zero(); }
  public:
    FloatType data() const { return _flt; }
    PrecisionType precision() const { return this->_flt.precision(); }
    CharacteristicsType characteristics() const { return this->_flt.precision(); }

    explicit Rounded(PrecisionType pr) : _flt(pr) { }
    Rounded(FloatType x) : _flt(x) { }
    Rounded(Rounded<FloatType> x, PrecisionType) : _flt(x._flt) { }

    template<class Y> requires Constructible<FloatType,Y,RoundingModeType,PrecisionType>
        Rounded(Y const& y, PrecisionType pr) : Rounded(FloatType(y,FloatType::get_rounding_mode(),pr)) { }

    template<class Y> requires Constructible<FloatType,Y,RoundingModeType,PrecisionType>
        Rounded<FloatType> operator=(Y const& y) { this->_flt=FloatType(y,FloatType::get_rounding_mode(),this->precision()); return *this; }

    explicit operator FloatType() const { return FloatType(this->_flt); }
    FloatType raw() const { return FloatType(this->_flt); }

    Rounded(Approximation<FloatType> const& x) : Rounded(reinterpret_cast<FloatType const&>(x)) { }
    operator Approximation<FloatType> () const;

    // Non-finiteness tests
    friend Bool is_nan(Rounded<FloatType> x) { return is_nan(x._flt); }

    // Integer conversions
    friend Rounded<FloatType> floor(Rounded<FloatType> x) { return Rounded<FloatType>(floor(x._flt)); }
    friend Rounded<FloatType> ceil(Rounded<FloatType> x) { return Rounded<FloatType>(ceil(x._flt)); }
    friend Rounded<FloatType> round(Rounded<FloatType> x) { return Rounded<FloatType>(round(x._flt)); }

    // Correctly rounded arithmetic
    friend Rounded<FloatType> nul(Rounded<FloatType> x) { return Rounded<FloatType>(nul(x._flt)); }
    friend Rounded<FloatType> pos(Rounded<FloatType> x) { return Rounded<FloatType>(pos(x._flt)); }
    friend Rounded<FloatType> neg(Rounded<FloatType> x) { return Rounded<FloatType>(neg(x._flt)); }
    friend Rounded<FloatType> hlf(Rounded<FloatType> x) { return Rounded<FloatType>(hlf(x._flt)); }
    friend Rounded<FloatType> sqr(Rounded<FloatType> x) { return Rounded<FloatType>(sqr(_rnd,x._flt)); }
    friend Rounded<FloatType> rec(Rounded<FloatType> x) { return Rounded<FloatType>(rec(_rnd,x._flt)); }
    friend Rounded<FloatType> add(Rounded<FloatType> x1, Rounded<FloatType> x2) { return Rounded<FloatType>(add(_rnd,x1._flt,x2._flt)); }
    friend Rounded<FloatType> sub(Rounded<FloatType> x1, Rounded<FloatType> x2) { return Rounded<FloatType>(sub(_rnd,x1._flt,x2._flt)); }
    friend Rounded<FloatType> mul(Rounded<FloatType> x1, Rounded<FloatType> x2) { return Rounded<FloatType>(mul(_rnd,x1._flt,x2._flt)); }
    friend Rounded<FloatType> div(Rounded<FloatType> x1, Rounded<FloatType> x2) { return Rounded<FloatType>(div(_rnd,x1._flt,x2._flt)); }
    friend Rounded<FloatType> fma(Rounded<FloatType> x1, Rounded<FloatType> x2, Rounded<FloatType> x3) { return Rounded<FloatType>(fma(_rnd,x1._flt,x2._flt,x3._flt)); }
    friend Rounded<FloatType> pow(Rounded<FloatType> x, Int n) { return Rounded<FloatType>(pow(_rnd,x._flt,n)); }
    friend Rounded<FloatType> sqrt(Rounded<FloatType> x) { return Rounded<FloatType>(sqrt(_rnd,x._flt)); }
    friend Rounded<FloatType> exp(Rounded<FloatType> x) { return Rounded<FloatType>(exp(_rnd,x._flt)); }
    friend Rounded<FloatType> log(Rounded<FloatType> x) { return Rounded<FloatType>(log(_rnd,x._flt)); }
    friend Rounded<FloatType> sin(Rounded<FloatType> x) { return Rounded<FloatType>(sin(_rnd,x._flt)); }
    friend Rounded<FloatType> cos(Rounded<FloatType> x) { return Rounded<FloatType>(cos(_rnd,x._flt)); }
    friend Rounded<FloatType> tan(Rounded<FloatType> x) { return Rounded<FloatType>(tan(_rnd,x._flt)); }
    friend Rounded<FloatType> asin(Rounded<FloatType> x) { return Rounded<FloatType>(asin(_rnd,x._flt)); }
    friend Rounded<FloatType> acos(Rounded<FloatType> x) { return Rounded<FloatType>(acos(_rnd,x._flt)); }
    friend Rounded<FloatType> atan(Rounded<FloatType> x) { return Rounded<FloatType>(atan(_rnd,x._flt)); }
    static Rounded<FloatType> pi(PrecisionType pr) { return Rounded<FloatType>(FloatType::pi(FloatType::get_rounding_mode(),pr)); }

    friend Rounded<FloatType> abs(Rounded<FloatType> x) { return Rounded<FloatType>(abs(x._flt)); }
    friend Rounded<FloatType> max(Rounded<FloatType> x1, Rounded<FloatType> x2) { return Rounded<FloatType>(max(x1._flt,x2._flt)); }
    friend Rounded<FloatType> min(Rounded<FloatType> x1, Rounded<FloatType> x2) { return Rounded<FloatType>(min(x1._flt,x2._flt)); }
    friend Rounded<FloatType> mag(Rounded<FloatType> x) { return Rounded<FloatType>(abs(x._flt)); }
    friend Rounded<FloatType> mig(Rounded<FloatType> x) { return Rounded<FloatType>(abs(x._flt)); }

    friend Rounded<FloatType> med(Rounded<FloatType> x1, Rounded<FloatType> x2) {
        return Rounded<FloatType>(hlf(add(CurrentRoundingMode(),x1._flt,x2._flt))); }
    friend Rounded<FloatType> rad(Rounded<FloatType> x1, Rounded<FloatType> x2) {
        return Rounded<FloatType>(hlf(sub(CurrentRoundingMode(),x2._flt,x1._flt))); }

    friend Rounded<FloatType> operator+(Rounded<FloatType> x) { return Rounded<FloatType>(pos(x._flt)); }
    friend Rounded<FloatType> operator-(Rounded<FloatType> x) { return Rounded<FloatType>(neg(x._flt)); }
    friend Rounded<FloatType> operator+(Rounded<FloatType> x1, Rounded<FloatType> x2) {
        return Rounded<FloatType>(add(_rnd,x1._flt,x2._flt)); }
    friend Rounded<FloatType> operator-(Rounded<FloatType> x1, Rounded<FloatType> x2) {
        return Rounded<FloatType>(sub(_rnd,x1._flt,x2._flt)); }
    friend Rounded<FloatType> operator*(Rounded<FloatType> x1, Rounded<FloatType> x2) {
        return Rounded<FloatType>(mul(_rnd,x1._flt,x2._flt)); }
    friend Rounded<FloatType> operator/(Rounded<FloatType> x1, Rounded<FloatType> x2) {
        return Rounded<FloatType>(div(_rnd,x1._flt,x2._flt)); }
    friend Rounded<FloatType>& operator+=(Rounded<FloatType>& x1, Rounded<FloatType> x2) { return x1=x1+x2; }
    friend Rounded<FloatType>& operator-=(Rounded<FloatType>& x1, Rounded<FloatType> x2) { return x1=x1-x2; }
    friend Rounded<FloatType>& operator*=(Rounded<FloatType>& x1, Rounded<FloatType> x2) { return x1=x1*x2; }
    friend Rounded<FloatType>& operator/=(Rounded<FloatType>& x1, Rounded<FloatType> x2) { return x1=x1/x2; }

    friend Rounded<FloatType> operator+(Rounded<FloatType> x1, ExactDouble x2) { return x1+FloatMP(x2,x1.precision()); }
    friend Rounded<FloatType> operator-(Rounded<FloatType> x1, ExactDouble x2) { return x1-FloatMP(x2,x1.precision()); }
    friend Rounded<FloatType> operator*(Rounded<FloatType> x1, ExactDouble x2) { return x1*FloatMP(x2,x1.precision()); }
    friend Rounded<FloatType> operator/(Rounded<FloatType> x1, ExactDouble x2) { return x1/FloatMP(x2,x1.precision()); }
    friend Rounded<FloatType> operator+(ExactDouble x1, Rounded<FloatType> x2) { return FloatMP(x1,x2.precision())+x2; }
    friend Rounded<FloatType> operator-(ExactDouble x1, Rounded<FloatType> x2) { return FloatMP(x1,x2.precision())-x2; }
    friend Rounded<FloatType> operator*(ExactDouble x1, Rounded<FloatType> x2) { return FloatMP(x1,x2.precision())*x2; }
    friend Rounded<FloatType> operator/(ExactDouble x1, Rounded<FloatType> x2) { return FloatMP(x1,x2.precision())/x2; }
    friend Rounded<FloatType>& operator+=(Rounded<FloatType>& x1, ExactDouble x2) { return x1=x1+x2; }
    friend Rounded<FloatType>& operator-=(Rounded<FloatType>& x1, ExactDouble x2) { return x1=x1-x2; }
    friend Rounded<FloatType>& operator*=(Rounded<FloatType>& x1, ExactDouble x2) { return x1=x1*x2; }
    friend Rounded<FloatType>& operator/=(Rounded<FloatType>& x1, ExactDouble x2) { return x1=x1/x2; }

    friend Rounded<FloatType> max(Rounded<FloatType> x1, ExactDouble x2) { return Rounded<FloatType>(max(x1._flt,x2)); }
    friend Rounded<FloatType> min(Rounded<FloatType> x1, ExactDouble x2) { return Rounded<FloatType>(min(x1._flt,x2)); }
    friend Rounded<FloatType> max(ExactDouble x1, Rounded<FloatType> x2) { return Rounded<FloatType>(max(x1,x2._flt)); }
    friend Rounded<FloatType> min(ExactDouble x1, Rounded<FloatType> x2) { return Rounded<FloatType>(min(x1,x2._flt)); }

    friend ApproximateKleenean operator==(Rounded<FloatType> x1, Rounded<FloatType> x2) { return x1._flt == x2._flt; }
    friend ApproximateKleenean operator!=(Rounded<FloatType> x1, Rounded<FloatType> x2) { return x1._flt != x2._flt; }
    friend ApproximateKleenean operator<=(Rounded<FloatType> x1, Rounded<FloatType> x2) { return x1._flt <= x2._flt; }
    friend ApproximateKleenean operator>=(Rounded<FloatType> x1, Rounded<FloatType> x2) { return x1._flt >= x2._flt; }
    friend ApproximateKleenean operator< (Rounded<FloatType> x1, Rounded<FloatType> x2) { return x1._flt <  x2._flt; }
    friend ApproximateKleenean operator> (Rounded<FloatType> x1, Rounded<FloatType> x2) { return x1._flt >  x2._flt; }

    friend ApproximateKleenean operator==(Rounded<FloatType> x1, ExactDouble x2) { return x1._flt == x2; }
    friend ApproximateKleenean operator!=(Rounded<FloatType> x1, ExactDouble x2) { return x1._flt != x2; }
    friend ApproximateKleenean operator<=(Rounded<FloatType> x1, ExactDouble x2) { return x1._flt <= x2; }
    friend ApproximateKleenean operator>=(Rounded<FloatType> x1, ExactDouble x2) { return x1._flt >= x2; }
    friend ApproximateKleenean operator< (Rounded<FloatType> x1, ExactDouble x2) { return x1._flt <  x2; }
    friend ApproximateKleenean operator> (Rounded<FloatType> x1, ExactDouble x2) { return x1._flt >  x2; }

    friend ApproximateKleenean operator==(ExactDouble x1, Rounded<FloatType> x2) { return x1 == x2._flt; }
    friend ApproximateKleenean operator!=(ExactDouble x1, Rounded<FloatType> x2) { return x1 != x2._flt; }
    friend ApproximateKleenean operator<=(ExactDouble x1, Rounded<FloatType> x2) { return x1 <= x2._flt; }
    friend ApproximateKleenean operator>=(ExactDouble x1, Rounded<FloatType> x2) { return x1 >= x2._flt; }
    friend ApproximateKleenean operator< (ExactDouble x1, Rounded<FloatType> x2) { return x1 <  x2._flt; }
    friend ApproximateKleenean operator> (ExactDouble x1, Rounded<FloatType> x2) { return x1 >  x2._flt; }

    friend Bool same(Rounded<FloatType> x1, Rounded<FloatType> x2) { return x1._flt == x2._flt; }

    friend OutputStream& operator<<(OutputStream& os, Rounded<FloatType> const& x) { return os << x.data(); }
};

template<class FLT> const CurrentRoundingMode Rounded<FLT>::_rnd=CurrentRoundingMode();

} // namespace Ariadne

#endif
