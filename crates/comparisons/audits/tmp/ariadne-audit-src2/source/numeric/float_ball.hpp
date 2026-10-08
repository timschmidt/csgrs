/***************************************************************************
 *            numeric/float_ball.hpp
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

/*! \file numeric/float_ball.hpp
 *  \brief Floating-point approximations to real numbers.
 */

#ifndef ARIADNE_FLOAT_BALL_HPP
#define ARIADNE_FLOAT_BALL_HPP

#include "utility/macros.hpp"

#include "foundations/logical.decl.hpp"
#include "number.decl.hpp"
#include "float.decl.hpp"

#include "positive.hpp"

#include "float_operations.hpp"
#include "float_traits.hpp"
#include "float_factory.hpp"

namespace Ariadne {

template<class PRE, class PR> requires DefaultConstructible<PRE> inline
    PRE _error_precision(PR const&) { return PRE(); }
template<class PRE, class PR> requires (not DefaultConstructible<PRE>) and Constructible<PRE,PR> inline
    PRE _error_precision(PR const& pr) { return PRE(pr); }

//! \ingroup NumericModule
//! \brief Floating point approximations to a real number with guaranteed error bounds.
//! \sa Real, FloatDP, FloatMP, Error, Float, Bounds, Approximation.
template<class F, class FE> class Ball
    : public DefineFloatOperations<Ball<F,FE>>
    , public ProvideConvertedFieldOperations<Bounds<F>,Ball<F,FE>>
{
    typedef ValidatedTag P; typedef typename F::PrecisionType PR; typedef typename FE::PrecisionType PRE;
    static_assert(Constructible<PR,PRE> or DefaultConstructible<PRE>);
  public:
    //! <p/>
    typedef P Paradigm;
    //! <p/>
    typedef Ball<F,FE> NumericType;
    //! <p/>
    typedef Number<P> GenericType;
    //! <p/>
    typedef F RawType;
    //! <p/>
    typedef FE RawErrorType;
    //! <p/>
    typedef PR PrecisionType;
    //! <p/>
    typedef PRE ErrorPrecisionType;
    //! <p/>
    typedef Pair<PR,PRE> CharacteristicsType;
  public:
    //! Construct a ball of radius \f$0\f$ about zero, using precision \a pr for the centre (value),
    //! and using either value \a pr for the precision of the radius (error), or the default value.
    explicit Ball(PrecisionType pr) : _v(0.0_x,pr), _e(0.0_x,_error_precision<PRE>(pr)) { }
    //! Construct a ball of radius \f$0\f$ about zero,
    //! using precision \a pr for the centre (value), and \a pre for the radius (error).
    explicit Ball(PrecisionType pr, ErrorPrecisionType pre) : _v(0.0_x,pr), _e(0.0_x,pre) { }
    explicit Ball(CharacteristicsType prs) : _v(0.0_x,std::get<0>(prs)), _e(0.0_x,std::get<1>(prs)) { }
    Ball(F const& v) : _v(v), _e(0.0_x,_error_precision<PRE>(v.precision())) { }
    explicit Ball(F const& v, PRE pre) : _v(v), _e(0.0_x,pre) { }
    //! Construct a ball of radius \a e about \a v.
    explicit Ball(F const& v, FE const& e) : _v(v), _e(e) { }
    //! Construct a ball of radius \a error about \a v.
    Ball(F const& v, Error<FE> const& error);
    //! Construct a ball containing the bounds \a x, with error bound of precision \a pre.
    Ball(Bounds<F> const& x, PRE pre);
    Ball(LowerBound<F> const& lower, UpperBound<F> const& upper) = delete;

    Ball(Decimal const& dv, Decimal const& de, PrecisionType pr);

    Ball(const ExactDouble& d, PR pr);
        Ball(const TwoExp& t, PR pr);
        Ball(const Integer& z, PR pr);
        Ball(const Dyadic& w, PR pr);
        Ball(const Decimal& d, PR pr);
        Ball(const Rational& q, PR pr);
        Ball(const Real& r, PR pr);
        Ball(const Ball<F,FE>& x, PR pr);
    Ball(const ValidatedNumber& y, PR pr);

    // FIXME: Constructors for other types
        Ball(const Integer& z, PR pr, PRE pre);
        Ball(const Dyadic& w, PR pr, PRE pre);
        Ball(const Rational& q, PR pr, PRE pre);
        Ball(const Real& q, PR pr, PRE pre);
    //! Construct a ball guaranteed to contain the generic validated number \a y,
    //! using precision \a pr for the centre (value), and \a pre for the radius (error).
    Ball(const ValidatedNumber& y, PR pr, PRE pre);

    //! Construct a ball containing the bounds \a x.
    //! The precision of the error is either that of \a F, or the default value.
    explicit Ball(Bounds<F> const& x);

    //! Assign from generic validated bounds \a y, keeping the same characteristics.
    Ball<F,FE>& operator=(const ValidatedNumber& y) { return *this=Ball<F,FE>(y,this->precision(),this->error_precision()); }

    //! Downcast to a generic validated value.
    operator ValidatedNumber () const;

    //! Create a ball from the generic validated number \a y with the same characteristics as \a this.
    Ball<F,FE> create(const ValidatedNumber& y) const { return Ball<F,FE>(y,this->precision(),this->error_precision()); }

    //! A lower bound.
    LowerBound<F> const lower() const;
    //! An upper bound.
    UpperBound<F> const upper() const;
    //! The centre value of the ball.
    F const value() const;
    //! The radius of the ball, giving a bound on the error of \ref value().
    Error<FE> const error() const;

    friend F value(Ball<F,FE> const& x) { return x.value(); }
    friend Error<FE> error(Ball<F,FE> const& x) { return x.error(); }

    RawType const lower_raw() const { return sub(down,_v,_e); }
    RawType const upper_raw() const { return add(up,_v,_e); }
    RawType const& value_raw() const { return _v; }
    RawErrorType const& error_raw() const { return _e; }
    //! Approximate by a builtin double-precision value. DEPRECATED
    double get_d() const { return _v.get_d(); }

    //! The precision of the floating-point type used for the centre value.
    PrecisionType precision() const { return _v.precision(); }
    //! The precision of the floating-point type used for the error bound.
    ErrorPrecisionType error_precision() const { return _e.precision(); }
    //! The compuational characteristics needed to create the ball; equivalent to the precision and error-precision.
    CharacteristicsType characteristics() const { return std::make_pair(_v.precision(),_e.precision()); }
    //! Downcast to a generic validated ball.
    GenericType generic() const { return this->operator GenericType(); }
    //! Add \a e to error bound.
    Ball<F,FE> pm(Error<FE> const& e) const;
    friend Approximation<F> round(Approximation<F> const& x);
  public:
#ifdef DOXYGEN
    //!@{
    //! \name Arithmetic operators
    friend Ball<F,FE> operator+(Ball<F,FE> const& x); //!< <p/>
    friend Ball<F,FE> operator-(Ball<F,FE> const& x); //!< <p/>
    friend Ball<F,FE> operator+(Ball<F,FE> const& x1, Ball<F,FE> const& x2); //!< <p/>
    friend Ball<F,FE> operator-(Ball<F,FE> const& x1, Ball<F,FE> const& x2); //!< <p/>
    friend Ball<F,FE> operator*(Ball<F,FE> const& x1, Ball<F,FE> const& x2); //!< <p/>
    friend Ball<F,FE> operator/(Ball<F,FE> const& x1, Ball<F,FE> const& x2); //!< <p/>
    friend Ball<F,FE>& operator+=(Ball<F,FE>& x1, Ball<F,FE> const& x2); //!< <p/>
    friend Ball<F,FE>& operator-=(Ball<F,FE>& x1, Ball<F,FE> const& x2); //!< <p/>
    friend Ball<F,FE>& operator*=(Ball<F,FE>& x1, Ball<F,FE> const& x2); //!< <p/>
    friend Ball<F,FE>& operator/=(Ball<F,FE>& x1, Ball<F,FE> const& x2); //!< <p/>
    //!@}

    //!@{
    //! \name Comparison operators
    friend ValidatedKleenean operator==(Ball<F,FE> const& x1, Ball<F,FE> const& x2); //!< <p/>
    friend ValidatedKleenean operator!=(Ball<F,FE> const& x1, Ball<F,FE> const& x2); //!< <p/>
    friend ValidatedKleenean operator<=(Ball<F,FE> const& x1, Ball<F,FE> const& x2); //!< <p/>
    friend ValidatedKleenean operator>=(Ball<F,FE> const& x1, Ball<F,FE> const& x2); //!< <p/>
    friend ValidatedKleenean operator< (Ball<F,FE> const& x1, Ball<F,FE> const& x2); //!< <p/>
    friend ValidatedKleenean operator> (Ball<F,FE> const& x1, Ball<F,FE> const& x2); //!< <p/>
    //!@}
#endif // DOXYGEN

    //!@{
    //! \name Arithmetic operations
    friend Ball<F,FE> nul(Ball<F,FE> const& x) {
        return Ball<F,FE>(nul(x._v),nul(x._e)); } //!< <p/>
    friend Ball<F,FE> pos(Ball<F,FE> const& x) {
        return Ball<F,FE>(pos(x._v),x._e); } //!< <p/>
    friend Ball<F,FE> neg(Ball<F,FE> const& x) {
        return Ball<F,FE>(neg(x._v),x._e); } //!< <p/>
    friend Ball<F,FE> hlf(Ball<F,FE> const& x) {
        return Ball<F,FE>(hlf(x._v),hlf(x._e)); } //!< <p/>
    friend Ball<F,FE> sqr(Ball<F,FE> const& x) {
        return Operations<Ball<F,FE>>::_sqr(x); } //!< <p/>
    friend Ball<F,FE> rec(Ball<F,FE> const& x) {
        return Operations<Ball<F,FE>>::_rec(x); } //!< <p/>

    friend Ball<F,FE> add(Ball<F,FE> const& x1, Ball<F,FE> const& x2) {
        return Operations<Ball<F,FE>>::_add(x1,x2); } //!< <p/>
    friend Ball<F,FE> sub(Ball<F,FE> const& x1, Ball<F,FE> const& x2) {
        return Operations<Ball<F,FE>>::_sub(x1,x2); } //!< <p/>
    friend Ball<F,FE> mul(Ball<F,FE> const& x1, Ball<F,FE> const& x2) {
        return Operations<Ball<F,FE>>::_mul(x1,x2); } //!< <p/>
    friend Ball<F,FE> div(Ball<F,FE> const& x1, Ball<F,FE> const& x2) {
        return Operations<Ball<F,FE>>::_div(x1,x2); } //!< <p/>
    friend Ball<F,FE> fma(Ball<F,FE> const& x1, Ball<F,FE> const& x2, Ball<F,FE> const& x3) {
        return Operations<Ball<F,FE>>::_fma(x1,x2,x3); } //!< <p/>
    friend Ball<F,FE> pow(Ball<F,FE> const& x, Int n) {
        return Operations<Ball<F,FE>>::_pow(x,n); } //!< <p/>
    friend Ball<F,FE> pow(Ball<F,FE> const& x, Nat m) {
        return Operations<Ball<F,FE>>::_pow(x,m); } //!< <p/>
    //!@}

    //!@{
    //! \name Algebraic and transcendental operations
    friend Ball<F,FE> sqrt(Ball<F,FE> const& x) {
        return Ball<F,FE>(sqrt(Bounds<F>(x)),x.error_precision()); } //!< <p/>
    friend Ball<F,FE> exp(Ball<F,FE> const& x) {
        return Ball<F,FE>(exp(Bounds<F>(x)),x.error_precision()); } //!< <p/>
    friend Ball<F,FE> log(Ball<F,FE> const& x) {
        return Ball<F,FE>(log(Bounds<F>(x)),x.error_precision()); } //!< <p/>
    friend Ball<F,FE> sin(Ball<F,FE> const& x) {
        return Ball<F,FE>(sin(Bounds<F>(x)),x.error_precision()); } //!< <p/>
    friend Ball<F,FE> cos(Ball<F,FE> const& x) {
        return Ball<F,FE>(cos(Bounds<F>(x)),x.error_precision()); } //!< <p/>
    friend Ball<F,FE> tan(Ball<F,FE> const& x) {
        return Ball<F,FE>(tan(Bounds<F>(x)),x.error_precision()); } //!< <p/>
    friend Ball<F,FE> asin(Ball<F,FE> const& x) {
        return Ball<F,FE>(asin(Bounds<F>(x)),x.error_precision()); } //!< <p/>
    friend Ball<F,FE> acos(Ball<F,FE> const& x) {
        return Ball<F,FE>(acos(Bounds<F>(x)),x.error_precision()); } //!< <p/>
    friend Ball<F,FE> atan(Ball<F,FE> const& x) {
        return Ball<F,FE>(atan(Bounds<F>(x)),x.error_precision()); } //!< <p/>
    //!@}

    //!@{
    //! \name Lattice operations
    friend Ball<F,FE> abs(Ball<F,FE> const& x) {
        return Operations<Ball<F,FE>>::_abs(x); } //!< <p/>
    friend Ball<F,FE> max(Ball<F,FE> const& x1, Ball<F,FE> const& x2) {
        return Operations<Ball<F,FE>>::_max(x1,x2); } //!< <p/>
    friend Ball<F,FE> min(Ball<F,FE> const& x1, Ball<F,FE> const& x2) {
        return Operations<Ball<F,FE>>::_min(x1,x2); } //!< <p/>
    friend PositiveLowerBound<F> mig(Ball<F,FE> const& x) {
        return Operations<Ball<F,FE>>::_mig(x); } //!< <p/>
    friend PositiveUpperBound<F> mag(Ball<F,FE> const& x) {
        return Operations<Ball<F,FE>>::_mag(x); } //!< <p/>
    //!@}

    //!@{
    //! \name Comparison operations
    friend ValidatedKleenean sgn(Ball<F,FE> const& x) {
        if (x._v>x._e) { return true; } else if (x._v<-x._e) { return false; } else { return indeterminate; } } //!< <p/>
    //! \brief Equality comparison operator. Tests equality of represented real-point value.
    friend LogicalType<ValidatedTag> eq(Ball<F,FE> const& x1, Ball<F,FE> const& x2) {
        return Operations<Ball<F,FE>>::_eq(x1,x2); } //!< <p/>
    //! \brief Strict less-than comparison operator. Tests equality of represented real-point value.
    friend LogicalType<ValidatedTag> lt(Ball<F,FE> const& x1, Ball<F,FE> const& x2) {
        return Operations<Ball<F,FE>>::_lt(x1,x2); } //!< <p/>
    //!@}


/*
public:
    // Mixed Ball-ValidatedNumber operations
    template<AValidatedNumber Y> friend Ball<F,FE> operator+(Ball<F,FE> const& x1, Y const& y2) {
        if constexpr (AnExactDyadic<Y>) { return operator+(x1,F(y2,x1.precision())); }
        else { return operator+(x1,Ball<F,FE>(y2,x1.precision(),x1.error_precision())); } }
    template<AValidatedNumber Y> friend Ball<F,FE> operator-(Ball<F,FE> const& x1, Y const& y2) {
        if constexpr (AnExactDyadic<Y>) { return operator-(x1,F(y2,x1.precision())); }
        else { return operator-(x1,Ball<F,FE>(y2,x1.precision(),x1.error_precision())); } }
    template<AValidatedNumber Y> friend Ball<F,FE> operator*(Ball<F,FE> const& x1, Y const& y2) {
        if constexpr (AnExactDyadic<Y>) { return operator*(x1,F(y2,x1.precision())); }
        else { return operator*(x1,Ball<F,FE>(y2,x1.precision(),x1.error_precision())); } }
    template<AValidatedNumber Y> friend Ball<F,FE> operator/(Ball<F,FE> const& x1, Y const& y2) {
        if constexpr (AnExactDyadic<Y>) { return operator/(x1,F(y2,x1.precision())); }
        else { return operator/(x1,Ball<F,FE>(y2,x1.precision(),x1.error_precision())); } }
    template<AValidatedNumber Y> friend Ball<F,FE> operator+(Y const& y1, Ball<F,FE> const& x2) {
        if constexpr (AnExactDyadic<Y>) { return operator+(F(y1,x2.precision()),x2); }
        else { return operator+(Ball<F,FE>(y1,x2.precision(),x2.error_precision()),x2); } }
    template<AValidatedNumber Y> friend Ball<F,FE> operator-(Y const& y1, Ball<F,FE> const& x2) {
        if constexpr (AnExactDyadic<Y>) { return operator-(F(y1,x2.precision()),x2); }
        else { return operator-(Ball<F,FE>(y1,x2.precision(),x2.error_precision()),x2); } }
    template<AValidatedNumber Y> friend Ball<F,FE> operator*(Y const& y1, Ball<F,FE> const& x2) {
        if constexpr (AnExactDyadic<Y>) { return operator*(F(y1,x2.precision()),x2); }
        else { return operator*(Ball<F,FE>(y1,x2.precision(),x2.error_precision()),x2); } }
    template<AValidatedNumber Y> friend Ball<F,FE> operator/(Y const& y1, Ball<F,FE> const& x2) {
        if constexpr (AnExactDyadic<Y>) { return operator/(F(y1,x2.precision()),x2); }
        else { return operator/(Ball<F,FE>(y1,x2.precision(),x2.error_precision()),x2); } }
*/
  public:
    friend Ball<F,FE> add(Ball<F,FE> const& x1, F const& x2) { return add(x1,Ball<F,FE>(x2,x1.error_precision())); }
    friend Ball<F,FE> sub(Ball<F,FE> const& x1, F const& x2) { return sub(x1,Ball<F,FE>(x2,x1.error_precision())); }
    friend Ball<F,FE> mul(Ball<F,FE> const& x1, F const& x2) { return mul(x1,Ball<F,FE>(x2,x1.error_precision())); }
    friend Ball<F,FE> div(Ball<F,FE> const& x1, F const& x2) { return div(x1,Ball<F,FE>(x2,x1.error_precision())); }
    friend Ball<F,FE> add(F const& x1, Ball<F,FE> const& x2) { return add(Ball<F,FE>(x1,x2.error_precision()),x2); }
    friend Ball<F,FE> sub(F const& x1, Ball<F,FE> const& x2) { return sub(Ball<F,FE>(x1,x2.error_precision()),x2); }
    friend Ball<F,FE> mul(F const& x1, Ball<F,FE> const& x2) { return mul(Ball<F,FE>(x1,x2.error_precision()),x2); }
    friend Ball<F,FE> div(F const& x1, Ball<F,FE> const& x2) { return div(Ball<F,FE>(x1,x2.error_precision()),x2); }

    friend Ball<F,FE> max(Ball<F,FE> const& x1, F const& x2) { return max(x1,Ball<F,FE>(x2,x1.error_precision())); }
    friend Ball<F,FE> min(Ball<F,FE> const& x1, F const& x2) { return min(x1,Ball<F,FE>(x2,x1.error_precision())); }
    friend Ball<F,FE> max(F const& x1, Ball<F,FE> const& x2) { return max(Ball<F,FE>(x1,x2.error_precision()),x2); }
    friend Ball<F,FE> min(F const& x1, Ball<F,FE> const& x2) { return min(Ball<F,FE>(x1,x2.error_precision()),x2); }

    friend Ball<F,FE> operator+(Ball<F,FE> const& x1, F const& x2) { return operator+(x1,Ball<F,FE>(x2,x1.error_precision())); }
    friend Ball<F,FE> operator-(Ball<F,FE> const& x1, F const& x2) { return operator-(x1,Ball<F,FE>(x2,x1.error_precision())); }
    friend Ball<F,FE> operator*(Ball<F,FE> const& x1, F const& x2) { return operator*(x1,Ball<F,FE>(x2,x1.error_precision())); }
    friend Ball<F,FE> operator/(Ball<F,FE> const& x1, F const& x2) { return operator/(x1,Ball<F,FE>(x2,x1.error_precision())); }
    friend Ball<F,FE> operator+(F const& x1, Ball<F,FE> const& x2) { return operator+(Ball<F,FE>(x1,x2.error_precision()),x2); }
    friend Ball<F,FE> operator-(F const& x1, Ball<F,FE> const& x2) { return operator-(Ball<F,FE>(x1,x2.error_precision()),x2); }
    friend Ball<F,FE> operator*(F const& x1, Ball<F,FE> const& x2) { return operator*(Ball<F,FE>(x1,x2.error_precision()),x2); }
    friend Ball<F,FE> operator/(F const& x1, Ball<F,FE> const& x2) { return operator/(Ball<F,FE>(x1,x2.error_precision()),x2); }

    friend ValidatedKleenean operator==(Ball<F,FE> const& x1, F const& x2) { return x1==Ball<F,FE>(x2,x1.error_precision()); }
    friend ValidatedKleenean operator!=(Ball<F,FE> const& x1, F const& x2) { return x1!=Ball<F,FE>(x2,x1.error_precision()); }
    friend ValidatedKleenean operator< (Ball<F,FE> const& x1, F const& x2) { return x1< Ball<F,FE>(x2,x1.error_precision()); }
    friend ValidatedKleenean operator> (Ball<F,FE> const& x1, F const& x2) { return x1> Ball<F,FE>(x2,x1.error_precision()); }
    friend ValidatedKleenean operator<=(Ball<F,FE> const& x1, F const& x2) { return x1<=Ball<F,FE>(x2,x1.error_precision()); }
    friend ValidatedKleenean operator>=(Ball<F,FE> const& x1, F const& x2) { return x1>=Ball<F,FE>(x2,x1.error_precision()); }
    friend ValidatedKleenean operator==(F const& x1, Ball<F,FE> const& x2) { return Ball<F,FE>(x1,x2.error_precision())==x2; }
    friend ValidatedKleenean operator!=(F const& x1, Ball<F,FE> const& x2) { return Ball<F,FE>(x1,x2.error_precision())!=x2; }
    friend ValidatedKleenean operator< (F const& x1, Ball<F,FE> const& x2) { return Ball<F,FE>(x1,x2.error_precision())< x2; }
    friend ValidatedKleenean operator> (F const& x1, Ball<F,FE> const& x2) { return Ball<F,FE>(x1,x2.error_precision())> x2; }
    friend ValidatedKleenean operator<=(F const& x1, Ball<F,FE> const& x2) { return Ball<F,FE>(x1,x2.error_precision())<=x2; }
    friend ValidatedKleenean operator>=(F const& x1, Ball<F,FE> const& x2) { return Ball<F,FE>(x1,x2.error_precision())>=x2; }

  public:
    //!@{
    //! \name Rounding operations

    //! Round lower and upper bounds to nearest integer values.
    friend Ball<F,FE> round(Ball<F,FE> const& x) {
        return Ball<F,FE>(round(x.lower_raw()),round(x.upper_raw())); }
    //! Round outward by 1 ulp. i.e. increase the error.
    friend Ball<F,FE> widen(Ball<F,FE> const& x) {
        const F m=std::numeric_limits<float>::min(); return Ball<F,FE>(sub(down,x._l,m),add(up,x._u,m)); }
    //! Round inward by 1 ulp. i.e. decrease the error.
    friend Ball<F,FE> narrow(Ball<F,FE> const& x) {
        const F m=std::numeric_limits<float>::min(); return Ball<F,FE>(add(up,x._l,m),add(down,x._u,m)); }
    //! Truncate to lower precision.
    friend Ball<F,FE> trunc(Ball<F,FE> const& x) {
        return Operations<Ball<F,FE>>::_trunc(x); }
    //! Truncate to lower precision.
    friend Ball<F,FE> trunc(Ball<F,FE> const& x, Nat n) {
        return Operations<Ball<F,FE>>::_trunc(x,n); }
    //! Round to the nearest integer to the midpoint.
    friend Integer cast_integer(Ball<F,FE> const& x) {
        return Operations<Ball<F,FE>>::_cast_integer(x); }
    //!@}

    //!@{
    //! \name Special value tests

    //! Tests whether \a x is NaN (not-a-number).
    friend Bool is_nan(Ball<F,FE> const& x) {
        return is_nan(x._v) || is_nan(x._e); }
    //! Tests whether \a x is a model of zero.
    friend auto is_zero(Ball<F,FE> const& x) -> LogicalType<ValidatedTag> {
        if(x.lower_raw()>0.0 || x.upper_raw()<0.0) { return false; }
        else if(x.lower_raw()==0.0 && x.upper_raw()==0.0) { return true; }
        else { return indeterminate; } }
    //! Tests whether \a x is a model of a positive number.
    friend auto is_positive(Ball<F,FE> const& x) -> LogicalType<ValidatedTag> {
        if(x.lower_raw()>=0.0) { return true; } else if(x.upper_raw()<0.0) { return false; } else { return indeterminate; } }
    //!@}


    //!@{
    //! \name Validated information tests and operations

    //! Tests is \a x1 and \a x2 have the same representation i.e. the same centre value and error bound.
    friend Bool same(Ball<F,FE> const& x1, Ball<F,FE> const& x2) {
        return x1._v==x2._v && x1._e==x2._e; }
        //! Tests is \a x1 is a valid approximation for \a x2 i.e. if \a x2 is within the error of the centre value of \a x1.
    friend Bool models(Ball<F,FE> const& x1, F const& x2) {
        return x1._l<=x2._v && x1._u >= x2._v; }
    //! Tests is \a x1 and \a x2 are consistent with being a model of the same value i.e. they intersect.
    friend Bool consistent(Ball<F,FE> const& x1, Ball<F,FE> const& x2) {
        return (x1._v >= x2._v ? sub(up,x1._v,x2._v) : sub(up,x2._v,x1._v)) <= add(down,x1._e,x2._e); }
    //! Tests is \a x1 and \a x2 are inconsistent with being a model of the same value i.e. they are disjoint.
    friend Bool inconsistent(Ball<F,FE> const& x1, Ball<F,FE> const& x2) {
        return (x1._v >= x2._v ? sub(down,x1._v,x2._v) : sub(down,x2._v,x1._v)) > add(up,x1._e,x2._e); }
    //! Tests is \a x1 is a tighter representation than \a x2.
    friend Bool refines(Ball<F,FE> const& x1, Ball<F,FE> const& x2) {
        return add ( up, (x1._v >= x2._v ? sub(up,x1._v,x2._v) : sub(up,x2._v,x1._v) ), x1._e ) <= x2._e; }
    //! The common refinement of \a x1 and \x2. Requires \a x1 and \a x2 to be consistent.
    friend Ball<F,FE> refinement(Ball<F,FE> const& x1, Ball<F,FE> const& x2) {
        return Ball<F,FE>(refinement(Bounds<F>(x1),Bounds<F>(x2)),x1.error_precision()); }
    friend Ball<F,FE> coarsening(Ball<F,FE> const& x1, Ball<F,FE> const& x2) {
        return Ball<F,FE>(coarsening(Bounds<F>(x1),Bounds<F>(x2)),x1.error_precision()); }
    //!@}

    //!@{
    //! \name Input/output operations
    friend OutputStream& operator<<(OutputStream& os, Ball<F,FE> const& x) {
        return Operations<Ball<F,FE>>::_write(os,x); } //!< Write to an output stream.
    friend InputStream& operator>>(InputStream& is, Ball<F,FE>& x) {
        return Operations<Ball<F,FE>>::_read(is,x); } //!< Read from an input stream.
    //!@}
  private: public:
    F _v; FE _e;
};

template<class PR> Ball(ValidatedNumber, PR) -> Ball<RawFloatType<PR>>;
template<class PR, class PRE> Ball(ValidatedNumber, PR, PRE) -> Ball<RawFloatType<PR>,RawFloatType<PRE>>;
template<class F, class FE> Ball(F,FE) -> Ball<F,FE>;

// Mixed Ball-value operations
template<ARawFloat F, class PRE> Ball<F,Float<PRE>> add(F const& x1, F const& x2, PRE pre) {
    typedef Float<PRE> FE; return Operations<Ball<F,FE>>::_add(x1,x2,pre); }
template<ARawFloat F, class PRE> Ball<F,Float<PRE>> sub(F const& x1, F const& x2, PRE pre) {
    typedef Float<PRE> FE; return Operations<Ball<F,FE>>::_sub(x1,x2,pre); }
template<ARawFloat F, class PRE> Ball<F,Float<PRE>> mul(F const& x1, F const& x2, PRE pre) {
    typedef Float<PRE> FE; return Operations<Ball<F,FE>>::_mul(x1,x2,pre); }
template<ARawFloat F, class PRE> Ball<F,Float<PRE>> div(F const& x1, F const& x2, PRE pre) {
    typedef Float<PRE> FE; return Operations<Ball<F,FE>>::_div(x1,x2,pre); }

template<class F, class FE> inline FloatBallFactory<PrecisionType<F>,PrecisionType<FE>> factory(Ball<F,FE> const& flt) {
    return FloatBallFactory<PrecisionType<F>,PrecisionType<FE>>(flt.precision(),flt.error_precision()); }
template<class PR, class PRE> inline FloatBall<PR,PRE> FloatBallFactory<PR,PRE>::create(Number<ValidatedTag> const& y) { return FloatBall<PR,PRE>(y,this->_pr,this->_pre); }
template<class PR, class PRE> inline FloatBall<PR,PRE> FloatBallFactory<PR,PRE>::create(Number<EffectiveTag> const& y) { return FloatBall<PR,PRE>(y,this->_pr,this->_pre); }
template<class PR, class PRE> inline FloatBall<PR,PRE> FloatBallFactory<PR,PRE>::create(Number<ExactTag> const& y) { return FloatBall<PR,PRE>(y,this->_pr,this->_pre); }
template<class PR, class PRE> inline FloatBall<PR,PRE> FloatBallFactory<PR,PRE>::create(Real const& y) { return FloatBall<PR,PRE>(y,this->_pr,this->_pre); }
template<class PR, class PRE> inline FloatBall<PR,PRE> FloatBallFactory<PR,PRE>::create(Rational const& y) { return FloatBall<PR,PRE>(y,this->_pr,this->_pre); }
template<class PR, class PRE> inline FloatBall<PR,PRE> FloatBallFactory<PR,PRE>::create(Dyadic const& y) { return FloatBall<PR,PRE>(y,this->_pr,this->_pre); }
template<class PR, class PRE> inline FloatBall<PR,PRE> FloatBallFactory<PR,PRE>::create(Integer const& y) { return FloatBall<PR,PRE>(y,this->_pr,this->_pre); }

template<class PR, class PRE> inline PositiveFloatBall<PR,PRE> FloatBallFactory<PR,PRE>::create(PositiveValidatedNumber const& y) { return PositiveFloatBall<PR,PRE>(y,this->_pr,this->_pre); }


template<class F, class FE> class Positive<Ball<F,FE>> : public Ball<F,FE> {
    using PR = typename Ball<F,FE>::PrecisionType;
    using PRE = typename Ball<F,FE>::ErrorPrecisionType;
  public:
    Positive() : Bounds<F>() { }
    template<BuiltinUnsignedIntegral M>
        Positive(M m, PRE pre) : Ball<F,FE>(m,pre) { }
    explicit Positive(PR pr, PRE pre) : Ball<F,FE>(pr,pre) { }
    explicit Positive(Ball<F,FE> const& x) : Ball<F,FE>(x) { }
};

template<class F, class FE> inline PositiveBall<F,FE> cast_positive(Ball<F,FE> const& x) {
    return PositiveBall<F,FE>(x); }



template<class F, class FE> struct Operations<Ball<F,FE>> {
    typedef typename FE::PrecisionType PRE;

    static FE _make_error(F const& e) {
        static_assert(SameAs<F,FE> or DefaultConstructible<PRE>);
        if constexpr (SameAs<F,FE>) { return e; }
        else if constexpr (DefaultConstructible<PRE>) { return FE(e,up,PRE()); }
    }

    static Ball<F,FE> _nul(Ball<F,FE> const& x) {
        return Ball<F,FE>(nul(x._v),nul(x._e));
    }

    static Ball<F,FE> _pos(Ball<F,FE> const& x) {
        return Ball<F,FE>(pos(x._v),x._e);
    }

    static Ball<F,FE> _neg(Ball<F,FE> const& x) {
        return Ball<F,FE>(neg(x._v),x._e);
    }

    static Ball<F,FE> _hlf(Ball<F,FE> const& x) {
        return Ball<F,FE>(hlf(x._v),hlf(x._e));
    }

    static Ball<F,FE> _sqr(Ball<F,FE> const& x) {
        Ball<F,FE> r=x*x;
        if(r._e>r._v) {
            r._e=hlf(add(up,r._e,_make_error(r._v)));
            r._v=F(Dyadic(r._e),upward,x.precision());
        }
        return r;
    }

    static Ball<F,FE> _rec(Ball<F,FE> const& x) {
        // Use this code to find value same as reciprocal value
        auto rv=rec(approx,x._v);
        auto ru=rec(up,sub(down,x._v,x._e));
        auto rl=rec(down,add(up,x._v,x._e));
        auto re=max(sub(up,ru,rv),sub(up,rv,rl));
        return Ball<F,FE>(rv,_make_error(re));
    #ifdef ARIADNE_UNDEFINED
        // Use this code to get same result as interval computation
        auto ru=rec(up,sub(down,x._v,x._e));
        auto rl=rec(down,add(up,x._v,x._e));
        auto re=hlf(sub(up,ru,rl));
        auto rv=hlf(add(near,rl,ru));
        return Ball<F,FE>(rv,re);
    #endif
    }

    static Ball<F,FE> _add(Ball<F,FE> const& x, Ball<F,FE> const& y) {
        auto rv=add(near,x._v,y._v);
        auto ru=add(up,x._v,y._v);
        auto rl=add(down,x._v,y._v);
        auto se=add(up,x._e,y._e);
        auto ae=hlf(FE(sub(up,ru,rl),up,se.precision()));
        auto re=add(up,ae,se);
        return Ball<F,FE>(rv,re);
    }

    static Ball<F,FE> _sub(Ball<F,FE> const& x, Ball<F,FE> const& y) {
        auto rv=sub(near,x._v,y._v);
        auto ru=sub(up,x._v,y._v);
        auto rl=sub(down,x._v,y._v);
        auto ae=_make_error(hlf(sub(up,ru,rl)));
        auto re=add(up,ae,add(up,x._e,y._e));
        return Ball<F,FE>(rv,re);
    }

    static Ball<F,FE> _mul(Ball<F,FE> const& x, Ball<F,FE> const& y) {
        auto rv=mul(near,x._v,y._v);
        auto ru=mul(up,x._v,y._v);
        auto rl=mul(down,x._v,y._v);
        auto re0=_make_error(hlf(sub(up,ru,rl)));
        auto re1=add(up,re0,mul(up,x._e,y._e));
        auto re2=add(up,mul(up,_make_error(abs(x._v)),y._e),mul(up,x._e,_make_error(abs(y._v))));
        auto re=add(up,re1,re2);
        return Ball<F,FE>(rv,re);
    }

    static Ball<F,FE> _div(Ball<F,FE> const& x, Ball<F,FE> const& y) {
        return x*rec(y);
    }

    static Ball<F,FE> _add(F const& x, F const& y, PRE pre) {
        auto rv=add(near,x,y);
        auto ru=add(up,x,y);
        auto rl=add(down,x,y);
        auto re=FE(hlf(sub(up,ru,rl)),up,pre);
        return Ball<F,FE>(rv,re);
    }

    static Ball<F,FE> _sub(F const& x, F const& y, PRE pre) {
        auto rv=sub(near,x,y);
        auto ru=sub(up,x,y);
        auto rl=sub(down,x,y);
        auto re=FE(hlf(sub(up,ru,rl)),up,pre);
        return Ball<F,FE>(rv,re);
    }

    static Ball<F,FE> _mul(F const& x, F const& y, PRE pre) {
        auto rv=mul(near,x,y);
        auto ru=mul(up,x,y);
        auto rl=mul(down,x,y);
        auto re=FE(hlf(sub(up,ru,rl)),up,pre);
        return Ball<F,FE>(rv,re);
    }

    static Ball<F,FE> _div(F const& x, F const& y, PRE pre) {
        auto rv=div(near,x,y);
        auto ru=div(up,x,y);
        auto rl=div(down,x,y);
        auto re=FE(hlf(sub(up,ru,rl)),up,pre);
        return Ball<F,FE>(rv,re);
    }

    static Ball<F,FE> _pow(Ball<F,FE> const& x, Nat m) {
        return Ball<F,FE>(pow(Bounds<F>(x),m));
    }

    static Ball<F,FE> _pow(Ball<F,FE> const& x, Int n) {
        return Ball<F,FE>(pow(Bounds<F>(x),n));
    }

    static Ball<F,FE> _sqrt(Ball<F,FE> const& x) {
        return Ball<F,FE>(sqrt(Bounds<F>(x)));
    }

    static Ball<F,FE> _exp(Ball<F,FE> const& x) {
        return Ball<F,FE>(exp(Bounds<F>(x)));
    }

    static Ball<F,FE> _log(Ball<F,FE> const& x) {
        return Ball<F,FE>(log(Bounds<F>(x)));
    }

    static Ball<F,FE> _sin(Ball<F,FE> const& x) {
        return Ball<F,FE>(sin(Bounds<F>(x)));
    }

    static Ball<F,FE> _cos(Ball<F,FE> const& x) {
        return Ball<F,FE>(cos(Bounds<F>(x)));
    }

    static Ball<F,FE> _tan(Ball<F,FE> const& x) {
        return Ball<F,FE>(tan(Bounds<F>(x)));
    }

    static Ball<F,FE> _asin(Ball<F,FE> const& x) {
        return Ball<F,FE>(asin(Bounds<F>(x)));
    }

    static Ball<F,FE> _acos(Ball<F,FE> const& x) {
        return Ball<F,FE>(acos(Bounds<F>(x)));
    }

    static Ball<F,FE> _atan(Ball<F,FE> const& x) {
        return Ball<F,FE>(atan(Bounds<F>(x)));
    }


    static Ball<F,FE> _abs(Ball<F,FE> const& x) {
        if(x._e<abs(x._v)) { return x; }
        else { auto rv=hlf(add(up,abs(x._v),x._e)); return Ball<F,FE>(rv,_make_error(rv)); }
    }

    static Ball<F,FE> _max(Ball<F,FE> const& x1, Ball<F,FE> const& x2) {
        return hlf((x1+x2)+abs(x1-x2));
    }

    static Ball<F,FE> _min(Ball<F,FE> const& x1, Ball<F,FE> const& x2) {
        return hlf((x1+x2)-abs(x1-x2));
    }

    static Error<F> _mag(Ball<F,FE> const& x) {
        return PositiveUpperBound<F>(add(up,abs(x._v),x._e));
    }

    static PositiveLowerBound<F> _mig(Ball<F,FE> const& x) {
        return PositiveLowerBound<F>(max(F(0,x._v.precision()),sub(down,abs(x._v),x._e)));
    }

    //! \related Bounds<F> \brief Strict greater-than comparison operator. Tests equality of represented real-point value.
    static ValidatedKleenean _eq(Ball<F,FE> const& x1, Ball<F,FE> const& x2) {
        return Bounds<F>(x1) == Bounds<F>(x2);
    }

    //! \related Bounds<F> \brief Strict greater-than comparison operator. Tests equality of represented real-point value.
    static ValidatedKleenean _lt(Ball<F,FE> const& x1, Ball<F,FE> const& x2) {
        return Bounds<F>(x1) <  Bounds<F>(x2);
    }

    static Bool _same(Ball<F,FE> const& x1, Ball<F,FE> const& x2) {
        return x1._v==x2._v && x1._e==x2._e;
    }

    static Bool _models(Ball<F,FE> const& x1, F const& x2) {
        return (x1._v>=x2 ? sub(up,x1._v,x2) : sub(up,x2,x1._v)) <= x1._e;
    }

    static Bool _consistent(Ball<F,FE> const& x1, Ball<F,FE> const& x2) {
        return consistent(Bounds<F>(x1),Bounds<F>(x2));
    }

    static Bool _inconsistent(Ball<F,FE> const& x1, Ball<F,FE> const& x2) {
        return inconsistent(Bounds<F>(x1),Bounds<F>(x2));
    }

    static Bool _refines(Ball<F,FE> const& x1, Ball<F,FE> const& x2) {
        return (x1._v>=x2._v ? sub(up,x1._v,x2._v) : sub(up,x2._v,x1._v)) <= sub(down,x2._e, x1._e);
    }

    static Ball<F,FE> _refinement(Ball<F,FE> const& x1, Ball<F,FE> const& x2) {
        return Ball<F,FE>(refinement(Bounds<F>(x1),Bounds<F>(x2)));
    }

    static Integer _cast_integer(Ball<F,FE> const& x);

    static OutputStream& _write(OutputStream& os, Ball<F,FE> const& x);

    static InputStream& _read(InputStream& is, Ball<F,FE>& x);
};



}

#endif
