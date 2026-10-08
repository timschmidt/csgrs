/***************************************************************************
 *            numeric/float_bounds.hpp
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

/*! \file numeric/float_bounds.hpp
 *  \brief Floating-point bounds for real numbers.
 */

#ifndef ARIADNE_FLOAT_BOUNDS_HPP
#define ARIADNE_FLOAT_BOUNDS_HPP

#include "utility/macros.hpp"

#include "foundations/logical.decl.hpp"
#include "number.decl.hpp"
#include "float.decl.hpp"

#include "positive.hpp"

#include "float_operations.hpp"
#include "float_traits.hpp"
#include "float_factory.hpp"

namespace Ariadne {

struct DefaultTag;

//! \ingroup NumericModule
//! \brief Validated bounds on a number with floating-point endpoints supporting outwardly-rounded arithmetic.
//! \details
//! Note that direct construction from a floating-point number is prohibited, since <c>%FloatDPBounds(3.3,dp)</c> would the singleton interval \f$[3.2999999999999998224,3.2999999999999998224]\f$ (the constant is first interpreted by the C++ compiler to give a C++ \c double, whereas <c>%FloatDPBounds(3.3_decimal,dp)</c> yields the interval \f$[3.2999999999999998224,3.3000000000000002665]\f$ enclosing \f$3.3\f$.
//!
//! Comparison tests on \c Bounds use the idea that an interval represents a single number with an unknown value.
//! Hence the result is of type \c ValidatedKleenean, which can take values { \c True, \c False, \c Indeterminate }.
//! Hence a test \f$[\underline{x},\overline{x}]\leq [\underline{y},\overline{y}]\f$ returns \c True if \f$\overline{x}\leq \underline{y}\f$, since in this case \f$x\leq x\f$ whenever \f$x_1\in[\underline{x},\overline{x}]\f$ and \f$y\in[\underline{y},\overline{y}]\f$, \c False if \f$\underline{x}>\overline{y}\f$, since in this case we know \f$x>y\f$, and \c Indeterminate otherwise, since in this case we can find \f$x,y\f$ making the result either true or false.
//! In the case of equality, the comparison \f$[\underline{x},\overline{x}]\f$==\f$[\underline{y},\overline{y}]\f$ only returns \c True if both intervals are singletons, since otherwise we can find values making the result either true of false.
//! To test equality of representation, use \c same(x,y)
//!
//! To obtain the lower and upper bounds of the possible values, use \c x.lower() and \c x.upper().
//! To obtain a best estimate of the value, use \c x.value(), which has an error at most \a x.error().
//! If \f$v\f$ and \f$e\f$ are the returned value and error for the bounds \f$[l,u]\f$, then it is guaranteed that \f$v-e\leq l\f$ and \f$v+e\geq u\f$ in exact arithmetic.
//!
//! To test if the bounds contain a number , use \c models(Bounds<F>,F), and to test if bounds are inconsistent use \c inconsistent(x,y), and to test if \c x provides a better approximation, use \c refines(x,y).
//! \sa Real, FloatDP, FloatMP, Float, Ball, UpperBound, LowerBound, Approximation.
//!
//! \par Python interface
//!
//! In the Python interface, %Ariadne validated bounds can be constructed from Python literals of the form \c {a:b} or (deprecated) \c [a,b] .
//! The former is preferred, as it cannot be confused with literals for other classes such as Vector and Array types.
//! Automatic conversion is used to convert FloatBounds literals of the form \c {a,b} to an FloatBounds in functions.
//!
//! Care must be taken when defining intervals using floating-point coefficients, since values are first converted to the nearest
//! representable value by the Python interpreter. <br><br>
//! \code
//!   FloatDPBounds({1.1:2.3}) # Create the interval [1.1000000000000001, 2.2999999999999998]
//!   FloatDPBounds({2.5:4.25}) # Create the interval [2.5, 4.25], which can be represented exactly
//!   FloatDPBounds([2.5,4.25]) # Alternative syntax for creating the interval [2.5, 4.25]
//! \endcode
template<class F> class Bounds
    : public DefineFloatOperations<Bounds<F>>
{
  protected:
    typedef ValidatedTag P; typedef typename F::RoundingModeType RND; typedef typename F::PrecisionType PR;
  public:
    //! <p/>
    typedef P Paradigm;
    //! <p/>
    typedef Bounds<F> NumericType;
    //! <p/>
    typedef Number<P> GenericType;
    //! <p/>
    typedef F RawType;
    //! <p/>
    typedef PR PrecisionType;
    //! <p/>
    typedef PR CharacteristicsType;
  public:
    //! Construct bounds of zero with precision \a pr.
    explicit Bounds(PrecisionType pr) : _l(0.0_x,pr), _u(0.0_x,pr) { }
    //! Construct bounds with value \a v.
    Bounds(RawType const& v) : _l(v), _u(v) { }
    //! Construct a lower bound of value \a l and an upper bound of value \a u.
    explicit Bounds(RawType const& l, RawType const& u) : _l(l), _u(u) { }
    //! Construct from a lower bound \a lower and an upper bound \a upper.
    Bounds(LowerBound<F> const& lower, UpperBound<F> const& upper);
    Bounds(LowerBound<F> const& lower, ValidatedUpperNumber const& upper);
    Bounds(ValidatedLowerNumber const& lower, UpperBound<F> const& upper);
    //! Construct from a lower bound \a lower and an upper bound \a upper, using precsion \a pr.
    Bounds(ValidatedLowerNumber const& lower, ValidatedUpperNumber const& upper, PR pr);
    template<BuiltinIntegral N1, BuiltinIntegral N2> Bounds(N1 n1, N2 n2, PR pr) : _l(n1,pr), _u(n2,pr) { }
    Bounds(ExactDouble const& dl, ExactDouble const& du, PrecisionType pr) : _l(dl,pr), _u(du,pr) { }
    Bounds(Dyadic const& wl, Dyadic const& wu, PrecisionType pr) : _l(wl,down,pr), _u(wu,up,pr) { }
    Bounds(Decimal const& dl, Decimal const& du, PrecisionType pr) : _l(dl,down,pr), _u(du,up,pr) { }
    Bounds(Rational const& ql, Rational const& qu, PrecisionType pr) : _l(ql,down,pr), _u(qu,up,pr) { }

    template<class FF> requires ConstructibleFrom<F,FF,RND,PR>
        Bounds(Bounds<FF> const& x, PR pr) : Bounds(F(x.lower_raw(),downward,pr),F(x.upper_raw(),upward,pr)) { }
    Bounds(const Bounds<F>& x, PR pr) : _l(x._l,down,pr), _u(x._u,up,pr) { }

    template<BuiltinIntegral N> Bounds(N n, PR pr) : _l(n,down,pr), _u(n,up,pr) { }
    Bounds(ExactDouble const& d, PR pr) : _l(d,pr), _u(d,pr) { }
        Bounds(TwoExp const& t, PR pr) : _l(t,pr), _u(t,pr) { }
        Bounds(const Integer& z, PR pr) : _l(z,down,pr), _u(z,up,pr) { }
        Bounds(const Dyadic& w, PR pr) : _l(w,down,pr), _u(w,up,pr) { }
        Bounds(const Decimal& d, PR pr) : _l(d,down,pr), _u(d,up,pr) { }
        Bounds(const Rational& q, PR pr) : _l(q,down,pr), _u(q,up,pr) { }
        Bounds(const Real& x, PR pr);
    //! Construct from generic validated bounds \a y, using precsion \a pr.
    Bounds(const ValidatedNumber& y, PR pr);


    //! Convert from a ball.
    template<class FE> Bounds(Ball<F,FE> const& x);

        Bounds<F>& operator=(const F& x) { return *this=Bounds<F>(x); }
    //! Assign from generic validated bounds \a y, keeping the same precision.
    Bounds<F>& operator=(const ValidatedNumber& y) { return *this=Bounds<F>(y,this->precision()); }
    template<BuiltinIntegral N> Bounds<F>& operator=(N n) { return *this=ValidatedNumber(n); }

    //! Downcast to generic validated bounds.
    operator ValidatedNumber () const;

    //! Create bounds from the generic validated number \a y with the same precision as \a this.
    Bounds<F> create(const ValidatedNumber& y) const { return Bounds<F>(y,this->precision()); }

    //! The lower bound.
    LowerBound<F> const lower() const;
    //! The upper bound.
    UpperBound<F> const upper() const;
    //! An approximation to the actual value.
    F const value() const;
    //! Bounds on the approximation given by value().
    Error<F> const error() const;

    friend F value(Bounds<F> const& x) { return x.value(); }
    friend Error<F> error(Bounds<F> const& x) { return x.error(); }

    RawType const& lower_raw() const { return _l; }
    RawType const& upper_raw() const { return _u; }
    RawType const value_raw() const { return hlf(add(near,_l,_u)); }
    RawType const error_raw() const { RawType v=value_raw(); return max(sub(up,_u,v),sub(up,v,_l)); }
    //! Approximate by a builtin double-precision value. DEPRECATED
    double get_d() const { return value_raw().get_d(); }

    //! The precision of the floating-point type used.
    PrecisionType precision() const { ARIADNE_DEBUG_ASSERT(_l.precision()==_u.precision()); return _u.precision(); }
    //! The compuational characteristics needed to create the bounds; equivalent to the precision.
    CharacteristicsType characteristics() const { ARIADNE_DEBUG_ASSERT(_l.precision()==_u.precision()); return _u.precision(); }
    //! Downcast to generic validated bounds.
    GenericType generic() const { return this->operator GenericType(); }

    //! Add \a e to upper bound, and subtract \a e from lower bound.
    Bounds<F> pm(Error<F> const& e) const;

    friend Approximation<F> round(Approximation<F> const& x);
    friend F midpoint(Bounds<F> const& x);
  public:
#ifdef DOXYGEN
    //!@{
    //! \name Arithmetic operators
    friend Bounds<F> operator+(Bounds<F> const& x); //!< <p/>
    friend Bounds<F> operator-(Bounds<F> const& x); //!< <p/>
    friend Bounds<F> operator+(Bounds<F> const& x1, Bounds<F> const& x2); //!< <p/>
    friend Bounds<F> operator-(Bounds<F> const& x1, Bounds<F> const& x2); //!< <p/>
    friend Bounds<F> operator*(Bounds<F> const& x1, Bounds<F> const& x2); //!< <p/>
    friend Bounds<F> operator/(Bounds<F> const& x1, Bounds<F> const& x2); //!< <p/>
    friend Bounds<F>& operator+=(Bounds<F>& x1, Bounds<F> const& x2); //!< <p/>
    friend Bounds<F>& operator-=(Bounds<F>& x1, Bounds<F> const& x2); //!< <p/>
    friend Bounds<F>& operator*=(Bounds<F>& x1, Bounds<F> const& x2); //!< <p/>
    friend Bounds<F>& operator/=(Bounds<F>& x1, Bounds<F> const& x2); //!< <p/>
    //!@}

    //!@{
    //! \name Comparison operators
    friend ValidatedKleenean operator==(Bounds<F> const& x1, Bounds<F> const& x2); //!< <p/>
    friend ValidatedKleenean operator!=(Bounds<F> const& x1, Bounds<F> const& x2); //!< <p/>
    friend ValidatedKleenean operator<=(Bounds<F> const& x1, Bounds<F> const& x2); //!< <p/>
    friend ValidatedKleenean operator>=(Bounds<F> const& x1, Bounds<F> const& x2); //!< <p/>
    friend ValidatedKleenean operator< (Bounds<F> const& x1, Bounds<F> const& x2); //!< <p/>
    friend ValidatedKleenean operator> (Bounds<F> const& x1, Bounds<F> const& x2); //!< <p/>
    //!@}
#endif // DOXYGEN

    //!@{
    //! \name Arithmetic operations
    friend Bounds<F> nul(Bounds<F> const& x) {
        return Bounds<F>(nul(x._l),nul(x._u)); } //!< <p/>
    friend Bounds<F> pos(Bounds<F> const& x) {
        return Bounds<F>(pos(x._l),pos(x._u)); } //!< <p/>
    friend Bounds<F> neg(Bounds<F> const& x) {
        return Bounds<F>(neg(x._u),neg(x._l)); } //!< <p/>
    friend Bounds<F> hlf(Bounds<F> const& x) {
        return Bounds<F>(hlf(x._l),hlf(x._u)); } //!< <p/>
    friend Bounds<F> sqr(Bounds<F> const& x) {
        if(x._l>0) { return Bounds<F>(mul(down,x._l,x._l),mul(up,x._u,x._u)); } //!< <p/>
        else if(x._u<0) { return Bounds<F>(mul(down,x._u,x._u),mul(up,x._l,x._l)); } //!< <p/>
        else { return Bounds<F>(nul(x._l),max(mul(up,x._l,x._l),mul(up,x._u,x._u))); } } //!< <p/>
    friend Bounds<F> rec(Bounds<F> const& x) {
        if(x._l>0 || x._u<0) {  return Bounds<F>(rec(down,x._u),rec(up,x._l)); } //!< <p/>
    //ARIADNE_THROW(DivideByZeroException,"FloatBounds rec(FloatBounds x)","x="<<x); //!< <p/>
        else { F inf_=F::inf(x.precision()); return Bounds<F>(-inf_,+inf_); } } //!< <p/>

    friend Bounds<F> add(Bounds<F> const& x1, Bounds<F> const& x2) {
        return Bounds<F>(add(down,x1._l,x2._l),add(up,x1._u,x2._u)); } //!< <p/>
    friend Bounds<F> sub(Bounds<F> const& x1, Bounds<F> const& x2) {
        return Bounds<F>(sub(down,x1._l,x2._u),sub(up,x1._u,x2._l)); } //!< <p/>
    friend Bounds<F> mul(Bounds<F> const& x1, Bounds<F> const& x2) {
        return Operations<Bounds<F>>::_mul(x1,x2); } //!< <p/>
    friend Bounds<F> div(Bounds<F> const& x1, Bounds<F> const& x2) {
        return Operations<Bounds<F>>::_div(x1,x2); } //!< <p/>
    friend Bounds<F> fma(Bounds<F> const& x1, Bounds<F> const& x2, Bounds<F> const& x3) {
        return Operations<Bounds<F>>::_fma(x1,x2,x3); } //!< <p/>
    friend Bounds<F> pow(Bounds<F> const& x, Int n) {
        if(n<0) { return pow(rec(x),Nat(-n)); } else return pow(x,Nat(n));} //!< <p/>
    friend Bounds<F> pow(Bounds<F> const& x, Nat m) {
        Bounds<F> y = (m%2==0) ? abs(x) : x;  Int n=static_cast<Int>(m); return Bounds<F>(pow(down,y._l,n),pow(up,y._u,n)); } //!< <p/>
    //!@}

    //!@{
    //! \name Algebraic and transcendental operations
    friend Bounds<F> sqrt(Bounds<F> const& x) {
        return Bounds<F>(sqrt(down,x.lower_raw()),sqrt(up,x.upper_raw())); } //!< <p/>
    friend Bounds<F> exp(Bounds<F> const& x) {
        return Bounds<F>(exp(down,x.lower_raw()),exp(up,x.upper_raw())); } //!< <p/>
    friend Bounds<F> log(Bounds<F> const& x) {
        return Bounds<F>(log(down,x.lower_raw()),log(up,x.upper_raw())); } //!< <p/>
    friend Bounds<F> sin(Bounds<F> const& x) {
        return Operations<Bounds<F>>::_sin(x); } //!< <p/>
    friend Bounds<F> cos(Bounds<F> const& x) {
        return Operations<Bounds<F>>::_cos(x); } //!< <p/>
    friend Bounds<F> tan(Bounds<F> const& x) {
        return Operations<Bounds<F>>::_tan(x); } //!< <p/>
    friend Bounds<F> asin(Bounds<F> const& x) {
        return Bounds<F>(asin(down,x.lower_raw()),asin(up,x.upper_raw())); } //!< <p/>
    friend Bounds<F> acos(Bounds<F> const& x) {
        return Bounds<F>(acos(down,x.upper_raw()),acos(up,x.lower_raw())); } //!< <p/>
    friend Bounds<F> atan(Bounds<F> const& x) {
        return Bounds<F>(atan(down,x._l),atan(up,x._u)); } //!< <p/>
    //!@}

    //!@{
    //! \name Lattice operations
    friend Bounds<F> abs(Bounds<F> const& x) {
        if(x.lower_raw()>=0) { return Bounds<F>(x.lower_raw(),x.upper_raw());} //!< <p/>
        else if(x.upper_raw()<=0) { return Bounds<F>(neg(x.upper_raw()),neg(x.lower_raw())); } //!< <p/>
        else { return Bounds<F>(F(0,x.precision()),max(neg(x.lower_raw()),x.upper_raw())); } } //!< <p/>
    friend Bounds<F> max(Bounds<F> const& x1, Bounds<F> const& x2) {
        return Bounds<F>(max(x1.lower_raw(),x2.lower_raw()),max(x1.upper_raw(),x2.upper_raw())); } //!< <p/>
    friend Bounds<F> min(Bounds<F> const& x1, Bounds<F> const& x2) {
        return Bounds<F>(min(x1.lower_raw(),x2.lower_raw()),min(x1.upper_raw(),x2.upper_raw())); } //!< <p/>
    friend PositiveLowerBound<F> mig(Bounds<F> const& x) {
        return PositiveLowerBound<F>(max(F(0,x.precision()),max(x._l,neg(x._u)))); } //!< <p/>
    friend PositiveUpperBound<F> mag(Bounds<F> const& x) {
        return PositiveUpperBound<F>(max(neg(x._l),x._u)); } //!< <p/>
    //!@}

    //!@{
    //! \name Comparison operations
    friend ValidatedKleenean sgn(Bounds<F> const& x) {
        if (x._l>0) { return true; } else if (x._u<0) { return false; } else { return indeterminate; } } //!< <p/>
    //! \brief Equality comparison operator. Tests equality of represented real-point value.
    friend LogicalType<ValidatedTag> eq(Bounds<F> const& x1, Bounds<F> const& x2) {
        if(x1.upper_raw()<x2.lower_raw() || x1.lower_raw()>x2.upper_raw()) { return false; } //!< <p/>
        else if(x1.lower_raw()==x2.upper_raw() && x1.upper_raw() == x2.lower_raw()) { return true; } //!< <p/>
        else { return indeterminate; } } //!< <p/>
    //! \brief Strict less-than comparison operator. Tests equality of represented real-point value.
    friend LogicalType<ValidatedTag> lt(Bounds<F> const& x1, Bounds<F> const& x2) {
        if(x1.upper_raw()< x2.lower_raw()) { return true; } //!< <p/>
        else if(x1.lower_raw()>=x2.upper_raw()) { return false; } //!< <p/>
        else { return indeterminate; } } //!< <p/>
    //!@}

    //!@{
    //! \name Rounding operations

    //! Round bounds to nearest integer values.
    friend Bounds<F> round(Bounds<F> const& x) {
        return Bounds<F>(round(x.lower_raw()),round(x.upper_raw())); }
    //! Round outward by 1 ulp.
    friend Bounds<F> widen(Bounds<F> const& x) {
        const F m=std::numeric_limits<float>::min(); return Bounds<F>(sub(down,x._l,m),add(up,x._u,m)); }
    //! Round inward by 1 ulp.
    friend Bounds<F> narrow(Bounds<F> const& x) {
        const F m=std::numeric_limits<float>::min(); return Bounds<F>(add(up,x._l,m),add(down,x._u,m)); }
    //! Truncate to lower precision.
    friend Bounds<F> trunc(Bounds<F> const& x) {
        return Operations<Bounds<F>>::_trunc(x); }
    //! Truncate to lower precision.
    friend Bounds<F> trunc(Bounds<F> const& x, Nat n) {
        return Operations<Bounds<F>>::_trunc(x,n); }
    //! Round to the nearest integer to the midpoint.
    friend Integer cast_integer(Bounds<F> const& x) {
        return Operations<Bounds<F>>::_cast_integer(x); }
    //@}

    //!@{
    //! \name Special value tests

    //! Tests whether \a x is NaN (not-a-number).
    friend Bool is_nan(Bounds<F> const& x) {
        return is_nan(x._l) || is_nan(x._u); }
    //! Tests whether \a x is a model of zero.
    friend auto is_zero(Bounds<F> const& x) -> LogicalType<ValidatedTag> {
        if(x.lower_raw()>0.0 || x.upper_raw()<0.0) { return false; }
        else if(x.lower_raw()==0.0 && x.upper_raw()==0.0) { return true; }
        else { return indeterminate; } }
    //! Tests whether \a x is a model of a positive number. Returns \c true if the lower bound is positive, \c false if the upper bound is strictly negative, and \c indeterminate otherwise.
    friend auto is_positive(Bounds<F> const& x) -> LogicalType<ValidatedTag> {
        if(x.lower_raw()>=0.0) { return true; } else if(x.upper_raw()<0.0) { return false; } else { return indeterminate; } } //!< <p/>
    //!@}

    //!@{
    //! \name Validated information tests and operations

    //! Tests is \a x1 and \a x2 have the same representation i.e. the same lower and upper bounds.
    friend Bool same(Bounds<F> const& x1, Bounds<F> const& x2) {
        return x1._l==x2._l && x1._u==x2._u; }
    //! Tests is \a x1 is a valid approximation for \a x2 i.e. if \a x2 is within the lower and upper bounds of \a x1.
    friend Bool models(Bounds<F> const& x1, F const& x2) {
        return x1._l<=x2 && x1._u >= x2; }
    //! Tests is \a x1 and \a x2 are consistent with being a model of the same value i.e. they intersect.
    friend Bool consistent(Bounds<F> const& x1, Bounds<F> const& x2) {
        return x1._l<=x2._u && x1._u >= x2._l; }
    //! Tests is \a x1 and \a x2 are inconsistent with being a model of the same value i.e. they are disjoint.
    friend Bool inconsistent(Bounds<F> const& x1, Bounds<F> const& x2) {
        return x1._l>x2._u || x1._u < x2._l; }
    //! Tests is \a x1 is a tighter representation than \a x2.
    friend Bool refines(Bounds<F> const& x1, Bounds<F> const& x2) {
        return x1._l>=x2._l && x1._u <= x2._u; }
    //! The common refinement of \a x1 and \x2. Requires \a x1 and \a x2 to be consistent.
    friend Bounds<F> refinement(Bounds<F> const& x1, Bounds<F> const& x2) {
        return Bounds<F>(max(x1._l,x2._l),min(x1._u,x2._u)); }
    //! Bounds for a value which is in either \a x1 or \a x2.
    friend Bounds<F> coarsening(Bounds<F> const& x1, Bounds<F> const& x2) {
        return Bounds<F>(min(x1._l,x2._l),max(x1._u,x2._u)); }
    //!@}

        // FIXME: Added functionality
        friend Bool same(Bounds<F> const& x, Bounds<Dyadic> const& w);
        friend Bool same(Bounds<F> const& x, Dyadic const& w) { return x._l==w && x._u==w; }
        friend Bool models(Bounds<F> const& x, Dyadic const& w) { return x._l<=w && w<=x._u; }
        friend Bounds<F> round(Bounds<F> const&);

    //!@{
    //! \name Input/output operations
    friend OutputStream& operator<<(OutputStream& os, Bounds<F> const& x) {
        return Operations<Bounds<F>>::_write(os,x); } //!< Write to an output stream.
    friend InputStream& operator>>(InputStream& is, Bounds<F>& x) {
        return Operations<Bounds<F>>::_read(is,x); } //!< Read from an input stream.
    //!@}
  public:
    // Value operations returning Bounds
    friend Bounds<F> operator+(F const& x1, F const& x2) { return add(x1,x2); }
    friend Bounds<F> operator-(F const& x1, F const& x2) { return sub(x1,x2); }
    friend Bounds<F> operator*(F const& x1, F const& x2) { return mul(x1,x2); }
    friend Bounds<F> operator/(F const& x1, F const& x2) { return div(x1,x2); }
    friend Bounds<F> add(F const& x1, F const& x2) { return Bounds<F>{add(down,x1,x2),add(up,x1,x2)}; }
    friend Bounds<F> sub(F const& x1, F const& x2) { return Bounds<F>{sub(down,x1,x2),sub(up,x1,x2)}; }
    friend Bounds<F> mul(F const& x1, F const& x2) { return Bounds<F>{mul(down,x1,x2),mul(up,x1,x2)}; }
    friend Bounds<F> div(F const& x1, F const& x2) { return Bounds<F>{div(down,x1,x2),div(up,x1,x2)}; }
    friend Bounds<F> pow(F const& x, Nat m) { return Bounds<F>{pow(down,x,m),pow(up,x,m)}; }
    friend Bounds<F> pow(F const& x, Int n) { return Bounds<F>{pow(down,x,n),pow(up,x,n)}; }
    friend Bounds<F> sqr(F const& x) { return Bounds<F>{sqr(down,x),sqr(up,x)}; }
    friend Bounds<F> rec(F const& x) { return Bounds<F>{rec(down,x),rec(up,x)}; }
    friend Bounds<F> sqrt(F const& x) { return Bounds<F>{sqrt(down,x),sqrt(up,x)}; }
    friend Bounds<F> exp(F const& x) { return Bounds<F>{exp(down,x),exp(up,x)}; }
    friend Bounds<F> log(F const& x) { return Bounds<F>{log(down,x),log(up,x)}; }
    friend Bounds<F> sin(F const& x) { return Bounds<F>{sin(down,x),sin(up,x)}; }
    friend Bounds<F> cos(F const& x) { return Bounds<F>{cos(down,x),cos(up,x)}; }
    friend Bounds<F> tan(F const& x) { return Bounds<F>{tan(down,x),tan(up,x)}; }
    friend Bounds<F> asin(F const& x) { return Bounds<F>{asin(down,x),asin(up,x)}; }
    friend Bounds<F> acos(F const& x) { return Bounds<F>{acos(down,x),acos(up,x)}; }
    friend Bounds<F> atan(F const& x) { return Bounds<F>{atan(down,x),atan(up,x)}; }
  public:
    // Mixed Bounds-value operations
    friend Bounds<F> add(Bounds<F> const& x1, F const& x2) {
        return Bounds<F>(add(down,x1._l,x2),add(up,x1._u,x2)); }
    friend Bounds<F> sub(Bounds<F> const& x1, F const& x2) {
        return Bounds<F>(sub(down,x1._l,x2),sub(up,x1._u,x2)); }
    friend Bounds<F> mul(Bounds<F> const& x1, F const& x2) {
        return mul(x1,Bounds<F>(x2)); }
    friend Bounds<F> div(Bounds<F> const& x1, F const& x2) {
        return div(x1,Bounds<F>(x2)); }
    friend Bounds<F> add(F const& x1, Bounds<F> const& x2) {
        return Bounds<F>(add(down,x1,x2._l),add(up,x1,x2._u)); }
    friend Bounds<F> sub(F const& x1, Bounds<F> const& x2) {
        return Bounds<F>(sub(down,x1,x2._u),sub(up,x1,x2._l)); }
    friend Bounds<F> mul(F const& x1, Bounds<F> const& x2) {
        return mul(Bounds<F>(x1),x2); }
    friend Bounds<F> div(F const& x1, Bounds<F> const& x2) {
        return div(Bounds<F>(x1),x2); }

    friend Bounds<F> max(Bounds<F> const& x1, F const& x2) {
        return Bounds<F>(max(down,x1._l,x2),max(up,x1._u,x2)); }
    friend Bounds<F> min(Bounds<F> const& x1, F const& x2) {
        return Bounds<F>(min(down,x1._l,x2),min(up,x1._u,x2)); }
    friend Bounds<F> max(F const& x1, Bounds<F> const& x2) {
        return Bounds<F>(max(down,x1,x2._l),max(up,x1,x2._u)); }
    friend Bounds<F> min(F const& x1, Bounds<F> const& x2) {
        return Bounds<F>(min(down,x1,x2._l),min(up,x1,x2._u)); }

    friend Bounds<F> operator+(Bounds<F> const& x1, F const& x2) { return add(x1,x2); }
    friend Bounds<F> operator-(Bounds<F> const& x1, F const& x2) { return sub(x1,x2); }
    friend Bounds<F> operator*(Bounds<F> const& x1, F const& x2) { return mul(x1,x2); }
    friend Bounds<F> operator/(Bounds<F> const& x1, F const& x2) { return div(x1,x2); }
    friend Bounds<F> operator+(F const& x1, Bounds<F> const& x2) { return add(x1,x2); }
    friend Bounds<F> operator-(F const& x1, Bounds<F> const& x2) { return sub(x1,x2); }
    friend Bounds<F> operator*(F const& x1, Bounds<F> const& x2) { return mul(x1,x2); }
    friend Bounds<F> operator/(F const& x1, Bounds<F> const& x2) { return div(x1,x2); }

    friend ValidatedKleenean operator==(Bounds<F> const& x1, F const& x2) { return x1==Bounds<F>(x2); }
    friend ValidatedKleenean operator!=(Bounds<F> const& x1, F const& x2) { return x1!=Bounds<F>(x2); }
    friend ValidatedKleenean operator< (Bounds<F> const& x1, F const& x2) { return x1< Bounds<F>(x2); }
    friend ValidatedKleenean operator> (Bounds<F> const& x1, F const& x2) { return x1> Bounds<F>(x2); }
    friend ValidatedKleenean operator<=(Bounds<F> const& x1, F const& x2) { return x1<=Bounds<F>(x2); }
    friend ValidatedKleenean operator>=(Bounds<F> const& x1, F const& x2) { return x1>=Bounds<F>(x2); }
    friend ValidatedKleenean operator==(F const& x1, Bounds<F> const& x2) { return Bounds<F>(x1)==x2; }
    friend ValidatedKleenean operator!=(F const& x1, Bounds<F> const& x2) { return Bounds<F>(x1)!=x2; }
    friend ValidatedKleenean operator< (F const& x1, Bounds<F> const& x2) { return Bounds<F>(x1)< x2; }
    friend ValidatedKleenean operator> (F const& x1, Bounds<F> const& x2) { return Bounds<F>(x1)> x2; }
    friend ValidatedKleenean operator<=(F const& x1, Bounds<F> const& x2) { return Bounds<F>(x1)<=x2; }
    friend ValidatedKleenean operator>=(F const& x1, Bounds<F> const& x2) { return Bounds<F>(x1)>=x2; }

/*
  public:
    // Mixed Bounds-ValidatedNumber operations
    template<AValidatedNumber Y> friend Bounds<F> operator+(Bounds<F> const& x1, Y const& y2) {
        if constexpr (AnExactDyadic<Y>) { return operator+(x1,F(y2,x1.precision())); }
        else { return operator+(x1,Bounds<F>(y2,x1.precision())); } }
    template<AValidatedNumber Y> friend Bounds<F> operator-(Bounds<F> const& x1, Y const& y2) {
        if constexpr (AnExactDyadic<Y>) { return operator-(x1,F(y2,x1.precision())); }
        else { return operator-(x1,Bounds<F>(y2,x1.precision())); } }
    template<AValidatedNumber Y> friend Bounds<F> operator*(Bounds<F> const& x1, Y const& y2) {
        if constexpr (AnExactDyadic<Y>) { return operator*(x1,F(y2,x1.precision())); }
        else { return operator*(x1,Bounds<F>(y2,x1.precision())); } }
    template<AValidatedNumber Y> friend Bounds<F> operator/(Bounds<F> const& x1, Y const& y2) {
        if constexpr (AnExactDyadic<Y>) { return operator/(x1,F(y2,x1.precision())); }
        else { return operator/(x1,Bounds<F>(y2,x1.precision())); } }
    template<AValidatedNumber Y> friend Bounds<F> operator+(Y const& y1, Bounds<F> const& x2) {
        if constexpr (AnExactDyadic<Y>) { return operator+(F(y1,x2.precision()),x2); }
        else { return operator+(Bounds<F>(y1,x2.precision()),x2); } }
    template<AValidatedNumber Y> friend Bounds<F> operator-(Y const& y1, Bounds<F> const& x2) {
        if constexpr (AnExactDyadic<Y>) { return operator-(F(y1,x2.precision()),x2); }
        else { return operator-(Bounds<F>(y1,x2.precision()),x2); } }
    template<AValidatedNumber Y> friend Bounds<F> operator*(Y const& y1, Bounds<F> const& x2) {
        if constexpr (AnExactDyadic<Y>) { return operator*(F(y1,x2.precision()),x2); }
        else { return operator*(Bounds<F>(y1,x2.precision()),x2); } }
    template<AValidatedNumber Y> friend Bounds<F> operator/(Y const& y1, Bounds<F> const& x2) {
        if constexpr (AnExactDyadic<Y>) { return operator/(F(y1,x2.precision()),x2); }
        else { return operator/(Bounds<F>(y1,x2.precision()),x2); } }
*/
  public:
    static Nat output_places;
    //! %Set the number of output places used to display the number. DEPRECATED
    static Void set_output_places(Nat p) { output_places=p; }
  private: public:
    RawType _l, _u;
};

template<class F> template<class FE> Bounds<F>::Bounds(Ball<F,FE> const& x) : Bounds(x.lower_raw(),x.upper_raw()) { }

template<class FE> inline Bounds<FE> make_bounds(Error<FE> const& e) { return pm(e); }
FloatDP midpoint(Bounds<FloatDP> const& x); // DEPRECATED


template<class PR> Bounds(ValidatedNumber, PR) -> Bounds<RawFloatType<PR>>;
template<class PR> Bounds(ValidatedLowerNumber, ValidatedUpperNumber, PR) -> Bounds<RawFloatType<PR>>;
template<class F> Bounds(F,F) -> Bounds<F>;


template<class F> inline FloatFactory<PrecisionType<F>> factory(Bounds<F> const& flt) { return FloatFactory<PrecisionType<F>>(flt.precision()); }
template<class PR> inline FloatBounds<PR> FloatFactory<PR>::create(Number<ValidatedTag> const& y) { return FloatBounds<PR>(y,_pr); }
template<class PR> inline FloatBounds<PR> FloatFactory<PR>::create(Number<EffectiveTag> const& y) { return FloatBounds<PR>(y,_pr); }
template<class PR> inline FloatBounds<PR> FloatFactory<PR>::create(Number<ExactTag> const& y) { return FloatBounds<PR>(y,_pr); }
template<class PR> inline FloatBounds<PR> FloatFactory<PR>::create(Real const& y) { return FloatBounds<PR>(y,_pr); }
template<class PR> inline FloatBounds<PR> FloatFactory<PR>::create(Rational const& y) { return FloatBounds<PR>(y,_pr); }
template<class PR> inline FloatBounds<PR> FloatFactory<PR>::create(Dyadic const& y) { return FloatBounds<PR>(y,_pr); }
template<class PR> inline FloatBounds<PR> FloatFactory<PR>::create(Integer const& y) { return FloatBounds<PR>(y,_pr); }

template<class PR> inline PositiveFloatBounds<PR> FloatFactory<PR>::create(PositiveValidatedNumber const& y) { return PositiveFloatBounds<PR>(y,_pr); }



template<class F> class Positive<Bounds<F>> : public Bounds<F>
    , public DeclarePositiveFloatOperations<PositiveBounds<F>>
{
    using typename Bounds<F>::PR;
  public:
    Positive() : Bounds<F>() { }
    explicit Positive(PR const& pr) : Bounds<F>(pr) { }
    template<BuiltinUnsignedIntegral M> Positive(M m, PR pr) : Bounds<F>(m,pr) { }
    explicit Positive(F const& x) : Bounds<F>(x) { }
    explicit Positive(F const& l, F const& u) : Bounds<F>(l,u) { }
    explicit Positive(Bounds<F> const& x) : Bounds<F>(x) { }
    Positive(Positive<LowerBound<F>> const& xl, Positive<UpperBound<F>> const& xu) : Bounds<F>(xl,xu) { }
    Positive(PositiveValidatedNumber const& y, PR pr) : Bounds<F>(y,pr) { }
  public:
    Positive<F> value() const { return cast_positive(this->Bounds<F>::value()); }
    Positive<LowerBound<F>> lower() const { return cast_positive(this->Bounds<F>::lower()); }
    Positive<UpperBound<F>> upper() const { return cast_positive(this->Bounds<F>::upper()); }
  public:
    friend PositiveBounds<F> nul(PositiveBounds<F> const& x) {
        return PositiveBounds<F>(nul(x._l),nul(x._u)); }
    friend PositiveBounds<F> hlf(PositiveBounds<F> const& x) {
        return PositiveBounds<F>(hlf(x._l),hlf(x._u)); }
    friend PositiveBounds<F> sqr(PositiveBounds<F> const& x) {
        return PositiveBounds<F>(sqr(down,x._l),sqr(up,x._u)); }
    friend PositiveBounds<F> rec(PositiveBounds<F> const& x) {
        return PositiveBounds<F>(rec(down,x._u),rec(up,x._l)); }
    friend PositiveBounds<F> add(PositiveBounds<F> const& x1, PositiveBounds<F> const& x2) {
        return PositiveBounds<F>(add(down,x1._l,x2._l),add(up,x1._u,x2._u)); }
    friend PositiveBounds<F> mul(PositiveBounds<F> const& x1, PositiveBounds<F> const& x2) {
        return PositiveBounds<F>(mul(down,x1._l,x2._l),mul(up,x1._u,x2._u)); }
    friend PositiveBounds<F> div(PositiveBounds<F> const& x1, PositiveBounds<F> const& x2) {
        return PositiveBounds<F>(div(down,x1._l,x2._u),div(up,x1._u,x2._l)); }
    friend PositiveBounds<F> pow(PositiveBounds<F> const& x, Nat m) {
        return PositiveBounds<F>(pow(down,x._l,static_cast<Int>(m)),pow(up,x._u,static_cast<Int>(m))); }
    friend PositiveBounds<F> sqrt(PositiveBounds<F> const& x) {
        return PositiveBounds<F>(sqrt(down,x._l),sqrt(up,x._u)); }
    friend PositiveBounds<F> atan(PositiveBounds<F> const& x) {
        return PositiveBounds<F>(atan(down,x._l),atan(up,x._u)); }
    friend PositiveBounds<F> max(PositiveBounds<F> const& x1, PositiveBounds<F> const& x2) {
        return PositiveBounds<F>(max(down,x1._l,x2._l),max(up,x1._u,x2._u)); }
    friend PositiveBounds<F> max(PositiveBounds<F> const& x1, Bounds<F> const& x2) {
        return PositiveBounds<F>(max(down,x1._l,x2._l),max(up,x1._u,x2._u)); }
    friend PositiveBounds<F> max(Bounds<F> const& x1, PositiveBounds<F> const& x2) {
        return PositiveBounds<F>(max(down,x1._l,x2._l),max(up,x1._u,x2._u)); }
    friend PositiveBounds<F> min(PositiveBounds<F> const& x1, PositiveBounds<F> const& x2) {
        return PositiveBounds<F>(min(down,x1._l,x2._l),min(up,x2._u,x2._u)); }
    friend PositiveBounds<F> abs(PositiveBounds<F> const& x) { return PositiveBounds<F>(abs(x._l),abs(x._u)); }

    friend Bounds<F> const& cast_unsigned(PositiveBounds<F> const& x) { return x; }
};

template<class F> inline PositiveBounds<F> cast_positive(Bounds<F> const& x) {
    return PositiveBounds<F>(x); }

extern template Ariadne::Nat Ariadne::Bounds<Ariadne::FloatDP>::output_places;
extern template Ariadne::Nat Ariadne::Bounds<Ariadne::FloatMP>::output_places;


template<class F> class Operations<Bounds<F>> {
    typedef PrecisionType<F> PR;
  public:
    static Bounds<F> _mul(Bounds<F> const& x1, Bounds<F> const& x2);
    static Bounds<F> _div(Bounds<F> const& x1, Bounds<F> const& x2);

    static Bounds<F> _fma(Bounds<F> const& x1, Bounds<F> const& x2, Bounds<F> const& x3);

    static Bounds<F> _pi(PR pr);
    static Bounds<F> _sin(Bounds<F> const& x);
    static Bounds<F> _cos(Bounds<F> const& x);
    static Bounds<F> _tan(Bounds<F> const& x);

    static Bounds<F> _trunc(Bounds<F> const& x);
    static Bounds<F> _trunc(Bounds<F> const& x, Nat n);

    static Integer _cast_integer(Bounds<F> const& x);

    // Mixed Bounded - Exact operations
    static Bounds<F> _add(Bounds<F> const& x1, F const& x2);
    static Bounds<F> _add(F const& x1, Bounds<F> const& x2);
    static Bounds<F> _sub(Bounds<F> const& x1, F const& x2);
    static Bounds<F> _sub(F const& x1, Bounds<F> const& x2);
    static Bounds<F> _mul(Bounds<F> const& x1, F const& x2);
    static Bounds<F> _mul(F const& x1, Bounds<F> const& x2);
    static Bounds<F> _div(Bounds<F> const& x1, F const& x2);
    static Bounds<F> _div(F const& x1, Bounds<F> const& x2);

    static OutputStream& _write(OutputStream& os, const Bounds<F>& x);
    static InputStream& _read(InputStream& is, Bounds<F>& x);
};


template<class F> inline auto Operations<Bounds<F>>::_mul(Bounds<F> const& x1, Bounds<F> const& x2) -> Bounds<F>
{
    const F& x1l=x1._l; const F& x1u=x1._u;
    const F& x2l=x2._l; const F& x2u=x2._u;
    PR pr(min(x1.precision(),x2.precision()));
    F rl(pr),ru(pr);
    if(x1l>=0) {
        if(x2l>=0) {
            rl=mul(down,x1l,x2l); ru=mul(up,x1u,x2u);
        } else if(x2u<=0) {
            rl=mul(down,x1u,x2l); ru=mul(up,x1l,x2u);
        } else {
            rl=mul(down,x1u,x2l); ru=mul(up,x1u,x2u);
        }
    }
    else if(x1u<=0) {
        if(x2l>=0) {
            rl=mul(down,x1l,x2u); ru=mul(up,x1u,x2l);
        } else if(x2u<=0) {
            rl=mul(down,x1u,x2u); ru=mul(up,x1l,x2l);
        } else {
            rl=mul(down,x1l,x2u); ru=mul(up,x1l,x2l);
        }
    } else {
        if(x2l>=0) {
            rl=mul(down,x1l,x2u); ru=mul(up,x1u,x2u);
        } else if(x2u<=0) {
            rl=mul(down,x1u,x2l); ru=mul(up,x1l,x2l);
        } else {
            rl=min(mul(down,x1u,x2l),mul(down,x1l,x2u));
            ru=max(mul(up,x1l,x2l),mul(up,x1u,x2u));
        }
    }
    return Bounds<F>(rl,ru);
}

template<class F> inline auto Operations<Bounds<F>>::_div(Bounds<F> const& x1, Bounds<F> const& x2) -> Bounds<F>
{
    const F& x1l=x1.lower_raw(); const F& x1u=x1.upper_raw();
    const F& x2l=x2.lower_raw(); const F& x2u=x2.upper_raw();
    PR pr(min(x1.precision(),x2.precision()));
    F rl(pr),ru(pr);

    // IMPORTANT: Need to be careful when one of the bounds is 0, since if x2l=-0.0 and x1u>0, then x2l>=0 but x1u/x2l=-inf
    if(x2l>0) {
        if(x1l>=0) {
            rl=div(down,x1l,x2u); ru=div(up,x1u,x2l);
        } else if(x1u<=0) {
            rl=div(down,x1l,x2l); ru=div(up,x1u,x2u);
        } else {
            rl=div(down,x1l,x2l); ru=div(up,x1u,x2l);
        }
    }
    else if(x2u<0) {
        if(x1l>=0) {
            rl=div(down,x1u,x2u); ru=div(up,x1l,x2l);
        } else if(x1u<=0) {
            rl=div(down,x1u,x2l); ru=div(up,x1l,x2u);
        } else {
            rl=div(down,x1u,x2u); ru=div(up,x1l,x2u);
        }
    }
    else {
        //ARIADNE_THROW(DivideByZeroException,"FloatBounds div(FloatBounds x1, FloatBounds x2)","x1="<<x1<<", x2="<<x2);
        rl=-F::inf(pr);
        ru=+F::inf(pr);
    }
    return Bounds<F>(rl,ru);
}

template<class F> inline auto Operations<Bounds<F>>::_fma(Bounds<F> const& x1, Bounds<F> const& x2, Bounds<F> const& x3) -> Bounds<F>
{
    return add(mul(x1,x2),x3);
}

}


#endif
