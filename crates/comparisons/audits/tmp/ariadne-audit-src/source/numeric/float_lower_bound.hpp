/***************************************************************************
 *            numeric/float_lower_bound.hpp
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

/*! \file numeric/float_lower_bound.hpp
 *  \brief Floating-point lower bounds for real numbers.
 */

#ifndef FLOAT_LOWER_BOUND_H
#define FLOAT_LOWER_BOUND_H

#include "foundations/logical.decl.hpp"
#include "number.decl.hpp"
#include "float.decl.hpp"

#include "positive.hpp"

#include "float_traits.hpp"
#include "float_operations.hpp"
#include "float_factory.hpp"

namespace Ariadne {

//! \ingroup NumericModule
//! \brief Floating-point lower bounds for real numbers.
//! \sa LowerReal, FloatDP, FloatMP, Bounds, UpperBound.
template<class F> class LowerBound
    : public DefineDirectedFloatOperations<LowerBound<F>,UpperBound<F>>
    , public DefineFloatOperations<Approximation<F>>
{
  protected:
    typedef LowerTag P; typedef typename F::RoundingModeType RND; typedef typename F::PrecisionType PR;
  public:
    //! <p/>
    typedef LowerTag Paradigm;
    //! <p/>
    typedef LowerBound<F> NumericType;
    //! <p/>
    typedef ValidatedLowerNumber GenericType;
    //! <p/>
    typedef F RawType;
    //! <p/>
    typedef PR PrecisionType;
    //! <p/>
    typedef PR CharacteristicsType;
  public:
    //! A lower bound of zero with precision \a pr.
    explicit LowerBound(PrecisionType pr) : _l(0.0_x,pr) { }
    //! A lower bound with value \a l.
    LowerBound(RawType const& l) : _l(l) { }

    template<BuiltinIntegral N> LowerBound(N n, PR pr) : LowerBound(ExactDouble(n),pr) { }
    LowerBound(const ExactDouble& d, PR pr) : _l(d,pr) { }
        LowerBound(const TwoExp& t, PR pr) : _l(t,pr) { }
        LowerBound(const Integer& z, PR pr) : _l(z,down,pr) { }
        LowerBound(const Dyadic& w, PR pr) : _l(w,down,pr) { }
        LowerBound(const Decimal& d, PR pr) : _l(d,down,pr) { }
        LowerBound(const Rational& q, PR pr) : _l(q,down,pr) { }
        LowerBound(const Real& r, PR pr);
        LowerBound(const F& x, PR pr);
        LowerBound(const Bounds<F>& x, PR pr);
    LowerBound(const LowerBound<F>& x, PR pr);
    //! A lower bound of type \p F from a generic lower bound \a y.
    LowerBound(const ValidatedLowerNumber& y, PR pr);
    template<class FF> requires Constructible<F,FF,RND,PR>
        LowerBound(const LowerBound<FF>& x, PR pr) : _l(x.raw(),down,pr) { }

    //! Convert from lower \em and upper bounds on a number.
    LowerBound(Bounds<F> const& x);
    template<class FE> LowerBound(Ball<F,FE> const& x);

        LowerBound<F>& operator=(const F& x) { return *this=LowerBound<F>(x); }
        LowerBound<F>& operator=(const Bounds<F>& x) { return *this=x.lower(); }
    //! Assign from the lower bound \a y, keeping the same properties.
    LowerBound<F>& operator=(const ValidatedLowerNumber& y) { return *this=LowerBound<F>(y,this->precision()); }
    //! Create a lower bound from the generic lower bound \a y with the same properties as \a this.
    LowerBound<F> create(const ValidatedLowerNumber& y) const { return LowerBound<F>(y,this->precision()); }
    //! Create a upper bound from the generic upper bound \a y with the same properties as \a this.
    UpperBound<F> create(const ValidatedUpperNumber& y) const { return UpperBound<F>(y,this->precision()); }

    //! Downcast to a generic lower bound.
    operator ValidatedLowerNumber () const;

    //! The precision of the floating-point type used.
    PrecisionType precision() const { return _l.precision(); }
    //! The compuational characteristics needed to create the lower bound; equivalent to the precision.
    CharacteristicsType characteristics() const { return _l.precision(); }
    //! Downcast to a generic lower bound.
    GenericType generic() const;
    //! The raw data used to represent the lower bound.
    RawType const& raw() const { return _l; }
    //! A mutable reference to the raw data used to represent the lower bound.
    RawType& raw() { return _l; }
    //! Under-approximate by a builtin double-precision value. DEPRECATED \deprecated
    double get_d() const { return _l.get_d(); }
  public:
#ifdef DOXYGEN
    //!@{
    //! \name Arithmetic operators
    friend LowerBound<F> operator+(LowerBound<F> const& x); //!< <p/>
    friend LowerBound<F> operator-(UpperBound<F> const& x); //!< <p/>
    friend UpperBound<F> operator-(LowerBound<F> const& x); //!< <p/>
    friend LowerBound<F> operator+(LowerBound<F> const& x1, LowerBound<F> const& x2); //!< <p/>
    friend LowerBound<F> operator-(LowerBound<F> const& x1, UpperBound<F> const& x2); //!< <p/>
    friend UpperBound<F> operator-(UpperBound<F> const& x1, LowerBound<F> const& x2); //!< <p/>
    friend LowerBound<F>& operator+=(LowerBound<F>& x1, LowerBound<F> const& x2); //!< <p/>
    friend LowerBound<F>& operator-=(LowerBound<F>& x1, UpperBound<F> const& x2); //!< <p/>
    friend UpperBound<F>& operator-=(UpperBound<F>& x1, LowerBound<F> const& x2); //!< <p/>

    friend Positive<LowerBound<F>> operator+(Positive<LowerBound<F>> const& x1, Positive<LowerBound<F>> const& x2); //!< <p/>
    friend Positive<LowerBound<F>> operator*(Positive<LowerBound<F>> const& x1, Positive<LowerBound<F>> const& x2); //!< <p/>
    friend Positive<LowerBound<F>> operator/(Positive<LowerBound<F>> const& x1, Positive<UpperBound<F>> const& x2); //!< <p/>
    friend Positive<UpperBound<F>> operator/(Positive<UpperBound<F>> const& x1, Positive<LowerBound<F>> const& x2); //!< <p/>
    friend Positive<LowerBound<F>>& operator+=(Positive<LowerBound<F>> & x1, Positive<LowerBound<F>> const& x2); //!< <p/>
    friend Positive<LowerBound<F>>& operator*=(Positive<LowerBound<F>> & x1, Positive<LowerBound<F>> const& x2); //!< <p/>
    friend Positive<LowerBound<F>>& operator/=(Positive<LowerBound<F>> & x1, Positive<UpperBound<F>> const& x2); //!< <p/>
    friend Positive<UpperBound<F>>& operator/=(Positive<UpperBound<F>> & x1, Positive<LowerBound<F>> const& x2); //!< <p/>
    //!@}

    //!@{
    //! \name Comparison operators
    friend ValidatedUpperKleenean operator<=(LowerBound<F> const& x1, UpperBound<F> const& x2); //!< <p/>
    friend ValidatedLowerKleenean operator>=(LowerBound<F> const& x1, UpperBound<F> const& x2); //!< <p/>
    friend ValidatedUpperKleenean operator< (LowerBound<F> const& x1, UpperBound<F> const& x2); //!< <p/>
    friend ValidatedLowerKleenean operator> (LowerBound<F> const& x1, UpperBound<F> const& x2); //!< <p/>
    friend ValidatedLowerKleenean operator<=(UpperBound<F> const& x1, LowerBound<F> const& x2); //!< <p/>
    friend ValidatedUpperKleenean operator>=(UpperBound<F> const& x1, LowerBound<F> const& x2); //!< <p/>
    friend ValidatedLowerKleenean operator< (UpperBound<F> const& x1, LowerBound<F> const& x2); //!< <p/>
    friend ValidatedUpperKleenean operator> (UpperBound<F> const& x1, LowerBound<F> const& x2); //!< <p/>
    //!@}
#endif // DOXYGEN

    //!@{
    //! \name Monotone arithmetic operations
    friend LowerBound<F> nul(LowerBound<F> const& x) {
        return LowerBound<F>(nul(x._l)); } //!< <p/>
    friend LowerBound<F> pos(LowerBound<F> const& x) {
        return LowerBound<F>(pos(x._l)); } //!< <p/>
    friend UpperBound<F> neg(LowerBound<F> const& x) {
        return UpperBound<F>(neg(x._l)); } //!< <p/>
    friend LowerBound<F> hlf(LowerBound<F> const& x) {
        return LowerBound<F>(hlf(x._l)); } //!< <p/>

    friend LowerBound<F> add(LowerBound<F> const& x1, LowerBound<F> const& x2) {
        return LowerBound<F>(add(down,x1._l,x2._l)); } //!< <p/>
//    friend Approximation<F> sub(LowerBound<F> const& x1, LowerBound<F> const& x2) {
//        return UpperBound<F>(sub(near,x1._l,x2._l)); } //!< <p/>
    friend LowerBound<F> sub(LowerBound<F> const& x1, UpperBound<F> const& x2) {
        return LowerBound<F>(sub(down,x1._l,x2._u)); } //!< <p/>
    //!@}

    //!@{
    //! \name Monotone algebraic and transcendental operations
    friend LowerBound<F> sqrt(LowerBound<F> const& x) {
        return LowerBound<F>(sqrt(down,x.raw())); } //!< <p/>
    friend LowerBound<F> exp(LowerBound<F> const& x) {
        return LowerBound<F>(exp(down,x.raw())); } //!< <p/>
    friend LowerBound<F> log(LowerBound<F> const& x) {
        return LowerBound<F>(log(down,x.raw())); } //!< <p/>
    friend LowerBound<F> atan(LowerBound<F> const& x) {
        return LowerBound<F>(atan(down,x.raw())); } //!< <p/>
    //!@}

    //!@{
    //! \name Lattice operations
    friend Approximation<F> abs(LowerBound<F> const& x) {
        return abs(Approximation<F>(x)); } //!< <p/>
    friend LowerBound<F> max(LowerBound<F> const& x1, LowerBound<F> const& x2) {
        return LowerBound<F>(max(x1._l,x2._l)); } //!< <p/>
    friend LowerBound<F> min(LowerBound<F> const& x1, LowerBound<F> const& x2) {
        return LowerBound<F>(min(x1._l,x2._l)); } //!< <p/>
    //!@}

    //!@{
    //! \name Comparison operations
    friend ValidatedNegatedSierpinskian eq(LowerBound<F> const& x1, UpperBound<F> const& x2) {
        if(x1._l>x2._u) { return false; } //!< <p/>
        else { return ValidatedNegatedSierpinskian(LogicalValue::INDETERMINATE); } } //!< <p/>
    friend ValidatedUpperKleenean lt(LowerBound<F> const& x1, UpperBound<F> const& x2) {
        if(x1._l>=x2._u) { return false; } //!< <p/>
        else { return ValidatedUpperKleenean(LogicalValue::LIKELY); } } //!< <p/>
    //!@}

    //!@{
    //! \name Rounding operations
    friend Integer cast_integer(LowerBound<F> const& x) {
        return floor(static_cast<Dyadic>(x._l)); } //!< <p/>
    //!@}

    //!@{
    //! \name Special value tests
    friend Bool is_nan(LowerBound<F> const& x) {
        return is_nan(x._l); } //!< <p/>
    //!@}

    //!@{
    //! \name Validated information tests and operations
    friend Bool same(LowerBound<F> const& x1, LowerBound<F> const& x2) {
        return x1._l==x2._l; } //!< <p/>
    friend Bool refines(LowerBound<F> const& x1, LowerBound<F> const& x2) {
        return x1._l>=x2._l; } //!< <p/>
    friend LowerBound<F> refinement(LowerBound<F> const& x1, LowerBound<F> const& x2) {
        return LowerBound<F>(max(x1._l,x2._l)); } //!< <p/>
    //!@}

    //!@{
    //! \name Input/output operations
    friend OutputStream& operator<<(OutputStream& os, LowerBound<F> const& x) {
        return write(os,x.raw(),DecimalPrecision{Bounds<F>::output_places},downward); } //!< Write to an output stream.
    friend InputStream& operator>>(InputStream& is, LowerBound<F>& x) {
        ARIADNE_NOT_IMPLEMENTED; } //!< Read from an input stream.
    //!@}
  public:
    friend UpperBound<F> neg(LowerBound<F> const& x);
    friend UpperBound<F> sub(UpperBound<F> const& x1, LowerBound<F> const& x2);
  public:
    friend LowerBound<F> operator*(PositiveBounds<F> const& x1, LowerBound<F> const& x2) {
        return LowerBound<F>(mul(down,x2.raw()>=0?x1.lower().raw():x1.upper().raw(),x2.raw())); }
    friend LowerBound<F> operator*(LowerBound<F> const& x1, PositiveBounds<F> const& x2) {
        return LowerBound<F>(mul(down,x1.raw(),x1.raw()>=0?x2.lower().raw():x2.upper().raw())); }
    friend LowerBound<F> operator/(LowerBound<F> const& x1, PositiveBounds<F> const& x2) {
        return LowerBound<F>(div(down,x1.raw(),x1.raw()>=0?x2.upper().raw():x2.lower().raw())); }
    // Needed to prevent ambiguity; useful as implementation is easier than PositiveBounds version.
    friend LowerBound<F> operator*(Positive<F> const& x1, LowerBound<F> const& x2) {
        return LowerBound<F>(mul(down,x1,x2.raw())); }
    friend LowerBound<F> operator*(LowerBound<F> const& x1, Positive<F> const& x2) {
        return LowerBound<F>(mul(down,x1.raw(),x2)); }
    friend LowerBound<F> operator/(LowerBound<F> const& x1, Positive<F> const& x2) {
        return LowerBound<F>(div(down,x1.raw(),x2)); }
  private: public:
    static Nat output_places;
    RawType _l;
};

template<class F> template<class FE> LowerBound<F>::LowerBound(Ball<F,FE> const& x) : LowerBound(x.lower_raw()) { }

template<class PR> LowerBound(ValidatedLowerNumber, PR) -> LowerBound<RawFloatType<PR>>;
template<class F> LowerBound(F) -> LowerBound<F>;

template<class F> inline FloatFactory<PrecisionType<F>> factory(LowerBound<F> const& flt) { return FloatFactory<PrecisionType<F>>(flt.precision()); }
template<class PR> inline FloatLowerBound<PR> FloatFactory<PR>::create(ValidatedLowerNumber const& y) { return FloatLowerBound<PR>(y,_pr); }
template<class PR> inline PositiveFloatLowerBound<PR> FloatFactory<PR>::create(PositiveValidatedLowerNumber const& y) { return PositiveFloatLowerBound<PR>(y,_pr); }

template<class F> class Positive<LowerBound<F>> : public LowerBound<F>
    , DefineConcreteGenericOperators<PositiveLowerBound<F>>
{
    using typename LowerBound<F>::PR;
  public:
    Positive() : LowerBound<F>() { }
    template<BuiltinUnsignedIntegral M>
        Positive(M m, PR const& pr) : LowerBound<F>(m,pr) { }
    explicit Positive(PR const& pr) : LowerBound<F>(pr) { }
    explicit Positive(F const& x) : LowerBound<F>(x) { }
    explicit Positive(LowerBound<F> const& x) : LowerBound<F>(x) { }
    Positive(PositiveValidatedLowerNumber const& y, PR pr) : LowerBound<F>(y,pr) { }
    Positive(Positive<F> const& x) : LowerBound<F>(x) { }
    Positive(PositiveBounds<F> const& x) : LowerBound<F>(x) { }
  public:
    friend PositiveLowerBound<F> nul(PositiveLowerBound<F> const& x) {
        return PositiveLowerBound<F>(nul(x.raw())); }
    friend PositiveLowerBound<F> pos(PositiveLowerBound<F> const& x) {
        return PositiveLowerBound<F>(pos(x.raw())); }
    friend PositiveLowerBound<F> sqr(PositiveLowerBound<F> const& x) {
        return PositiveLowerBound<F>(sqr(down,x.raw())); }
    friend PositiveLowerBound<F> rec(PositiveUpperBound<F> const& x) {
        return PositiveLowerBound<F>(rec(down,x.raw())); }
    friend PositiveLowerBound<F> add(PositiveLowerBound<F> const& x1, PositiveLowerBound<F> const& x2) {
        return PositiveLowerBound<F>(add(down,x1.raw(),x2.raw())); }
    friend PositiveLowerBound<F> mul(PositiveLowerBound<F> const& x1, PositiveLowerBound<F> const& x2) {
        return PositiveLowerBound<F>(mul(down,x1.raw(),x2.raw())); }
    friend PositiveLowerBound<F> div(PositiveLowerBound<F> const& x1, PositiveUpperBound<F> const& x2) {
        return PositiveLowerBound<F>(div(down,x1.raw(),x2.raw())); }
    friend PositiveLowerBound<F> max(PositiveLowerBound<F> const& x1, PositiveLowerBound<F> const& x2) {
        return PositiveLowerBound<F>(max(x1.raw(),x2.raw())); }
    friend PositiveLowerBound<F> max(PositiveLowerBound<F> const& x1, LowerBound<F> const& x2) {
        return PositiveLowerBound<F>(max(x1.raw(),x2.raw())); }
    friend PositiveLowerBound<F> max(LowerBound<F> const& x1, PositiveLowerBound<F> const& x2) {
        return PositiveLowerBound<F>(max(x1.raw(),x2.raw())); }
    friend PositiveLowerBound<F> min(PositiveLowerBound<F> const& x1, PositiveLowerBound<F> const& x2) {
        return PositiveLowerBound<F>(min(x1.raw(),x2.raw())); }

    friend PositiveUpperBound<F> rec(PositiveLowerBound<F> const& x);
    friend PositiveUpperBound<F> div(PositiveUpperBound<F> const& x1, PositiveLowerBound<F> const& x2);

    friend PositiveLowerBound<F> pow(PositiveLowerBound<F> const& x, Nat m) {
        return PositiveLowerBound<F>(pow(down,x.raw(),static_cast<int>(m))); }
    friend Approximation<F> pow(LowerBound<F> const& x, Int n) {
        return pow(Approximation<F>(x),n); }
  public:
    friend PositiveLowerBound<F> operator+(PositiveLowerBound<F> const& x1, PositiveLowerBound<F> const& x2) {
        return PositiveLowerBound<F>(add(down,x1.raw(),x2.raw())); }
    friend PositiveLowerBound<F> operator*(PositiveLowerBound<F> const& x1, PositiveLowerBound<F> const& x2) {
        return PositiveLowerBound<F>(mul(down,x1.raw(),x2.raw())); }
    friend PositiveLowerBound<F> operator/(PositiveLowerBound<F> const& x1, PositiveUpperBound<F> const& x2) {
        return PositiveLowerBound<F>(mul(down,x1.raw(),x2.raw())); }
    friend PositiveLowerBound<F>& operator+=(PositiveLowerBound<F>& x1, PositiveLowerBound<F> const& x2) {
        return x1=x1+x2; }
    friend PositiveLowerBound<F>& operator*=(PositiveLowerBound<F>& x1, PositiveLowerBound<F> const& x2) {
        return x1=x1*x2; }
    friend PositiveLowerBound<F>& operator/=(PositiveLowerBound<F>& x1, PositiveUpperBound<F> const& x2) {
        return x1=x1/x2; }
  public:
    // Needed to prevent ambiguity
    friend PositiveLowerBound<F> operator*(PositiveBounds<F> const& x1, PositiveLowerBound<F> const& x2) {
        return PositiveLowerBound<F>(mul(down,x1.lower().raw(),x2.raw())); }
    friend PositiveLowerBound<F> operator*(PositiveLowerBound<F> const& x1, PositiveBounds<F> const& x2) {
        return PositiveLowerBound<F>(mul(down,x1.raw(),x2.lower().raw())); }
    friend PositiveLowerBound<F> operator/(PositiveLowerBound<F> const& x1, PositiveBounds<F> const& x2) {
        return PositiveLowerBound<F>(div(down,x1.raw(),x2.upper().raw())); }
    friend PositiveLowerBound<F> operator*(Positive<F> const& x1, PositiveLowerBound<F> const& x2) {
        return PositiveLowerBound<F>(mul(down,x1,x2.raw())); }
    friend PositiveLowerBound<F> operator*(PositiveLowerBound<F> const& x1, Positive<F> const& x2) {
        return PositiveLowerBound<F>(mul(down,x1.raw(),x2)); }
    friend PositiveLowerBound<F> operator/(PositiveLowerBound<F> const& x1, Positive<F> const& x2) {
        return PositiveLowerBound<F>(div(down,x1.raw(),x2)); }
    friend PositiveLowerBound<F> operator/(Positive<F> const& x1, PositiveUpperBound<F> const& x2) {
        return PositiveLowerBound<F>(div(down,x1,x2.raw())); }
    friend PositiveLowerBound<F> operator/(PositiveLowerBound<F> const& x1, Nat m2) {
        return PositiveLowerBound<F>(div(down,x1.raw(),F(m2,x1.precision()))); }
    friend PositiveUpperBound<F> operator/(Positive<F> const& x1, PositiveLowerBound<F> const& x2);
};

template<class F> inline PositiveLowerBound<F> cast_positive(LowerBound<F> const& x) {
    return PositiveLowerBound<F>(x); }

}

#endif
