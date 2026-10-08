/***************************************************************************
 *            numeric/float_approximation.hpp
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

/*! \file numeric/float_approximation.hpp
 *  \brief Floating-point approximations to real numbers.
 */

#ifndef ARIADNE_FLOAT_APPROXIMATION_HPP
#define ARIADNE_FLOAT_APPROXIMATION_HPP

#include "foundations/logical.decl.hpp"
#include "number.decl.hpp"
#include "float.decl.hpp"

#include "positive.hpp"

#include "float_traits.hpp"
#include "float_operations.hpp"
#include "float_factory.hpp"

namespace Ariadne {

//! \ingroup NumericModule
//! \brief Floating point number approximations to real numbers supporting approxiamate arithmetic.
//! \details
//! The \c Approximation<F> class represents approximate floating-point numbers.
//! Operations are performed approximately, with no guarantees on the output.
//! \sa Real, NaiveReal, FloatDP, FloatMP, Float, Ball, Bounds.
template<class F> class Approximation
    : public DefineFloatOperations<Approximation<F>>
{
  protected:
    typedef ApproximateTag P; typedef typename F::RoundingModeType RND; typedef typename F::PrecisionType PR;
  public:
    //! <p/>
    typedef ApproximateTag Paradigm;
    //! <p/>
    typedef Approximation<F> NumericType;
    //! <p/>
    typedef ApproximateNumber GenericType;
    //! <p/>
    typedef F RawType;
    //! <p/>
    typedef PR PrecisionType;
    //! <p/>
    typedef PR CharacteristicsType;
  public:

    //! <p/>
    explicit Approximation(PrecisionType pr) : _a(0.0_x,pr) { }
    //! <p/>
    Approximation(RawType const& a) : _a(a) { }

        Approximation(double d, PR pr) : _a(cast_exact(d),near,pr) { }
        Approximation(ApproximateDouble d, PR pr) : _a(cast_exact(d),near,pr) { }
        Approximation(ExactDouble const& d, PR pr) : _a(d,pr) { }
        Approximation(TwoExp const& t, PR pr) :_a(t,pr) { }
        Approximation(const Integer& z, PR pr) : _a(z,near,pr) { }
        Approximation(const Dyadic& w, PR pr) : _a(w,near,pr) { }
        Approximation(const Decimal& d, PR pr) : _a(d,near,pr) { }
        Approximation(const Rational& q, PR pr) : _a(q,near,pr) { }
        Approximation(const Real& r, PR pr); // : _a(r.get(pr)) { }
        Approximation(const Approximation<F>& x, PR pr) : _a(x.raw(),near,pr) { }
    //! <p/>
    Approximation(const ApproximateNumber& y, PR pr); // : _a(y.get(ApproximateTag(),pr)) { }

    //! <p/>
    template<class FF> requires Constructible<F,FF,RND,PR>
        Approximation(const Approximation<FF>& x, PR pr) : _a(x.raw(),near,pr) { }

    //! <p/>
    Approximation(Error<F> const& x); // FIXME: Remove
    //! <p/>
    template<class FE> Approximation(Ball<F,FE> const& x);
    //! <p/>
    Approximation(Bounds<F> const& x);
    //! <p/>
    Approximation(UpperBound<F> const& x);
    //! <p/>
    Approximation(LowerBound<F> const& x);

    //! <p/>
    template<BuiltinIntegral N> Approximation<F>& operator=(N n) { this->_a=n; return *this; }
    //! <p/>
    template<BuiltinFloatingPoint D> Approximation<F>& operator=(D x) { this->_a=ExactDouble(x); return *this; }
        Approximation<F>& operator=(const LowerBound<F>& x) { return *this=Approximation<F>(x); }
        Approximation<F>& operator=(const UpperBound<F>& x) { return *this=Approximation<F>(x); }
        Approximation<F>& operator=(const Bounds<F>& x) { return *this=Approximation<F>(x); }
        Approximation<F>& operator=(const F& x) { return *this=Approximation<F>(x); }
    //! <p/>
    Approximation<F>& operator=(const ApproximateNumber& y) { return *this=Approximation<F>(y,this->precision()); }
    //! <p/>
    Approximation<F> create(const ApproximateNumber& y) const { return Approximation<F>(y,this->precision()); }

    //! <p/>
    operator ApproximateNumber () const;

    //! <p/>
    PrecisionType precision() const { return _a.precision(); }
    //! <p/>
    CharacteristicsType characteristics() const { return _a.precision(); }
    //! <p/>
    GenericType generic() const { return this->operator GenericType(); }
    //! <p/>
    explicit operator RawType () const { return this->_a; }
    //! <p/>
    explicit operator ApproximateDouble () const { return this->_a.get_d(); }
    //! <p/>
    RawType const& raw() const { return this->_a; }
    //! <p/>
    RawType& raw() { return this->_a; }
    //! <p/> DEPRECATED
    double get_d() const { return this->_a.get_d(); }
  public:
#ifdef DOXYGEN
    //!@{
    //! \name Arithmetic operators
    friend Approximation<F> operator+(Approximation<F> const& x); //!< <p/>
    friend Approximation<F> operator-(Approximation<F> const& x); //!< <p/>
    friend Approximation<F> operator+(Approximation<F> const& x1, Approximation<F> const& x2); //!< <p/>
    friend Approximation<F> operator-(Approximation<F> const& x1, Approximation<F> const& x2); //!< <p/>
    friend Approximation<F> operator*(Approximation<F> const& x1, Approximation<F> const& x2); //!< <p/>
    friend Approximation<F> operator/(Approximation<F> const& x1, Approximation<F> const& x2); //!< <p/>
    friend Approximation<F>& operator+=(Approximation<F>& x1, Approximation<F> const& x2); //!< <p/>
    friend Approximation<F>& operator-=(Approximation<F>& x1, Approximation<F> const& x2); //!< <p/>
    friend Approximation<F>& operator*=(Approximation<F>& x1, Approximation<F> const& x2); //!< <p/>
    friend Approximation<F>& operator/=(Approximation<F>& x1, Approximation<F> const& x2); //!< <p/>
    //!@}

    //!@{
    //! \name Comparison operators
    friend ApproximateKleenean operator==(Approximation<F> const& x1, Approximation<F> const& x2); //!< <p/>
    friend ApproximateKleenean operator!=(Approximation<F> const& x1, Approximation<F> const& x2); //!< <p/>
    friend ApproximateKleenean operator<=(Approximation<F> const& x1, Approximation<F> const& x2); //!< <p/>
    friend ApproximateKleenean operator>=(Approximation<F> const& x1, Approximation<F> const& x2); //!< <p/>
    friend ApproximateKleenean operator< (Approximation<F> const& x1, Approximation<F> const& x2); //!< <p/>
    friend ApproximateKleenean operator> (Approximation<F> const& x1, Approximation<F> const& x2); //!< <p/>
    //!@}
#endif // DOXYGEN

    //!@{
    //! \name Arithmetic operations
    friend Approximation<F> nul(Approximation<F> const& x) {
        return Approximation<F>(nul(x._a)); } //!< <p/>
    friend Approximation<F> pos(Approximation<F> const& x) {
        return Approximation<F>(pos(x._a)); } //!< <p/>
    friend Approximation<F> neg(Approximation<F> const& x) {
        return Approximation<F>(neg(x._a)); } //!< <p/>
    friend Approximation<F> hlf(Approximation<F> const& x) {
        return Approximation<F>(hlf(x._a)); } //!< <p/>
    friend Approximation<F> sqr(Approximation<F> const& x) {
        return Approximation<F>(mul(near,x._a,x._a)); } //!< <p/>
    friend Approximation<F> rec(Approximation<F> const& x) {
        return Approximation<F>(rec(near,x._a)); } //!< <p/>

    friend Approximation<F> add(Approximation<F> const& x1, Approximation<F> const& x2) {
        return Approximation<F>(add(near,x1._a,x2._a)); } //!< <p/>
    friend Approximation<F> sub(Approximation<F> const& x1, Approximation<F> const& x2) {
        return Approximation<F>(sub(near,x1._a,x2._a)); } //!< <p/>
    friend Approximation<F> mul(Approximation<F> const& x1, Approximation<F> const& x2) {
        return Approximation<F>(mul(near,x1._a,x2._a)); } //!< <p/>
    friend Approximation<F> div(Approximation<F> const& x1, Approximation<F> const& x2) {
        return Approximation<F>(div(near,x1._a,x2._a)); } //!< <p/>
    friend Approximation<F> fma(Approximation<F> const& x1, Approximation<F> const& x2, Approximation<F> const& x3) {
        return Approximation<F>(fma(near,x1._a,x2._a,x3._a)); } //!< <p/>

    friend Approximation<F> pow(Approximation<F> const& x, Nat m) {
        return Approximation<F>(pow(approx,x._a,static_cast<Int>(m))); } //!< <p/>
    friend Approximation<F> pow(Approximation<F> const& x, Int n) {
        return Approximation<F>(pow(approx,x._a,n)); } //!< <p/>
    //!@}

    //!@{
    //! \name Algebraic and transcendental operations
    friend Approximation<F> sqrt(Approximation<F> const& x) {
        return Approximation<F>(sqrt(approx,x._a)); } //!< <p/>
    friend Approximation<F> exp(Approximation<F> const& x) {
        return Approximation<F>(exp(approx,x._a)); } //!< <p/>
    friend Approximation<F> log(Approximation<F> const& x) {
        return Approximation<F>(log(approx,x._a)); } //!< <p/>
    friend Approximation<F> sin(Approximation<F> const& x) {
        return Approximation<F>(sin(approx,x._a)); } //!< <p/>
    friend Approximation<F> cos(Approximation<F> const& x) {
        return Approximation<F>(cos(approx,x._a)); } //!< <p/>
    friend Approximation<F> tan(Approximation<F> const& x) {
        return Approximation<F>(tan(approx,x._a)); } //!< <p/>
    friend Approximation<F> asin(Approximation<F> const& x) {
        return Approximation<F>(asin(approx,x._a)); } //!< <p/>
    friend Approximation<F> acos(Approximation<F> const& x) {
        return Approximation<F>(acos(approx,x._a)); } //!< <p/>
    friend Approximation<F> atan(Approximation<F> const& x) {
        return Approximation<F>(atan(approx,x._a)); } //!< <p/>
    //!@}

    //!@{
    //! \name Lattice operations
    friend Approximation<F> abs(Approximation<F> const& x) {
        return Approximation<F>(abs(x._a)); } //!< <p/>
    friend Approximation<F> max(Approximation<F> const& x, Approximation<F> const& y) {
        return Approximation<F>(max(x._a,y._a)); } //!< <p/>
    friend Approximation<F> min(Approximation<F> const& x, Approximation<F> const& y) {
        return Approximation<F>(min(x._a,y._a)); } //!< <p/>
    friend PositiveApproximation<F> mag(Approximation<F> const& x) {
        return PositiveApproximation<F>(abs(x._a)); } //!< <p/>
    friend PositiveApproximation<F> mig(Approximation<F> const& x) {
        return PositiveApproximation<F>(abs(x._a)); } //!< <p/>
    //!@}

    //!@{
    //! \name Comparison operations
    friend ApproximateKleenean sgn(Approximation<F> const& x) {
        return ApproximateKleenean(x._a>0 ? LogicalValue::LIKELY : x._a<0 ? LogicalValue::UNLIKELY : LogicalValue::INDETERMINATE); } //!< <p/>
    friend ApproximateKleenean eq(Approximation<F> const& x1, Approximation<F> const& x2) {
        return x1._a==x2._a; } //!< <p/>
    friend ApproximateKleenean lt(Approximation<F> const& x1, Approximation<F> const& x2) {
        return x1._a< x2._a; } //!< <p/>
    //@}

    //!@{
    //! \name Rounding operations
    friend Approximation<F> floor(Approximation<F> const& x) {
        return Approximation<F>(floor(x._a)); } //!< <p/>
    friend Approximation<F> ceil(Approximation<F> const& x) {
        return Approximation<F>(ceil(x._a)); } //!< <p/>
    friend Approximation<F> round(Approximation<F> const& x) {
        return Approximation<F>(round(x._a)); } //!< <p/>
    friend Integer cast_integer(Approximation<F> const& x) {
        return round(static_cast<Dyadic>(x._a)); } //!< <p/>
    //!@}

    //!@{
    //! \name Special value tests
    friend Bool is_nan(Approximation<F> const& x) {
        return is_nan(x._a); } //!< <p/>
    //!@}

    //!@{
    //! \name Approximate information tests
    friend Bool same(Approximation<F> const& x1, Approximation<F> const& x2) {
        return x1._a==x2._a; } //!< Test if \a x1 and \a x2 are the same approximation. They might still possibly represent different numbers.
    //!}

    //!@{
    //! \name Input/output operations
    friend OutputStream& operator<<(OutputStream& os, Approximation<F> const& x) {
        os << "~"; write(os,x.raw(),DecimalPrecision{Approximation<F>::output_places},to_nearest); os << "~"; return os; } //!< Write to an output stream.
    friend InputStream& operator>>(InputStream& is, Approximation<F>& x) {
        char c; is >> c; assert(c=='~'); is >> x._a; is >> c; assert(c=='~'); return is; } //!< Read from an input stream.
    //!@}
  public:
    static Nat output_places;
    //! <p/>
    static Void set_output_places(Nat p) { output_places=p; }
    //! <p/>
    Approximation<F> pm(Approximation<F> _e) { return *this; }
  public:
    RawType _a;
};

template<class F> template<class FE> Approximation<F>::Approximation(Ball<F,FE> const& x) : Approximation(x.value_raw()) { }

template<class PR> Approximation(ApproximateNumber, PR) -> Approximation<RawFloatType<PR>>;
template<class F> Approximation(F) -> Approximation<F>;

template<class F> inline FloatFactory<PrecisionType<F>> factory(Approximation<F> const& flt) { return FloatFactory<PrecisionType<F>>(flt.precision()); }
template<class PR> inline FloatApproximation<PR> FloatFactory<PR>::create(ApproximateNumber const& y) { return FloatApproximation<PR>(y,_pr); }
template<class PR> inline PositiveFloatApproximation<PR> FloatFactory<PR>::create(PositiveApproximateNumber const& y) { return PositiveFloatApproximation<PR>(y,_pr); }
template<class PR> template<BuiltinFloatingPoint D> inline
    FloatApproximation<PR> FloatFactory<PR>::create(D const& y) { return FloatApproximation<PR>(y,_pr); }

template<class F> class Positive<Approximation<F>> : public Approximation<F>
    , public DeclarePositiveFloatOperations<PositiveApproximation<F>>
    , public DefineSemiFieldOperators<PositiveApproximation<F>>
    , public DefineConcreteGenericOperators<PositiveApproximation<F>>
{
    using typename Approximation<F>::PR;
  public:
    Positive() : Approximation<F>() { }
    template<BuiltinUnsignedIntegral M>
        Positive(M m) : Approximation<F>(m) { }
    explicit Positive(PR const& pr) : Approximation<F>(pr) { }
    explicit Positive(F const& x) : Approximation<F>(x) { }
    explicit Positive(Approximation<F> const& x) : Approximation<F>(x) { }
    Positive(PositiveApproximateNumber const& y, PR pr) : Approximation<F>(y,pr) { }
    Positive(PositiveLowerBound<F> const& x) : Approximation<F>(x) { }
    Positive(PositiveUpperBound<F> const& x) : Approximation<F>(x) { }
    Positive(Positive<F> const& x) : Approximation<F>(x) { }
    Positive(Error<F> const& x) : Approximation<F>(x) { }
  public:
    friend PositiveApproximation<F> nul(PositiveApproximation<F> const& x) { return PositiveApproximation<F>(nul(x.raw())); }
    friend PositiveApproximation<F> hlf(PositiveApproximation<F> const& x) { return PositiveApproximation<F>(hlf(x.raw())); }
    friend PositiveApproximation<F> sqr(PositiveApproximation<F> const& x) { return PositiveApproximation<F>(sqr(near,x.raw())); }
    friend PositiveApproximation<F> rec(PositiveApproximation<F> const& x) { return PositiveApproximation<F>(rec(near,x.raw())); }
    friend PositiveApproximation<F> add(PositiveApproximation<F> const& x1, PositiveApproximation<F> const& x2) {
        return PositiveApproximation<F>(add(near,x1.raw(),x2.raw())); }
    friend PositiveApproximation<F> mul(PositiveApproximation<F> const& x1, PositiveApproximation<F> const& x2) {
        return PositiveApproximation<F>(mul(near,x1.raw(),x2.raw())); }
    friend PositiveApproximation<F> div(PositiveApproximation<F> const& x1, PositiveApproximation<F> const& x2) {
        return PositiveApproximation<F>(div(near,x1.raw(),x2.raw())); }
    friend PositiveApproximation<F> pow(PositiveApproximation<F> const& x, Nat m) { return PositiveApproximation<F>(pow(near,x.raw(),static_cast<Int>(m))); }
    friend PositiveApproximation<F> pow(PositiveApproximation<F> const& x, Int n) { return PositiveApproximation<F>(pow(near,x.raw(),n)); }
    friend PositiveApproximation<F> sqrt(PositiveApproximation<F> const& x) { return PositiveApproximation<F>(sqrt(near,x.raw())); }
    friend PositiveApproximation<F> exp(PositiveApproximation<F> const& x) { return PositiveApproximation<F>(exp(near,x.raw())); }
    friend Approximation<F> log(PositiveApproximation<F> const& x) { return Approximation<F>(log(near,x.raw())); }
    friend PositiveApproximation<F> atan(PositiveApproximation<F> const& x) { return PositiveApproximation<F>(atan(near,x.raw())); }
    friend PositiveApproximation<F> max(PositiveApproximation<F> const& x1, PositiveApproximation<F> const& x2) {
        return PositiveApproximation<F>(max(near,x1.raw(),x2.raw())); }
    friend PositiveApproximation<F> max(PositiveApproximation<F> const& x1, Approximation<F> const& x2) {
        return PositiveApproximation<F>(max(near,x1.raw(),x2.raw())); }
    friend PositiveApproximation<F> max(Approximation<F> const& x1, PositiveApproximation<F> const& x2) {
        return PositiveApproximation<F>(max(near,x1.raw(),x2.raw())); }
    friend PositiveApproximation<F> min(PositiveApproximation<F> const& x1, PositiveApproximation<F> const& x2) {
        return PositiveApproximation<F>(min(near,x1.raw(),x2.raw())); }
    friend PositiveApproximation<F> abs(PositiveApproximation<F> const& x) { return PositiveApproximation<F>(abs(x)); }
};

template<class F> inline PositiveApproximation<F> cast_positive(Approximation<F> const& x) {
    return PositiveApproximation<F>(x); }

extern template Ariadne::Nat Ariadne::Approximation<Ariadne::FloatDP>::output_places;
extern template Ariadne::Nat Ariadne::Approximation<Ariadne::FloatMP>::output_places;

}

#endif
