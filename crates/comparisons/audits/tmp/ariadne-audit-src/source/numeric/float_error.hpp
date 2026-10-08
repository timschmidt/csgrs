/***************************************************************************
 *            numeric/float_error.hpp
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

/*! \file numeric/float_error.hpp
 *  \brief Floating-point error bounds for metric spaces.
 */

#ifndef ARIADNE_FLOAT_ERROR_HPP
#define ARIADNE_FLOAT_ERROR_HPP

#include "utility/macros.hpp"
#include "utility/metaprogramming.hpp"

#include "foundations/logical.decl.hpp"
#include "number.decl.hpp"
#include "float.decl.hpp"

#include "builtin.hpp"
#include "positive.hpp"

#include "float_traits.hpp"
#include "float_operations.hpp"
#include "float_factory.hpp"

#include "float_upper_bound.hpp"

namespace Ariadne {

//! \ingroup NumericModule
//! \brief Floating-point upper bounds for positive real numbers, suitable for use as an upper bound for an error in a metric space.
//! \sa FloatDP, FloatMP, Float, UpperBound.
template<class F> class Error
    : public DefineDirectedFloatOperations<UpperBound<F>,LowerBound<F>>

    , public DeclarePositiveDirectedNumericOperations<PositiveUpperBound<F>,PositiveLowerBound<F>>
    , public ProvideConcreteGenericDirectedSemiFieldOperations<PositiveUpperBound<F>,PositiveLowerBound<F>,Nat,Nat>
{
    using PR=typename F::PrecisionType;
  private: public:
    F _e;
  public:
    typedef PositiveValidatedUpperNumber GenericType;
    typedef PR PrecisionType;
    typedef PR CharacteristicsType;
  public:
    Error(Positive<F> const& x);
    Error(PositiveBounds<F> const& x);
    //! Convert from a positive upper bound \a x on the value to be used to represent an error-bound.
    Error(PositiveUpperBound<F> const& x) : _e(x._u) { }
    operator PositiveUpperBound<F> const& () const { return reinterpret_cast<PositiveUpperBound<F>const&>(*this); }
    operator PositiveUpperBound<F>& () { return reinterpret_cast<PositiveUpperBound<F>&>(*this); }
  public:
    explicit Error(PR const& pr) : _e(pr) { }
    //! Treat \a a as an upper-bound for an error.
    //! \pre Requires that \a e is positive (or zero).
    //! \todo Check error not being negative to allow for NaN as a valid input.
    explicit Error(F const& e) : _e(e) { ARIADNE_PRECONDITION_MSG(!(this->_e<0),"e="<<*this); }
    //! Treat the natural number \a m as an upper-bound for an error, represented with precision \a pr.
    template<BuiltinUnsignedIntegral M> Error(M m, PR pr) : _e(m,pr) { }
    explicit Error(UpperBound<F> const& x) : Error(x._u) { }
    //! Treat \a y as an upper-bound for an error, represented with precision \a pr.
    //! \pre Requires that \a y is not definitely strictly negative.
    explicit Error(ValidatedUpperNumber const& y, PR pr) : Error(UpperBound<F>(y,pr)) { }
    explicit Error(const ExactDouble& d, PR pr) : Error(UpperBound<F>(d,pr)) { }
    explicit Error(const TwoExp& t, PR pr) : Error(UpperBound<F>(t,pr)) { }
    //! Assign from the natural number \a m, keeping the same precision.
    template<BuiltinUnsignedIntegral M> Error<F>& operator=(Nat m) {
        reinterpret_cast<UpperBound<F>&>(*this)=m; return *this; }
    //! Assign from the generic error bound \a y, keeping the same precision.
    Error<F>& operator=(ValidatedErrorNumber y);
    operator ValidatedErrorNumber() const;
    friend Bounds<F> pm(Error<F> const& x) { return Bounds<F>(-x._e,+x._e); } //!< <p/>
  public:
    //! Downcast to a generic error bound.
    ValidatedErrorNumber generic() const;
    //! The precision of the floating-point type used.
    PrecisionType precision() const { return _e.precision(); }
    //! The compuational characteristics needed to create the error bound; equivalent to the precision.
    CharacteristicsType characteristics() const { return _e.precision(); }
    //! The raw data used to represent the error bound.
    F const& raw() const { return _e; }
    //! A mutable reference to the raw data used to represent the error bound.
    F& raw() { return _e; }
  public:
    //!@{
    //! \name Arithmetic operators
    friend Error<F> operator+(Error<F> const& x1, Error<F> const& x2) { return Error<F>(add(up,x1._e,x2._e)); } //!< <p/>
    friend Error<F> operator*(Error<F> const& x1, Error<F> const& x2) { return Error<F>(mul(up,x1._e,x2._e)); } //!< <p/>
    friend Error<F>& operator+=(Error<F>& x1, Error<F> const& x2) { return x1=x1+x2; } //!< <p/>
    friend Error<F>& operator*=(Error<F>& x1, Error<F> const& x2) { return x1=x1*x2; } //!< <p/>

    friend UpperBound<F> operator+(Error<F> const& x) { return UpperBound<F>(+x._e); } //!< <p/>
    friend LowerBound<F> operator-(Error<F> const& x) { return LowerBound<F>(-x._e); } //!< <p/>
    friend UpperBound<F> operator+(F const& x1, Error<F> const& x2) { return UpperBound<F>(add(up,x1,x2._e)); } //!< <p/>
    friend LowerBound<F> operator-(F const& x1, Error<F> const& x2) { return LowerBound<F>(sub(down,x1,x2._e)); } //!< <p/>
    //!@}

    //!@{
    //! \name Monotone arithmetic operations
    friend Error<F> nul(Error<F> const& x) { return Error<F>(0u,x.precision()); } //!< <p/>
    friend UpperBound<F> pos(Error<F> const& x) { return UpperBound<F>(pos(x._e)); } //!< <p/>
    friend LowerBound<F> neg(Error<F> const& x) { return LowerBound<F>(neg(x._e)); } //!< <p/>
    friend Error<F> add(Error<F> const& x1, Error<F> const& x2) { return Error<F>(add(up,x1._e,x2._e)); } //!< <p/>
    friend Error<F> mul(Error<F> const& x1, Error<F> const& x2) { return Error<F>(mul(up,x1._e,x2._e)); } //!< <p/>
    friend Error<F> sqr(Error<F> const& x) { return Error<F>(sqr(up,x._e)); } //!< <p/>
    friend Error<F> pow(Error<F> const& x, Nat m) { return Error<F>(pow(up,x._e,static_cast<Int>(m))); } //!< <p/>
    friend Error<F> fma(Error<F> const& x1, Error<F> const& x2, Error<F> const& x3) { return Error<F>(fma(up,x1._e,x2._e,x3._e)); } //!< <p/>
    //!@}

    //!@{
    //! \name Monotone algebraic and transcendental operations
    friend Error<F> sqrt(Error<F> const& x) { return Error<F>(sqrt(up,x._e)); } //!< <p/>
    friend Error<F> exp(Error<F> const& x) { return Error<F>(exp(up,x._e)); } //!< <p/>
    friend UpperBound<F> log(Error<F> const& x) { return UpperBound<F>(log(up,x._e)); } //!< <p/>
    friend UpperBound<F> log2(Error<F> const& x) { return log(x)/cast_positive(log(Bounds<F>(2u,x.precision()))); } //!< <p/>
    //!@}

    //!@{
    //! \name Lattice operations
    friend Error<F> max(Error<F> const& x1, Error<F> const& x2) { return Error<F>(max(x1._e,x2._e)); } //!< <p/>
    friend Error<F> min(Error<F> const& x1, Error<F> const& x2) { return Error<F>(min(x1._e,x2._e)); } //!< <p/>
    friend Error<F> abs(Error<F> const& x) { return x; } //!< <p/>
    friend Error<F> mag(Error<F> const& x) { return x; } //!< <p/>
    //!@}

        friend Error<F> operator+(Error<F> const& x1, Positive<F> const& x2) { return Error<F>(add(up,x1._e,x2)); }
        friend Error<F> operator+(Positive<F> const& x1, Error<F> const& x2) { return Error<F>(add(up,x1,x2._e)); }
        friend Error<F> operator*(Error<F> const& x1, Positive<F> const& x2) { return Error<F>(mul(up,x1._e,x2)); }
        friend Error<F> operator*(Positive<F> const& x1, Error<F> const& x2) { return Error<F>(mul(up,x1,x2._e)); }

        friend Error<F> operator+(Error<F> const& x1, PositiveBounds<F> const& x2) { return Error<F>(add(up,x1._e,x2._u)); }
        friend Error<F> operator+(PositiveBounds<F> const& x1, Error<F> const& x2) { return Error<F>(add(up,x1._u,x2._e)); }
        friend Error<F> operator*(Error<F> const& x1, PositiveBounds<F> const& x2) { return Error<F>(mul(up,x1._e,x2._u)); }
        friend Error<F> operator*(PositiveBounds<F> const& x1, Error<F> const& x2) { return Error<F>(mul(up,x1._u,x2._e)); }

        friend Error<F> operator+(Error<F> const& x1, PositiveUpperBound<F> const& x2) { return Error<F>(add(up,x1._e,x2._u)); }
        friend Error<F> operator+(PositiveUpperBound<F> const& x1, Error<F> const& x2) { return Error<F>(add(up,x1._u,x2._e)); }
        friend Error<F> operator*(Error<F> const& x1, PositiveUpperBound<F> const& x2) { return Error<F>(mul(up,x1._e,x2._u)); }
        friend Error<F> operator*(PositiveUpperBound<F> const& x1, Error<F> const& x2) { return Error<F>(mul(up,x1._u,x2._e)); }

        friend Error<F> operator/(Error<F> const& x1, PositiveLowerBound<F> const& x2) { return Error<F>(div(up,x1._e,x2._l)); }

        friend Error<F> operator+(Error<F> const& x1, Nat m2) { return Error<F>(add(up,x1._e,F(m2,up,x1.precision()))); }
        friend Error<F> operator+(Nat m1, Error<F> const& x2) { return Error<F>(add(up,F(m1,up,x2.precision()),x2._e)); }
        friend Error<F> operator*(Error<F> const& x1, Nat m2) { return Error<F>(mul(up,x1._e,F(m2,up,x1.precision()))); }
        friend Error<F> operator*(Nat m1, Error<F> const& x2) { return Error<F>(mul(up,F(m1,up,x2.precision()),x2._e)); }
        friend Error<F> operator/(Error<F> const& x1, Nat m2) { return Error<F>(div(up,x1._e,F(m2,down,x1.precision()))); }

        friend Approximation<F> operator-(Error<F> const& x1, Error<F> const& x2) { return Approximation<F>(sub(up,x1._e,x2._e)); }
        friend LowerBound<F> operator-(PositiveBounds<F> const& x1, Error<F> const& x2) { return LowerBound<F>(sub(down,x1._l,x2._e)); }
        friend UpperBound<F> operator-(UpperBound<F> const& x1, LowerBound<F> const& x2);
        friend LowerBound<F> operator-(LowerBound<F> const& x1, UpperBound<F> const& x2);

    /*
        friend PositiveUpperBound<F> operator+(PositiveUpperBound<F> const& x1, PositiveUpperBound<F> const& x2);
        friend PositiveUpperBound<F> operator*(PositiveUpperBound<F> const& x1, PositiveUpperBound<F> const& x2);
        friend PositiveUpperBound<F> add(PositiveUpperBound<F> const& x1, PositiveUpperBound<F> const& x2);
        friend PositiveUpperBound<F> mul(PositiveUpperBound<F> const& x1, PositiveUpperBound<F> const& x2);
        friend PositiveUpperBound<F> max(PositiveUpperBound<F> const& x1, PositiveUpperBound<F> const& x2);
        friend PositiveUpperBound<F> min(PositiveUpperBound<F> const& x1, PositiveUpperBound<F> const& x2);
    */
        friend Error<F>& operator+=(Error<F>& x1, PositiveUpperBound<F> const& x2) { return x1=x1+x2; }
        friend Error<F>& operator+=(Error<F>& x1, PositiveBounds<F> const& x2) { return x1=x1+x2; }
        friend Error<F>& operator+=(Error<F>& x1, Positive<F> const& x2) { return x1=x1+x2; }

        friend Error<F>& operator*=(Error<F>& x1, PositiveUpperBound<F> const& x2) { return x1=x1*x2; }
        friend Error<F>& operator*=(Error<F>& x1, PositiveBounds<F> const& x2) { return x1=x1*x2; }
        friend Error<F>& operator*=(Error<F>& x1, Positive<F> const& x2) { return x1=x1*x2; }

  public:
    //!@{
    //! \name Validated information tests and operations
    friend Bool same(Error<F> const& x1, Error<F> const& x2) { return x1._e==x2._e; }
    friend Bool same(Error<F> const& x1, Dyadic const& x2) { return x1._e==x2; }
    //! <p/>
    friend Bool refines(Error<F> const& x1, Error<F> const& x2) { return x1._e<=x2._e; }
    //! <p/>
    friend Error<F> refinement(Error<F> const& x1, Error<F> const& x2) { return Error<F>(min(x1._e,x2._e)); }
    //!@}
  public:
    friend Error<F>const& cast_positive(Error<F> const& x) { return x; }
  public:
    //!@{
    //! \name Input/output operations
    friend OutputStream& operator<<(OutputStream& os, Error<F> const& x) {
        return write(os,x.raw(),DecimalPrecision{Error<F>::output_places},upward); } //!< Write to an output stream.
    friend InputStream& operator>>(InputStream& is, Error<F>& x) {
        UpperBound<F> xu; is >> xu; x=Error<F>(xu); return is; } //!< Read from an input stream.
    //!@}
  public:
    static Nat output_places;
    //! Set the number of decimal places used for the output. DEPRECATED
    static Void set_output_places(Nat p) { output_places=p; }
};

template<class PR> Error(ValidatedUpperNumber, PR) -> Error<RawFloatType<PR>>;
template<class F> Error(F) -> Error<F>;

extern template Ariadne::Nat Ariadne::Error<Ariadne::FloatDP>::output_places;
extern template Ariadne::Nat Ariadne::Error<Ariadne::FloatMP>::output_places;


}

#endif
