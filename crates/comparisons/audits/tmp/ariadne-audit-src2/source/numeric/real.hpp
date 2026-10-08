/***************************************************************************
 *            numeric/real.hpp
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

/*! \file numeric/real.hpp
 *  \brief
 */

#ifndef ARIADNE_REAL_HPP
#define ARIADNE_REAL_HPP

#include "utility/handle.hpp"

#include "foundations/logical.decl.hpp"
#include "numeric/number.decl.hpp"
#include "numeric/float.decl.hpp"

#include "numeric/arithmetic.hpp"

namespace Ariadne {

class Accuracy;

class Real;
class LowerReal;
class UpperReal;
class NaiveReal;

class PositiveReal;
class PositiveUpperReal;

class ValidatedReal;

template<> struct IsNumber<Real> : True { };

class Dyadic;
template<class F> class Bounds;
template<class F> class LowerBound;
template<class F> class UpperBound;
template<class F> class Approximation;

template<class X> class ConvergentSequence;
template<class X> class FastCauchySequence;

//! \ingroup NumericModule
extern const Real pi;
extern const Real infinity;

class RealInterface;

//! \ingroup NumericModule
//! \brief %Real number type \f$\R\f$ supporting elementary functions, comparisons and limits.
//! \details Effectively computable real numbers can be computed to arbitrary given accuracy as a (dyadic) rational approximation;
//! equivalently, as arbitrarily tight (dyadic) rational intervals.
//! They can be given as fast-converging Cauchy sequence of (dyadic) rationals,
//! or as a nested sequence of nested (dyadic) rational intervals whose intersection is the number itself.
//!
//! \b Example
//! The follow code creates a real number, computes it to an accuracy of 128 binary digits, a
//! and outputs the result as floating-point bounds with 96 bits of precision:
//! \snippet tutorials/numeric_usage.cpp Real_usage
//!
//! \sa LowerReal, UpperReal, NaiveReal, ValidatedReal,
//! Rational, Decimal, Dyadic, Integer
class Real
    : public Handle<const RealInterface>
    , public DeclareRealOperations<Real,PositiveReal>
    , public DeclareAnalyticFieldOperations<Real>
    , public DeclareLatticeOperations<Real,PositiveReal>
    , public DeclareComparisonOperations<Real,Kleenean,NegatedSierpinskian>
    , public DefineFieldOperators<Real>
{
  public:
    typedef RealInterface Interface;
    typedef EffectiveTag Paradigm;
    typedef Real NumericType;
  public:

    //! \name Constructors
    //!@{
    Real(); //!< Default constructor yields the integer \c 0 as a real number.
    explicit Real(SharedPointer<const Interface>); //!< Construct from any class implementing the real number interface.
    explicit Real(ConvergentSequence<DyadicBounds> const&); //!< Construct from a sequence of dyadic bounds converging to a singleton intersection.
    explicit Real(FastCauchySequence<Dyadic> const&); //!< Construct from a fast convergent sequence of dyadic numbers i.e. \f$|w_m-w_n|\leq 2 ^\min(m,n)\f$.
    //!@}

    //! \name Conversion operators
    //!@{
    explicit Real(double) = delete; //!< Unsafe construction from a double is DEPRECATED

#ifdef DOXYGEN
    Real(Int n); //!< Convert from a builtin integer.
#else
    template<BuiltinUnsignedIntegral M> Real(M m);
    template<BuiltinSignedIntegral N> Real(N n);
#endif
    Real(ExactDouble d); //!< Construct from a double-precision value representing a number exactly.
    Real(Integer const& n); //!< Construct from an integer.
    Real(Dyadic const& d); //!< Construct from a dyadic number \a d=p/2<sup>q</sup> for integers \a p, \a q.
    Real(Decimal const& d); //!< Construct from a decimal number, given by its decimal expansion.
    Real(Rational const& q); //!< Construct from a rational number.

    explicit Real(FloatDP x); //!< DEPRECATED
    explicit Real(EffectiveNumber r); //!< DEPRECATED

    operator EffectiveNumber() const; //!< Convert to an effective number for computations.
    //!@}

    //! \name Convert to real number types describing less information.
    //!@{
    UpperReal upper() const; //!< A real number allowing only computation of upper bounds.
    LowerReal lower() const; //!< A real number allowing only computation of lower bounds.
    //FloatDPApproximation approx() const; //!< A real number allowing only computation of approximations with no guarantees on the accuracy.
    //!@}

    //! \name Computation of rigorous validated bounds on the number.
    //!@{
    //! Compute a concrete approximation with an error of at most \a 2<sup>-acc</sup> i.e. \a acc binary digits.
    ValidatedReal compute(Accuracy acc) const;
    //! Compute a concrete approximation with a bound on the error. The error bound converges to \a 0 as \a eff approaches infinity.
    //! The time taken should be roughly polynomial in \a eff.
    ValidatedReal compute(Effort eff) const;
    //! Compute a concrete approximation using dyadic bounds.
    DyadicBounds compute_get(Effort eff) const;
    //! Compute a concrete approximation using double-precision.
    FloatDPBounds compute_get(Effort eff, DoublePrecision pr) const;
    //! Compute a concrete approximation using the given precision.
    FloatMPBounds compute_get(Effort eff, MultiplePrecision pr) const;
    //! Compute a concrete approximation using double-precision.
    FloatDPBounds get(DoublePrecision pr) const;
    //! Compute a concrete approximation using the given precision.
    FloatMPBounds get(MultiplePrecision pr) const;
    //! Compute a concrete approximation using double-precision.
    FloatDPBounds compute_using(DoublePrecision pr) const;
    //! Compute a concrete approximation using the given precision.
    FloatMPBounds compute_using(MultiplePrecision pr) const;
    //!@}

    //! \name Standard arithmetic operators
    //!@{
    friend Real operator+(Real const& r); //!< Unary plus.
    friend Real operator-(Real const& r); //!< Unary minus.
    friend Real operator+(Real const& r1, Real const& r2); //!< Plus.
    friend Real operator-(Real const& r1, Real const& r2); //!< Minus.
    friend Real operator*(Real const& r1, Real const& r2); //!< Times.
    friend Real operator/(Real const& r1, Real const& r2); //!< Divides.
    friend Real& operator+=(Real& r1, Real const& r2); //!< Inplace plus.
    friend Real& operator-=(Real& r1, Real const& r2); //!< Inplace minus.
    friend Real& operator*=(Real& r1, Real const& r2); //!< Inplace times.
    friend Real& operator/=(Real& r1, Real const& r2); //!< Inplace divides.
    //!@}

    //! \name Standard comparison operators.
    //!@{
    friend NegatedSierpinskian operator==(Real const& r1, Real const& r2); //!< Equality is undecidable and may only robustly be falsified.
    friend Sierpinskian operator!=(Real const& r1, Real const& r2); //!< Inequality is undecidable and may only robustly be verified.
    friend Kleenean operator<=(Real const& r1, Real const& r2); //!< Comparison \c leq.
    friend Kleenean operator>=(Real const& r1, Real const& r2); //!< Comparison .
    friend Kleenean operator< (Real const& r1, Real const& r2); //!< Comparison .
    friend Kleenean operator> (Real const& r1, Real const& r2); //!< Comparison .

    friend Boolean operator>(Real const& r1, Pair<Real,Real> lu); //!< Given \a l<u, returns \a true if r>l and \a false if r<u.
    //!@}

    //! \name Named arithmetical functions
    //!@{
    friend Real nul(Real const& r); //!< Zero \a 0.
    friend Real pos(Real const& r); //!< Identity \a +r.
    friend Real neg(Real const& r); //!< Negative \a -r.
    friend Real hlf(Real const& r); //!< Half \a r÷2.
    friend Real sqr(Real const& r); //!< Square \a r<sup>2</sup>.
    friend Real rec(Real const& r); //!< Reciprocal \a 1/r.
    friend Real add(Real const& r1, Real const& r2); //!< \brief Add \a r1+r2.
    friend Real sub(Real const& r1, Real const& r2); //!< \brief Subtract \a r1-r2.
    friend Real mul(Real const& r1, Real const& r2); //!< \brief Multiply \a r1×r2.
    friend Real div(Real const& r1, Real const& r2); //!< \brief Divide \a r1÷r2.
    friend Real fma(Real const& r1, Real const& r2, Real const& r3); //!< \brief Fused multiply-and-add \a r1×r2+r3.
    friend Real pow(Real const& r, Int n); //!< \brief Power \a r<sup>n</sup>.
    //!@}

    //! \name Algebraic and transcendental functions
    //!@{
    friend Real sqrt(Real const& r); //!< The square root of \a r, √\a r. Requires \a r ≥ 0.
    friend Real exp(Real const& r); //!< The natural exponent of \a r, \em e<sup>r</sup>.
    friend Real log(Real const& r); //!< The natural logarithm of \a r, log<em><sub>e</sub> r</em> or ln <em>r</em>. Requires \a r ≥ 0.
    friend Real sin(Real const& r); //!< The sine of \a r.
    friend Real cos(Real const& r); //!< The cosine of \a r.
    friend Real tan(Real const& r); //!< The tangent of \a r, sin(\a r)/cos(\a r).
    friend Real asin(Real const& r); //!< The arc-sine of \a r.
    friend Real acos(Real const& r); //!< The arc-cosine of \a r.
    friend Real atan(Real const& r); //!< The arc-tangent of \a r.
    //!@}

    //! \name Lattice operations
    //!@{
    friend PositiveReal abs(Real const& r); //!< Absolute value \a |r|.
    friend Real min(Real const& r1, Real const& r2); //!< Mimimum \a r1∧r2.
    friend Real max(Real const& r1, Real const& r2); //!< Maximum \a r1∨r2.
    //!@}

    //! \name Operations based on the metric structure.
    //!@{
    friend PositiveReal dist(Real const& r1, Real const& r2);
        //!< The distance |\a r <sub>1</sub>-\a r <sub>2</sub>| between \a r<sub>1</sub> and \a r<sub>2</sub>.
    friend PositiveUpperReal mag(Real const& r);
        //!< An over-approximation to the absolute value of \a r.
    friend FloatDPError mag(Real const&, DoublePrecision);
    //!@}

    //! \name Comparison operations.
    //!@{
    friend NegatedSierpinskian eq(Real const& r1, Real const& r2); //!< Returns \c false if \a r1!=r2 and \c indeterminate if \a r1==r2.
    friend Kleenean lt(Real const& r1, Real const& r2); //!< Returns \c true if \a r1<r2, \c false if \a r1\>r2, and \c indetermiate if \a r1==r2.
    friend Kleenean leq(Real const& r1, Real const& r2); //!< Returns \c true if \a r1<r2, \c false if \a r1\>r2, and \c indetermiate if \a r1==r2.

    friend Kleenean sgn(Real const& r); //!< Returns \c true if \a r>0, \c false if \a r<0, and \c indeterminate if \a r==0.
    friend ValidatedKleenean check_sgn(Real r, Effort eff); //!< Equivalent to \c sgn(r).check(eff).
    friend Boolean nondeterministic_greater(Real const& r, Rational const& a, Rational const& b); //!< Given \a a<b, returns \a true if r>a and \a false if r<b.

    friend Real choose(Case<LowerKleenean,Real> const& c1, Case<LowerKleenean,Real> const& c2);
        //!< A nonextensional choice, between the value of any of the valid cases.
    friend Real when(Case<UpperKleenean,Real> const& c1, Case<UpperKleenean,Real> const& c2);
        //!< A value equal to that of each of the valid cases.
    //!@}

    //! \name Limit operations.
    //!@{
    friend Real limit(ConvergentSequence<DyadicBounds> const& qbs);
        //!< Create a real number from a sequence of dyadic bounds whose width converges to 0.
    friend Real limit(FastCauchySequence<Dyadic> const& qs);
        //!< Create a real number from a sequence of dyadic numbers \f$q_n\f$ for which
        //!< \f$|q_{n_1}-q_{n_2}|\leq 2^{\min(n_1,n_2)}\f$
    friend Real limit(FastCauchySequence<Real> const& rs);
        //!< The limit of a sequence of real numbers \f$r_n\f$ for which
        //!< \f$|r_{n_1}-r_{n_2}|\leq 2^{\min(n_1,n_2)}\f$
    //!@}

    //! \name Rounding operations.
    //!@{
    friend Integer round(Real const& r);
        //!< Round to a "nearby" integer. This integer <em>need not</em> be the <em>closest</em> integer.
    //!@}


    //! \name Operations on the representation.
    //!@{
    friend Bool same(Real const&, Real const&); //!< Test equivalence of representation.
    //!@}

    //! \name Input/output operations
    //!@{
    friend OutputStream& operator<<(OutputStream& os, Real const& r); //!< Write to an output stream.
    //!@}
    double get_d() const;
  private:
    Real(std::int64_t n, Void*);
    Real(std::uint64_t m, Void*);
};

template<BuiltinUnsignedIntegral M> inline Real::Real(M m) : Real(std::uint64_t(m),nullptr) { }
template<BuiltinSignedIntegral N> inline Real::Real(N n) : Real(std::int64_t(n),nullptr) { }

Real choose(Case<LowerKleenean,Real> const& c1, Case<LowerKleenean,Real> const& c2);
Real when(Case<UpperKleenean,Real> const& c1, Case<UpperKleenean,Real> const& c2);


//! \ingroup UserNumericTypeSubModule
//! \brief Computable lower real numbers defined by conversion to concrete floats.
class PositiveReal : public Real
{
  public:
    PositiveReal() : Real() { }
    template<ConvertibleTo<Real> Y>
        PositiveReal(Positive<Y> const& p) : Real(p) { }
    explicit PositiveReal(Real const& r) : Real(r) { }
    PositiveBounds<Dyadic> compute_get(Effort) const;
  public:
    friend PositiveReal max(PositiveReal const& pr1, PositiveReal const& pr2);
    friend PositiveReal max(Real const& r1, PositiveReal const& pr2);
    friend PositiveReal max(PositiveReal const& pr1, Real const& r2);

    friend PositiveReal min(PositiveReal const& pr1, PositiveReal const& pr2);

    friend PositiveReal rec(PositiveReal const& pr);
    friend PositiveReal add(PositiveReal const& pr1, PositiveReal const& pr2);
    friend PositiveReal mul(PositiveReal const& pr1, PositiveReal const& pr2);
    friend PositiveReal div(PositiveReal const& pr1, PositiveReal const& pr2);
};

PositiveReal cast_positive(Real const& x);

ValidatedKleenean check_sgn(Real r, Effort eff);

} // namespace Ariadne

#endif
