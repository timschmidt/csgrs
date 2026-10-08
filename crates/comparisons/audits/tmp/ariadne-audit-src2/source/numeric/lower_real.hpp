/***************************************************************************
 *            numeric/lower_real.hpp
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

/*! \file numeric/lower_real.hpp
 *  \brief Real numbers, including lower, upper and naive types, and validated versions.
 */



#ifndef ARIADNE_LOWER_REAL_HPP
#define ARIADNE_LOWER_REAL_HPP


#include "utility/typedefs.hpp"
#include "utility/pointer.hpp"
#include "utility/handle.hpp"

#include "foundations/logical.decl.hpp"
#include "numeric/number.decl.hpp"
#include "numeric/float.decl.hpp"

#include "foundations/paradigm.hpp"
#include "numeric/arithmetic.hpp"
#include "numeric/sequence.hpp"

#include "numeric/dyadic.hpp"

#include "numeric/real.hpp"
#include "numeric/validated_real.hpp"
#include "numeric/approximate_real.hpp"

namespace Ariadne {

class Accuracy;

class Real;
class LowerReal;
class UpperReal;
class NaiveReal;
class PositiveReal;
class PositiveLowerReal;
class PositiveUpperReal;
class PositiveNaiveReal;

class ValidatedReal;
class ValidatedUpperReal;
class ValidatedLowerReal;
class ApproximateReal;


class RealInterface;
using LowerRealInterface = RealInterface;
using UpperRealInterface = RealInterface;
using NaiveRealInterface = RealInterface;

class ValidatedRealInterface;



//! \ingroup NumericModule
//! \brief Lower real number type \f$\R_<\f$.
//! \details An effectively computable <em>lower real</em> is a real number for which it is possible to compute arbitrarily accurate (dyadic) rational lower bounds;
//! equivalently, a bounded increasing sequence of (dyadic) rationals converging to the number.
//! However, the convergence rate is not known, and there may be arbitrarily large upward jumps in the sequence.
//!
//! Multiplication and division are unsupported, since given \f$x \geq \underline{x}\f$ and \f$y\geq\underline{y}\f$,
//! we cannot deduce \f$x \times y\geq\underline{x}\times\underline{y}\f$ if the bounds are negative.
//! However, multiplication and reciprocation of positive lower reals <em>is</em> supported.
//!
//! %Positive lower real numbers are a natural type for representing the measure of an open set,
//! or the separation between two points in a metric space.
//! \sa Real, UpperReal, NaiveReal
class LowerReal
    : public Handle<const LowerRealInterface>
{
  public:
    typedef EffectiveTag Paradigm;
    typedef LowerReal NumericType;
  public:
    LowerReal(Real);
    explicit LowerReal(SharedPointer<const Interface>);
  public:
    //!@{
    //! \name Computation of rigorous validated lower bounds on the number
    ValidatedLowerReal compute(Effort eff) const;
    DyadicLowerBound compute_get(Effort eff) const;
    FloatDPLowerBound compute_get(Effort eff, DoublePrecision pr) const;
    FloatMPLowerBound compute_get(Effort eff, MultiplePrecision pr) const;
    //!@}
  public:
    //!@{
    //! \name Lattice operations
    friend PositiveNaiveReal abs(LowerReal const& r) = delete; //!< \em No absolute value operator!
    friend LowerReal min(LowerReal const& r1, LowerReal const& r2); //!< The mimimum of \a r1 and \a r2.
    friend LowerReal max(LowerReal const& r1, LowerReal const& r2); //!< The maximum of \a r1 and \a r2.
    //!@}

    //!@{
    //! \name Named arithmetical functions
    friend LowerReal pos(LowerReal const& r); //!< Identity \a +r.
    friend UpperReal neg(LowerReal const& r); //!< Negative \a -r.
    friend LowerReal neg(UpperReal const& r); //!< Negative \a -r.
    friend LowerReal hlf(LowerReal const& r); //!< Half \a r÷2.
    friend LowerReal add(LowerReal const& r1, LowerReal const& r2); //!< \brief Sum \a r1+r2.
    friend LowerReal sub(LowerReal const& r1, UpperReal const& r2); //!< \brief Difference \a r1-r2.
    friend UpperReal sub(UpperReal const& r1, LowerReal const& r2); //!< \brief Difference \a r1-r2.

    friend NaiveReal mul(LowerReal const& r1, LowerReal const& r2) = delete; //!< \brief \em No multiplication operator, since non-monotone!
    friend NaiveReal div(LowerReal const& r1, UpperReal const& r2) = delete; //!< \brief \em No division operator, since non-monotone!
    friend NaiveReal rec(LowerReal const& r) = delete; //!< \brief \em No reciprocal operation, since non-monotone!
    //!@}

    //!@{
    //! \name Named arithmetical functions on positive numbers
    friend LowerReal mul(LowerReal const& r1, PositiveReal const& r2); //!< Multiplication \a r1×r2 preserves monotonicity.
    friend LowerReal mul(PositiveReal const& r1, LowerReal const& r2); //!< Multiplication \a r1×r2 preserves monotonicity.
    friend LowerReal div(LowerReal const& r1, PositiveReal const& r2); //!< Division \a r1÷r2 preserves monotonicity.
    friend LowerReal div(PositiveReal const& r1, UpperReal const& r2); //!< Division \a r1÷r2 preserves monotonicity.
    friend UpperReal div(PositiveReal const& r1, LowerReal const& r2); //!< Division \a r1÷r2 preserves monotonicity.

    friend PositiveLowerReal add(PositiveLowerReal const& r1, PositiveLowerReal const& r2); //!< Addition \a r1+r2 preserves positivity.
    friend PositiveLowerReal mul(PositiveLowerReal const& r1, PositiveLowerReal const& r2); //!< Multiplication \a r1×r2 preserves positivity.
    friend PositiveLowerReal div(PositiveLowerReal const& r1, PositiveUpperReal const& r2); //!< Division \a r1÷r2 preserves positivity.
    friend PositiveUpperReal div(PositiveUpperReal const& r1, PositiveLowerReal const& r2); //!< Division \a r1÷r2 preserves positivity.
    friend PositiveLowerReal rec(PositiveUpperReal const& r); //!< Reciprocal \a 1/r preserves positivity.
    friend PositiveUpperReal rec(PositiveLowerReal const& r); //!< Reciprocal \a 1/r preserves positivity.
    //!@}

    //!@{
    //! \name Algebraic and transcendental functions
    friend PositiveLowerReal sqrt(PositiveLowerReal const& r); //!< The square root of \a r, √\a r. Requires \a r ≥ 0.
    friend PositiveLowerReal exp(LowerReal const& r); //!< The natural exponent of \a r, \em e<sup>r</sup>.
    friend LowerReal log(PositiveLowerReal const& r); //!< The natural logarithm of \a r. Requires \a r ≥ 0.
    friend LowerReal atan(LowerReal const& r); //!< The arc-tangent of \a r.
    friend PositiveLowerReal atan(PositiveLowerReal const& r); //!< Arc-tangent preserves positivity

    friend NaiveReal sin(LowerReal const& r); //!< No sine operation, since non-monotone!
    friend NaiveReal cos(LowerReal const& r); //!< No cosine operation, since non-monotone!
    friend NaiveReal tan(LowerReal const& r); //!< No tangent operation, since non-monotone!
    //!@}

    //!@{
    //! \name Limit operations.
    friend LowerReal limit(IncreasingSequence<Dyadic> const& qlbs);
        //!< Create a real number from an increasing sequence of dyadic lower bounds.
    //!@}

    //!@{
    //! \name Input/output operations
    friend OutputStream& operator<<(OutputStream& os, LowerReal const& r); //< Write to an output stream.
    //!@}

    //!@{
    //! \name Mixed operations
    friend Real min(LowerReal const& lr1, Real const& r2);
    friend Real min(Real const& r1, LowerReal const& lr2);
    //!@}
};

//! \ingroup UserNumericTypeSubModule
//! \brief Computable lower real numbers defined by conversion to concrete floats.
class PositiveLowerReal : public LowerReal
{
  public:
    PositiveLowerReal(PositiveReal r) : LowerReal(r) { }
    explicit PositiveLowerReal(LowerReal r) : LowerReal(r) { }
    PositiveLowerBound<Dyadic> compute_get(Effort) const;
  public:
    PositiveLowerReal rec(PositiveUpperReal const&);
    PositiveUpperReal rec(PositiveLowerReal const&);
    PositiveLowerReal add(PositiveLowerReal const&, PositiveLowerReal const&);
    PositiveLowerReal mul(PositiveLowerReal const&, PositiveLowerReal const&);
    PositiveLowerReal div(PositiveLowerReal const&, PositiveUpperReal const&);
    PositiveUpperReal div(PositiveUpperReal const&, PositiveLowerReal const&);
};

PositiveUpperReal rec(PositiveLowerReal plr);
PositiveLowerReal add(PositiveLowerReal plr1, PositiveLowerReal plr2);
PositiveLowerReal mul(PositiveLowerReal plr1, PositiveLowerReal plr2);
PositiveLowerReal div(PositiveLowerReal plr1, PositiveUpperReal pur2);

class ValidatedLowerReal
    : public Handle<const ValidatedRealInterface>
{
  public:
    ValidatedLowerReal(DyadicLowerBound const&);
    ValidatedLowerReal(ValidatedReal const&);
    DyadicLowerBound get() const;
    FloatDPLowerBound get(DoublePrecision) const;
    FloatMPLowerBound get(MultiplePrecision) const;
    friend OutputStream& operator<<(OutputStream&, ValidatedLowerReal const&);
};

} // namespace Ariadne

#endif
