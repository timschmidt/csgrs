/***************************************************************************
 *            numeric/upper_real.hpp
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

/*! \file numeric/upper_real.hpp
 *  \brief Real numbers, including lower, upper and naive types, and validated versions.
 */



#ifndef ARIADNE_UPPER_REAL_HPP
#define ARIADNE_UPPER_REAL_HPP


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
//! \brief Upper real number type \f$\R_>\f$.
//! \details An effectively computable <em>upper real</em> is a real number for which it is possible to compute arbitrarily accurate (dyadic) rational upper bounds;
//! equivalently, a bounded decreasing sequence of (dyadic) rationals converging to the number.
//! However, the convergence rate is not known, and there may be arbitrarily large downward jumps in the sequence.
//!
//! Multiplication and division are unsupported, since given \f$x \leq \overline{x}\f$ and \f$y\leq\overline{y}\f$,
//! we cannot deduce \f$x \times y\leq\overline{x}\times\overline{y}\f$.
//! However, multiplication and reciprocation of positive upper reals <em>is</em> supported.
//!
//! %Positive upper real numbers are a natural type for representing upper bounds of a distance or norm,
//! including the radius of a ball, or the measure of an closed set.
//! \sa Real, LowerReal, NaiveReal
class UpperReal
    : public Handle<const UpperRealInterface>
{
  public:
    typedef EffectiveTag Paradigm;
    typedef UpperReal NumericType;
  public:
    UpperReal(Real);
    explicit UpperReal(SharedPointer<const Interface>);
  public:
    //!@{
    //! \name Computation of rigorous validated upper bounds on the number
    ValidatedUpperReal compute(Effort eff) const;
    DyadicUpperBound compute_get(Effort eff) const;
    FloatDPUpperBound compute_get(Effort eff, DoublePrecision pr) const;
    FloatMPUpperBound compute_get(Effort eff, MultiplePrecision pr) const;
    //!@}

  public:
    //!@{
    //! \name Lattice operations
    friend PositiveNaiveReal abs(UpperReal const& r) = delete; //!< \em No absolute value operator!
    friend UpperReal min(UpperReal const& r1, UpperReal const& r2); //!< The mimimum of \a r1 and \a r2.
    friend UpperReal max(UpperReal const& r1, UpperReal const& r2); //!< The maximum of \a r1 and \a r2.
    //!@}

    //!@{
    //! \name Named arithmetical functions
    friend UpperReal pos(UpperReal const& r); //!< Identity \a +r.
    friend LowerReal neg(UpperReal const& r); //!< Negative \a -r.
    friend UpperReal neg(LowerReal const& r); //!< Negative \a -r.
    friend UpperReal hlf(UpperReal const& r); //!< Half \a r÷2.
    friend UpperReal add(UpperReal const& r1, UpperReal const& r2); //!< \brief Sum \a r1+r2.
    friend UpperReal sub(UpperReal const& r1, LowerReal const& r2); //!< \brief Difference \a r1-r2.
    friend LowerReal sub(LowerReal const& r1, UpperReal const& r2); //!< \brief Difference \a r1-r2.

    friend NaiveReal mul(UpperReal const& r1, UpperReal const& r2) = delete; //!< \brief \em No multiplication operator, since non-monotone!
    friend NaiveReal mul(UpperReal const& r1, LowerReal const& r2) = delete; //!< \brief \em No division operator, since non-monotone!
    friend NaiveReal rec(UpperReal const& r) = delete; //!< \brief \em No reciprocal operation, since non-monotone!
    //!@}

    //!@{
    //! \name Named arithmetical functions on positive numbers
    friend UpperReal mul(UpperReal const& r1, PositiveReal const& r2); //!< Multiplication \a r1×r2 preserves monotonicity.
    friend UpperReal mul(PositiveReal const& r1, UpperReal const& r2); //!< Multiplication \a r1×r2 preserves monotonicity.
    friend UpperReal div(UpperReal const& r1, PositiveReal const& r2); //!< Division \a r1÷r2 preserves monotonicity.
    friend UpperReal div(PositiveReal const& r1, LowerReal const& r2); //!< Division \a r1÷r2 preserves monotonicity.
    friend LowerReal div(PositiveReal const& r1, UpperReal const& r2); //!< Division \a r1÷r2 preserves monotonicity.

    friend PositiveUpperReal add(PositiveUpperReal const& r1, PositiveUpperReal const& r2); //!< Addition \a r1+r2 preserves positivity.
    friend PositiveUpperReal mul(PositiveUpperReal const& r1, PositiveUpperReal const& r2); //!< Multiplication \a r1×r2 preserves positivity.
    friend PositiveUpperReal div(PositiveUpperReal const& r1, PositiveLowerReal const& r2); //!< Division \a r1÷r2 preserves positivity.
    friend PositiveLowerReal div(PositiveLowerReal const& r1, PositiveUpperReal const& r2); //!< Division \a r1÷r2 preserves positivity.
    friend PositiveUpperReal rec(PositiveLowerReal const& r); //!< Reciprocal \a 1/r preserves positivity.
    friend PositiveLowerReal rec(PositiveUpperReal const& r); //!< Reciprocal \a 1/r preserves positivity.
    //!@}

    //!@{
    //! \name Algebraic and transcendental functions
    friend PositiveUpperReal sqrt(PositiveUpperReal const& r); //!< The square root of \a r, √\a r. Requires \a r ≥ 0.
    friend PositiveUpperReal exp(UpperReal const& r); //!< The natural exponent of \a r, \em e<sup>r</sup>.
    friend UpperReal log(PositiveUpperReal const& r); //!< The natural logarithm of \a r. Requires \a r ≥ 0.
    friend UpperReal atan(UpperReal const& r); //!< The arc-tangent of \a r.
    friend PositiveUpperReal atan(PositiveUpperReal const& r); //!< Arc-tangent preserves positivity
    friend NaiveReal sin(UpperReal const& r); //!< No sine operation, since non-monotone!
    friend NaiveReal cos(UpperReal const& r); //!< No cosine operation, since non-monotone!
    friend NaiveReal tan(UpperReal const& r); //!< No tangent operation, since non-monotone!
    //!@}

    //!@{
    //! \name Limit operations.
    friend UpperReal limit(DecreasingSequence<Dyadic> const& qubs);
        //!< Create a real number from an decreasing sequence of dyadic upper bounds.
    //!@}

    //!@{
    //! \name Input/output operations
    friend OutputStream& operator<<(OutputStream& os, UpperReal const& r); //< Write to an output stream.
    //!@}

    //!@{
    //! \name Mixed operations
    friend Real max(UpperReal const& ur1, Real const& r2);
    friend Real max(Real r1, UpperReal const& ur2);
    //!@}
};

//! \ingroup UserNumericTypeSubModule
//! \brief Computable upper real numbers defined by conversion to concrete floats.
class PositiveUpperReal : public UpperReal
{
  public:
    PositiveUpperReal(PositiveReal r) : UpperReal(r) { }
    explicit PositiveUpperReal(UpperReal r) : UpperReal(r) { }
    PositiveUpperBound<Dyadic> compute_get(Effort) const;
  public:
    PositiveUpperReal rec(PositiveLowerReal const&);
    PositiveLowerReal rec(PositiveUpperReal const&);
    PositiveUpperReal add(PositiveUpperReal const&, PositiveUpperReal const&);
    PositiveUpperReal mul(PositiveUpperReal const&, PositiveUpperReal const&);
    PositiveUpperReal div(PositiveUpperReal const&, PositiveLowerReal const&);
    PositiveLowerReal div(PositiveLowerReal const&, PositiveUpperReal const&);
};

PositiveLowerReal rec(PositiveUpperReal pur);
PositiveUpperReal add(PositiveUpperReal pur1, PositiveUpperReal pur2);
PositiveUpperReal mul(PositiveUpperReal pur1, PositiveUpperReal pur2);
PositiveUpperReal div(PositiveUpperReal pur1, PositiveLowerReal plr2);


class ValidatedUpperReal
    : public Handle<const ValidatedRealInterface>
{
  public:
    ValidatedUpperReal(DyadicUpperBound const&);
    ValidatedUpperReal(ValidatedReal const&);
    DyadicUpperBound get() const;
    FloatDPUpperBound get(DoublePrecision) const;
    FloatMPUpperBound get(MultiplePrecision) const;
    friend OutputStream& operator<<(OutputStream&, ValidatedUpperReal const&);
};

} // namespace Ariadne

#endif
