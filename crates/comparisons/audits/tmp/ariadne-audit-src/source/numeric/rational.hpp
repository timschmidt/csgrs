/***************************************************************************
 *            numeric/rational.hpp
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

/*! \file numeric/rational.hpp
 *  \brief
 */



#ifndef ARIADNE_RATIONAL_HPP
#define ARIADNE_RATIONAL_HPP

#include "numeric/gmp.hpp"
#include "utility/typedefs.hpp"
#include "utility/metaprogramming.hpp"
#include "utility/string.hpp"
#include "numeric/integer.hpp"
#include "numeric/arithmetic.hpp"
#include "numeric/number.decl.hpp"
#include "numeric/float.decl.hpp"

namespace Ariadne {

class Int64;

enum class Comparison : ComparableEnumerationType;


/************ Rational *******************************************************/

//! \ingroup NumericModule
//! \brief %Rational numbers.
//! \sa Integer, Dyadic, Decimal, Real
class Rational
{
  public:
    mpq_t _mpq;
  public:
    typedef ExactTag Paradigm;
    typedef Rational NumericType;
  public:
    ~Rational();
    Rational();
    Rational(const Integer&, const Integer&);
    template<BuiltinIntegral N> Rational(N n);
    Rational(Int64);
    explicit Rational(FloatDP const&);
    explicit Rational(FloatMP const&);
    Rational(const ExactDouble&);
    Rational(const Integer&);
    Rational(const Dyadic&);
    explicit Rational(const String&);
    explicit Rational(const mpq_t);
    Rational(const Rational&);
    Rational(Rational&&);
    Rational& operator=(const Rational&);
    Rational& operator=(Rational&&);
    operator ExactNumber () const;

    static Rational inf(Sign sgn);
    static Rational inf();
    static Rational nan();

    Integer get_num() const;
    Integer get_den() const;
    Integer numerator() const;
    Natural denominator() const;

    //! \name Arithmetic operators
    //!@{
    friend Rational operator+(Rational const& q) { return pos(q); }
    friend Rational operator-(Rational const& q) { return neg(q); }
    friend Rational operator+(Rational const& q1, Rational const& q2) { return add(q1,q2); }
    friend Rational operator-(Rational const& q1, Rational const& q2) { return sub(q1,q2); }
    friend Rational operator*(Rational const& q1, Rational const& q2) { return mul(q1,q2); }
    friend Rational operator/(Rational const& q1, Rational const& q2) { return div(q1,q2); }
    friend Rational& operator+=(Rational& q1, Rational const& q2) { return q1=add(q1,q2); }
    friend Rational& operator-=(Rational& q1, Rational const& q2) { return q1=sub(q1,q2); }
    friend Rational& operator*=(Rational& q1, Rational const& q2) { return q1=mul(q1,q2); }
    friend Rational& operator/=(Rational& q1, Rational const& q2) { return q1=div(q1,q2); }
    //@}

    friend Rational operator/(Integer const& z1, Integer const& z2);

    //! \name Comparison operators
    //!@{
    friend Boolean operator==(Rational const& q1, Rational const& q2) { return eq(q1,q2); }
    friend Boolean operator!=(Rational const& q1, Rational const& q2) { return !eq(q1,q2); }
    friend Boolean operator<=(Rational const& q1, Rational const& q2) { return !lt(q2,q1); }
    friend Boolean operator>=(Rational const& q1, Rational const& q2) { return !lt(q1,q2); }
    friend Boolean operator< (Rational const& q1, Rational const& q2) { return lt(q1,q2); }
    friend Boolean operator> (Rational const& q1, Rational const& q2) { return lt(q2,q1); }
    //@}

    //! \name Named arithmetical functions
    //!@{
    friend Rational nul(Rational const& q); //!< Zero \a 0.
    friend Rational pos(Rational const& q); //!< Identity \a +q.
    friend Rational neg(Rational const& q); //!< Negative \a -q.
    friend Rational hlf(Rational const& q); //!< Half \a q÷2.
    friend Positive<Rational> sqr(Rational const& q); //!< Square \a q<sup>2</sup>.
    friend Rational rec(Rational const& q); //!< Reciprocal \a 1/q.
    friend Rational add(Rational const& q1, Rational const& q2); //!< \brief Add \a q1+q2.
    friend Rational sub(Rational const& q1, Rational const& q2); //!< \brief Subtract \a q1-q2.
    friend Rational mul(Rational const& q1, Rational const& q2); //!< \brief Multiply \a q1×q2.
    friend Rational div(Rational const& q1, Rational const& q2); //!< \brief Divide \a q1÷q2.
    friend Rational fma(Rational const& q1, Rational const& q2, Rational const& q3); //!< \brief Fused multiply-and-add \a q1×q2+q3.
    friend Rational pow(Rational const& q, Int n); //!< \brief Power \a q<sup>n</sup>.
    //!@}

    friend Real sqrt(Real const&);
    friend Real exp(Real const&);
    friend Real log(Real const&);
    friend Real sin(Real const&);
    friend Real cos(Real const&);
    friend Real tan(Real const&);
    friend Real asin(Real const&);
    friend Real acos(Real const&);
    friend Real atan(Real const&);

    //! \name Lattice operations
    //!@{
    friend Positive<Rational> abs(Rational const& q); //!< Absolute value \a |q|.
    friend Rational min(Rational const& q1, Rational const& q2); //!< Minimum \a q1∧q2.
    friend Positive<Rational> min(Positive<Rational> const& q1, Positive<Rational> const& q2);
    friend Rational max(Rational const& q1, Rational const& q2); //!< Maximum \a q1∨q2.
    friend Positive<Rational> max(Rational const& q1, Positive<Rational> const& q2);
    friend Positive<Rational> max(Positive<Rational> const& q1, Rational const& q2);
    friend Positive<Rational> max(Positive<Rational> const& q1, Positive<Rational> const& q2);

    friend Rational mag(Rational const& q);
    friend Rational mig(Rational const& q);
    //!@}

    //!@{
    //! \name Rounding operations
    friend Integer floor(Rational const& q); //!< Round \a q down to the nearest lower integer.
    friend Integer round(Rational const& q); //!< Round \a q to the nearest integer. %Rounding of halves is implementation-dependent.
    friend Integer ceil(Rational const& q); //!< Round \a q up to the nearest higher integer.
    //!@}

    //!@{
    //! \name Comparison operations
    friend Sign sgn(Rational const& q); //!< The sign of \a q.
    friend Comparison cmp(Rational const& q1, Rational const& q2); //!< Compares which of \a q1 and \a q2 is larger.
    friend Boolean eq(Rational const& q1, Rational const& q2); //!< Tests if \a q1 is equal to \a q2.
    friend Boolean lt(Rational const& q1, Rational const& q2); //!< Tests if \a q1 is less than \a q2.
    //!@}

    //!@{
    //! \name Special value tests
    friend Bool is_nan(Rational const& q); //!< Tests whether \a q is NaN (not-a-number).
    friend Bool is_inf(Rational const& q); //!< Tests whether \a q is ±∞.
    friend Bool is_finite(Rational const& q); //!< Tests whether \a q is finite.
    friend Bool is_zero(Rational const& q); //!< Tests whether \a q is zero.
    //!@}

    friend Comparison cmp(Rational const& q1, ExactDouble const& d2);
    friend Comparison cmp(ExactDouble const& d1, Rational const& q2);

    //!@{
    //! \name Input/output operations
    friend OutputStream& operator<<(OutputStream& os, Rational const& q); //!< Write to an output stream.
    friend InputStream& operator>>(InputStream& os, Rational& q); //!< Read from an input stream.
    friend Rational operator"" _q(long double x);
    //! \brief Alternative for operator""_q for use in Python interface.
    friend Rational q_(long double x);
    //!@}
  public:
    double get_d() const;
    mpq_t const& get_mpq() const;
  private:
    friend class Dyadic;
};
template<> struct IsNumber<Rational> : True { };
Rational operator"" _q(unsigned long long int n);
Rational operator"" _q(long double x);

template<BuiltinIntegral N> inline Rational::Rational(N n) : Rational(Int64(n)) { }

OutputStream& write(OutputStream& os, mpz_t const z);
InputStream& operator>>(InputStream& is, Rational& q1);

template<> class Positive<Rational> : public Rational {
  public:
    Positive() : Rational() { }
    template<BuiltinUnsignedIntegral M> Positive(M m) : Rational(m) { }
    explicit Positive(Rational const& q) : Rational(q) { ARIADNE_ASSERT(q>=0); }
};
inline Positive<Rational> cast_positive(Rational const& q) { return Positive<Rational>(q); }

using PositiveRational = Positive<Rational>;

using RationalBounds = Bounds<Rational>; //!< Alias for rational bounds on a number. //!< \ingroup NumericModule

template<> class Bounds<Rational> {
    Rational _l, _u;
  public:
    typedef ValidatedTag Paradigm;
    Bounds(Rational q) : _l(q), _u(q) { }
    Bounds(Rational l, Rational u) : _l(l), _u(u) { }
    template<class X> requires Constructible<Rational,X> Bounds(Bounds<X> const& x)
        : Bounds(Rational(x.lower_raw()),Rational(x.upper_raw())) { }
    operator ValidatedNumber() const;
    Bounds<Rational> pm(Rational e) { return RationalBounds(_l-e,_u+e); }
    Rational lower() const { return _l; }
    Rational upper() const { return _u; }
    Rational lower_raw() const { return _l; }
    Rational upper_raw() const { return _u; }
    friend RationalBounds operator+(RationalBounds const& q) { return RationalBounds(+q._l,+q._u); }
    friend RationalBounds operator-(RationalBounds const& q) { return RationalBounds(-q._u,-q._l); }
    friend RationalBounds operator+(RationalBounds const& q1, RationalBounds const& q2) { return RationalBounds(q1._l+q2._l,q1._u+q2._u); }
    friend RationalBounds operator-(RationalBounds const& q1, RationalBounds const& q2) { return RationalBounds(q1._l-q2._u,q1._u-q2._l); }
    friend RationalBounds operator*(RationalBounds const& q1, RationalBounds const& q2);
    friend RationalBounds operator/(RationalBounds const& q1, RationalBounds const& q2);
    friend RationalBounds add(RationalBounds const& q1, RationalBounds const& q2);
    friend RationalBounds sub(RationalBounds const& q1, RationalBounds const& q2);
    friend RationalBounds mul(RationalBounds const& q1, RationalBounds const& q2);
    friend RationalBounds div(RationalBounds const& q1, RationalBounds const& q2);
    friend RationalBounds pow(RationalBounds const& q, Nat m);
    friend RationalBounds pow(RationalBounds const& q, Int n);
    friend RationalBounds nul(RationalBounds const& q);
    friend RationalBounds pos(RationalBounds const& q);
    friend RationalBounds neg(RationalBounds const& q);
    friend RationalBounds hlf(RationalBounds const& q);
    friend RationalBounds sqr(RationalBounds const& q);
    friend RationalBounds rec(RationalBounds const& q);
    friend RationalBounds abs(RationalBounds const& q);
    friend RationalBounds max(RationalBounds const& q1, RationalBounds const& q2);
    friend RationalBounds min(RationalBounds const& q1, RationalBounds const& q2);
    friend ValidatedKleenean operator==(RationalBounds const& q1, RationalBounds const& q2);
    friend ValidatedKleenean operator!=(RationalBounds const& q1, RationalBounds const& q2);
    friend ValidatedKleenean operator<=(RationalBounds const& q1, RationalBounds const& q2);
    friend ValidatedKleenean operator>=(RationalBounds const& q1, RationalBounds const& q2);
    friend ValidatedKleenean operator< (RationalBounds const& q1, RationalBounds const& q2);
    friend ValidatedKleenean operator> (RationalBounds const& q1, RationalBounds const& q2);

    friend Boolean inconsistent(RationalBounds const& q1, RationalBounds const& q2) { return q1._l>q2._u || q1._u < q2._l; }
    friend Boolean refines(RationalBounds const& q1, RationalBounds const& q2) { return q1._l>=q2._l and q1._u<=q2._u; }
    friend RationalBounds refinement(RationalBounds const& q1, RationalBounds const& q2) { return RationalBounds(max(q1._l,q2._l),min(q1._u,q2._u)); }
    friend RationalBounds coarsening(RationalBounds const& q1, RationalBounds const& q2) { return RationalBounds(min(q1._l,q2._l),max(q1._u,q2._u)); }
    friend OutputStream& operator<<(OutputStream& os, RationalBounds y) { return os << "[" << y._l << ":" << y._u << "]"; }
};

} // namespace Ariadne

#endif
