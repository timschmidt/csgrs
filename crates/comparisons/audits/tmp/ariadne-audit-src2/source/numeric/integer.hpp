/***************************************************************************
 *            numeric/integer.hpp
 *
 *  Copyright  2013-22  Pieter Collins
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

/*! \file numeric/integer.hpp
 *  \brief
 */



#ifndef ARIADNE_INTEGER_HPP
#define ARIADNE_INTEGER_HPP

#include "numeric/gmp.hpp"

#include <cassert>

#include "utility/typedefs.hpp"
#include "utility/metaprogramming.hpp"
#include "numeric/sign.hpp"
#include "foundations/logical.hpp"
#include "numeric/arithmetic.hpp"
#include "numeric/int.hpp"
#include "numeric/number.decl.hpp"

namespace Ariadne {

struct ExactTag;
class Integer;
template<> struct IsNumber<Integer> : True { };

template<> struct IsNumber<Nat> : True { };
template<> struct IsNumber<Int> : True { };

//! \ingroup NumericModule
//! \brief Arbitrarily-sized integers.
//! \sa Natural, Dyadic, Decimal, Rational, Real
class Integer
{
  public:
    mpz_t _mpz;
  public:
    typedef ExactTag Paradigm; //!< <p/>
  public:
    ~Integer(); //!< Destructor
    Integer(); //!< Default constructor yielding \c 0
    template<BuiltinUnsignedIntegral M> Integer(M m); //!< Convert from a builtin unsigned (positive) integral type
    template<BuiltinSignedIntegral N> Integer(N n); //!< Convert from a builtin signed (positive or negative) integral type
    explicit Integer(const mpz_t); //!< Construct from a raw GMP integer
    explicit Integer(String const&); //!< Construct from a string literal
    Integer(const Integer&); //!< Copy constructor
    Integer(Integer&&); //!< Move constructor
    Integer& operator=(const Integer&); //!< %Assignment
    Integer& operator=(Integer&&); //!< Move assignment
    operator ExactNumber () const; //!< Convert to a generic exact number
    String literal() const; //!< A string literal

    friend Integer operator"" _z(unsigned long long int n); //!< An integer literal

    //!@{
    //! \name Arithmetic operators
    friend Integer& operator++(Integer& z); //!< <p/>
    friend Integer& operator--(Integer& z); //!< <p/>
    friend Integer operator+(Integer const& z) { return pos(z); } //!< <p/>
    friend Integer operator-(Integer const& z) { return neg(z); } //!< <p/>
    friend Integer operator+(Integer const& z1, Integer const& z2) { return add(z1,z2); } //!< <p/>
    friend Integer operator-(Integer const& z1, Integer const& z2) { return sub(z1,z2); } //!< <p/>
    friend Integer operator*(Integer const& z1, Integer const& z2) { return mul(z1,z2); } //!< <p/>
    friend Rational operator/(Integer const& z1, Integer const& z2); //!< <p/>
    friend Integer operator%(Integer const& z1, Integer const& z2); //!< <p/>
    friend Integer& operator+=(Integer& z1, Integer const& z2); //!< <p/>
    friend Integer& operator-=(Integer& z1, Integer const& z2); //!< <p/>
    friend Integer& operator*=(Integer& z1, Integer const& z2); //!< <p/>
    //!@}

    //!@{
    //! \name Comparison operators
    friend Boolean operator==(Integer const& z1, Integer const& z2) { return eq(z1,z2); } //!< <p/>
    friend Boolean operator!=(Integer const& z1, Integer const& z2) { return !eq(z1,z2); } //!< <p/>
    friend Boolean operator<=(Integer const& z1, Integer const& z2) { return !lt(z2,z1); } //!< <p/>
    friend Boolean operator>=(Integer const& z1, Integer const& z2) { return !lt(z1,z2); } //!< <p/>
    friend Boolean operator< (Integer const& z1, Integer const& z2) { return lt(z1,z2); } //!< <p/>
    friend Boolean operator> (Integer const& z1, Integer const& z2) { return lt(z2,z1); } //!< <p/>
    //!@}

    //!@{
    //! \name Arithmetic operations
    friend Int log2floor(Natural const& n); //!< \f$\lfloor\log_{2}n\rfloor\f$. Returns \c -1 if \a n is \c 0.

    friend Integer nul(Integer const& z); //!< Zero \a 0.
    friend Integer pos(Integer const& z); //!< Identity \a +z.
    friend Integer neg(Integer const& z); //!< Negative \a -z.
    friend Natural sqr(Integer const& z); //!< Square \a z<sup>2</sup>.
    friend Dyadic hlf(Integer const& z); //!< Half \a z÷2.
    friend Dyadic shft(Integer const& z, Int n); //!< Bit-shift \a z×2<sup>n</sup>.
    friend Rational rec(Integer const& z); //!< Reciprocal \a 1/z.

    friend Integer add(Integer const& z1, Integer const& z2); //!< \brief Sum \a z1+z2.
    friend Integer sub(Integer const& z1, Integer const& z2); //!< \brief Difference \a z1-z2.
    friend Integer mul(Integer const& z1, Integer const& z2); //!< \brief Product \a z1×z2.
    friend Rational div(Integer const& z1, Integer const& z2); //!< \brief Quotient \a z1÷z2.
    friend Integer fma(Integer const& z1, Integer const& z2, Integer const& z3); //!< \brief Fused multiply-and-add \a z1×z2+z3.
    friend Integer pow(Integer const& z, Nat m); //!< \brief Power \a z<sup>m</sup>.
    friend Rational pow(Integer const& z, Int n); //!< \brief Power \a z<sup>n</sup>.

    friend Integer quot(Integer const& z1, Integer const& z2); //!< <p/>
    friend Integer rem(Integer const& z1, Integer const& z2); //!< <p/>
    //!@}

    //!@{
    //! \name Lattice operations
    friend Natural abs(Integer const& z); //!< Absolute value \a |z|.
    friend Integer min(Integer const& z1, Integer const& z2); //!< Minimum \a z1∧z2.
    friend Integer max(Integer const& z1, Integer const& z2); //!< Maximum \a z1∨z2.
    //!@}

    //!@{
    //! \name Comparison operations
    friend Sign sgn(Integer const& z); //!< The sign of \a z.
    friend Comparison cmp(Integer const& w1, Integer const& w2); //!< Compares which of \a w1 and \a w2 is larger.
    friend Boolean eq(Integer const& w1, Integer const& w2); //!< Tests if \a w1 is equal to \a w2.
    friend Boolean lt(Integer const& w1, Integer const& w2); //!< Tests if \a w1 is less than \a w2.
    //!@}

    //!@{
    //! \name Special value tests
    friend Bool is_nan(Integer const& z); //!< Tests whether \a z is NaN (not-a-number).
    friend Bool is_inf(Integer const& z); //!< Tests whether \a z is ±∞.
    friend Bool is_finite(Integer const& z); //!< Tests whether \a z is finite.
    friend Bool is_zero(Integer const& z); //!< Tests whether \a z is zero.
    //!@}

    //!@{
    //! \name Input/output operations
    friend OutputStream& operator<<(OutputStream& os, Integer const& z); //!< <p/>
    //!@}

  public:
    template<BuiltinIntegral N> N get() const; //!< Convert to a builtin integral value of type \a N. Throws a \c std::runtime_error if the value does not fit in type \a N.
    long int get_si() const;
    mpz_t const& get_mpz() const;
  private:
  public:
    Integer(Nat32 m);
    Integer(Int32 n);
    Integer(Nat64 m);
    Integer(Int64 n);
};

template<BuiltinUnsignedIntegral M> inline Integer::Integer(M m) : Integer(Nat64(m)) { }
template<BuiltinSignedIntegral N> inline Integer::Integer(N n) : Integer(Int64(n)) { }
Integer operator"" _z(unsigned long long int n);
Integer operator"" _z(const char* str, std::size_t);

template<BuiltinIntegral N> inline N Integer::get() const {
    N n=static_cast<N>(this->get_si()); ARIADNE_ASSERT(Integer(n)==*this); return n; }

template<class R, class A> R integer_cast(const A& a) {
    return cast_integer(a).template get<R>(); }

template<> class Positive<Integer> : public Integer {
  public:
    Positive() : Integer() { }
    template<BuiltinUnsignedIntegral M> Positive(M m) : Integer(m) { }
    Positive(int n) = delete;
    explicit Positive(Integer const& z) : Integer(z) { assert(z>=0); }
};

//! \brief A positive integer.
class Natural : public Positive<Integer> {
  public:
    Natural() : Positive<Integer>() { }
    template<BuiltinUnsignedIntegral M> Natural(M m) : Positive<Integer>(m) { }
    Natural(int n) = delete;
    explicit Natural(Integer const& z) : Positive<Integer>(z) { assert(z>=Integer(0)); }
    friend Natural& operator++(Natural& n) { ++static_cast<Integer&>(n); return n; }
    friend Natural& operator+=(Natural& n1, Natural const& n2) { static_cast<Integer&>(n1)+=n2; return n1; }
    friend Natural operator+(Natural const& n1, Natural const& n2) { return Natural(static_cast<Integer const&>(n1)+static_cast<Integer const&>(n2)); }
    friend Natural operator*(Natural const& n1, Natural const& n2) { return Natural(static_cast<Integer const&>(n1)*static_cast<Integer const&>(n2)); }
};

inline Natural cast_positive(Integer const& z) { return Natural(z); }

} // namespace Ariadne

#endif
