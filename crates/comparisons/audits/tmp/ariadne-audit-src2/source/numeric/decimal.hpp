/***************************************************************************
 *            numeric/decimal.hpp
 *
 *  Copyright  2014-20  Pieter Collins
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

/*! \file numeric/decimal.hpp
 *  \brief ExactTag decimal numbers, useful for user input.
 */

#ifndef ARIADNE_DECIMAL_HPP
#define ARIADNE_DECIMAL_HPP

#include <string>
#include <iostream>
#include "utility/typedefs.hpp"
#include "numeric/number.decl.hpp"
#include "numeric/integer.hpp"

namespace Ariadne {

//! \ingroup NumericModule
//! \brief A decimal number.
//! \sa Integer, Dyadic, Rational, Real
class Decimal
{
  public:
    static const Integer _ten;
    Integer _p; Nat _q;
  public:
    typedef ExactTag Paradigm;
    //! \brief Default constructor creates the number 0 (zero).
    Decimal();
    //! \brief Construct the number p/10^q.
    explicit Decimal(Integer p, Nat q);
    explicit Decimal(Integer p, Int q) = delete;
    //! \brief Construct from a builtin integral type.
    template<BuiltinIntegral N> Decimal(N n);
    //! \brief Construct from a double-precision floating-point number representation.
    explicit Decimal(double d);
    //! \brief Construct from a string representation.
    explicit Decimal(String const&);
    //! \brief Convert from a dyadic.
    Decimal(Dyadic const&);
    //! \brief Convert to a rational number.
    operator Rational () const;
    //! \brief Convert to an generic number.
    operator ExactNumber () const;
    //! \brief A string literal
    String literal() const;

    //! \name Arithmetic operators
    //!@{
    friend Decimal operator+(Decimal const& d); //!< Unary plus \a +d.
    friend Decimal operator-(Decimal const& d); //!< Unary minus \a -d.
    friend Decimal operator+(Decimal const& d1, Decimal const& d2); //!< Plus \a d1+d2.
    friend Decimal operator-(Decimal const& d1, Decimal const& d2); //!< Minus \a d1-d2.
    friend Decimal operator*(Decimal const& d1, Decimal const& d2); //!< Times \a d1*d2.
    friend Rational operator/(Decimal const& d1, Decimal const& d2); //!< Divides \a d1/d2, yielding a rational.
    friend Decimal& operator+=(Decimal& d1, Decimal const& d2); //!< Inplace plus \a d1:=d1+d2.
    //!@}

    //!@{
    //! \name Comparison operators
    friend Boolean operator==(Decimal const& d1, Decimal const& d2) { return cmp(d1,d2)==Comparison::EQUAL; } //!< <p/>
    friend Boolean operator!=(Decimal const& d1, Decimal const& d2) { return cmp(d1,d2)!=Comparison::EQUAL; } //!< <p/>
    friend Boolean operator<=(Decimal const& d1, Decimal const& d2) { return cmp(d2,d1)!=Comparison::GREATER; } //!< <p/>
    friend Boolean operator>=(Decimal const& d1, Decimal const& d2) { return cmp(d1,d2)!=Comparison::LESS; } //!< <p/>
    friend Boolean operator< (Decimal const& d1, Decimal const& d2) { return cmp(d1,d2)==Comparison::LESS; } //!< <p/>
    friend Boolean operator> (Decimal const& d1, Decimal const& d2) { return cmp(d2,d1)==Comparison::GREATER; } //!< <p/>
    //!@}

    //!@{
    //! \name Arithmetic operations
    friend Decimal nul(Decimal const& d); //!< Zero \a 0.
    friend Decimal sqr(Decimal const& d); //!< Square \a d<sup>2</sup>.
    friend Decimal hlf(Decimal const& d); //!< Half \a d/2.
    //!@}

    //!@{
    //! \name Lattice operations
    friend Decimal abs(Decimal const& d); //!< Absolute value \a |d|.
    friend Decimal min(Decimal const& d1, Decimal const& d2); //!< Minimum \a d1∧d2.
    friend Decimal max(Decimal const& d1, Decimal const& d2); //!< Maximum \a d1∨d2.
    //!@}

    //!@{
    //! \name Comparison operations
    friend Comparison cmp(Decimal const& d1, Decimal const& d2); //!< Comparison of two decimal values.
    //!@}

    //! \brief Write to an output stream.
    friend OutputStream& operator<<(OutputStream& os, Decimal const& d);
    //! \brief Construct from an integer literal.
    friend Decimal operator"" _decimal (unsigned long long int n);
    //! \brief Construct from a floating-point literal.
    friend Decimal operator"" _decimal (long double dbl);
    //! \brief Construct from a string literal.
    friend Decimal operator"" _decimal (const char* str, std::size_t);

    //! \brief Shorthand for operator""_decimal.
    friend Decimal operator"" _dec (long double dbl);

    //! \brief Alternative for operator""_decimal for use in Python interface.
    friend Decimal dec_(long double dbl);

    void canonicalize();
};
Decimal operator"" _dec (unsigned long long int n);
Decimal operator"" _dec (long double dbl);
Decimal operator"" _dec (const char* str, std::size_t);
Decimal operator"" _decimal (unsigned long long int n);
Decimal operator"" _decimal (long double dbl);
Decimal operator"" _decimal (const char* str, std::size_t);

template<BuiltinIntegral N> inline Decimal::Decimal(N n) : Decimal(n,0u) { }


template<> class Positive<Decimal> : public Decimal {
  public:
    Positive() : Decimal() { }
    template<BuiltinUnsignedIntegral M> Positive(M m) : Decimal(m) { }
    Positive(int n) = delete;
    explicit Positive(Decimal const& w) : Decimal(w) { ARIADNE_ASSERT(w>=0); }
};
inline Positive<Decimal> cast_positive(Decimal const& w) { return Positive<Decimal>(w); }
using PositiveDecimal = Positive<Decimal>;

template<> class Bounds<Decimal> {
    Decimal _l, _u;
  public:
    Bounds(Decimal w) : _l(w), _u(w) { }
    template<ConvertibleTo<Decimal> X> Bounds(X const& x)
        : Bounds(Decimal(x)) { }
    Bounds(Decimal l, Decimal u) : _l(l), _u(u) { }
    template<class X> requires Constructible<Decimal,X> Bounds(Bounds<X> const& x)
        : Bounds(Decimal(x.lower_raw()),Decimal(x.upper_raw())) { }
    operator ValidatedNumber() const;
    Decimal lower() const { return _l; }
    Decimal upper() const { return _u; }
    Decimal lower_raw() const { return _l; }
    Decimal upper_raw() const { return _u; }
    friend OutputStream& operator<<(OutputStream& os, Bounds<Decimal> y) { return os << "[" << y._l << ":" << y._u << "]"; }
};


} // namespace Ariadne

#endif
