/***************************************************************************
 *            numeric/twoexp.hpp
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

/*! \file numeric/twoexp.hpp
 *  \brief
 */

#ifndef ARIADNE_TWOEXP_HPP
#define ARIADNE_TWOEXP_HPP

#include <cmath>
#include "float.decl.hpp"

namespace Ariadne {

/************ TwoExp ***************************************************/

class ExactDouble;

class Integer;
class Dyadic;

struct Two;

//! \ingroup NumericModule
//! \brief A class representing a number of the form  \c 2<sup><i>n</i></sup> for some <i>n</i>.
//! Useful since floating-point numbers can be exactly multiplied and divided by powers of \c 2.
class TwoExp {
    Int _n;
  public:
    explicit TwoExp(Int n) : _n(n) { }
    constexpr TwoExp(Two const&) : _n(1) { }
    Int exponent() const { return this->_n; }
    // NOTE: Use std::pow(2.0,_n) not (1<<_n) since latter does not handle very large exponents
    FloatDP get_raw(DoublePrecision pr) const;
    FloatMP get_raw(MultiplePrecision pr) const;
    double get_d() const { return std::pow(2.0,this->_n); }
    friend TwoExp rec(TwoExp w) { return TwoExp(-w._n); }
    friend Dyadic operator+(TwoExp);
    friend Dyadic operator-(TwoExp);
    friend Dyadic operator*(Dyadic, TwoExp);
    friend Dyadic operator/(Dyadic, TwoExp);
    friend OutputStream& operator<<(OutputStream& os, TwoExp);
};
inline TwoExp exp2(Int n) { return TwoExp(n); }

//! \ingroup NumericModule
//! \brief The integer constant 2. Only used for creating dyadic numbers.
//! TODO: Should not be a subtype of std::integral_constant<uint,2u> to avoid conversion to builtin integers yielding e.g. 1/two=0.
struct Two
  : public std::integral_constant<uint,2u>
{
    friend Dyadic operator/(Int n, Two);
    friend Dyadic operator/(ExactDouble d, Two);
    friend Dyadic operator/(Integer z, Two);
    friend Dyadic operator/(Dyadic, Two);
    friend TwoExp operator^(Two, Nat m) { return TwoExp(static_cast<Int>(m)); }
    friend TwoExp operator^(Two, Int n) { return TwoExp(n); }
    friend TwoExp pow(Two, int n) { return TwoExp(n); }
};
static const Two two = Two();
static const Two _2 = Two();

} // namespace Ariadne

#endif
