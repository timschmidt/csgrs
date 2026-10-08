/***************************************************************************
 *            numeric/bits.hpp
 *
 *  Copyright  2020  Pieter Collins
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

/*! \file numeric/bits.hpp
 *  \brief A class representing count of binary digits.
 */

#ifndef ARIADNE_BITS_HPP
#define ARIADNE_BITS_HPP

#include <iosfwd>
#include <cassert>

namespace Ariadne {

using OutputStream = std::ostream;

//! \brief A count of a number of binary digits, usable to define an accuracy or precision specification.
class Bits {
    unsigned long int _bits;
    explicit Bits(unsigned long long int bits) : _bits(bits) {
        assert(static_cast<unsigned long long int>(this->_bits)==bits); }
  public:
    operator unsigned long int () const { return this->_bits; }
    friend Bits operator""_bits (unsigned long long int);
    friend OutputStream& operator<<(OutputStream& os, Bits const& bits);
};
inline Bits operator""_bits (unsigned long long int bits) { return Bits(bits); }

} // namespace Ariadne

#endif // ARIADNE_BITS_HPP
