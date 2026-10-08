/***************************************************************************
 *            numeric/int.hpp
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

/*! \file numeric/int.hpp
 *  \brief Fixed-width integral classes
 */

#ifndef ARIADNE_INT_HPP
#define ARIADNE_INT_HPP

#include <cassert>
#include "numeric/concepts.hpp"

namespace Ariadne {

/************  Ints ********************************************************/

class Nat32 {
    uint32_t _m;
  public:
    Nat32() : _m(0u) { }
    template<BuiltinUnsignedIntegral M> Nat32(M m) : _m(m) { assert(_m==m); }
    template<BuiltinSignedIntegral N> Nat32(N n) : _m(n) { assert(n>=0); assert((int64_t)_m==n); }
    uint32_t get_ui() const { return _m; }
};

class Nat64 {
    uint64_t _m;
  public:
    Nat64() : _m(0u) { }
    template<BuiltinUnsignedIntegral M> Nat64(M m) : _m(m) { assert(_m==m); }
    template<BuiltinSignedIntegral N> Nat64(N n) : _m(static_cast<uint64_t>(n)) { assert(n>=0); assert((int64_t)_m==n);
        assert(uint64_t(int64_t(_m))==_m); }
    uint64_t get_ui() const { return _m; }
};

class Int32 {
    int32_t _n;
  public:
    Int32() : _n(0) { }
    template<BuiltinUnsignedIntegral M> Int32(M m) : _n(static_cast<int32_t>(m)) { assert(_n>=0); assert((uint32_t)_n==m); }
    template<BuiltinSignedIntegral N> Int32(N n) : _n(n) { assert(_n==n); }
    int32_t get_si() const { return _n; }
};

class Int64 {
    int64_t _n;
  public:
    Int64() : _n(0) { }
    template<BuiltinUnsignedIntegral M> Int64(M m) : _n(static_cast<int64_t>(m)) { assert(_n>=0); assert((uint64_t)_n==m); }
    template<BuiltinSignedIntegral N> Int64(N n) : _n(n) { assert(_n==n); }
    int64_t get_si() const { return _n; }
};

uint32_t fac(uint8_t n);
uint16_t fac(uint16_t n);
uint32_t fac(uint32_t n);
uint64_t fac(uint64_t n);
uint32_t bin(uint8_t n, uint8_t k);
uint16_t bin(uint16_t n, uint16_t k);
uint32_t bin(uint32_t n, uint32_t k);
uint64_t bin(uint64_t n, uint64_t k);

} // namespace Ariadne

#endif
