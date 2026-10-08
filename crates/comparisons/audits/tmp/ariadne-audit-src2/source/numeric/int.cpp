/***************************************************************************
 *            numeric/int.cpp
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

#include "int.hpp"

#include "utility/stdlib.hpp"
#include "utility/macros.hpp"

#include <cassert>
#include <limits>

namespace Ariadne {

uint32_t
fac(uint8_t n)
{
    ARIADNE_ASSERT(n<13); // Maximum factorial in 32 bits
    uint32_t  r=1;
    for(uint8_t i=1; i<=n; ++i) {
        r*=i;
    }
    return r;
}

uint32_t
bin(uint8_t n, uint8_t k)
{
    ARIADNE_ASSERT(n<32);  // Maximum computable bin(n,n/2) using 32 bits
                           // Note that this is shorter than the maximum representable factorial
    if(k>n+1) { ARIADNE_FAIL_MSG("bin("<<n<<","<<k<<")"); }
    if(k==n+1) { return 0; }
    ARIADNE_ASSERT(k<=n);
    uint32_t r=1;
    for(uint8_t i=1; i<=k; ++i) {
        r*=(n+1u-i);
        r/=i;
    }
    return r;
}

uint16_t
fac(uint16_t n)
{
    ARIADNE_ASSERT(n<9); // Maximum factorial in 16 bits
    uint16_t  r=1;
    for(uint16_t i=1; i<=n; ++i) {
        r*=i;
    }
    return r;
}


uint16_t
bin(uint16_t n, uint16_t k)
{
    ARIADNE_ASSERT(n<16);  // Maximum computable bin(n,n/2) using 16 bits
                           // Note that this is shorter than the maximum representable factorial
    if(k>n+1) { ARIADNE_FAIL_MSG("bin("<<n<<","<<k<<")"); }
    if(k==n+1) { return 0; }
    ARIADNE_ASSERT(k<=n);
    uint16_t r=1;
    for(uint16_t i=1; i<=k; ++i) {
        r*=(n+1-i);
        r/=i;
    }
    return r;
}

uint32_t
fac(uint32_t n)
{
    ARIADNE_ASSERT(n<13); // Maximum factorial in 32 bits
    uint32_t  r=1;
    for(uint32_t i=1; i<=n; ++i) {
        r*=i;
    }
    return r;
}


uint32_t
bin(uint32_t n, uint32_t k)
{
    ARIADNE_ASSERT(n<31);  // Maximum computable bin(n,n/2) using 32 bits
                           // Note that this is shorter than the maximum representable factorial
    if(k>n+1) { ARIADNE_FAIL_MSG("bin("<<n<<","<<k<<")"); }
    if(k==n+1) { return 0; }
    ARIADNE_ASSERT(k<=n);
    uint32_t r=1;
    for(uint32_t i=1; i<=k; ++i) {
        r*=(n+1-i);
        r/=i;
    }
    return r;
}



uint64_t
fac(uint64_t n)
{
    ARIADNE_ASSERT(n<21); // Maximum factorial in 64 bits
    uint64_t  r=1;
    for(uint64_t i=1; i<=n; ++i) {
        r*=i;
    }
    return r;
}


uint64_t
bin(uint64_t n, uint64_t k)
{
    ARIADNE_ASSERT(n<63);  // Maximum computable bin(n,n/2) using 64 bits
                           // Note that this is shorter than the maximum representable factorial
    if(k>n+1) { ARIADNE_FAIL_MSG("bin("<<n<<","<<k<<")"); }
    if(k==n+1) { return 0; }
    ARIADNE_ASSERT(k<=n);
    uint64_t r=1;
    for(uint64_t i=1; i<=k; ++i) {
        r*=(n+1-i);
        r/=i;
    }
    return r;
}

} // namespace Ariadne
