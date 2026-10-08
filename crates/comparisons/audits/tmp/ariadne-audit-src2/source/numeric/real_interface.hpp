/***************************************************************************
 *            numeric/real_interface.hpp
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

#ifndef ARIADNE_REAL_INTERFACE_HPP
#define ARIADNE_REAL_INTERFACE_HPP

#include <iosfwd>

namespace Ariadne {

class Real;
class ValidatedReal;

class Dyadic;

class DoublePrecision;
class MultiplePrecision;
template<class... PRS> class Float;
using FloatDP=Float<DP>;
using FloatMP=Float<MP>;

template<class F> class Bounds;
using DyadicBounds = Bounds<Dyadic>;
using FloatDPBounds = Bounds<FloatDP>;
using FloatMPBounds = Bounds<FloatMP>;

using OutputStream = std::ostream;

class Real;

class RealInterface {
  public:
    virtual ~RealInterface() = default;
    virtual ValidatedReal _compute(Accuracy) const = 0;
    virtual ValidatedReal _compute(Effort) const = 0;
    virtual DyadicBounds _compute_get(Effort) const = 0;
    virtual FloatDPBounds _compute_get(Effort, DoublePrecision) const = 0;
    virtual FloatMPBounds _compute_get(Effort, MultiplePrecision) const = 0;
  public:
    virtual OutputStream& _write(OutputStream& os) const = 0;
};

class ValidatedRealInterface {
  public:
    virtual ~ValidatedRealInterface() = default;
    virtual DyadicBounds _get() const = 0;
    virtual FloatDPBounds _get(DoublePrecision) const = 0;
    virtual FloatMPBounds _get(MultiplePrecision) const = 0;
  public:
    virtual OutputStream& _write(OutputStream& os) const = 0;
};

} // namespace Ariadne

#endif /* ARIADNE_REAL_INTERFACE_HPP */
