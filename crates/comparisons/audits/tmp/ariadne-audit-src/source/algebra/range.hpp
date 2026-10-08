/***************************************************************************
 *            algebra/range.hpp
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

/*! \file algebra/range.hpp
 *  \brief Index ranges.
 */

#ifndef ARIADNE_RANGE_HPP
#define ARIADNE_RANGE_HPP

#include "utility/typedefs.hpp"

namespace Ariadne {

//! \ingroup LinearAlgebraSubModule
//! \brief A range of integer values from a \em start value up to, but not including, a \em stop value.
class Range {
    SizeType _start; SizeType _stop;
  public:
    Range(SizeType start, SizeType stop) : _start(start), _stop(stop) { } //!< .
    SizeType operator[](SizeType i) const { return _start+i; } //!< .
    SizeType size() const { return this->_stop-this->_start; } //!< .
    SizeType start() const { return this->_start; } //!< .
    SizeType stop() const { return this->_stop; } //!< .
    SizeType stride() const { return 1u; } //!< Always equal to \a 1.
};
inline Range range(SizeType stop) { return Range(0u,stop); } //!< \relates Range
inline Range range(SizeType start, SizeType stop) { return Range(start,stop); } //!< \relates Range

struct RangeIterator {
    explicit inline RangeIterator(SizeType i) : _i(i) { }
    inline RangeIterator& operator++() { ++this->_i; return *this; }
    inline SizeType operator*() const { return this->_i; }
    friend inline bool operator!=(RangeIterator iter1, RangeIterator iter2) { return iter1._i != iter2._i; }
  private:
    SizeType _i;
};
inline RangeIterator begin(Range rng) { return RangeIterator(rng.start()); }
inline RangeIterator end(Range rng) { return RangeIterator(rng.stop()); }

} // namespace Ariadne

#endif
