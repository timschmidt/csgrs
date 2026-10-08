/***************************************************************************
 *            range.hpp
 *
 *  Copyright  2023  Pieter Collins
 *
 ****************************************************************************/

/*
 * This file is part of Helper, under the MIT license.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

/*! \file range.hpp
 *  \brief Index ranges.
 */

#ifndef HELPER_RANGE_HPP
#define HELPER_RANGE_HPP

#include <cstddef>

namespace Helper {

//! \ingroup LinearAlgebraModule
//! \brief A range of integer values from a \em start value up to, but not including, a \em stop value.
class Range {
    size_t _start; size_t _stop;
  public:
    Range(size_t start, size_t stop) : _start(start), _stop(stop) { } //!< .
    size_t operator[](size_t i) const { return _start+i; } //!< .
    size_t size() const { return this->_stop-this->_start; } //!< .
    size_t start() const { return this->_start; } //!< .
    size_t stop() const { return this->_stop; } //!< .
    size_t stride() const { return 1u; } //!< Always equal to \a 1.
};
inline Range range(size_t stop) { return Range(0u,stop); } //!< \relates Range
inline Range range(size_t start, size_t stop) { return Range(start,stop); } //!< \relates Range

struct RangeIterator {
    explicit inline RangeIterator(size_t i) : _i(i) { }
    inline RangeIterator& operator++() { ++this->_i; return *this; }
    inline size_t operator*() const { return this->_i; }
    friend inline bool operator!=(RangeIterator iter1, RangeIterator iter2) { return iter1._i != iter2._i; }
  private:
    size_t _i;
};
inline RangeIterator begin(Range rng) { return RangeIterator(rng.start()); }
inline RangeIterator end(Range rng) { return RangeIterator(rng.stop()); }

} // namespace Helper

#endif // HELPER_RANGE_HPP
