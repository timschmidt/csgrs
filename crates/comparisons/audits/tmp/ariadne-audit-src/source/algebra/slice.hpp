/***************************************************************************
 *            algebra/slice.hpp
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

/*! \file algebra/slice.hpp
 *  \brief Slices.
 */

#ifndef ARIADNE_SLICE_HPP
#define ARIADNE_SLICE_HPP

#include "utility/typedefs.hpp"

namespace Ariadne {

class Slice {
    SizeType _size; SizeType _start; SizeType _stride;
  public:
    Slice(SizeType size, SizeType start, SizeType stride) : _size(size), _start(start), _stride(stride) { }
    SizeType operator[](SizeType i) { return _start+i*_stride; }
    SizeType size() const { return this->_size; }
    SizeType start() const { return this->_start; }
    SizeType stride() const { return this->_stride; }
    SizeType stop() const { return this->_start+this->_size*this->_stride; }
};
inline Slice slice(SizeType size_, SizeType start, SizeType stride) { return Slice(size_,start,stride); }

} // namespace Ariadne

#endif
