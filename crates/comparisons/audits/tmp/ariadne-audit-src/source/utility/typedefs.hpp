/***************************************************************************
 *            utility/typedefs.hpp
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

/*! \file utility/typedefs.hpp
 *  \brief
 */



#ifndef ARIADNE_TYPEDEFS_HPP
#define ARIADNE_TYPEDEFS_HPP

#include <cstdint>
#include <iosfwd>
#include <type_traits>
#include <memory>

namespace Ariadne {


class Real;

template<class T> class Variable;

using uchar = unsigned char;
using uint = unsigned int;

using RealVariable = Variable<Real>;

//! \brief Internal name for standard output stream.
using OutputStream = std::ostream;
//! \brief Internal name for standard input stream.
using InputStream = std::istream;
//! Internal name for standard string stream.
using StringStream = std::stringstream;

//! Internal name for void type.
using Void = void;
//! Internal name for builtin boolean type.
using Bool = bool;
//! Internal name for builtin char type.
using Char = char;
//! Internal name for builtin byte type (8 bits).
using Byte = std::int8_t;
//! Internal name for builtin unsigned integers.
using Nat = uint;
//! Internal name for builtin integers.
using Int = int;
//! Internal name for builtin double-precision floats.
using Dbl = double;
//! Internal name for builtin double-precision floats.
using Double = double;


//! Internal name for builtin double-precision floats.
using StringType = std::string;
//! A thin wrapper around a std::string.
class String;


//! Internal name for standard size type, used for sizes of containers.
using SizeType = std::size_t;
//! Internal name for standard difference type of container indices and pointers.
using PointerDifferenceType = std::ptrdiff_t;

//! Internal name for unsigned integer used as a step counter
using CounterType = std::uint32_t;

class Nat32;
class Int32;
class Nat64;
class Int64;

using std::declval;

//! Internal alias for standard unique pointer.
template<class T> using UniquePointer = std::unique_ptr<T>;
//! Internal alias for standard shared pointer.
template<class T> using SharedPointer = std::shared_ptr<T>;
//! Internal alias for standard initializer list.
template<class T> using InitializerList = std::initializer_list<T>;
//! Internal alias for standard pair.
template<class T1, class T2> using Pair = std::pair<T1,T2>;
//! Internal alias for standard tuple.
template<class... TS> using Tuple = std::tuple<TS...>;

//! A class for encoding comparisons for use in ordered containers.
template<class T> class ContainerComparison;

//! A class wrapper for C-style arrays.
template<class T> class Array;
//! A thin wrapper around a std::vector.
template<class T> class List;
//! A thin wrapper around a std::set.
template<class T, class CMP=ContainerComparison<T>> class Set;
//! A thin wrapper around a std::map.
template<class K, class T, class CMP=ContainerComparison<K>> class Map;

//! A tag for the size of a scalar object.
struct SizeOne { operator SizeType() const { return 1u; } };
//! A tag for an index into a scalar object.
struct IndexZero { operator SizeType() const { return 0u; } };

//! The type used for the degree of an index.
typedef std::uint16_t DegreeType;
//! The type used for the dimension of a geometric object.
typedef SizeType DimensionType;

//! The type used for the subspace grid
typedef Map<RealVariable , Nat> subspace;

#if (defined __arm || defined __aarch64__)
typedef short ComparableEnumerationType;
#else
typedef char ComparableEnumerationType;
#endif


} // namespace Ariadne

#endif
