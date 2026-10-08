/***************************************************************************
 *            tuple.hpp
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

/*! \file tuple.hpp
 *  \brief Pair and Tuple types, and types to be used as lvalues in assignments.
 */

#ifndef HELPER_TUPLE_HPP
#define HELPER_TUPLE_HPP

#include <utility>
#include <tuple>

namespace Helper {

template<class T1, class T2> using Pair = std::pair<T1,T2>;
using std::make_pair;
template<class T1, class T2> constexpr Pair<T1&,T2&> make_lpair(T1& t1, T2& t2) { return Pair<T1&,T2&>(t1,t2); }

template<class... TS> using Tuple = std::tuple<TS...>;
using std::make_tuple;
template<class... TS> constexpr Tuple<TS&...> make_ltuple(TS&... ts) { return Tuple<TS&...>(std::tie(ts...)); }


template<class T> inline decltype(auto) get_first(T&& t) { return std::get<0>(std::forward<T>(t)); }
template<class T> inline decltype(auto) get_second(T&& t) { return std::get<1>(std::forward<T>(t)); }
template<class T> inline decltype(auto) get_third(T&& t) { return std::get<2>(std::forward<T>(t)); }
template<class T> inline decltype(auto) get_fourth(T&& t) { return std::get<3>(std::forward<T>(t)); }
template<class T> inline decltype(auto) get_fifth(T&& t) { return std::get<4>(std::forward<T>(t)); }


} // namespace Helper

#endif /* HELPER_TUPLE_HPP */
