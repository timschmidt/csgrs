/***************************************************************************
 *            variant.hpp
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

/*! \file variant.hpp
 *  \brief
 */



#ifndef HELPER_VARIANT_HPP
#define HELPER_VARIANT_HPP

#include <variant>

#include "metaprogramming.hpp"

namespace Helper {

//! Internal alias for standard variant.
template<class... TS> using Variant = std::variant<TS...>;

template<class C, class... TS> class CodedVariant {
    C _code;
  public:
    explicit CodedVariant(C code) : _code(code) { }
    template<OneOf<TS...> T> CodedVariant(T const&) : _code(T::code()) { }
    template<class V> inline decltype(auto) accept(V const& v) const;
    C code() const { return _code; }
};
template<class T, class C, class... TS> bool holds_alternative(CodedVariant<C,TS...> const& var) { return var.code()==T::code(); }

} // namespace Helper

#endif
