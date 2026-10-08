/***************************************************************************
 *            utility/variant.hpp
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

/*! \file utility/variant.hpp
 *  \brief
 */



#ifndef ARIADNE_VARIANT_HPP
#define ARIADNE_VARIANT_HPP

#include <variant>

#include "metaprogramming.hpp"

namespace Ariadne {

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

} // namespace Ariadne

#endif
