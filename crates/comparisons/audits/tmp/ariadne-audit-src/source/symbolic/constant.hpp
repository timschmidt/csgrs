/***************************************************************************
 *            symbolic/constant.hpp
 *
 *  Copyright  2008-20  Pieter Collins
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

/*! \file symbolic/constant.hpp
 *  \brief Named constants
 */

#ifndef ARIADNE_CONSTANT_HPP
#define ARIADNE_CONSTANT_HPP

#include "utility/string.hpp"
#include "symbolic/identifier.hpp"

namespace Ariadne {

//! \ingroup SymbolicModule
//! \brief A named constant of type \a T.
//! \details
//! \par \b Example
//! \snippet tutorials/symbolic_usage.cpp Constant_usage
//! \see Variable, Expression
template<class T> class Constant
    : public T
{
  public:
    //! The constant with value \a value, displayed as the value itself.
    explicit Constant(const T& value) : T(value), _name("") { }
    //! The constant with name \a name and value \a value.
    explicit Constant(const Identifier& name, const T& value) : T(value), _name(name) { }
    //! The name of the constant.
    const Identifier& name() const { return _name; }
    //! The value of the constant.
    const T& value() const { return *this; }
    const T& val() const { return *this; }
    //! Write to an output stream.
    friend OutputStream& operator<<(OutputStream& os, Constant<T> const& c) {
        if (c.name().empty()) { return os << c.value(); } else { return os << c._name << "(=" << c.value() << ")"; } }
  private:
    Identifier _name;
};

template<> class Constant<String>
    : public String
{
  public:
    explicit Constant(const String& value) : String(value) { }
    const Identifier& name() const { return static_cast<const Identifier&>(static_cast<const std::string&>(*this)); }
    const String& value() const { return *this; }
    const String& val() const { return *this; }
};

} // namespace Ariadne

#endif /* ARIADNE_CONSTANT_HPP */
