/***************************************************************************
 *            utility/builtin.hpp
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

/*! \file utility/builtin.hpp
 *  \brief
 */



#ifndef ARIADNE_BUILTIN_HPP
#define ARIADNE_BUILTIN_HPP

#include "metaprogramming.hpp"

namespace Ariadne {

class builtin_uint {
    long unsigned int _m;
  public:
    template<BuiltinUnsigned M> builtin_uint(M m) : _m(m) { }
    explicit operator long unsigned int() const { return _m; }
    long unsigned int get_ui() const { return _m; }
};

class builtin_int {
    long int _n;
  public:
    template<BuiltinIntegral N> builtin_int(N n) : _n(n) { }
    explicit operator long int () const { return _n; }
    long int get_si() const { return _n; }
};

class builtin_double {
    double _x;
  public:
    template<BuiltinFloatingPoint X> builtin_double(X x) : _x(x) { }
    explicit operator double () const { return _x; }
    double get_d() const { return _x; }
};

class exact_double {
    double _x;
  private:
    exact_double(double x) : _x(x) { }
  public:
    friend exact_double cast_exact(double);
    exact_double() : _x(0.0) { }
    explicit operator double () const { return _x; }
    double get_d() const { return _x; }
};

exact_double cast_exact(double x) { return exact_double(x); }

} // namespace Ariadne

#endif

