/***************************************************************************
 *            utility/prototype.hpp
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

/*! \file utility/prototype.hpp
 *  \brief
 */

#ifndef ARIADNE_PROTOTYPE_HPP
#define ARIADNE_PROTOTYPE_HPP

#include <utility>

namespace Ariadne {

template<class T> class Prototype {
    T _prototype;
  public:
    Prototype(T t) : _prototype(t) { }
    template<class X> X convert(X const& x) { return _prototype.convert(x); }
};
template<class T> inline Prototype<T> make_prototype(T const& t) {
    return Prototype<T>(t); }

} // namespace Ariadne

#endif
