/***************************************************************************
 *            utility/clonable.hpp
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

/*! \file utility/clonable.hpp
 *  \brief 
 */



#ifndef ARIADNE_CLONABLE_HPP
#define ARIADNE_CLONABLE_HPP

namespace Ariadne {

/************ ClonableInterface **********************************************/

class ClonableInterface {
  public:
    virtual ~ClonableInterface() = default;
  protected:
    virtual ClonableInterface* _copy() const = 0;
    virtual ClonableInterface* _move() = 0;
};

} // namespace Ariadne

#endif
