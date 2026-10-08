/***************************************************************************
 *            io/drawer_interface.hpp
 *
 *  Copyright  2011-20  Pieter Collins
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

/*! \file io/drawer_interface.hpp
 *  \brief Interface for drawing affine and nonlinear sets.
 */

#ifndef ARIADNE_DRAWER_INTERFACE_HPP
#define ARIADNE_DRAWER_INTERFACE_HPP

#include <iosfwd>
#include "utility/writable.hpp"

namespace Ariadne {

typedef Void Void;
class CanvasInterface;
struct Projection2d;
typedef Projection2d Projection2d;
class ValidatedConstrainedImageSet;

//! \brief A class for computing outer approximations to sets defined by functions.
class DrawerInterface : public WritableInterface
{
  public:
    virtual Void draw(CanvasInterface& cnvs, const Projection2d& proj, const ValidatedConstrainedImageSet& set) const = 0;
};

//! \brief A class for computing outer approximations to sets defined by functions.
class Drawer
    : public Handle<const DrawerInterface>
{
  public:
    using Handle<const DrawerInterface>::Handle;
    inline Void draw(CanvasInterface& cnvs, const Projection2d& proj, const ValidatedConstrainedImageSet& set) const {
        return this->_ptr->draw(cnvs,proj,set); }
    friend inline OutputStream& operator<<(OutputStream& os, Drawer const& drw) {
        return drw._ptr->_write(os); }
};


} //namespace Ariadne

#endif /* ARIADNE_DRAWER_INTERFACE_HPP */
