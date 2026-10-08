/***************************************************************************
 *            io/drawer.hpp
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

/*! \file io/drawer.hpp
 *  \brief Class for drawing affine and nonlinear sets.
 */

#ifndef ARIADNE_DRAWER_HPP
#define ARIADNE_DRAWER_HPP

#include <iosfwd>
#include "utility/declarations.hpp"
#include "io/drawer_interface.hpp"

namespace Ariadne {

//! \brief Draw a box over-approximation to the set.
class BoxDrawer : public DrawerInterface
{
  public:
    Void draw(CanvasInterface& cnvs, const Projection2d& proj, const ValidatedConstrainedImageSet& set) const;
    OutputStream& _write(OutputStream& os) const;
};

//! \brief Draw an affine over-approximation to the set.
class AffineDrawer : public DrawerInterface
{
    Nat _splittings;
  public:
    AffineDrawer(Nat splittings) : _splittings(splittings) { }
    Void draw(CanvasInterface& cnvs, const Projection2d& proj, const ValidatedConstrainedImageSet& set) const;
    OutputStream& _write(OutputStream& os) const;
};

//! \brief Pave the set and draw the computed cells.
class GridDrawer : public DrawerInterface
{
    Nat _fineness;
  public:
    GridDrawer(Nat fineness) : _fineness(fineness) { }
    Void draw(CanvasInterface& cnvs, const Projection2d& proj, const ValidatedConstrainedImageSet& set) const;
    OutputStream& _write(OutputStream& os) const;
};


} //namespace Ariadne

#endif /* ARIADNE_DRAWER_HPP */
