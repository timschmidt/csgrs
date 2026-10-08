/***************************************************************************
 *            io/null_graphics.hpp
 *
 *  Copyright  2020-21  Mirko Albanese, Luca Geretti
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

/*! \file io/null_graphics.hpp
 *  \brief Null output in the case of missing graphics support.
 */

#ifndef ARIADNE_NULL_GRAPHICS_HPP
#define ARIADNE_NULL_GRAPHICS_HPP

#include "io/figure.hpp"
#include "io/graphics_backend_interface.hpp"
#include "io/drawer_interface.hpp"

namespace Ariadne {

class NullCanvas : public CanvasInterface
{
public:
    virtual Void initialise(StringType x, StringType y, StringType z, double lx, double ux, double ly, double uy, double lz, double uz) { };
    virtual Void initialise(StringType x, StringType y, double lx, double ux, double ly, double uy) { }
    virtual Void finalise() { }

    virtual Void write(const char* filename) const { }

    virtual Void move_to(double x, double y) { }
    virtual Void line_to(double x, double y) { }
    virtual Void circle(double x, double y, double r) { }
    virtual Void dot(double x, double y) { }
    virtual Void stroke() { }
    virtual Void fill() { }

    virtual Void fill_boundary(List<Point2d> const& boundary) { }

    virtual Void set_line_width(double lw) { }
    virtual Void set_line_colour(double r, double g, double b) { }
    virtual Void set_fill_opacity(double fo) { }
    virtual Void set_fill_colour(double r, double g, double b) { }

    virtual Void set_heat_map(Bool b) { }
    virtual Void set_colour_palette() { }
    virtual Void fill_3d() { }

    virtual Vector2d scaling() const { return Vector2d(0,0); }
    virtual Box2d bounds() const { return Box2d(0,0,0,0); }
};

class NullGraphicsBackend : public GraphicsBackendInterface {
  public:
    SharedPointer<CanvasInterface> make_canvas(const char* cfilename, Nat drawing_width, Nat drawing_height, Bool is_animated) const override { return std::make_shared<NullCanvas>(); }
};

class NullDrawer : public DrawerInterface
{
  public:
    Void draw(CanvasInterface& cnvs, const Projection2d& proj, const ValidatedConstrainedImageSet& set) const { }
    OutputStream& _write(OutputStream& os) const { return os << "NullDrawer()"; }
};

} // namespace Ariadne

#endif // ARIADNE_NULL_GRAPHICS_HPP


