/***************************************************************************
 *            io/cairo.hpp
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

#ifndef ARIADNE_CAIRO_HPP
#define ARIADNE_CAIRO_HPP

#include "config.hpp"

#ifdef HAVE_CAIRO_H

#include <cairo/cairo.h>
#include "io/figure.hpp"

namespace Ariadne {

template<SizeType N, class X> class Tensor;

struct ImageSize2d {
    Nat nx,ny;
    ImageSize2d(Nat _nx,Nat _ny) : nx(_nx), ny(_ny) { }
    ImageSize2d(Int _nx,Int _ny) {
        ARIADNE_ASSERT(_nx > 0 && _ny > 0);
        nx = static_cast<Nat>(_nx);
        ny= static_cast<Nat>(_ny);
    }
};

class CairoCanvas
    : public CanvasBase
{
    friend class Figure;
  private:
    cairo_t *cr;
    double lw; // The line width in pixels
    double dr; // The dot radius in pixels
    Colour lc,fc; // The line and fill colours
  public:
    ~CairoCanvas();
    CairoCanvas(const ImageSize2d& size);
    Void initialise(StringType x, StringType y, StringType z, double xl, double xu, double yl, double yu, double lz, double uz);
    Void initialise(StringType x, StringType y, double xl, double xu, double yl, double yu);
    Void write(const char* filename) const;
    Void finalise();
    Void move_to(double x, double y);
    Void line_to(double x, double y);
    Void circle(double x, double y, double r);
    Void dot(double x, double y);
    Void stroke();
    Void fill();

    Void set_dot_radius(double dr);
    Void set_line_width(double lw);
    Void set_line_colour(double r, double g, double b);
    Void set_fill_opacity(double o);
    Void set_fill_colour(double r, double g, double b);

    Void set_colour_palette();
    Void fill_3d();
    Void set_heat_map(Bool b);
  public:
    ImageSize2d size_in_pixels() const;
};

class CairoGraphicsBackend : public GraphicsBackendInterface {
  public:
    SharedPointer<CanvasInterface> make_canvas(const char* cfilename, Nat drawing_width, Nat drawing_height, Bool is_animated) const;
};

} // namespace Ariadne

#endif // HAVE_CAIRO_H

#endif // ARIADNE_CAIRO_HPP


