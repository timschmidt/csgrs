/***************************************************************************
 *            io/gnuplot.hpp
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

#ifndef ARIADNE_GNUPLOT_HPP
#define ARIADNE_GNUPLOT_HPP

#include "config.hpp"

#ifdef HAVE_GNUPLOT_H

#include "io/figure.hpp"
#include "io/gnuplot-iostream.hpp"

namespace Ariadne {

struct  _Range
{       
    double Xmin;
    double Xmax;
    double Ymin;
    double Ymax;
    double Zmin;
    double Zmax; 
};

struct  _Labels
{
    String xLabel = "";
    String yLabel = "";
    String zLabel = "";
};

class GnuplotCanvas : public CanvasBase
{
    friend class Figure;
  private:
    Gnuplot *gnuplot;
    List<Point2d> geom;
    Colour lc, fc;
    double lw;
    _Range rng;
    Nat dim;
    Point2d Cpoint;
    double dr;
    bool isdot;
    Nat sizeX;
    Nat sizeY;
    bool isMultiplot;
    bool isColourPalette;
    bool isanimate;
    _Labels labels;

    Mutex _mux;

  public:
    ~GnuplotCanvas();
    // Constructors - Create the canvas
    //Create canvas with dimensions
    GnuplotCanvas(String filename, Nat X = 800, Nat Y = 800, Bool is_anim = false);

    //CanvasInterface
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
    Vector2d scaling() const;
    Box2d bounds() const;

    Void set_colour_palette();
    Void fill_3d();
    Void set_heat_map(Bool b);

    //Set Multiplot - Multiple plot on same screen
    void set_multiplot(bool s);
    // Set Labels and Title
    void set_labels(String xLabel, String yLabel, String zLabel = "");
    // Set X, Y Range
    void set_range_2d(double minX, double maxX, double minY, double maxY);

    void set_range_3d(double minX, double maxX, double minY,  double maxY, double minZ, double maxZ);
};

class GnuplotGraphicsBackend : public GraphicsBackendInterface {
  public:
    SharedPointer<CanvasInterface> make_canvas(const char* cfilename, Nat drawing_width, Nat drawing_height, Bool is_animated) const;
};

} // namespace Ariadne

#endif // HAVE_GNUPLOT_H

#endif // ARIADNE_GNUPLOT_HPP