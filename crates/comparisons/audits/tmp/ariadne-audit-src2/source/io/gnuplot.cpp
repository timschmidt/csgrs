/***************************************************************************
 *            io/gnuplot.cpp
 *
 *  Copyright  2008-21  Mirko Albanese, Luca Geretti
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

#include "utility/standard.hpp"
#include "config.hpp"

#include "utility/macros.hpp"
#include "helper/stlio.hpp"
#include "numeric/numeric.hpp"
#include "function/function.hpp"
#include "geometry/point.hpp"
#include "geometry/box.hpp"
#include "symbolic/variable.hpp"
#include "symbolic/space.hpp"
#include "symbolic/expression_set.hpp"
#include "io/geometry2d.hpp"
#include "io/figure.hpp"
#include "io/gnuplot.hpp"
#include "conclog/logging.hpp"

using namespace ConcLog;

namespace Ariadne {

#ifdef HAVE_GNUPLOT_H

SharedPointer<CanvasInterface> GnuplotGraphicsBackend::make_canvas(const char* cfilename, Nat drawing_width, Nat drawing_height, Bool is_animated) const {
    return std::make_shared<GnuplotCanvas>(cfilename, drawing_width, drawing_height, is_animated);
}

GnuplotCanvas::GnuplotCanvas(String cfilename, Nat X, Nat Y, Bool is_anim): lc(0.0, 0.0, 0.0, 0.0),
                                            fc(1.0, 1.0, 1.0, 1.0),
                                            lw(0.0),
                                            dr(2.0),
                                            isdot(false),
                                            sizeX(X),
                                            sizeY(Y),
                                            isMultiplot(false),
                                            isColourPalette(false),
                                            isanimate(is_anim)

{
    //Start the pipe communication storing all the gnuplot command into the .gnu file and in the
    //meantime these command are executed by gnuplot
    gnuplot = new Gnuplot("tee "+cfilename+".gnu | gnuplot > /dev/null 2>&1");

    // Disable legend
    *gnuplot << "unset key\n";
    //*gnuplot << "set key noautotitle\n";
    if(not(isanimate)){
        *gnuplot << "set terminal pngcairo nocrop ";
    } else {
        *gnuplot << "set terminal gif animate nocrop ";
    }

    *gnuplot << "size " << to_string(this->sizeX) << ", " << to_string(this->sizeY) << "\n";

    if(not(isanimate)) {
        *gnuplot << "set output \"" << cfilename << ".png\"\n";
        this->set_multiplot(true);
    } else {
        *gnuplot << "set output \"" << cfilename << ".gif\"\n";
        *gnuplot << "unset multiplot\n";
        this->set_multiplot(false);
    }
    
    //Se the initial size of the drawing point
    this->geom.resize(1);
    this->dim = 0;
}

GnuplotCanvas::~GnuplotCanvas() {
    delete gnuplot;
}

void GnuplotCanvas::initialise(StringType x, StringType y, StringType z, double xl, double xu, double yl, double yu, double zl, double zu)
{
    this->set_labels(x, y, z);
    this->set_range_3d(xl, xu, yl, yu, zl, zu);
}

void GnuplotCanvas::initialise(StringType x, StringType y, double xl, double xu, double yl, double yu)
{
    this->set_labels(x, y);
    this->set_range_2d(xl, xu, yl, yu);
}

void GnuplotCanvas::finalise() { }
void GnuplotCanvas::circle(double x, double y, double r) { }
void GnuplotCanvas::stroke() {
    char hex_string[20];

    if (this->isdot)
    {
        //Plot a dot
        *gnuplot << "plot \"<echo '" << to_string(this->Cpoint.x) << " " << to_string(this->Cpoint.y) << "'\" w points pt 7 ps " << to_string(this->dr) << "\n";
        this->isdot = false;
    }
    else{
        //Plot a line path with a colored lines
        *gnuplot << "plot '-' ";
        *gnuplot << "w lines ";
        *gnuplot << "lw " << to_string(this->lw) << " ";
        *gnuplot << "lc rgb \"#";
        if (this->lc.red < 9) { *gnuplot << "0" << this->lc.red;}
        else if (this->lc.red > 255){ *gnuplot << "FF";}
        else{   
            snprintf(hex_string, 20, "%X", std::make_unsigned<int>::type(this->lc.red));
            *gnuplot << hex_string;
            }
        if (this->lc.green < 9) { *gnuplot << "0" << this->lc.green;}
        else if (this->lc.green > 255){ *gnuplot << "FF";}
        else{
            snprintf(hex_string, 20, "%X", std::make_unsigned<int>::type(this->lc.green));
            *gnuplot << hex_string;
            }
        if (this->lc.blue < 9) { *gnuplot << "0" << this->lc.blue;}
        else if (this->lc.blue > 255){ *gnuplot << "FF";}
        else{snprintf(hex_string, 20, "%X", std::make_unsigned<int>::type(this->lc.blue));
            *gnuplot <<hex_string;
            }
        *gnuplot << "\"\n";

        //Fill the data into gnuplot
        for (SizeType i = 0; i < this->dim; i++)
        {
            *gnuplot << to_string(this->geom[i].x) << " " << to_string(this->geom[i].y) << "\n";
        } 
        *gnuplot << "e\n";
    }

    //Reset the dimension of the current object
    this->dim = 0;  
 }

void GnuplotCanvas::move_to(double x, double y)
{
    this->Cpoint.x = this->geom[0].x = x;
    this->Cpoint.y = this->geom[0].y = y;
    this->dim = 1;
}

void GnuplotCanvas::line_to(double x, double y)
{
    this->dim++;

    if (this->dim >= this->geom.size())
    {
        this->geom.resize(this->geom.size()+this->geom.size());
        this->geom.resize(this->geom.size()+this->geom.size());
    }
  
    this->Cpoint.x = this->geom[this->dim-1].x = x;
    this->Cpoint.y = this->geom[this->dim-1].y = y;

}

void GnuplotCanvas::dot(double x, double y)
{
    this->isdot = true;
    this->Cpoint.x = x;
    this->Cpoint.y = y;
}

void GnuplotCanvas::fill()
{   
    char hex_string[20];
    if (this->isdot)
    {
        //Plot a dot
        *gnuplot << "plot \"<echo '" << to_string(this->Cpoint.x) << " " << to_string(this->Cpoint.y) << "'\" w points pt 7 ps " << to_string(this->dr) << "\n";
        this->isdot = false;
    }
    else
    {
        //Plot a line or a filled closed line
        *gnuplot << "plot '-' ";
        if(!isColourPalette){
            *gnuplot << "w filledcurves ";
            *gnuplot << "fc rgb \"#";
            if (this->fc.red < 9) { *gnuplot << "0" << this->fc.red;}
            else if (this->fc.red > 255){ *gnuplot << "FF";}
            else{   
                snprintf(hex_string, 20, "%X", std::make_unsigned<int>::type(this->fc.red));
                *gnuplot << hex_string;
                }
            if (this->fc.green < 9) { *gnuplot << "0" << this->fc.green;}
            else if (this->fc.green > 255){ *gnuplot << "FF";}
            else{
                snprintf(hex_string, 20, "%X", std::make_unsigned<int>::type(this->fc.green));
                *gnuplot << hex_string;
                }
            if (this->fc.blue < 9) { *gnuplot << "0" << this->fc.blue;}
            else if (this->fc.blue > 255){ *gnuplot << "FF";}
            else{snprintf(hex_string, 20, "%X", std::make_unsigned<int>::type(this->fc.blue));
                *gnuplot <<hex_string;
                }
            *gnuplot << "\" ";
            *gnuplot << "fs solid " << this->fc.opacity << " border ";

            if(this->lw!=0){
                *gnuplot << "lc rgb \"#";
                if (this->lc.red < 9) { *gnuplot << "0" << this->lc.red;}
                else if (this->lc.red > 255){ *gnuplot << "FF";}
                else{   
                    snprintf(hex_string, 20, "%X", std::make_unsigned<int>::type(this->lc.red));
                    *gnuplot << hex_string;
                    }
                if (this->lc.green < 9) { *gnuplot << "0" << this->lc.green;}
                else if (this->lc.green > 255){ *gnuplot << "FF";}
                else{
                    snprintf(hex_string, 20, "%X", std::make_unsigned<int>::type(this->lc.green));
                    *gnuplot << hex_string;
                    }
                if (this->lc.blue < 9) { *gnuplot << "0" << this->lc.blue;}
                else if (this->lc.blue > 255){ *gnuplot << "FF";}
                else{snprintf(hex_string, 20, "%X", std::make_unsigned<int>::type(this->lc.blue));
                    *gnuplot <<hex_string;
                    }
                *gnuplot << "\"\n";
            }else{ *gnuplot << "\n"; }

            //Fill the data into gnuplot
            for (SizeType i = 0; i < this->dim; i++)
            {
                //X and Y coordinates
                *gnuplot << to_string(this->geom[i].x) << " " << to_string(this->geom[i].y) << "\n";
            } 
        }else{
            *gnuplot << "u ::1 w lines lw " << to_string(this->lw) << " linecolor palette\n";

            //Fill the data into gnuplot
            for (SizeType i = 0; i < this->dim; i++)
            {
                //Y value because it reppresent the colour palette value
                *gnuplot <<  to_string(this->geom[i].y) << "\n";
            } 
        }
    }
    *gnuplot << "e\n";

    //Reset the dimension of the current object
    this->dim = 0;       
}

Void GnuplotCanvas::fill_3d(){
    if (this->isdot)
    {
        *gnuplot << "splot \"<echo '" << to_string(this->Cpoint.x) << " " << to_string(this->Cpoint.y) << "'\" w points pt 7 ps " << to_string(this->dr) << "\n";
        this->isdot = false;
    }
    else{
            //Plot a 3d graphics
            *gnuplot << "splot '-' w pm3d ";
            *gnuplot << "\n";
            for (SizeType i = 0; i <= this->dim; i++)
            {
                if(this->geom[i].x == std::numeric_limits<double>::lowest() && this->geom[i].y == std::numeric_limits<double>::max()){
                    *gnuplot << "\n";
                }else{
                    *gnuplot << to_string(this->geom[i].y) << "\n";
                }
            }
            *gnuplot << "e\n";  
    }

    //Reset the dimension of the current object
    this->dim = 0; 
}

void GnuplotCanvas::write(const char* filename) const
{
    *gnuplot << "quit\n";
}

void GnuplotCanvas::set_line_width(double _lw)
{
    this->lw = _lw;
}

void GnuplotCanvas::set_line_colour(double _r, double _g, double _b) 
{
    this->lc.red = std::round(_r*255);
    this->lc.green = std::round(_g*255);
    this->lc.blue = std::round(_b*255); 
}

void GnuplotCanvas::set_fill_opacity(double _fo)
{
    this->fc.opacity = _fo;

    *gnuplot << "set style fill transparent solid " << to_string(_fo) << "\n";
}

void GnuplotCanvas::set_fill_colour(double _r, double _g, double _b) 
{
    this->fc.red = std::round(_r*255);
    this->fc.green = std::round(_g*255);
    this->fc.blue = std::round(_b*255);
}

Void GnuplotCanvas::set_colour_palette() {
    isColourPalette = true;
    *gnuplot << "set cbrange [" << to_string(-1) << ":" << to_string(1) << "]\n";
    *gnuplot << "set cbtics " << to_string(0.2) << "\n";
    *gnuplot << "set palette defined\n";
}

void GnuplotCanvas::set_heat_map(Bool hm)
{
    if(hm){
        *gnuplot << "set view map\n";
    }
    else{
        *gnuplot << "unset view\n";
    } 
}

void GnuplotCanvas::set_multiplot(bool s)
{

    if (this->isMultiplot == s){ }
    else{
        this->isMultiplot = s;
        if (s){ *gnuplot << "set multiplot\n"; }
        else { *gnuplot << "unset multiplot\n"; }
    }
}

void GnuplotCanvas::set_labels(String _xLabel, String _yLabel, String _zLabel)
{
    *gnuplot << "set xlabel '" << _xLabel << "'\n";
    this->labels.xLabel = _xLabel;
    *gnuplot << "set ylabel '" << _yLabel << "'\n";
    this->labels.yLabel = _yLabel;
    if (_zLabel != "")
    {
        *gnuplot << "set zlabel '" << _zLabel << "'\n";
        this->labels.zLabel = _zLabel;
    }
}

void GnuplotCanvas::set_range_2d(double minX, double maxX, double minY, double maxY)
{
    *gnuplot << "set xrange [" << to_string(minX) <<":" << to_string(maxX) << "] \n";
    this->rng.Xmin = minX;
    this->rng.Xmax = maxX;
    *gnuplot << "set yrange [" << to_string(minY) <<":" << to_string(maxY) << "] \n";
    this->rng.Ymin = minY;
    this->rng.Ymax = maxY;
}

void GnuplotCanvas::set_range_3d(double minX, double maxX, double minY, double maxY, double minZ, double maxZ)
{
    *gnuplot << "set xrange [" << to_string(minX) <<":" << to_string(maxX) << "] \n";
    this->rng.Xmin = minX;
    this->rng.Xmax = maxX;
    *gnuplot << "set yrange [" << to_string(minY) <<":" << to_string(maxY) << "] \n";
    this->rng.Ymin = minY;
    this->rng.Ymax = maxY;
    *gnuplot << "set zrange [" << to_string(minZ) <<":" << to_string(maxZ) << "] \n";
    this->rng.Zmin = minZ;
    this->rng.Zmax = maxZ;
}
#endif

} // namespace Ariadne

