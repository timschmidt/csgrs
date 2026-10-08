/***************************************************************************
 *            test_gnuplot.cpp
 *
 *  Copyright  2008-21  Mirko Albanese
 *
 ****************************************************************************
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



#include "../test.hpp"
#include "ariadne.hpp"
#include "numeric/numeric.hpp"
#include "numeric/float_bounds.hpp"
#include "algebra/tensor.hpp"
#include "algebra/tensor.tpl.hpp"

#ifdef HAVE_GNUPLOT_H

#include "io/gnuplot.hpp"
#include "dynamics/1D_pde.hpp"
#include "dynamics/2D_pde.hpp"

using namespace Ariadne;
using namespace std;

class TestGnuplot
{
    public:

        TestGnuplot(){ GraphicsManager::instance().set_backend(GnuplotGraphicsBackend()); };
        ~TestGnuplot(){};

        void test_fixed_precision() {
            ARIADNE_TEST_CALL(test_point(double_precision));
            ARIADNE_TEST_CALL(test_box(double_precision));
            ARIADNE_TEST_CALL(test_interpolateCurve(double_precision));
        }

        template<class PR>
        void test_variable_precision(PR pr)
        {
            ARIADNE_TEST_CALL(test_stringAnimation(pr));
            ARIADNE_TEST_CALL(test_gauss3D(pr));
            ARIADNE_TEST_CALL(test_gauss3DProjXY(pr));
            ARIADNE_TEST_CALL(test_gauss3DProjXZ(pr));
            ARIADNE_TEST_CALL(test_gauss3DProjYZ(pr));
            ARIADNE_TEST_CALL(test_gauss3DAnimation(pr));
        }

        template< class PR>
        void test_point(PR pr)
        {
            //2d point
            Point<Float<PR>> pt1(2, pr);
            pt1[0]=Float<PR>(cast_exact(2.),pr);
            pt1[1]=Float<PR>(cast_exact(4.),pr);
            Figure g1 = Figure(ApproximateBoxType({{0,5},{0,5}}), Projection2d(2,0,1));
            g1.draw(pt1);
            g1.write("test_gnuplot-point2d");

        }

        template< class PR>
        void test_box(PR pr)
        {
            Box<Interval<Float<PR>>> box({{1,4},{2,3}});
            Figure g1 = Figure(ApproximateBoxType({{0,5},{1,4}}), Projection2d(2,0,1));
            g1.draw(box);
            g1.write("test_gnuplot-box");
        }

        template<class PR>
        void test_interpolateCurve(PR pr)
        {
            InterpolatedCurve cv(0,Point<Float<PR>>(2,Float<PR>(0.0_x,pr)));
            for(Int i=1; i<=10; ++i) {
                Point<Float<PR>> pt(2,pr); pt[0]=Float<PR>(cast_exact(i/10.),pr); pt[1]=Float<PR>(cast_exact(i*i/100.),pr);
                cv.insert(i,pt);
            }
            Figure g(ApproximateBoxType({{-1,+1},{-1,+1}}),Projection2d(2,0,1));
            g.set_bounding_box(cv.bounding_box());
            g.set_line_colour(1,0,0);
            g.set_line_colour(0,0,0);
            g.draw(cv);
            g.write("test_gnuplot-curve");

        }

        template<class PR>
        void test_stringAnimation(PR pr)
        {
            SizeType Nx = 10;             // # point in space

            Parameter1D<PR> stringModel(pr);

            stringModel.length = 10;  // Length
            stringModel.tension = 100000;
            stringModel.mass = 1;

            Float<PR> c = sqrt((stringModel.tension/(stringModel.mass/stringModel.length))).value();

            stringModel.x0 = (0.85_q*stringModel.length).value();  // Point of max amplitube - Triangular IC
            Float<PR> wavelength = (stringModel.length*2).value();

            Float<PR> frequency = (c/wavelength).value(); // Frequency

            stringModel.amp = cast_exact(ApproximateDouble(0.8));
            stringModel.damping = 100;

            stringModel.CourantNumber = cast_exact(ApproximateDouble(0.8));

            stringModel.k = ((2*pi)/wavelength).value();
            stringModel.omega = (2*pi*frequency).value();

            Tensor<2, Float<PR>> data = pde_1d_solver(stringModel, Nx, pr);

            Figure fig1 = Figure(ApproximateBoxType({{0,Nx-1},{-1,1}}), Projection2d(2,0,1));
            fig1.set_line_colour(0.0,0.0,0.0);
            fig1.set_line_width(4.0);
            fig1.set_fill_style(false);
            fig1.set_fill_colour(1.0,1.0,1.0);
            fig1.set_animated(true);
            fig1.draw(data);
            fig1.write("test_gnuplot-StringEvolution");


            RealVariable x("x"), y("y");
            Axes2d axes(0.0<=x<=Nx-1,-1.0<=y<=1.0);
            LabelledFigure fig2=LabelledFigure(axes);
            fig2 << line_colour(0.0,0.0,0.0);
            fig2 << line_width(4.0);
            fig2 << fill_style(false);
            fig2 << fill_colour(1.0,1.0,1.0);
            fig2 << set_animated(true);

            fig2.draw(data);

            fig2.write("test_gnuplot-LabelledFigure-StringEvolution");


        }//String Evolution over time

        template<class PR>
        void test_gauss3D(PR pr)
        {
            Float<PR> zb(0.0_x, pr);

            int dim = 20;
            Tensor<3, Float<PR>> data({SizeType(dim), SizeType(dim), 1}, zb);

            data = gaussian_function(data, dim, dim, pr);

            Figure fig1 = Figure(ApproximateBoxType({{0,dim-1},{0,dim-1},{0,1}}), Projection3d(3,0,1,2));
            fig1.draw(data);
            fig1.write("test_gnuplot-Gauss3D");

            RealVariable x("x"), y("y"), z("z");
            Axes3d axes(0<=x<=dim-1,0<=y<=dim-1,0<=z<=1);
            LabelledFigure fig2=LabelledFigure(axes);
            fig2.draw(data);
            fig2.write("test_gnuplot-LabelledFigure-Gauss3D");

        }//Gauss 3D

        template< class PR>
        void test_gauss3DProjXY(PR pr)
        {
            Float<PR> zb(0.0_x, pr);

            int dim = 20;
            Tensor<3, Float<PR>> data({SizeType(dim), SizeType(dim), 1}, zb);

            data = gaussian_function(data, dim, dim, pr);

            Figure fig1 = Figure(ApproximateBoxType({{0, dim-1},{0, dim-1}, {0,1}}), Projection2d(3,0,1));
            fig1.draw(data);
            fig1.write("test_gnuplot-Gauss3DProjXY");

            RealVariable x("x"), y("y");
            Axes2d axes(0<=x<=dim-1,0<=y<=dim-1);
            LabelledFigure fig2=LabelledFigure(axes);
            fig2.draw(data);
            fig2.write("test_gnuplot-LabelledFigure-Gauss3DProjXY");

        }

        template< class PR>
        void test_gauss3DProjXZ(PR pr)
        {
            Float<PR> zb(0.0_x, pr);

            int dim = 20;
            Tensor<3, Float<PR>> data({SizeType(dim), SizeType(dim), 1}, zb);

            data = gaussian_function(data, dim, dim, pr);

            Figure fig1 = Figure(ApproximateBoxType({{0, dim-1},{0, dim-1}, {0,1}}), Projection2d(3,0,2));
            fig1.draw(data);
            fig1.write("test_gnuplot-Gauss3DProjXZ");

            RealVariable x("x"), y("z");
            Axes2d axes(0<=x<=dim-1,0<=y<=1);
            LabelledFigure fig2=LabelledFigure(axes);
            fig2.draw(data);
            fig2.write("test_gnuplot-LabelledFigure-Gauss3DProjXZ");

        }

        template< class PR>
        void test_gauss3DProjYZ(PR pr)
        {
            Float<PR> zb(0.0_x, pr);

            int dim = 20;
            Tensor<3, Float<PR>> data({SizeType(dim), SizeType(dim), 1}, zb);

            data = gaussian_function(data, dim, dim, pr);

            Figure fig1 = Figure(ApproximateBoxType({{0, dim-1},{0, dim-1}, {0,1}}), Projection2d(3,1,2));
            fig1.draw(data);
            fig1.write("test_gnuplot-Gauss3DProjYZ");

            RealVariable x("y"), y("z");
            Axes2d axes(0<=x<=dim-1,0<=y<=1);
            LabelledFigure fig2=LabelledFigure(axes);
            fig2.draw(data);
            fig2.write("test_gnuplot-LabelledFigure-Gauss3DProjYZ");

        }

        template<class PR>
        void test_gauss3DAnimation(PR pr)
        {
            int Nx = 10;   //Mesh 1° dim
            int Ny = 10;   //Mesh 2° dim

            Parameter2D<PR> firstDim(pr), secondDim(pr);
            firstDim.length = 10;
            secondDim.length = 10;
            firstDim.x0 = 5;
            secondDim.x0 = 5;

            Tensor<3, Float<PR>> data = pde_2d_solver(firstDim, secondDim, SizeType(Nx), SizeType(Ny), pr);

            Figure fig1 = Figure(ApproximateBoxType({{0,Nx-1}, {0,Ny-1}, {-1,1}}), Projection3d(3, 0, 1, 2));
            fig1.set_animated(true);
            fig1.draw(data);
            fig1.write("test_gnuplot-Gauss3DAnimation");

            RealVariable x("x"), y("y"), z("z");
            Axes3d axes(0<=x<=Nx-1,0<=y<=Ny-1,-1<=z<=1);
            LabelledFigure fig2=LabelledFigure(axes);
            fig2 << set_animated(true);
            fig2.draw(data);
            fig2.write("test_gnuplot-LabelledFigure-Gauss3DAnimation");
        }
};

int main(int argc, const char** argv) {

    TestGnuplot testGnuplot;

    testGnuplot.test_fixed_precision();
    testGnuplot.test_variable_precision(double_precision);
    testGnuplot.test_variable_precision(MultiplePrecision(128));

    return ARIADNE_TEST_FAILURES;
}

#else
int main(int arc, const char** argv)
{}

#endif

