/***************************************************************************
 *            test_graphics.cpp
 *
 *  Copyright  2008-20  Pieter Collins
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

#include "config.hpp"
#include "../test.hpp"

#include "function/function.hpp"
#include "io/figure.hpp"
#include "io/drawer.hpp"
#include "geometry/point.hpp"
#include "geometry/box.hpp"
#include "geometry/curve.hpp"
#include "geometry/function_set.hpp"
#include "geometry/grid_paving.hpp"
#include "symbolic/expression_set.hpp"

using namespace Ariadne;

class TestGraphics {
  public:
    void test() {
        ARIADNE_TEST_CALL(test_point2d())
        ARIADNE_TEST_CALL(test_colour())
        ARIADNE_TEST_CALL(test_graphics_properties())
        ARIADNE_TEST_CALL(test_construct_figure())
        ARIADNE_TEST_CALL(test_figure_projection())
        ARIADNE_TEST_CALL(test_construct_labelled_figure())
        ARIADNE_TEST_CALL(test_labelled_figure_projection())
        ARIADNE_TEST_CALL(test_default_drawer())
    }

    void test_point2d() {
        ARIADNE_TEST_PRINT(Point2d(1,1))
    }

    void test_colour() {
        ARIADNE_TEST_PRINT(Colour(1,1,1,0))
        ARIADNE_TEST_PRINT(Colour("white",1,1,1))
    }

    void test_graphics_properties() {
        GraphicsProperties p;
        p.set_line_colour(0,0,1)
         .set_fill_opacity(0.5)
         .set_fill_colour(0,1,0);
        ARIADNE_TEST_PRINT(p)
    }

    void test_construct_figure() {
        GraphicsBoundingBoxType bx3({{FloatDPApproximation(0.0,dp),FloatDPApproximation(1.0,dp)},
                                    {FloatDPApproximation(-1.0,dp),FloatDPApproximation(0.0,dp)},
                                    {FloatDPApproximation(-2.0,dp),FloatDPApproximation(2.0,dp)}});
        ARIADNE_TEST_EXECUTE(Figure(bx3,0,1))
        ARIADNE_TEST_EXECUTE(Figure(bx3,{0,1}))
        ARIADNE_TEST_EXECUTE(Figure(bx3,0,1,2))
        ARIADNE_TEST_EXECUTE(Figure(bx3,Projection2d(3,0,1)))
        ARIADNE_TEST_EXECUTE(Figure(bx3,Projection3d(3,2,1,0)))
        GraphicsBoundingBoxType bx1({{FloatDPApproximation(0.0,dp),FloatDPApproximation(1.0,dp)}});
        ARIADNE_TEST_FAIL(Figure(bx1,0,1))
        ARIADNE_TEST_FAIL(Figure(bx1,{0,1}))
        ARIADNE_TEST_FAIL(Figure(bx1,0,1,2))
        ARIADNE_TEST_FAIL(Figure(bx1,Projection2d(2,0,1)))
        ARIADNE_TEST_FAIL(Figure(bx1,Projection3d(3,2,1,0)))
        Figure fig(bx3,0,1);
        ARIADNE_TEST_PRINT(fig.get_bounding_box())
        auto const& p = fig.properties();
        ARIADNE_TEST_PRINT(p)
    }

    void test_figure_projection() {
        GraphicsBoundingBoxType bx({{FloatDPApproximation(0.0,dp),FloatDPApproximation(1.0,dp)},
                                    {FloatDPApproximation(-1.0,dp),FloatDPApproximation(0.0,dp)},
                                    {FloatDPApproximation(-2.0,dp),FloatDPApproximation(2.0,dp)}});
        Figure f(bx,0,1,2);
        f.set_projection(3,0,2,1);
        ARIADNE_TEST_FAIL(f.set_projection(4,0,1))
        ARIADNE_TEST_FAIL(f.set_projection(4,0,1,2))
        ARIADNE_TEST_EXECUTE(f.set_projection(3,0,1))
        ARIADNE_TEST_EXECUTE(f.set_projection(3,0,1,2))
        ARIADNE_TEST_FAIL(f.set_projection_map(Projection2d(4,0,1)))
        ARIADNE_TEST_FAIL(f.set_projection_map(Projection3d(4,0,1,2)))
        ARIADNE_TEST_EXECUTE(f.set_projection_map(Projection2d(3,0,1)))
        ARIADNE_TEST_EXECUTE(f.set_projection_map(Projection3d(3,2,0,1)))
        ARIADNE_TEST_PRINT(f.get_2d_projection_map())
        ARIADNE_TEST_PRINT(f.get_3d_projection_map())
    }

    void test_construct_labelled_figure() {
        RealVariable x("x"), y("y"), z("z");
        GraphicsBoundingBoxType bx1d({{FloatDPApproximation(0.0,dp),FloatDPApproximation(1.0,dp)}});
        GraphicsBoundingBoxType bx2d({{FloatDPApproximation(0.0,dp),FloatDPApproximation(1.0,dp)},
                                      {FloatDPApproximation(-1.0,dp),FloatDPApproximation(0.0,dp)}});
        GraphicsBoundingBoxType bx3d({{FloatDPApproximation(0.0,dp),FloatDPApproximation(1.0,dp)},
                                    {FloatDPApproximation(-1.0,dp),FloatDPApproximation(0.0,dp)},
                                    {FloatDPApproximation(-2.0,dp),FloatDPApproximation(2.0,dp)}});
        ARIADNE_TEST_FAIL(LabelledFigure(Variables2d({x,y}),VariablesBox<ApproximateIntervalType>(RealSpace({x}),bx1d)))
        ARIADNE_TEST_EXECUTE(LabelledFigure(Variables2d({x,y}),VariablesBox<ApproximateIntervalType>(RealSpace({x,y}),bx2d)))
        ARIADNE_TEST_FAIL(LabelledFigure(Variables3d({x,y,z}),VariablesBox<ApproximateIntervalType>(RealSpace({x,y}),bx2d)))
        ARIADNE_TEST_EXECUTE(LabelledFigure(Variables3d({x,y,z}),VariablesBox<ApproximateIntervalType>(RealSpace({x,y,z}),bx3d)))
        LabelledFigure f(Variables3d({x,y,z}),VariablesBox<ApproximateIntervalType>(RealSpace({x,y,z}),bx3d));
        auto const& p = f.properties();
        ARIADNE_TEST_PRINT(p)
    }

    void test_labelled_figure_projection() {
        RealVariable x("x"), y("y"), z("z");
        LabelledFigure f(Axes2d({0<=x<=1,-1<=y<=2}));
        ARIADNE_TEST_EXECUTE(f.set_axes(Axes2d({0<=z<=1,-1<=y<=2})))
        ARIADNE_TEST_EXECUTE(f.set_axes(Axes3d({0<=z<=1,-1<=y<=2,0<=x<=2})))
        ARIADNE_TEST_EXECUTE(f.set_bounds(x,ApproximateDoubleInterval(1.0,2.0)))
        ARIADNE_TEST_EXECUTE(f.set_bounds({{x,ApproximateDoubleInterval(1.0,2.0)},{y,ApproximateDoubleInterval(0.0,1.0)}}))
        ARIADNE_TEST_EXECUTE(f.set_bounding_box(VariablesBox<ApproximateIntervalType>(RealSpace({x}),GraphicsBoundingBoxType({{FloatDPApproximation(0.0,dp),FloatDPApproximation(1.0,dp)}}))))
    }

    void test_default_drawer()
    {
        ExactBoxType bx1(2); bx1[0]=ExactIntervalType(-0.2_pr,0.2_pr); bx1[1]=ExactIntervalType(-0.1_pr,0.10_pr);
        ExactBoxType bx2(2); bx2[0]=ExactIntervalType(0.1_pr,0.3_pr); bx2[1]=ExactIntervalType(0.05_pr,0.15_pr);
        ExactBoxType bx3(2); bx3[0]=ExactIntervalType(0.2_pr,0.4_pr); bx3[1]=ExactIntervalType(0.10_pr,0.25_pr);
        ExactBoxType bx4(2); bx4[0]=ExactIntervalType(0.25_pr,0.5_pr); bx4[1]=ExactIntervalType(0.20_pr,0.50_pr);
        ExactBoxType bx5(2); bx5[0]=ExactIntervalType(0.4_pr,0.8_pr); bx5[1]=ExactIntervalType(0.40_pr,1.1_pr);

        Real p(0.5_x);
        EffectiveVectorMultivariateFunction x=EffectiveVectorMultivariateFunction::identity(3);
        EffectiveVectorMultivariateFunction afn1={0.05_dec*x[0]+0.05_dec*x[2]+0.15_dec,0.05_dec*x[1]+0.05_dec*x[2]+0.6_dec};
        ValidatedConstrainedImageSet s1(ExactBoxType::unit_box(3),afn1);
        ApproximateBoxType bbx1=widen(s1.bounding_box(),0.25_x);

        EffectiveVectorMultivariateFunction rf(1u, sqr(x[0])+sqr(x[1])-sqr(p));
        ConstraintSet cs1(rf,RealBox(1u,RealInterval(-1,0)));

        {
            ExactDouble h=10000;
            Figure g(ExactBoxType{{-1.0_x,+1.0_x},{-h,+h}},Projection2d(2,0,1));
            g.set_fill_colour(0.5,1.0,1.0);
            g.set_line_width(10);
            g << ExactBoxType({{-0.5_x,+0.0_x},{-h/two, +h/two}});
            g.set_line_width(1);
            g << ExactBoxType({{0.25_x,+0.75_x},{-h/two, +h/two}});
            g.write("test_graphics-canvas");
        }

        Figure g(ApproximateBoxType({{-1,+1},{-1,+1}}),Projection2d(2,0,1));
        g << fill_colour(0.5,1.0,1.0)
          << line_colour(0.0,0.0,0.0)
          << bx1
          << bx2
          << bx3
          << bx4
          << bx5;
        g.write("test_graphics-bx1");
        g.clear();

        g.set_fill_colour(1.0,0.5,1.0);
        g.draw(bx1);
        g.draw(bx2);
        g.set_fill_colour(magenta);
        g.draw(bx5);
        g.write("test_graphics-bx2");
        g.clear();


        ExactBoxType bx2d(2); bx2d[0]=ExactIntervalType(0.2_pr,0.4_pr); bx2d[1]=ExactIntervalType(0.2_pr,0.5_pr);
        ExactBoxType bx3d(3); bx3d[0]=ExactIntervalType(0.2_pr,0.4_pr); bx3d[1]=ExactIntervalType(0.2_pr,0.5_pr); bx3d[2]=ExactIntervalType(0.2_pr,0.7_pr);
        g.set_bounding_box(bx3d.bounding_box());
        g.set_projection(3,0,1);
        g.draw(bx3d);
        g.write("test_graphics-bx3");
        g.clear();

        plot("test_graphics_set",Projection2d(2,0,1),bbx1,{{orange,s1}});

        InterpolatedCurve cv(0,Point<FloatDP>(2,FloatDP(0.0_x,dp)));
        for(Int i=1; i<=10; ++i) {
            Point<FloatDP> pt(2,dp); pt[0]=FloatDP(cast_exact(i/10.),dp); pt[1]=FloatDP(cast_exact(i*i/100.),dp);
            cv.insert(i,pt);
        }
        g.set_bounding_box(cv.bounding_box());
        g.set_projection(2,0,1);
        g.set_line_colour(1,0,0);
        g.draw(cv);
        g.set_line_colour(0,0,0);
        g.write("test_graphics-curve");
        g.clear();

        GridTreePaving gts(2);
        gts.adjoin_outer_approximation(bx1, 4);
        gts.adjoin_outer_approximation(bx2, 5);
        gts.adjoin_outer_approximation(bx3, 6);
        gts.adjoin_outer_approximation(bx4, 7);
        gts.adjoin_outer_approximation(bx5, 8);
        gts.recombine();

        ExactBoxType bbox(2); bbox[0]=ExactIntervalType(-2,2); bbox[1]=ExactIntervalType(-2,2);
        g.set_bounding_box(bbox);
        g.set_projection(2,0,1);
        g << gts;
        g.write("test_graphics-paving");
        g.clear();
    }
};

Int main(Int argc, char **argv)
{
    TestGraphics().test();

    return ARIADNE_TEST_FAILURES;
}

