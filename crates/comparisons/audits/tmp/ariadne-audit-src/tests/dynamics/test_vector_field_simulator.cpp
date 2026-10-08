/***************************************************************************
 *            test_vector_field_simulator.cpp
 *
 *  Copyright  2006-20  Luca Geretti, Mirko Albanese
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

#include <fstream>
#include <iostream>

#include "config.hpp"
#include "utility/tuple.hpp"
#include "algebra/vector.hpp"
#include "algebra/matrix.hpp"
#include "algebra/algebra.hpp"
#include "function/function.hpp"
#include "function/constraint.hpp"
#include "dynamics/enclosure.hpp"
#include "geometry/box.hpp"
#include "geometry/list_set.hpp"
#include "solvers/integrator.hpp"
#include "symbolic/expression_set.hpp"
#include "dynamics/vector_field_simulator.hpp"
#include "io/figure.hpp"
#include "io/command_line_interface.hpp"
#include "dynamics/orbit.hpp"

#include "../test.hpp"

using namespace Ariadne;
using namespace std;

class TestVectorFieldSimulator
{
  public:
    Void test() const {
        ARIADNE_TEST_CALL(test_multiple_real_expr_vdp_cycle());
        ARIADNE_TEST_CALL(test_multiple_real_box_vdp_cycle());
        ARIADNE_TEST_CALL(test_multiple_trajectories_x());
        ARIADNE_TEST_CALL(test_multiple_trajectories_xy());
    }

    Void test_multiple_trajectories_x() const {
        typedef VectorFieldSimulator::ApproximateListPointType ListPointType;

        Real mu=Dyadic(0.5_x);
        RealVariable x("x"), y("y"), z("z");

        VectorField vanderpol({dot(x)=y,dot(y)=mu*(1-x*x)*y-x},{let(z)=sqrt(sqr(x)+sqr(y))});
        ARIADNE_TEST_PRINT(vanderpol);

        RealExpressionBoundedConstraintSet initial_set({-2.0_dec<=x<=-1.5_dec,0.5_dec<=y<=1});

        Real time = 1.0_dec;

        VectorFieldSimulator simulator(vanderpol);
        simulator.configuration().set_step_size(0.05);
        simulator.configuration().set_num_subdivisions(0);
        simulator.configuration().set_discretisation_type(DiscretisationType::Mince);

        SizeType trajectory_number = 4;

        for (SizeType t=1; t<=trajectory_number; t++) {
            simulator.configuration().insert_subspace(x, t);
            ARIADNE_TEST_PRINT(simulator.configuration());

            Orbit<ListPointType> orbit = simulator.orbit(initial_set,time);

            List<LabelledInterpolatedCurve> curves = orbit.curves();

            ARIADNE_TEST_EQUALS(curves.size(),t);

            simulator.configuration().remove_keys();
        }
    }

    Void test_multiple_trajectories_xy() const {
        typedef VectorFieldSimulator::ApproximateListPointType ListPointType;

        Real mu=Dyadic(0.5_x);
        RealVariable x("x"), y("y"), z("z");

        VectorField vanderpol({dot(x)=y,dot(y)=mu*(1-x*x)*y-x},{let(z)=sqrt(sqr(x)+sqr(y))});
        ARIADNE_TEST_PRINT(vanderpol);

        RealExpressionBoundedConstraintSet initial_set({-2.0_dec<=x<=-1.5_dec,0.5_dec<=y<=1});

        Real time = 1.0_dec;

        VectorFieldSimulator simulator(vanderpol);
        simulator.configuration().set_step_size(0.05);
        simulator.configuration().set_num_subdivisions(0);
        simulator.configuration().set_discretisation_type(DiscretisationType::Mince);

        SizeType trajectory_number = 4;

        for (SizeType first=1; first<=trajectory_number; first++) {
            for (SizeType second=1; second<=trajectory_number; second++){
                simulator.configuration().insert_subspace(x, first);
                simulator.configuration().insert_subspace(y, second);
                ARIADNE_TEST_PRINT(simulator.configuration());

                Orbit<ListPointType> orbit = simulator.orbit(initial_set,time);

                List<LabelledInterpolatedCurve> curves = orbit.curves();

                ARIADNE_TEST_EQUALS(curves.size(), first*second);

                simulator.configuration().remove_keys();
            }
        }
    }

    Void test_multiple_real_expr_vdp_cycle() const {

        typedef VectorFieldSimulator::ApproximateListPointType ListPointType;

        Real mu=Dyadic(0.5_x);
        RealVariable x("x"), y("y"), z("z");

        VectorField vanderpol({dot(x)=y,dot(y)=mu*(1-x*x)*y-x},{let(z)=sqrt(sqr(x)+sqr(y))});
        ARIADNE_TEST_PRINT(vanderpol);

        RealExpressionBoundedConstraintSet initial_set({-2.0_dec<=x<=-1.5_dec,0.5_dec<=y<=1});

        Real time = 6.3_dec;

        VectorFieldSimulator simulator(vanderpol);
        simulator.configuration().set_step_size(0.05);
        simulator.configuration().set_num_subdivisions(0);
        simulator.configuration().set_discretisation_type(DiscretisationType::Mince);
        simulator.configuration().insert_subspace(x, 1);
        simulator.configuration().insert_subspace(y, 1);

        ARIADNE_TEST_PRINT(simulator.configuration());

        Orbit<ListPointType> orbit = simulator.orbit(initial_set,time);

        LabelledFigure fig({-2.5_dec<=x<=2.5_dec,-2.5_dec<=y<=2.5_dec});
        fig << orbit;
        fig.write("test_real_expr_vector_field_simulator_xy");

        LabelledFigure fig2({0<=TimeVariable()<=6.3_dec,0<=z<=2.5_dec});
        fig2 << orbit;
        fig2.write("test_real_expr_vector_field_simulator_tz");

    }

    Void test_multiple_real_box_vdp_cycle() const{
        typedef VectorFieldSimulator::ApproximateListPointType ListPointType;
      
        Real mu=Dyadic(0.5_x);
        RealVariable x("x"), y("y"), z("z");

        VectorField vanderpol({dot(x)=y,dot(y)=mu*(1-x*x)*y-x},{let(z)=sqrt(sqr(x)+sqr(y))});
        ARIADNE_TEST_PRINT(vanderpol);

        RealVariablesBox initial_set({-2.0_dec<=x<=-1.5_dec,0.5_dec<=y<=1});
        
        Real time = 6.3_dec;

        VectorFieldSimulator simulator(vanderpol);
        simulator.configuration().set_step_size(0.05);
        simulator.configuration().set_num_subdivisions(0);
        simulator.configuration().set_discretisation_type(DiscretisationType::Mince);
        simulator.configuration().insert_subspace(x, 1);
        simulator.configuration().insert_subspace(y, 1);

        ARIADNE_TEST_PRINT(simulator.configuration());

        Orbit<ListPointType> orbit = simulator.orbit(initial_set,time);

        LabelledFigure fig({-2.5<=x<=2.5,-3<=y<=3});
        fig << orbit;
        fig.write("test_real_box_list_vector_field_simulator_xy");

        LabelledFigure fig2({0<=TimeVariable()<=6.3_dec,0<=z<=2.5_dec});
        fig2 << orbit;
        fig2.write("test_real_box_list_vector_field_simulator_tz");
    }
};

Int main(Int argc, const char* argv[])
{
    if (not CommandLineInterface::instance().acquire(argc,argv)) return -1;
    TestVectorFieldSimulator().test();
    return ARIADNE_TEST_FAILURES;
}
