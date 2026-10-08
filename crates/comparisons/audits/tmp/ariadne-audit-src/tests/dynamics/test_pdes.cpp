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

#include "dynamics/first_order_pde.hpp"

#include "numeric/numeric.hpp"
#include "function/function.hpp"

#include "dynamics/1D_pde.hpp"
#include "dynamics/2D_pde.hpp"

using namespace Ariadne;
using namespace std;

class TestPdes
{
    public:

        TestPdes(){ };
        ~TestPdes(){ };

        template<class PR>
        void test(PR pr)
        {
            ARIADNE_TEST_CALL(test_sinusoidal_string(pr));
            ARIADNE_TEST_CALL(test_triangular_string(pr));
            ARIADNE_TEST_CALL(test_gauss_3d(pr));
            ARIADNE_TEST_CALL(test_first_order_pde(pr));
        }

        template<class PR>
        void test_sinusoidal_string(PR pr)
        {
            SizeType Nx = 200;             // # point in space

            Parameter1D<PR> stringModel(pr);

            stringModel.length = 200;  // Length
            stringModel.tension = 10000;
            stringModel.mass = cast_exact(ApproximateDouble(0.9));

            Float<PR> c = sqrt((stringModel.tension/(stringModel.mass/stringModel.length))).value();

            Float<PR> wavelength = (stringModel.length*2).value();

            Float<PR> frequency = (c/wavelength).value(); // Frequency

            stringModel.amp = cast_exact(ApproximateDouble(0.8));
            stringModel.damping = 100;

            stringModel.CourantNumber = cast_exact(ApproximateDouble(0.8));

            stringModel.k = ((2*pi)/wavelength).value();
            stringModel.omega = (2*pi*frequency).value();

            stringModel.is_triangular = false;

            Tensor<2, Float<PR>> data = pde_1d_solver(stringModel, Nx, pr);

            Float<PR> tollerance(cast_exact(ApproximateDouble(1.2)),pr);

            //Check values
            for (SizeType time = 0; time < data.size(1); time++)
            {
                for (SizeType x1 = 0; x1 < Nx; x1++)
                {
                    if ( (data[{x1, time}]) > (stringModel.amp+tollerance).value() )
                    {
                        ARIADNE_TEST_ASSERT(abs(data[{x1, time}]) <= (stringModel.amp+tollerance));
                        return;
                    }

                }

            }


        }//String Evolution over time

        template<class PR>
        void test_triangular_string(PR pr)
        {
            SizeType Nx = 200;             // # point in space

            Parameter1D<PR> stringModel(pr);

            stringModel.length = 200;  // Length
            stringModel.tension = 10000;
            stringModel.mass = cast_exact(ApproximateDouble(0.9));

            Float<PR> c = sqrt((stringModel.tension/(stringModel.mass/stringModel.length))).value();

            stringModel.x0 = (0.85_q*stringModel.length).value();  // Point of max amplitube - Triangular IC
            Float<PR> wavelength = (stringModel.length*2).value();

            Float<PR> frequency = (c/wavelength).value(); // Frequency

            stringModel.amp = cast_exact(ApproximateDouble(0.8));
            stringModel.damping = 100;

            stringModel.CourantNumber = cast_exact(ApproximateDouble(0.8));

            stringModel.k = ((2*pi)/wavelength).value();
            stringModel.omega = (2*pi*frequency).value();

            stringModel.is_triangular = true;

            Float<PR> tollerance(cast_exact(ApproximateDouble(1.2)),pr);

            Tensor<2, Float<PR>> data = pde_1d_solver(stringModel, Nx, pr);

            //Check values
            for (SizeType time = 0; time < data.size(1); time++)
            {
                for (SizeType x1 = 0; x1 < Nx; x1++)
                {
                    if ( (data[{x1, time}]) > (stringModel.amp+tollerance).value() )
                    {
                        ARIADNE_TEST_ASSERT(abs(data[{x1, time}]) <= (stringModel.amp+tollerance));
                        return;
                    }

                }

            }

        }//String Evolution over time

        template<class PR>
        void test_gauss_3d(PR pr)
        {
            int Nx = 25;   //Mesh 1° dim
            int Ny = 25;   //Mesh 2° dim

            Parameter2D<PR> firstDim(pr), secondDim(pr);
            firstDim.length = 25;
            secondDim.length = 25;

            firstDim.x0 = 15;
            secondDim.x0 = 15;

            Float<PR> tollerance(cast_exact(ApproximateDouble(1.2)),pr);

            Tensor<3, Float<PR>> data = pde_2d_solver(firstDim, secondDim, SizeType(Nx), SizeType(Ny), pr);

            //Check values
            for (SizeType time = 0; time < data.size(1); time++)
            {
                for (SizeType x2 = 0; x2 < SizeType(Ny); x2++)
                {
                    for(SizeType x1 = 0; x1 < SizeType(Nx); x1++)
                    {
                        if ( (data[{x1, x2, time}]) > tollerance )
                        {
                            ARIADNE_TEST_ASSERT(abs(data[{x1, x2, time}]) <= tollerance);
                            return;
                        }
                    }
                }
            }

        }

        template <class PR>
        void test_first_order_pde(PR pr)
        {
            // DimensionType m=2;
            DimensionType n=3;

            Real rho = 1/2_q; rho=1/2_q;
            Real c = 1; c=1;

            Matrix<Real> I=Matrix<Real>::identity(n);

            Matrix<Real> A={{rho,0,0},{0,rho,0},{0,0,1}};
            Matrix<Real> B0={{0,0,1},{0,0,0},{rho*c*c,0,0}};
            Matrix<Real> B1={{0,0,0},{0,0,1},{0,rho*c*c,0}};;

            Array<Matrix<Real>> Bs={B0,B1};

            auto D0=DiagonalMatrix<Real>(Array<Real>{c,0,-c});
            auto D1=DiagonalMatrix<Real>(Array<Real>{c,0,-c});
            Array<DiagonalMatrix<Real>> Ds={D0,D1};

            Real det=sqrt(1+sqr(rho)*sqr(c));
            Real c1=1/det; Real cr=(rho*c)/det;
            Matrix<Real> T0={{c1,0,c1},{0,1,0},{cr,0,-cr}};
            Matrix<Real> T1={{0,1,0},{c1,0,c1},{cr,0,-cr}};
            Array<Matrix<Real>> Ts={T0,T1};

            Vector<Real> v00=column(T0,0);
            Vector<Real> v01=column(T0,1);
            Vector<Real> v02=column(T0,2);
            Vector<Real> v10=column(T1,0);
            Vector<Real> v11=column(T1,1);
            Vector<Real> v12=column(T1,2);

            auto z=EffectiveScalarMultivariateFunction::zero(EuclideanDomain(2));
            auto x=EffectiveVectorMultivariateFunction::identity(EuclideanDomain(2));

            EffectiveVectorMultivariateFunction f=EffectiveVectorMultivariateFunction::zeros(3,EuclideanDomain(2+1));
            EffectiveVectorMultivariateFunction phi0={sin(4*x[0]),sin(6*x[1]),z};

            FirstOrderPDE pde{A,Bs,Ds,Ts,f};

            auto solution=first_order_pde(pde,phi0,pr);
            auto uts=solution.uts;


            // Check values
            for (SizeType step = 0; step < uts.size(2); step++)
            {
                for (SizeType x1 = 0; x1 < uts.size(1); x1++)
                {
                    for (SizeType x0 = 0; x0 < uts.size(0); x0++)
                    {
                        for (SizeType i = 0; i < n; i++)
                        {
                            if (uts[{x0,x1,step}].at(i).lower().get_d() > 30000.0 || uts[{x0,x1,step}].at(i).upper().get_d() > 30000.0){
                                ARIADNE_TEST_ASSERT((abs(uts[{x0,x1,step}].at(i).lower().get_d()) <= 30000.0));
                                ARIADNE_TEST_ASSERT((abs(uts[{x0,x1,step}].at(i).upper().get_d()) <= 30000.0));
                                std::cout << "lower: " << uts[{x0,x1,step}].at(i).lower().get_d() << "\nupper: " << uts[{x0,x1,step}].at(i).upper().get_d() << std::endl;
                                return;
                            }
                        }


                    }

                }

            }


        }
};

int main(int argc, const char** argv) {

    TestPdes testPdes;

    testPdes.test(double_precision);

    return ARIADNE_TEST_FAILURES;
}



