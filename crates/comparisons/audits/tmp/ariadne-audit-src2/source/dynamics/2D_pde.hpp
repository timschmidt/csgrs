#include "numeric/numeric.hpp"
#include "utility/array.hpp"
#include "algebra/tensor.hpp"

namespace Ariadne
{
    template<class PR>
    class Parameter2D
    {
        public:
            Parameter2D(PR pr)  :   length(0, pr),
                                    damping(0, pr),
                                    x0(0, pr)
            {}
            Float<PR> length;     //Length
            Float<PR> damping;    //damping
            Float<PR> x0;
    }; 



    // Set initial condition
    template<class PR>
    Tensor<2, Float<PR>> set_ic_2d(Parameter2D<PR>firstDim, Parameter2D<PR>secondDim, SizeType Nx, SizeType Ny, Array<Float<PR>> spacePointX, Array<Float<PR>> spacePointY, PR pr)
    {
        Float<PR> zb(0, pr);
        Tensor<2, Float<PR>> u({Nx, Ny}, zb);
        for (SizeType i = 0; i < Nx; i++)
        {
            for (SizeType j = 0; j < Ny; j++)
            {
                u[{i, j}] = (exp(-pow(spacePointX[i]-firstDim.x0, 2) - pow(spacePointY[j]-secondDim.x0, 2)).value());
            }
        }
        return u;
    }

    template<class PR>
    Array<Float<PR>> linspace2d(Float<PR> L, SizeType n, PR pr)
    {
        Array<Float<PR>> linspaced(n,Float<PR>(0, pr));
        if (n == 0)
            return linspaced;
        if (n == 1)
        {
            linspaced[0] = L;
            return linspaced;
        }
        Float<PR> delta = (L/n).value();
        for (SizeType i = 0; i < (n - 1); i++)
        {
            linspaced[i] = ((0 + delta*i)).value();
        }
        linspaced[n - 1] = L;
        
        return linspaced;
    }

    // Solving the one dimensional pde
    template<class PR>
    Tensor<3, Float<PR>> pde_2d_solver(Parameter2D<PR>& firstDim, Parameter2D<PR>& secondDim, SizeType Nx, SizeType Ny, PR pr)
    {
        auto spaceX = linspace2d(firstDim.length, Nx, pr);    //Mesh point x
        auto spaceY = linspace2d(secondDim.length, Ny, pr);   //Mesh point y

        auto dx = (spaceX[1] - spaceX[0]).value();
        auto dy = (spaceY[1] - spaceY[0]).value();

        Float<PR> c(cast_exact(ApproximateDouble(5.0)), pr);
        Float<PR> T(cast_exact(ApproximateDouble(2.0)), pr);

        Float<PR> zb(0, pr);

        Float<PR> dt(cast_exact(ApproximateDouble(0.09)), pr);
        
        Float<PR> Nt = round(T/dt).value();
        SizeType Ntime = Nt.get_d();
        auto time = linspace2d(T, Ntime, pr);                   //Mesh point t

        Float<PR> Cx = (c*dt/dx).value();                       //Courant Numbers
        Float<PR> Cy = (c*dt/dy).value();
        Float<PR> Cx2 = pow(Cx, 2).value();
        Float<PR> Cy2 = pow(Cy, 2).value();

        Tensor<2, Float<PR>> u({Nx, Ny}, zb);
        Tensor<3, Float<PR>> uts({Nx, Ny, Ntime}, zb);

        u = set_ic_2d(firstDim, secondDim, Nx, Ny, spaceX, spaceY, pr);
        for (SizeType x = 0; x < Nx; x++)
        {
            for (SizeType y = 0; y < Ny; y++)
            {
                uts[{x, y, 0}] = u[{x, y}];
            }   
        }

        //Compute first time step
        SizeType n = 0;
        for (SizeType i = 1; i < Nx - 1; i++)
        {
            for (SizeType j = 1; j < Ny - 1; j++)
            {
                auto u_xx = (uts[{i-1, j, n}] - 2*uts[{i, j, n}] + uts[{i+1, j, n}]).value();
                auto u_yy = (uts[{i, j-1, n}] - 2*uts[{i, j, n}] + uts[{i, j+1, n}]).value();
                uts[{i, j, n+1}] = (uts[{i, j, n}] + Cx2*u_xx/2u + Cy2*u_yy/2u).value();
            }
        }
        //Compute each time step
        for (n = 1; n < Ntime; n++)
        {
            for (SizeType i = 1; i < Nx - 1; i++)
            {
                for (SizeType j = 1; j < Ny - 1; j++)
                {
                    auto u_xx = (uts[{i-1, j, n}] - 2*uts[{i, j, n}] + uts[{i+1, j, n}]).value();
                    auto u_yy = (uts[{i, j-1, n}] - 2*uts[{i, j, n}] + uts[{i, j+1, n}]).value();
                    uts[{i, j, n+1}] = (2*uts[{i, j, n}] - uts[{i, j, n-1}] + Cx2*u_xx + Cy2*u_yy).value();
                }
                
            }
        }
    return uts;
    }

    template<class PR>
    Tensor<3, Float<PR>>gaussian_function(Tensor<3, Float<PR>>&tensor, int sizeX, int sizeY, PR pr)
    {
        for (SizeType step = 0; step < 1; step++)
        {
            for (SizeType x = 0; x < tensor.size(0); x++)
            {
                for (SizeType y = 0; y < tensor.size(1); y++)
                {
                    tensor[{x, y, step}] = (exp(-pow(Float<PR>(int(x)-sizeX/2, pr), 2).value()/16 - pow(Float<PR>(int(y)-sizeY/2, pr), 2).value()/16).value());};
                }   
            }
        return tensor;
    }


         
    

}