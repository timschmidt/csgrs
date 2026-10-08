#include "1D_pde.hpp"

//using namespace std;
namespace Ariadne{

    template<class X>
    Array<Float<X>> linspace1d(Float<X> L, SizeType n)
    {
        Array<Float<X>> linspaced(n, Float<X>(0.0));
        if (n == 0)
            return linspaced;
        if (n == 1)
        {
            linspaced[0] = L;
            return linspaced;
        }
        Float<X> delta = (L/(n - 1));
        for (SizeType i = 0; i < (n - 1); i++)
        {
            linspaced[i] = (0 + delta*i);
        }
        linspaced[n - 1] = L;
        
        return linspaced;
    }

    // Set initial condition
    template<class X>
    Tensor<2, Float<X>> set_ic(Tensor<2, Float<X>>& uts,/* std::function<Float<X>(Float<X>)> &phi0, */SizeType Nx, Array<Float<X>> spacePoint, Parameter1D<X>& stringModel, bool isTriangular)
    {
        if (!isTriangular)
        {
            for (SizeType i = 0; i < Nx; i++){
            //uts[{i, 0}] = phi0(spacePoint[i]);
                uts[{i, 0}] = sin(stringModel.k*spacePoint[i]); //Sinusoind init condition
            }
        }
        else{
            for (SizeType i = 0; i < Nx; i++){
                if (definitely(spacePoint[i] < stringModel.x0))
                    uts[{i, 0}] = (stringModel.amp*(spacePoint[i]/stringModel.x0));//.value();
                else
                    uts[{i, 0}] (stringModel.amp*(stringModel.length - spacePoint[i])/(stringModel.length - stringModel.x0));//.value();
            }
        }

        return uts;
    }

    //Solving one dimensional pde
    template<class PR>
    Tensor<2, Float<PR>> pde_1d_solver(/*std::function<Float<PR>(Float<PR>)>& phi0, std::function<Float<PR>(Float<PR>, Float<PR>)>& source, */Parameter1D<PR>& stringParameter, SizeType Nx)
    {
        Float<PR> c = sqrt((stringParameter.tension/(stringParameter.mass/stringParameter.length)));
        //FloatDP c = sqrt((stringParameter.tension/(stringParameter.mass/stringParameter.length))).value();

        Float<PR> T = 0.060;
        Float<PR> zb = 0;

        Float<PR> C2 = pow(stringParameter.CourantNumber, 2);
        //FloatDP C2 = pow(stringParameter.CourantNumber, 2).value();

        Array<Float<PR>> space = linspace1d(stringParameter.length, Nx);
        //Array<FloatDP> space = linspace1D(stringParameter.length, Nx);

        Float<PR> dx = (space[1] - space[0]);
        //FloatDP dx = (space[1] - space[0]).value();
        Float<PR> dt = (stringParameter.CourantNumber*dx/c);
        //FloatDP dt = (stringParameter.CourantNumber*dx/c).value();
        //FFloatDP Nt = round(T/dt).value();
        Float<PR> Nt = round(T/dt);
        //FloatDP Nt = round(T/dt).value();
        SizeType Ntime = Nt.get_d();

        Array<Float<PR>> time = linspace1d(T, Ntime);

        Tensor<2, Float<PR>> uts({Nx, Ntime}, zb);

        uts = set_ic(uts, /*phi0, */Nx, space);

        // Set first Time step
        SizeType n = 0;
        uts[{0, n}] = 0;
        uts[{Nx - 1, n}] = 0;
        for (SizeType i = 1; i < Nx; i++)
        {
            uts[{i, n+1}] = (uts[{i, n}] - C2*(uts[{i-1, n}] - 2u*uts[{i, n}] + uts[{i+1, n}])/2u
                + pow(dt,2)*sin(stringParameter.k*space[i])*cos(stringParameter.omega*time[n]));//source(space[i], time[n]));//.value();
        }// first time step

        // For each time step - main loop
        for (n = 1; n < Nt.get_d(); n++)
        {
            uts[{0, n}] = 0;
            uts[{Nx - 1, n}] = 0;
            // Update inner points
            for (SizeType i = 1; i < Nx; i++)
            {
                uts[{i, n+1}] = ((1/(stringParameter.damping*dt/2u+1))*(2*uts[{i, n}] - uts[{i, n-1}] + (stringParameter.damping*dt/2)*uts[{i, n-1}] +
                    C2*(uts[{i-1, n}] - 2*uts[{i, n}] + uts[{i+1, n}])) + 1000u*pow(dt, 2)*sin(stringParameter.k*space[i])*cos(stringParameter.omega*time[n]));//source(space[i], time[n]));//.value();
            }
        }//main loop
        return uts;
    }
        
}// namespace Ariadne