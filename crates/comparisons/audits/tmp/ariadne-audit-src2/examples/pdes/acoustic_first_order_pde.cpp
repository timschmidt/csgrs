/***************************************************************************
 *            acoustic_first_order_pde.cpp
 *
 *  Copyright  2018-20  Pieter Collins, Svetlana Selivanova
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include "dynamics/first_order_pde.hpp"

#include "numeric/numeric.hpp"
#include "function/function.hpp"

using namespace Ariadne;

int main() {
    // DimensionType m=2;
    DimensionType n=3;

    Real rho = 1/2_q; rho=1/2_q;
    Real c = 3; c=1;

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
    EffectiveVectorMultivariateFunction phi0={sin(2*x[0]),sin(3*x[1]),z};

    FirstOrderPDE pde{A,Bs,Ds,Ts,f};
    auto pr=multiple_precision(128_bits);

    auto solution=first_order_pde(pde,phi0,pr);
    auto h=solution.h;
    auto tau=solution.tau;
    auto uts=solution.uts;
    auto error=solution.error;
    std::cout << "uts="<<uts<<"\n";
    std::cout << "h="<<h<<", tau="<<tau<<", error="<<error<<"\n";

    // Check derivatives
    SizeType i0=2; SizeType i1=3; SizeType k=0;
    auto dudt=(uts[{i0,i1,k+1}]-uts[{i0,i1,k}])/tau;
    auto dudx0=(uts[{i0+1,i1,k}]-uts[{i0-1,i1,k}])/h/2;
    auto dudx1=(uts[{i0,i1+1,k}]-uts[{i0,i1-1,k}])/h/2;
    std::cout << "Check A*dudt + B0*dudx0 + B1*dudx1=" << A*dudt + B0*dudx0 + B1*dudx1 <<std::endl;
}
