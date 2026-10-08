/***************************************************************************
 *            function_demonstration.cpp
 *
 *  Copyright  2009-21  Pieter Collins
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

#include "ariadne.hpp"

using namespace Ariadne;

void print() { CONCLOG_PRINTLN(""); }
template<class T> void print(const char* label, T const& expr) { CONCLOG_PRINTLN(label << ": " << (expr)) }


void function_demonstration() {
    //! [Function demonstration]

    // Create a user-defined scalar-valued function
    auto x=EffectiveScalarMultivariateFunction::identity(2);
    auto sf=sqrt(sqr(x[0])+sqr(x[1]));
    print("sf:",sf);
    auto a=FloatDPApproximationVector({4,3},dp);
    auto sfa=sf(a);
    print("sf(a):",sfa);
    auto b=FloatDPBoundsVector({4,3},dp);
    auto sfb=sf(b);
    print("sf(b):",sfb);

    x=EffectiveVectorMultivariateFunction::identity(3);
    auto vf=EffectiveVectorMultivariateFunction({sqrt(sqr(x[0])+sqr(x[1])),x[1]+x[2]});
    print("vf:",vf);
    a=FloatDPApproximationVector({4,3,0},dp);
    auto vfa=vf(a);
    print("vf(a):",vfa);
    b=FloatDPBoundsVector({4,3,0},dp);
    auto vfb=evaluate(vf,b);
    print("evaluate(vf,b):",vfb);

    auto dsf0=derivative(sf,0);
    print("derivative(sf,0):",dsf0);

    auto p0=FloatDPApproximationMultivariatePolynomial::coordinate(2,0,dp);
    auto p1=FloatDPApproximationMultivariatePolynomial::coordinate(2,1,dp);
    print("p1:",p1);
    auto a3=FloatDPApproximation(3,dp);
    auto a5=FloatDPApproximation(5,dp);
    auto q=p0*(p0+p1*a3)+a5;
    print("q:",q);
    //! [Function demonstration]
}

void calculus_demonstration() {
    //! [Calculus demonstration]

    // Create a box to act as the domain of a Taylor function
    auto dom=BoxDomainType({{4,7},{1,6},{-1,+1}});
    print("dom:",dom);

    // Create a sweeper to control the accuracy of a Taylor function
    SweeperDP swp=ThresholdSweeperDP(dp,1e-8);
    swp=GradedSweeperDP(dp,6);
    print("swp:",swp);

    // Create the scalar Taylor model representing a constant function with given value on the domain dom
    auto tc=ValidatedScalarMultivariateTaylorFunctionModelDP::constant(dom,4.2_dec,swp);
    print("tc:",tc);

    // Create the scalar Taylor model representing the function x1 on the domain dom
    auto t1=ValidatedScalarMultivariateTaylorFunctionModelDP::coordinate(dom,1,swp);
    print("t1:",t1);

    // Create all Taylor variables on the domain dom
    auto t=ValidatedScalarMultivariateTaylorFunctionModelDP::identity(dom,swp);
    print("t[0]:",t[0]);
    print("t[1]:",t[1]);
    print("t[2]:",t[2]);

    // Make shorthands for the variable names
    ValidatedScalarMultivariateTaylorFunctionModelDP tx=t[0];
    ValidatedScalarMultivariateTaylorFunctionModelDP ty=t[1];
    ValidatedScalarMultivariateTaylorFunctionModelDP tz=t[2];
    print("tx:",tx);
    print("ty:",ty);
    print("tz:",tz);

    //Create a ValidatedScalarMultivariateTaylorFunctionModelDP from a Function
    auto p=EffectiveScalarMultivariateFunction::coordinate(3,0);
    auto tp=ValidatedScalarMultivariateTaylorFunctionModelDP(dom,p,swp);
    print("p:",p);
    print("tp:",tp);

    // The domain D of tx.
    print("tx.domain():",tx.domain());
    // A not-necessarily tight over-approximation to p(D)+/-e.
    print("tx.codomain():",tx.codomain());
    // An over-approximation to p(D)+/-e.
    print("tx.range():",tx.range());

    // Define some constants
    auto n=int(3);
    auto c=FloatDP(0.75_x,dp);
    auto b=FloatDPBounds(0.625_x,0.875_x,dp);

    // Arithmetic on Taylor models
    +tx; -tx; tx+ty; tx-ty; tx*ty; tx/ty;

    // Mixed arithmetic
    tx+c; tx-c; tx*c; tx/c;
    c+tx; c-tx; c*tx; c/tx;

    tx+b; tx-b; tx*b; tx/b;
    b+tx; b-tx; b*tx; b/tx;

    // Lattice functions
    min(tx,ty); max(tx,ty); abs(tx);

    // Univariate arithmetical functions
    neg(tx);
    rec(tx);
    sqr(tx);
    pow(tx,n);

    // Algebraic and transcendental functions
    sqrt(tx);
    exp(tx); log(tx);
    sin(tx), cos(tx), tan(tx/2);

    // Inplace operations
    tx+=ty; tx-=ty;
    tx+=c; tx-=c; tx*=c; tx/=c;
    tx+=b; tx-=b; tx*=b; tx/=b;

    // Set coordinate functions tx0, tx1
    dom=BoxDomainType({{-1,1},{-1,1}});
    auto tx0=ValidatedScalarMultivariateTaylorFunctionModelDP::coordinate(dom,0,swp);
    auto tx1=ValidatedScalarMultivariateTaylorFunctionModelDP::coordinate(dom,1,swp);

    auto tf=ValidatedScalarMultivariateTaylorFunctionModelDP(1+2*tx0+3*tx0*tx1+tx0*tx0);
    // Compute an antiderivative of f with respect to x[0] whose value is zero at the midpoint of the domain of x[0]
    auto Itf0=antiderivative(tf,0);
    // Compute an antiderivative of f with respect to x[1] whose value is zero when x[1]=c
    auto Itf1=antiderivative(tf,1,c);
    print("antiderivative(tf,0):",Itf0);
    print("antiderivative(tf,1):",Itf1);

    // Restrict to a subdomain
    auto subdom=BoxDomainType({{-0.125_x,0.125_x},{0.5_x,0.75_x}});
    auto w=tx0*tx1;
    auto rw=restriction(w,subdom);
    print("restriction(w,subdom):",rw);

    // Join tf and tg into a VectorTaylorFunction join(f,g)(x)=[f(x),g(x)]
    auto tg=ValidatedVectorMultivariateTaylorFunctionModelDP({exp(tx0)*cos(tx1)});
    auto tfg=join(tf,tg);
    print("join(tf,tg):",tfg);

    // Combine tf and tg into a VectorTaylorFunction combine(f,g)(x,y)=[f(x),g(y)]
    tfg=combine(tf,tg);
    print("combine(tf,tg):",tfg);

    // Function composition
    dom=BoxDomainType({{4,7},{1,6},{-1,1}});
    auto th=ValidatedVectorMultivariateTaylorFunctionModelDP::identity(dom,swp);
    auto cd=th.codomain();

    auto f=EffectiveScalarMultivariateFunction::coordinate(3,0);
    auto g=EffectiveVectorMultivariateFunction::zeros(2,3);

    tf=ValidatedScalarMultivariateTaylorFunctionModelDP(cd,f,swp);
    tg=ValidatedVectorMultivariateTaylorFunctionModelDP(cd,g,swp);

    // Compose an scalar multivariate function and a vector Taylor function
    auto fth=compose(f,th);
    print("compose(f,th):",fth);

    // Compose a vector multivariate function and a vector Taylor function
    auto gth=compose(g,th);
    print("compose(g,th):",gth);

    // Compose a scalar and a vector Taylor function
    assert(subset(th.codomain(),tf.domain()));
    auto tfh=compose(tf,th);
    print("compose(tf,th):",tfh);

    // Compose two vector Taylor functions
    assert(subset(th.codomain(),tg.domain()));
    auto tgh=compose(tg,th);
    print("compose(tg,th):",tgh);

    // Consistency and refinement checking
    dom=BoxDomainType({{-1,+1},{-1,+1}});
    auto tf1=tx0+tx0*tx0/2;
    auto tf2=tx0+FloatDPBounds(-1,1,dp);
    print("tf1:",tf1);
    print("tf2:",tf1);
    auto ic=inconsistent(tf1,tf2);
    print("inconsistent(tf1,tf2):",ic);
    auto rf=refines(tf1,tf2);
    print("refines(tf1,tf2):",rf);
    tf=refinement(tf1,tf2);
    print("refinement(tf1,tf2):",tf);
    //! [Calculus demonstration]
}

int main(int argc, const char* argv[]) {
    if (not CommandLineInterface::instance().acquire(argc,argv)) return -1;

    function_demonstration();
    print();
    calculus_demonstration();
}



