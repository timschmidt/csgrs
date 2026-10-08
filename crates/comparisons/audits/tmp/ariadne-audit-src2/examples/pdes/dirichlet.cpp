/***************************************************************************
 *            dirichlet.cpp
 *
 *  Copyright  2018-20  Pieter Collins
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 3 of the License, or
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

#include "ariadne.hpp"

#include "function/taylor_function.hpp"

#define PRINT(expr) { std::cout << #expr << ": " << (expr) << std::endl; }
#define PRINTLN { std::cout << std::endl; }

namespace Ariadne {

template<class F> using ValidatedScalarTaylorFunctionModel = ScaledFunctionPatch<ValidatedTaylorModel<F>>;
using ValidatedScalarTaylorFunctionModelMP = ScaledFunctionPatch<ValidatedTaylorModelMP>;
using ApproximateScalarTaylorFunctionModelDP = ScaledFunctionPatch<ApproximateTaylorModelDP>;

template<class F> Bounds<F> integral(ValidatedScalarTaylorFunctionModel<F> const& tf, Real const& a, Real const& b) {
    ARIADNE_ASSERT(tf.argument_size()==1);
    auto pr=tf.properties().precision();
    Vector<Bounds<F>> ax({a},pr);
    Vector<Bounds<F>> bx({b},pr);
    auto integral_tf = antiderivative(tf, 0);
    return integral_tf(bx)-integral_tf(ax);
}


template<class F> ValidatedScalarTaylorFunctionModel<F> fourier_function(List<Bounds<F>> as, ValidatedScalarTaylorFunctionModel<F> const& tz) {
    ValidatedScalarTaylorFunctionModel<F> tx=ValidatedScalarTaylorFunctionModel<F>::coordinate(tz.domain(), 0, tz.properties());
    auto tr=tz;
    tr+=as[0];
    for(SizeType i=1; i!=as.size(); ++i) {
        tr+=as[i]*cos(i*pi*tx);
    }
    return tr;
}


template<class F> List<Bounds<F>> fourier_coefficients(ValidatedScalarTaylorFunctionModel<F> const& tf, SizeType n) {
    ARIADNE_ASSERT(tf.argument_size()==1);
    auto pr=tf.properties().precision();
    ValidatedScalarTaylorFunctionModel<F> tx=ValidatedScalarTaylorFunctionModel<F>::coordinate(tf.domain(), 0, tf.properties());
    List<Bounds<F>> as(n, Bounds<F>(pr));
    as[0]=integral(tf,0,1);
    for(SizeType i=1; i!=n; ++i) {
        as[i]=2*integral(tf*cos(i*pi*tx),0,1);
    }
    return as;
}

template<class F> Bounds<F> fourier_norm_square(List<Bounds<F>> const& as) {
    Bounds<F> r=sqr(as[0]);
    for(SizeType i=1; i!=as.size(); ++i) {
        r+=sqr(as[i])/2;
    }
    return r;
}

template<class F> Void gnuplot(String filename, ValidatedScalarTaylorFunctionModel<F> const& tf) {
    ARIADNE_ASSERT(tf.argument_size()==1);
    std::ofstream ofs(filename);
    auto pr=tf.properties().precision();
    IntervalDomainType dom=tf.domain()[0];
    Nat m=100;
    F ax(Dyadic(dom.lower_bound()),pr);
    F bx(Dyadic(dom.upper_bound()),pr);
    Vector<Bounds<F>> x(1u,pr);
    for(SizeType i=0; i<=m; ++i) {
        x[0] = ((m-i)*ax+i*bx)/m;
        Bounds<F> tfx=unchecked_evaluate(tf,x);
//        ofs << (i!=0?";":"") << x[0].get_d() << "," << tfx.lower()<<","<<tfx.upper();
        ofs << x[0].get_d() << " " << tfx.lower()<<" "<<tfx.upper() << "\n";
    }
    ofs.close();
}

//! \brief Solve the Dirichlet boundary value problem u<sub>xx</sub>(x) + u(x) = f(x); u(0)=u(1)=0.
//! \details A solution is given by given as a sum of a particular solution, plus a solution to the homogeneous equation to correct the boundary values.
//! A particular solution \f$u_p\f$ is given by \f$u_p(x)=\sum_{k=0}^{\infty}b_k\cos(k\pi x)\f$ where \f$b_k=a_k/(1-\pi^2k^2)\f$ and \f$f(x)=\sum_{k=0}^{\infty}a_k\cos(k\pi x)\f$ is the Fourier series of \f$f\f$.
//! This solution has \f$\alpha=u_p(0)=\sum_{k=0}^{\infty}a_k\f$ and \f$\beta=u_p(1)=\sum_{k=0}^{\infty}(-1)^ka_k\f$.
//!
//! The homogeneous equation has general solution \f$u_h(x)=A\cos(x)+B\sin(x)\f$ with \f$u_h(0)=A\f$ and \f$u_h(1)=A\cos(1)+B\sin(1)\f$.
//! The boundary conditions imply \f$A=-\alpha\f$ and \f$B=(\alpha\cos(1)-\beta)/\sin(1)\f$.
//!
//! A computed solution uses only finitely many terms of the Fourier series for \f$u\f$.
//! If the last tert is that in \f$\cos(n\pi x)\f$, then the error in the particular solution is \f$\sum_{k=n}^{\infty} a_k/(k^2\pi^2-1)\,\cos(k\pi x)\f$,
//! which has absolute value less than \f$\sum_{k=n+1}^{\infty}|a_k|/(k^2\pi^2-1)\f$.
//! By the Cauchy-Schwarz inequality, this is bounded by \f$\bigl(\sum{k=n+1}^\infty}{a_k}^2\bigr)^{1/2} \cdot \bigl(\sum_{k=n+1}^{\infty}1/(k^2\pi^2-1)^2\bigr\)^{1/2}\f$.
//! The constant (in \f$f\f$) factor can be bounded by over-approximating the sum by an integral, yielding
//!   \f[ \| u|_n - u \|_\infty \leq \frac{1}{3^{1/2} \pi^2 n^{3/2}} \f]
//! A weaker, but easier bound is given by \fC_n ||f-f_n||_2\f$, where \f$C_n\f$ is given by
//!   \f[ \sum_{k=n}^{\infty}1/(k^2\pi^2-1)^2 = \frac{1}{\pi^2} \sum_{k=n}^{\infty}1/(k^2-1/\pi^2)^2 \leq \frac{1}{\pi^2} \sum_{k=n}^{\infty}1/(k^2-1/2^2)
//!           = \frac{1}{\pi^2} \sum_{k=n}^{\infty} \frac{1}{k-1/2}-\frac{1}{k+1/2} = \frac{1}{\pi^2} \frac{1}{n-1/2} = C_n^2 . \f]
void dirichlet(EffectiveScalarMultivariateFunction f) {

    ARIADNE_ASSERT(f.argument_size()==1);
    PRINT(f)

    EffectiveScalarMultivariateFunction x=EffectiveScalarMultivariateFunction::coordinate(EuclideanDomain(1),0);

    // Set precision parameters of function model
    DoublePrecision pr;
    // MultiplePrecision pr(128);
    typedef RawFloat<decltype(pr)> F;
    double eps=1e-14;
    ThresholdSweeper<F> swp(pr,eps);

    // Define left and right endpoints of interval, and representative test point
    Vector<F> a({0.0_x},pr);
    Vector<F> b({1.0_x},pr);
    auto xv=(a+2*b)/3;

    // Define domain of problem, and zero and coordinate functions.
    IntervalDomainType dom(0,1);
        BoxDomainType bxdom(1u,dom);
    ValidatedScalarTaylorFunctionModel<F> tz(bxdom, swp);
    ValidatedScalarTaylorFunctionModel<F> tx=ValidatedScalarTaylorFunctionModel<F>::coordinate(bxdom, 0, swp);

    // Convert f to a polynomial function model
    ValidatedScalarTaylorFunctionModel<F> tf(bxdom, f, swp);
    PRINT(tf);
    gnuplot("dirichlet-f.dat",tf);

    // Set the number of terms to use for the Fourier series
    SizeType n=10;
    Array<FloatDPBounds> sums(n,FloatDPBounds(pr));

    // Compute the Fourier coefficients of f
    List<Bounds<F>> as=fourier_coefficients(tf,n);

    // Compute the square of the L2 norm of f
    Bounds<F> sqrnormL2f = integral(tf*tf,0,1);
    PRINT(sqrnormL2f);
    // Compute the square of the L2 norm of the Fourier sum
    Bounds<F> sqrnormL2p=fourier_norm_square(as);
    PRINT(sqrnormL2p);
    //for(SizeType i=0; i!=n; ++i) { PRINT(make_tuple(i,as[i],sums[i])); }
    PRINT(sqrnormL2p/sqrnormL2f);
    auto sqrerrorL2=sqrnormL2f - sqrnormL2p;
    PRINT(sqrerrorL2);
    auto errorL2=sqrt(sqrerrorL2);
    PRINT(errorL2);
    auto relerrorL2=sqrt(1-sqrnormL2p/sqrnormL2f);
    PRINT(relerrorL2);
    PRINTLN;

    // Compute the Fourier coefficients of the particular solution
    List<FloatDPBounds> cs(n,FloatDPBounds(dp));
    for(SizeType i=0; i!=n; ++i) {
        cs[i] = as[i]/(1-sqr(i*pi));
    }

    PRINT(as);
    PRINT(cs);
    PRINTLN;

    // Compute the particular solution
    auto tv = fourier_function(cs,tz);
    PRINT(tv);

    // Compute the uniform error of the particular solution
    auto ve=mag(sqrerrorL2/(3*n*n*n*pow(pi,4)));
    PRINT(ve);

    tv=tv + Bounds<F>(-ve,+ve);
    PRINTLN;




    // Compute the solution to the homogeneous equation needed to match the boundary conditions
    Bounds<F> tva=tv(a);
    Bounds<F> tvb=tv(b);
    auto tw=tva*cos(tx)+(tvb-tva*cos(b[0]))/sin(b[0])*sin(tx);

//    PRINT(tv(a));
//    PRINT(tv(b));
//    PRINT(tw(a));
//    PRINT(tw(b));

    // Compute the solution tu
    auto tu=tv-tw;
    PRINT(tu.error());
    PRINT(tu(a));
    PRINT(tu(b));
    PRINTLN

    // Plot the solution
    gnuplot("dirichlet-u.dat",tu);

    // Compute the error of the midpoint function
    auto ctu=tu;
    ctu.clobber();
    auto ddctu=derivative(derivative(ctu,0),0);
    auto te=ddctu+ctu-tf;
    PRINT(te.model());
    PRINT(norm(te.model()));
    PRINT(te(xv));

    gnuplot("dirichlet-e.dat",te);

}

} // namespace Ariadne


int main() {
    using namespace Ariadne;

    RealVariable x("x");
    RealSpace args({x});

    // Define the right-hand side function f in u''+u=f
    auto f=make_function(args, 1/sqrt(2-x*x)-3/4_q);

    dirichlet(f);
}
