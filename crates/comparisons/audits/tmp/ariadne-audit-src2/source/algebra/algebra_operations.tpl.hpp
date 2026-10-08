/***************************************************************************
 *            algebra_operations.tpl.hpp
 *
 *  Copyright  2011-20  Pieter Collins
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

#include "utility/exceptions.hpp"
#include "algebra/operations.hpp"

#include "algebra/series.hpp"
#include "function/taylor_series.hpp"

namespace Ariadne {

template<class A> struct IsNormedAlgebra : False { };
template<class A> struct IsGradedAlgebra : False { };

template<class A> concept ANormedAlgebra = IsNormedAlgebra<A>::value;
template<class A> concept AGradedAlgebra = IsNormedAlgebra<A>::value;

template<AGradedAlgebra A> A
compose(const Series<typename A::NumericType>& x, const A& y)
{
    Nat d=y.degree();

    A w=y - y.value();
    A r=y.create();
    r+=x[d];
    for(Nat n=1; n<=d; ++n) {
        r=r*w;
        r+=x[d-n];
    }
    return r;
}


template<class X> class TaylorSeries;

template<ANormedAlgebra A> A compose(const TaylorSeries<FloatDPBounds>& ts, const A& tv, double eps)
{
    //std::cerr<<"_compose(TaylorSeries,A,ErrorTag)\n";
    //std::cerr<<"\n  ts="<<ts<<"\n  tv="<<tv<<"\n";
    FloatDP& vref=const_cast<FloatDP&>(tv.value());
    FloatDP vtmp=vref;
    vref=0;
    A r(tv.argument_size());
    r+=ts[ts.degree()];
    for(Nat i=1; i<=ts.degree(); ++i) {
        //std::cerr<<"    r="<<r<<std::endl;
        r=r*tv;
        r+=ts[ts.degree()-i];
        r.sweep(eps);
    }
    //std::cerr<<"    r="<<r<<std::endl;
    r+=ts.error();
    //std::cerr<<"    r="<<r<<std::endl;
    vref=vtmp;
    return r;
}

template<ANormedAlgebra A> A compose(const TaylorSeries<FloatDPBounds>& ts, const A& tm)
{
    return _compose(ts,tm,tm.tolerance());
}


// Compose using the Taylor formula directly. The final term is the Taylor series computed
// over the range of the series. This method tends to suffer from blow-up of the
// truncation error
template<ANormedAlgebra A> A _compose1(const AnalyticFunction& fn, const A& tm, double eps)
{
    static const Nat DEGREE=18;
    static const ExactDouble TRUNCATION_ERROR=1e-8_pr;
    Nat d=DEGREE;
    FloatDP c=tm.value();
    FloatDPBounds r=tm.range();
    Series<FloatDPBounds> centre_series=fn.series(FloatDPBounds(c));
    Series<FloatDPBounds> range_series=fn.series(r);

    FloatDPUpperBound truncation_error_estimate=mag(range_series[d])*pow(mag(r-c),d);
    if(truncation_error_estimate.raw()>TRUNCATION_ERROR) {
        ARIADNE_WARN("Truncation error estimate "<<truncation_error_estimate
                     <<" is greater than maximum allowable truncation error "<<TRUNCATION_ERROR);
    }

    A x=tm-c;
    A res(tm.argument_size(),tm.accuracy_ptr());
    res+=range_series[d];
    for(Nat i=0; i!=d; ++i) {
        //std::cerr<<"i="<<i<<" r="<<res<<"\n";
        res=centre_series[d-i-1]+x*res;
        res.sweep(eps);
    }
    //std::cerr<<"i="<<d<<" r="<<res<<"\n";
    return res;
}

// Compose using the Taylor formula with a constant truncation error. This method
// is usually better than _compose1 since there is no blow-up of the trunction
// error. The radius of convergence of this method is still quite low,
// typically only half of the radius of convergence of the power series itself
template<ANormedAlgebra A> A _compose2(const AnalyticFunction& fn, const A& tm, double eps)
{
    static const Nat DEGREE=20;
    static const ExactDouble TRUNCATION_ERROR=1e-8_pr;
    Nat d=DEGREE;
    FloatDP c=tm.value();
    FloatDPBounds r=tm.range();
    Series<FloatDPBounds> centre_series=fn.series(FloatDPBounds(c));
    Series<FloatDPBounds> range_series=fn.series(r);

    //std::cerr<<"c="<<c<<" r="<<r<<" r-c="<<r-c<<" e="<<mag(r-c)<<"\n";
    //std::cerr<<"cs[d]="<<centre_series[d]<<" rs[d]="<<range_series[d]<<"\n";
    //std::cerr<<"cs="<<centre_series<<"\nrs="<<range_series<<"\n";
    FloatDPError truncation_error=mag(range_series[d]-centre_series[d])*pow(mag(r-c),d);
    //std::cerr<<"te="<<truncation_error<<"\n";
    if(truncation_error.raw()>TRUNCATION_ERROR) {
        ARIADNE_WARN("Truncation error estimate "<<truncation_error
                 <<" is greater than maximum allowable truncation error "<<TRUNCATION_ERROR);
    }

    A x=tm-c;
    A res(tm.argument_size(),tm.accuracy_ptr());
    res+=centre_series[d];
    for(Nat i=0; i!=d; ++i) {
        res=centre_series[d-i-1]+x*res;
        res.sweep(eps);
    }
    res+=FloatDPBounds(-truncation_error,+truncation_error);
    return res;
}

template<class F> Error<F> error_bound(Bounds<F> const& b, F const& c) {
    return Error<F>(max(b.upper()-c,c-b.lower()));
}

template<class F> Error<F> error_bound(Bounds<F> const& b, Bounds<F> const& c) {
    return Error<F>(max(b.upper()-c.lower(),c.upper()-b.lower()));
}

// Compose using the Taylor formula with a constant truncation error. This method
// is usually better than _compose1 since there is no blow-up of the trunction
// error. This method is better than _compose2 since the truncation error is
// assumed at the ends of the intervals
template<ANormedAlgebra A> A _compose3(const AnalyticFunction& fn, const A& tm, FloatDP eps)
{
    static const Nat DEGREE=20;
    static const ExactDouble TRUNCATION_ERROR=1e-8_pr;
    Nat d=DEGREE;
    FloatDP c=tm.value();
    FloatDPBounds r=tm.range();
    Series<FloatDPBounds> centre_series=fn.series(FloatDPBounds(c));
    Series<FloatDPBounds> range_series=fn.series(r);

    //std::cerr<<"c="<<c<<" r="<<r<<" r-c="<<r-c<<" e="<<mag(r-c)<<"\n";
    //std::cerr<<"cs[d]="<<centre_series[d]<<" rs[d]="<<range_series[d]<<"\n";
    //std::cerr<<"cs="<<centre_series<<"\nrs="<<range_series<<"\n";
    FloatDPError se=error_bound(range_series[d],centre_series[d]);
    FloatDPError e=error_bound(r,c);
    FloatDPError p=pow(e,d);
    //std::cerr<<"se="<<se<<" e="<<e<<" p="<<p<<std::endl;
    // FIXME: Here we assume the dth derivative of f is monotone increasing
    FloatDP truncation_error=(se*p).raw();
    //std::cerr<<"te="<<truncation_error<<"\n";
    if(truncation_error>TRUNCATION_ERROR) {
        ARIADNE_WARN("Truncation error estimate "<<truncation_error
                 <<" is greater than maximum allowable truncation error "<<TRUNCATION_ERROR);
    }

    A x=tm;
    A res(tm.argument_size(),tm.accuracy_ptr());
    res+=centre_series[d];
    for(Nat i=0; i!=d; ++i) {
        res=centre_series[d-i-1]+x*res;
        //res.sweep(eps);
    }
    res+=FloatDPBounds(-truncation_error,+truncation_error);
    return res;
}


template<ANormedAlgebra A> A _compose(const AnalyticFunction& fn, const A& tm, FloatDP eps)
{
    return _compose3(fn,tm,eps);
}



///////////////////////////////////////////////////////////////////////////////

// Algebraic and trancendental functions
//   bounded domain (rec,sqrt,log,tan)
//   unbounded domain (exp,sin,cos)

namespace {
inline Int pow2(Nat k) { return 1<<k; }
inline Int powm1(Nat k) { return (k%2) ? -1 : +1; }
}


template<class A> A NormedAlgebraOperations<A>::apply(Sqrt, const A& x)
{
    // Use a special routine to minimise errors
    // Given range [rl,ru], rescale by constant a such that rl/a=1-d; ru/a=1+d
    auto tol=cast_exact(x.tolerance());
    auto avg=cast_exact(x.average());
    auto rad=cast_exact(x.radius());
    auto pr=avg.precision();

    if(avg<=rad) {
        ARIADNE_THROW(DomainException,"sqrt(A x)","x="<<x);
    }

    auto eps=mag(rad)/abs(avg);
    ARIADNE_DEBUG_ASSERT(decide(eps<1));

    Series<X> sqrt_series=Series<X>(Sqrt(),X(1,pr));
    Nat d=integer_cast<Nat>(log((1-eps)*tol)/log(eps)+1);
    
    auto trunc_err=pow(eps,d)/cast_positive(1-eps)*mag(sqrt_series[d]);
    ARIADNE_DEBUG_ASSERT(0<=trunc_err.raw());

    A y=x/avg-1;
    A z=x.create();
    z+=sqrt_series[d-1];
    for(Nat i=0; i!=d; ++i) {
        z=sqrt_series[d-i-1] + z * y;
    }
    z+=z.create_ball(trunc_err);
    z*=sqrt(avg);
    return z;
}

template<class A> A NormedAlgebraOperations<A>::apply(Rec, const A& x)
{
    // Use a special routine to minimise errors
    // Given range [rl,ru], rescale by constant a such that rl/a=1-d; ru/a=1+d
    auto tol=cast_exact(x.tolerance());
    auto avg=cast_exact(x.average());
    auto rad=cast_exact(x.radius());

    if(decide(rad>=abs(avg))) {
        ARIADNE_THROW(DivideByZeroException,"rec(A x)","x="<<x<<", avg="<<avg<<", rad="<<rad);
    }

    auto eps=mag(rad)/abs(avg);
    ARIADNE_DEBUG_ASSERT(decide(eps<1));

    // Compute the degree and truncation error
    Nat d=integer_cast<Nat>((log((1-eps)*tol)/log(eps))+1);
    auto te=pow(eps,d)/cast_positive(1-eps);

    A y=1-x/avg;
    A z=x.create();
    z+=(d%2?-1:+1);
    for(Nat i=0; i!=d; ++i) {
        z=1 + z * y;
    }

    z+=z.create_ball(te);
    z/=avg;
    return z;
}

template<class A> A NormedAlgebraOperations<A>::apply(Log, const A& x)
{
    // Use a special routine to minimise errors
    // Given range [rl,ru], rescale by constant a such that rl/a=1-d; ru/a=1+d
    auto tol=cast_exact(x.tolerance());
    auto avg=cast_exact(x.average());
    auto rad=cast_exact(x.radius());
    auto pr=avg.precision();

    if(avg<=rad) {
        ARIADNE_THROW(DomainException,"log(A x)","x="<<x);
    }

    auto eps=mag(rad)/abs(avg);
    ARIADNE_DEBUG_ASSERT(decide(eps<1));

    Nat d=integer_cast<Nat>(log((1-eps)*tol)/log(eps)+1);
    auto trunc_err=pow(eps,d)/cast_positive(1-eps)/d;

    A y=x/avg-1;
    A z=x.create();
    z+=X(d%2?-1:+1,pr)/d;
    for(Nat i=1; i!=d; ++i) {
        z=X((d-i)%2?+1:-1,pr)/(d-i) + z * y;
    }
    z=z*y;

    z+=z.create_ball(trunc_err);
    z+=log(static_cast<X>(avg));
    return z;
}

// Use special code to utilise exp(ax+b)=exp(x)^a*exp(b)
template<class A> A NormedAlgebraOperations<A>::apply(Exp, const A& x)
{
    auto avg=x.average();
    auto rad=x.radius();
    auto tol = cast_exact(x.tolerance());

    // Scale to unit interval
    Nat sfp=0; // A number such that 2^sfp>rad(x.range())
    while(decide(Dyadic(pow(two,static_cast<int>(sfp)))<rad)) { ++sfp; }
    Dyadic sf=pow(two,static_cast<int>(sfp));
    A y = (x-avg)/sf;
    auto yrad=rad*mag((avg-avg)+sf);

    // Find the required degree
    Nat deg = 0;
    auto term_mag=pow(yrad,0u);
    auto trunc_err=term_mag;
    do {
        ++deg;
        term_mag *= (yrad/deg);
        trunc_err = term_mag*(2u*deg+2u)/deg;
    } while(decide(trunc_err>tol));

    A z=x.create_constant(1);
    for(Nat i=0; i!=deg; ++i) {
        z/=(deg-i);
        z=y*z+1;
    }
    z+=z.create_ball(trunc_err);

    // Square r a total of sfp times
    for(Nat i=0; i!=sfp; ++i) {
        z=z*z;
    }

    // Multiply by exp(xv)
    z*=exp(avg);

    return z;
}

// Use special code to utilise sin(x+2pi)=sin(x)
// and that the power series is of the form x*f(x^2)
template<class A> A NormedAlgebraOperations<A>::apply(Sin, const A& x)
{
    // FIXME: Truncation error may be incorrect

    auto tol = cast_exact(x.tolerance());
    auto avg=x.average();
    auto rad=x.radius();
    Int n=integer_cast<Int>( round(avg/pi) );

    // Range reduce; use sin(x)=sin(x-2*n*pi)=sin((2*n+1)*pi-x)
    A y=(n%2) ? n*pi-x : x-n*pi;

    A s=sqr(y);

    // Find the required degree
    Nat deg = 0;
    auto term_mag=pow(rad,0u);
    auto trunc_err=term_mag;
    do {
        ++deg;
        term_mag *= (rad/deg);
        trunc_err = term_mag*(2u*deg+2u)/deg;
    } while(decide(trunc_err>tol));

    // Compute x(1-y/6+y^2/120-y^3/5040+... = x(1-y/6*(1-y/20*(1-y/42*...)
    A z=x.create_constant(1);
    for(DegreeType i=0; i!=deg; ++i) {
        z/=(-Int(2*(deg-i)*(2*(deg-i)+1)));
        z*=s;
        z+=1;
    }
    z=y*z;

    z+=z.create_ball(trunc_err);

    return z;
}

// Use special code to utilise sin(x+2pi)=sin(x)
// and that the power series is of the form f(x^2)
template<class A> A NormedAlgebraOperations<A>::apply(Cos, const A& x)
{
    auto tol = cast_exact(x.tolerance());
    auto avg=x.average();
    auto rad=x.radius();

    // Range reduce; use cos(x)=cos(x-2*n*pi)=-cos(x-pi)
    Int n=integer_cast<Int>( round(avg/pi) );
    A y=x-n*pi;
    int c=(n%2)?-1:+1; // If n is odd, take minus the usual series

    A s=sqr(y);

    // Find the required degree
    Nat deg = 0;
    auto term_mag=pow(rad,0u);
    auto trunc_err=term_mag;
    do {
        ++deg;
        term_mag *= (rad/deg);
        trunc_err=term_mag*(2u*deg+2u)/deg;
    } while(decide(trunc_err>tol));

    // Compute 1-y/2+y^2/24-y^3/720+... = (1-y/2*(1-y/12*(1-y/30*...)
    A z=x.create_constant(c);
    for(DegreeType i=0; i!=deg; ++i) {
        z/=(-Int(2*(deg-i)*(2*(deg-i)-1)));
        z*=s;
        z+=c;
    }

    z+=z.create_ball(trunc_err);

    return z;
}

template<class A> A NormedAlgebraOperations<A>::apply(Tan, const A& x)
{
    return sin(x)*rec(cos(x));
}

template<class A> A NormedAlgebraOperations<A>::apply(Asin, const A& x)
{
    ARIADNE_NOT_IMPLEMENTED;
/*
    static const Nat DEG=18;
    typedef typename A::NumericType X;
    FloatDP xavg = x.average();
    FloatDP xrad = x.radius();
    FloatDPBounds xrng = xavg + FloatDPBounds(-xrad,+xrad);
    return compose(TaylorSeries(DEG,&Series<X>::asin,xavg,xrng),x);
*/
}

template<class A> A NormedAlgebraOperations<A>::apply(Acos, const A& x)
{
    ARIADNE_NOT_IMPLEMENTED;
/*
    static const Nat DEG=18;
    typedef typename A::NumericType X;
    FloatDP xavg = x.average();
    FloatDP xrad = x.radius();
    FloatDPBounds xrng = xavg + FloatDPBounds(-xrad,+xrad);
    return compose(TaylorSeries(DEG,&Series<X>::acos,xavg,xrng),x);
*/
}

template<class A> A NormedAlgebraOperations<A>::apply(Atan, const A& x)
{
    ARIADNE_NOT_IMPLEMENTED;
/*
    static const Nat DEG=18;
    typedef typename A::NumericType X;
    FloatDP xavg = x.average();
    FloatDP xrad = x.radius();
    FloatDPBounds xrng = xavg + FloatDPBounds(-xrad,+xrad);
    return compose(TaylorSeries(DEG,&Series<X>::atan,xavg,xrng),x);
*/
}


} // namespace Ariadne
