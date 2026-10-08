/***************************************************************************
 *            profile_arithmetic.cpp
 *
 *  Copyright 2008--17  Pieter Collins
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

#include <cassert>
#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <algorithm>

#include "utility/stopwatch.hpp"

#if defined HAVE_FENV_H && defined HAVE_GMPXX_H

#include <fenv.h>
#include <gmpxx.h>

// Needed for MAC OS X, where Nat is not defined
typedef unsigned Int Nat;

// Machine epsilon, approximately 2.2e-16;
const double eps=1./(1<<26)/(1<<26);

#if defined __GNUC__ && ( defined __i386__ || defined __x86_64 || defined _M_IX86 || defined _M_X86 )
    #if ( defined __SSE_MATH__ &&  defined __SSE2__ )
        #define ARIADNE_SSE_ROUNDING
        #define ARIADNE_C99_ROUNDING
    #elif __GNUC__ >= 5 || ( __GNUC__ == 4 && __GNUC_MINOR__ >= 3 )
        #define ARIADNE_GCC_ROUNDING
    #else
        #define ARIADNE_C99_ROUNDING
    #endif
#else
    #define ARIADNE_BOOST_ROUNDING
    #define ARIADNE_C99_ROUNDING
#endif


typedef Int rounding_mode_t;
static const rounding_mode_t round_nearest = FE_TONEAREST;
static const rounding_mode_t round_down = FE_DOWNWARD;
static const rounding_mode_t round_up = FE_UPWARD;

rounding_mode_t grnd=FE_UPWARD;

inline double max(double x, double y) { return x>=y ? x : y; }
inline double min(double x, double y) { return x<=y ? x : y; }
inline double abs(double x) { return x>=0 ? x : -x; }
inline mpq_class abs(mpq_class x) { return x>=0 ? x : mpq_class(-x); }


#define MCW_EM          0x003f          /* interrupt exception masks   */
#define EM_INVALID      0x0001          /*   invalid                   */
#define EM_DENORMAL     0x0002          /*   denormal                  */
#define EM_ZERODIVIDE   0x0004          /*   zero divide               */
#define EM_OVERFLOW     0x0008          /*   overflow                  */
#define EM_UNDERFLOW    0x0010          /*   underflow                 */
#define EM_INEXACT      0x0020          /*   inexact (precision)       */

#define MCW_IC          0x1000          /* infinity control            */
#define IC_AFFINE       0x1000          /*   affine                    */
#define IC_PROJECTIVE   0x0000          /*   projective                */

#define MCW_RC          0x0c00          /*  rounding control           */
#define RC_CHOP         0x0c00          /*    chop                     */
#define RC_UP           0x0800          /*    up                       */
#define RC_DOWN         0x0400          /*    down                     */
#define RC_NEAR         0x0000          /*    near                     */

#define MCW_PC          0x0300          /*  precision control          */
#define PC_24           0x0000          /*    24 bits                  */
#define PC_53           0x0200          /*    53 bits                  */
#define PC_64           0x0300          /*    64 bits                  */

/**** math coprocessor default control word and rounding modes (80x87) */

#define CW_DEFAULT\
        (IC_AFFINE      | RC_NEAR       | PC_64         |\
         EM_DENORMAL    | EM_OVERFLOW   | EM_UNDERFLOW  | EM_INEXACT)

#define CW_ROUND_CHOP   ((CW_DEFAULT & ~MCW_RC) | RC_CHOP)
#define CW_ROUND_UP     ((CW_DEFAULT & ~MCW_RC) | RC_UP)
#define CW_ROUND_DOWN   ((CW_DEFAULT & ~MCW_RC) | RC_DOWN)
#define CW_ROUND_NEAR   ((CW_DEFAULT & ~MCW_RC) | RC_NEAR)

//unsigned short ROUND_UP      = CW_ROUND_UP;
//unsigned short ROUND_DOWN    = CW_ROUND_DOWN;
//unsigned short ROUND_NEAREST = CW_ROUND_NEAR;
unsigned short ROUND_UP      = 2943;
unsigned short ROUND_DOWN    = 1919;
unsigned short ROUND_NEAREST = 895;

inline rounding_mode_t get_control_word() { rounding_mode_t grnd; asm volatile ("fstcw %0" : "=m" (grnd) ); return grnd; }

#if defined ARIADNE_C99_ROUNDING

inline Void set_round(up,) { fesetround(FE_UPWARD);  }
inline Void set_round(down,) { fesetround(FE_DOWNWARD);  }
inline Void set_round_nearest() { fesetround(FE_TONEAREST);  }

inline Void FloatDP::set_rounding_mode(rounding_mode_t rnd) { fesetround(rnd); }
inline rounding_mode_t FloatDP::get_rounding_mode() { return fegetround(); }

inline Void c_set_round_nearest() { fesetround(FE_TONEAREST); }

#elif defined ARIADNE_GCC_ROUNDING

inline Void set_round(up,) { asm volatile ("fldcw ROUND_UP"); }
inline Void set_round(down,) { asm volatile ("fldcw ROUND_DOWN"); }
inline Void set_round_nearest() { asm volatile ("fldcw ROUND_NEAREST"); }

inline Void c_set_round_nearest() { fesetround(FE_TONEAREST); }

//inline Void FloatDP::set_rounding_mode(rounding_mode_t rnd) { asm volatile ("fldcw (%0)" : : "r" (rnd) ); }
//inline rounding_mode_t FloatDP::get_rounding_mode() { rounding_mode_t rnd asm volatile ("fldcw (%0)" : : "r" (rnd) ); return rnd; }

inline rounding_mode_t FloatDP::get_rounding_mode() { rounding_mode_t grnd; asm volatile ("fstcw %0" : "=m" (grnd) ); return grnd; }
inline Void FloatDP::set_rounding_mode(rounding_mode_t rnd) { asm volatile ("fldcw %0" : : "m" (rnd) ); }

#elif defined ARIADNE_GCC_MACRO_ROUNDING

#define set_round(up,)           asm("fldcw ROUND_UP")
#define set_round(down,)         asm("fldcw ROUND_DOWN")
#define set_round_nearest()      asm("fldcw ROUND_NEAREST")

inline Void FloatDP::set_rounding_mode(rounding_mode_t rnd) { fesetround(rnd); }
inline rounding_mode_t FloatDP::get_rounding_mode() { return fegetround(); }

inline Void c_set_round_nearest() { fesetround(FE_TONEAREST); }

#endif

double rndm() {
    double w=double(1<<16)*(1<<15);
    Int r1=Int(2*Nat(std::rand()))/2;
    unsigned Int r2=std::rand();
    unsigned Int r3=std::rand();
    double r=((double(r3)/w+double(r2))/w+double(r1))/(1<<28);
    //std::cerr<<"r="<<r<<" r1,2,3="<<r1<<","<<r2<<","<<r3<<"\n";
    return r;
}

inline double add_rnd(double x, double y) {
    return x+y;
}

inline double add_opp(double x, double y) {
    volatile double t=-x;
    t=t-y;
    return -t;
}

inline double mul_opp(double x, double y) {
    volatile double t=-x;
    t=t*y;
    return -t;
}

inline Void acc_rnd(double& r, double x, double y) {
    double m=x*y;
    std::cerr<<"  acc_rnd: r="<<r<<" m="<<m;
    r+=x*y;
    std::cerr<<" r="<<r<<"\n";
}

inline Void acc_opp(double& r, double x, double y) {
    volatile double t=-x;
    t=t*y;
    std::cerr<<"  acc_opp: r="<<r<<" m="<<-t;
    t=t-r;
    r=-t;
    std::cerr<<" r="<<r<<"\n";
    return;
    std::cerr<<"  acc_opp: r="<<r;
    r=add_opp(r,mul_opp(x,y));
    std::cerr<<" m="<<mul_opp(x,y)<<" r="<<r<<"\n";
    return;
}



Void dot_lu_rat(double& l, double& u, SizeType n, const double* x, const double* y);
Void dot_md_rat(mpq_class& m, SizeType n, const double* x, const double* y);
Void dot_lu_std(double& l, double& u, SizeType n, const double* x, const double* y);
Void dot_lu_opp(double& l, double& u, SizeType n, const double* x, const double* y);
Void dot_lu_ivl(double& l, double& u, SizeType n, const double* x, const double* y);
Void dot_lu_ord(double& l, double& u, SizeType n, const double* x, const double* y);
Void dot_mr_std(double& m, double& r, SizeType n, const double* x, const double* y);
Void dot_mr_mid(double& m, double& r, SizeType n, const double* x, const double* y);
Void dot_mr_csy(double& m, double& r, SizeType n, const double* x, const double* y);

Void add_mr_std(double& e, SizeType n, double* r, const double* x, const double* y);
Void add_mr_buf(double& e, SizeType n, double* r, const double* x, const double* y);
Void add_mr_csy(double& e, SizeType n, double* r, const double* x, const double* y);

Void scal_mr_std(double& e, SizeType n, double* r, const double* x, const double& c);
Void scal_mr_csy(double& e, SizeType n, double* r, const double* x, const double& c);


Void profile_dot(Int n, Int nn) {
    double* x=new double[n];
    double* y=new double[n];

    for(Int i=0; i!=n; ++i) {
        x[i]=rndm();
        y[i]=rndm();
    }

    double m=0,r=0,l=0,u=0;

    Stopwatch<Milliseconds> sw; double t=0;

    double ql=0; double qu=0;
    dot_lu_rat(ql,qu,n,x,y);
    std::cout<<"lu_rat:        e="<<(qu-ql)/2<<" l="<<ql<<" u="<<qu<<std::endl;
    assert(ql<=qu);

    mpq_class qm=0;
    dot_md_rat(qm,n,x,y);

    for(Int i=0; i!=nn; ++i) {
        l=0; u=0;
        dot_lu_std(l,u,n,x,y);
    }
    sw.click();
    t=sw.elapsed();
    std::cout<<"lu_std: t(us)="<<std::setprecision(5)<<t
             <<std::setprecision(20)<<" e="<<(u-l)/2<<" l="<<l<<" u="<<u<<std::endl;
    assert(l<=ql && qu<=u);

    sw.reset();
    for(Int i=0; i!=nn; ++i) {
        l=0; u=0;
        dot_lu_opp(l,u,n,x,y);
    }
    sw.click();
    t=sw.elapsed();
    std::cout<<"lu_opp: t(us)="<<std::setprecision(5)<<t<<std::setprecision(20)
             <<" e="<<(u-l)/2<<" l="<<l<<" u="<<u<<std::endl;
    assert(l<=ql && qu<=u);

    sw.reset();
    for(Int i=0; i!=nn; ++i) {
        l=0; u=0;
        dot_lu_ivl(l,u,n,x,y);
    }
    sw.click();
    t=sw.elapsed();
    std::cout<<"lu_ivl: t(us)="<<std::setprecision(5)<<t<<std::setprecision(20)
             <<" e="<<(u-l)/2<<" l="<<l<<" u="<<u<<std::endl;
    assert(l<=ql && qu<=u);



    sw.reset();
    for(Int i=0; i!=nn; ++i) {
        m=0; r=0;
        dot_mr_std(m,r,n,x,y);
    }
    sw.click();
    t=sw.elapsed();
    std::cout<<"mr_std: t(us)="<<std::setprecision(5)<<t<<std::setprecision(20)
             <<" r="<<r<<" m="<<m<<" e="<<mpq_class(abs(mpq_class(m)-qm)).get_d()<<std::endl;
    assert(abs(mpq_class(m)-qm)<=r);

    sw.reset();
    for(Int i=0; i!=nn; ++i) {
        m=0; r=0;
        dot_mr_mid(m,r,n,x,y);
    }
    sw.click();
    t=sw.elapsed();
    std::cout<<"mr_mid: t(us)="<<std::setprecision(5)<<t<<std::setprecision(20)
             <<" r="<<r<<" m="<<m<<" e="<<mpq_class(abs(mpq_class(m)-qm)).get_d()<<std::endl;
    assert(abs(mpq_class(m)-qm)<=r);

    sw.reset();
    for(Int i=0; i!=nn; ++i) {
        m=0; r=0;
        dot_mr_csy(m,r,n,x,y);
    }
    sw.click();
    t=sw.elapsed();
    std::cout<<"mr_csy: t(us)="<<std::setprecision(5)<<t<<std::setprecision(20)
             <<" r="<<r<<" m="<<m<<" e="<<mpq_class(abs(mpq_class(m)-qm)).get_d()<<std::endl;
    assert(abs(mpq_class(m)-qm)<=r);


    delete[] x;
    delete[] y;
}


Void profile_add(Int n, Int nn) {
    double* x=new double[n];
    double* y=new double[n];
    double* z=new double[n];

    for(Int i=0; i!=n; ++i) {
        x[i]=rndm();
        y[i]=rndm();
    }

    double r=0;

    boost::timer tm; double t=0;

    sw.reset();
    for(Int i=0; i!=nn; ++i) {
        r=0;
        add_mr_std(r,n,z,x,y);
    }
    sw.click();
    t=sw.elapsed();
    std::cout<<"add_mr_std: t="<<std::setprecision(5)<<t<<std::setprecision(20)
             <<" r="<<r<<std::endl;

    sw.reset();
    for(Int i=0; i!=nn; ++i) {
        r=0;
        add_mr_buf(r,n,z,x,y);
    }
    sw.click();
    t=sw.elapsed();
    std::cout<<"add_mr_buf: t="<<std::setprecision(5)<<t<<std::setprecision(20)
             <<" r="<<r<<std::endl;

    sw.reset();
    for(Int i=0; i!=nn; ++i) {
        r=0;
        add_mr_csy(r,n,z,x,y);
    }
    sw.click();
    t=sw.elapsed();
    std::cout<<"add_mr_csy: t="<<std::setprecision(5)<<t<<std::setprecision(20)
             <<" r="<<r<<std::endl;
}


 Void profile_scal(Int n, Int nn) {
    double* x=new double[n];
    double* z=new double[n];

    double c=rndm();
    for(Int i=0; i!=n; ++i) {
        x[i]=rndm();
    }


    double r=0;

    boost::timer tm; double t=0;

    sw.reset();
    for(Int i=0; i!=nn; ++i) {
        r=0;
        scal_mr_std(r,n,z,x,c);
    }
    sw.click();
    t=sw.elapsed();
    std::cout<<"scal_mr_std: t="<<std::setprecision(5)<<t<<std::setprecision(20)
             <<" r="<<r<<std::endl;

    sw.reset();
    for(Int i=0; i!=nn; ++i) {
        r=0;
        scal_mr_csy(r,n,z,x,c);
    }
    sw.click();
    t=sw.elapsed();
    std::cout<<"scal_mr_csy: t="<<std::setprecision(5)<<t<<std::setprecision(20)
             <<" r="<<r<<std::endl;
}


Void test_rounding(volatile double p, volatile double q)
{
    std::cout<<"Testing correct rounding\n";
    rounding_mode_t fcw=get_control_word();
    std::cout<<"Initial control word="<<fcw<<"\n";
    rounding_mode_t rnd=FloatDP::get_rounding_mode();
    std::cout<<"Initial rounding mode="<<rnd<<"\n";

    set_round(up,);
    std::cout<<"Up rounding mode="<<FloatDP::get_rounding_mode()<<"\n";
    volatile double xu=p/q;
    std::cout<<"  Computed "<<p<<"/"<<q<<"="<<xu<<std::endl;

    set_round(down,);
    std::cout<<"Down rounding mode="<<FloatDP::get_rounding_mode()<<"\n";
    volatile double xl=p/q;
    std::cout<<"  Computed "<<p<<"/"<<q<<"="<<xl<<std::endl;

    set_round_nearest();
    std::cout<<"Nearest rounding mode="<<FloatDP::get_rounding_mode()<<"\n";
    volatile double xn=p/q;
    std::cout<<"  Computed "<<p<<"/"<<q<<"="<<xn<<std::endl;

    std::cout<<p<<"/"<<q<<" ~ "<<xn<<std::endl;
    std::cout<<xl<<" < "<<p<<"/"<<q<<" < "<<xu<<std::endl;
    FloatDP::set_rounding_mode(rnd);
    std::cout<<"Restored rounding mode="<<FloatDP::get_rounding_mode()<<"\n";
}

Int main(Int argc, const char* argv[]) {
    std::cout<<std::setprecision(20);
    std::cerr<<std::setprecision(20);
    //srand((unsigned)time(0));

    Int n=(1<<10);
    Int nn=(1<<12);
    if(argc>1) {
        n=(1<<atoi(argv[1]));
    }
    if(argc>2) {
        nn=(1<<atoi(argv[2]));
    }
    std::cout<<"\nTries="<<n<<"\nVector size="<<nn<<"\n"<<std::endl;

    profile_scal(n,nn);
    profile_add(n,nn);
    profile_dot(n,nn);
    return 0;

    double* x=new double[n];
    double* y=new double[n];
    double* z=new double[n];

    for(Int i=0; i!=n; ++i) {
        x[i]=rndm();
        y[i]=rndm();
        z[i]=rndm();
    }

    double* w=new double[n];
    double* v=new double[n];
    for(Int i=0; i!=n; ++i) {
        v[i]=z[i];
        w[i]=z[i];
    }

    Int nnn=std::min(n,4);
    for(Int i=0; i!=nnn; ++i) {
        FloatDP::set_rounding_mode(round_down);
        double r=add_rnd(x[i],y[i]);
        std::cerr<<"add_rnd_down: x="<<x[i]<<" y="<<y[i]<<" r="<<r<<"\n";
        FloatDP::set_rounding_mode(round_up);
        double o=add_opp(x[i],y[i]);
        std::cerr<<"add_opp_up:   x="<<x[i]<<" y="<<y[i]<<" r="<<o<<"\n";
        assert(r==o);
    }
    FloatDP::set_rounding_mode(round_nearest);

    for(Int i=0; i!=nnn; ++i) {
        std::cerr<<"x="<<x[i]<<" y="<<y[i]<<" z="<<z[i]<<"\n";
        FloatDP::set_rounding_mode(round_down);
        acc_rnd(z[i],x[i],y[i]);
        FloatDP::set_rounding_mode(round_up);
        acc_opp(w[i],x[i],y[i]);
        std::cerr<<" r="<<z[i]<<" o="<<w[i]<<"\n\n\n";
        assert(z[i]==w[i]);
    }
    FloatDP::set_rounding_mode(round_nearest);

}














Void dot_lu_rat(double& l, double& u, SizeType n, const double* x, const double* y) {
    FloatDP::set_rounding_mode(round_nearest);
    assert(l==u);
    mpq_class r=l;
    for(SizeType i=0; i!=n; ++i) {
        r+=mpq_class(x[i])*mpq_class(y[i]);
    }
    //mpf_class::set_precision(128);
    //std::cerr<<"r="<<r<<"~"<<mpf_class(r)<<"\n";
    mpq_class a(r.get_d());
    double d=a.get_d();
    assert(mpq_class(d)==a);
    assert(d==a);
    double e=abs(d)*(eps/16);
    assert(e>0);
    l=d;
    Int m=0;
    while(l<r) {
        ++m;
        l=d+m*e;
    }
    while(l>r) {
        --m;
        l=d+m*e;
    }
    ++m;
    u=d+m*e;

    assert(mpq_class(l)<=r);
    assert(mpq_class(u)>=r);
}


Void dot_md_rat(mpq_class& m, SizeType n, const double* x, const double* y) {
    set_round_nearest();
    for(SizeType i=0; i!=n; ++i) {
        m+=mpq_class(x[i])*mpq_class(y[i]);
    }
}

Void dot_lu_ivl(double& l, double& u, SizeType n, const double* x, const double* y) {
    for(SizeType i=0; i!=n; ++i) {
        set_round(up,);
        u+=x[i]*y[i];
        set_round(down,);
        l+=x[i]*y[i];
    }
    set_round_nearest();
}

Void dot_lu_std(double& l, double& u, SizeType n, const double* x, const double* y) {
    set_round(up,);
    for(SizeType i=0; i!=n; ++i) {
        u+=x[i]*y[i];
        //std::cerr<<" i="<<i<<" u="<<u<<"\n";
    }
    set_round(down,);
    for(SizeType i=0; i!=n; ++i) {
        l+=x[i]*y[i];
        //std::cerr<<" i="<<i<<" l="<<l<<"\n";
    }
    set_round_nearest();
}

Void dot_lu_opp(double& l, double& u, SizeType n, const double* x, const double* y) {
    set_round(up,);
    register volatile double uu=u;
    register volatile double ll=-l;
    register volatile double t;
    for(SizeType i=0; i!=n; ++i) {
        uu+=x[i]*y[i];
        t=-x[i];
        t=t*y[i];
        ll+=t;
        //l+=t*y[i];
        //std::cerr<<" i="<<i<<" l="<<(-l)<<" u="<<u<<"\n";
    }
    u=uu;
    l=-ll;
    set_round_nearest();
}

Void dot_lu_opp2(double& l, double& u, SizeType n, const double* x, const double* y) {
    set_round(up,);
    register volatile double t;
    for(SizeType i=0; i!=n; ++i) {
        u+=x[i]*y[i];
        t=(-x[i]);
        t=t*y[i];
        t=t-l;
        l=-t;
        //std::cerr<<" i="<<i<<" l="<<(-l)<<" u="<<u<<"\n";
    }
    set_round_nearest();
}

Void dot_lu_ord(double& l, double& u, SizeType n, const double* x, const double* y) {

}

Void dot_mr_std(double& m, double& r, SizeType n, const double* x, const double* y) {
    set_round(down,);
    volatile double l=m-r;
    for(SizeType i=0; i!=n; ++i) {
        l+=x[i]*y[i];
    }
    set_round(up,);
    volatile double u=m+r;
    for(SizeType i=0; i!=n; ++i) {
        u+=x[i]*y[i];
    }
    set_round_nearest();
    m=(u+l)/2;
    set_round(up,);
    r=max(u-m,m-l);
    set_round_nearest();
}

Void dot_mr_mid(double& m, double& r, SizeType n, const double* x, const double* y) {
    volatile double a=m;
    for(SizeType i=0; i!=n; ++i) {
        a+=x[i]*y[i];
    }
    set_round(down,);
    volatile double l=m-r;
    for(SizeType i=0; i!=n; ++i) {
        l+=x[i]*y[i];
    }
    set_round(up,);
    volatile double u=m+r;
    for(SizeType i=0; i!=n; ++i) {
        u+=x[i]*y[i];
    }
    m=a;
    r=r+max(u-m,m-l);
    set_round_nearest();
}

Void dot_mr_csy(double& m, double& r, SizeType n, const double* x, const double* y) {
    double e=0;
    for(SizeType i=0; i!=n; ++i) {
        double p=x[i]*y[i];
        m+=p;
        e+=abs(p)*(eps/2);
        e+=abs(m)*(eps/2);
    }
    r+=e;
}




Void add_mr_std(double& e, SizeType n, double* r, const double* x, const double* y)
{
    for(SizeType i=0; i!=n; ++i) {
        volatile double a=x[i]+y[i];
        set_round(down,);
        volatile double l=x[i]+y[i];
        set_round(up,);
        volatile double u=x[i]+y[i];
        r[i]=a;
        e+=(u-l)/2;
        set_round_nearest();
    }
}

Void add_mr_buf(double& e, SizeType n, double* r, const double* x, const double* y)
{
    double* z=new double[n];
    for(SizeType i=0; i!=n; ++i) {
        z[i]=x[i]+y[i];
    }
    set_round(up,);
    for(SizeType i=0; i!=n; ++i) {
        volatile double u=x[i]+y[i];
        volatile double t=-x[i];
        volatile double ml=t-y[i];
        e+=(u+ml)/2;
    }
    set_round_nearest();
    delete[] z;
}

Void add_mr_csy(double& e, SizeType n, double* r, const double* x, const double* y)
{
    double d=0;
    for(SizeType i=0; i!=n; ++i) {
        r[i]=x[i]+y[i];
        d+=abs(r[i]);
    }
    d=d*(1+eps*n/2)*eps/2;
    e=e+d;
    e=e*(1+eps/2);
}


Void scal_mr_std(double& e, SizeType n, double* r, const double* x, const double& c)
{
    for(SizeType i=0; i!=n; ++i) {
        volatile double a=x[i]*c;
        set_round(down,);
        volatile double l=x[i]*c;
        set_round(up,);
        volatile double u=x[i]*c;
        r[i]=a;
        e+=(u-l)/2;
        set_round_nearest();
    }
}

Void scal_mr_csy(double& e, SizeType n, double* r, const double* x, const double& c)
{
    double d=0;
    for(SizeType i=0; i!=n; ++i) {
        r[i]=x[i]*c;
        d+=abs(r[i]);
    }
    d=d*(1+eps*n/2)*eps/2;
    e=e+d;
    e=e*(1+eps/2);
}

#else // No fenv.h or no gmpxx.h

Int main() {
    std::cout << "SKIPPED " << std::endl;
}

#endif
