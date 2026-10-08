/***************************************************************************
 *            solvers/linear_programming.cpp
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

#include "function/functional.hpp"
#include "config.hpp"

#include "utility/tuple.hpp"
#include "numeric/numeric.hpp"
#include "algebra/vector.hpp"
#include "algebra/matrix.hpp"
#include "algebra/diagonal_matrix.hpp"
#include "function/affine.hpp"
#include "solvers/linear_programming.hpp"

#include "utility/macros.hpp"
#include "conclog/logging.hpp"

using namespace ConcLog;

namespace Ariadne {

typedef Vector<UpperIntervalType> UpperIntervalVectorType;
typedef Matrix<UpperIntervalType> UpperIntervalMatrixType;

// Return r[i]=x[i]-c for i=1,...,n
template<class X> Vector<X> esub(const Vector<X>& x, const X& c);
// Return r[i]=x[i]*y[i] for i=1,...,n
template<class X> Vector<X> emul(const Vector<X>& x, const Vector<X>& y);
// Return r[i]=x[i]*y[i] for i=1,...,n
template<class X> Vector<X> ediv(const Vector<X>& x, const Vector<X>& z);
// Return r[i]=x[i]*y[i]+z for i=1,...,n
template<class X> Vector<X> efma(const Vector<X>& x, const Vector<X>& y, const X& z);
// Return r[i]=1/y[i] for i=1,...,n
template<class X> Vector<X> erec(const Vector<X>& v);

template<class X> Bool is_nan(Vector<X> const& v);

// Compute R=ADA^T for diagonal D
template<class XA, class XD> Matrix<ArithmeticType<XA,XD>> adat(const Matrix<XA>& A, const Vector<XD>& D);

template<class X, class XX> XX compute_mu(const Vector<X>& xl, const Vector<X>& xu,
                                          const Vector<XX>& x, const Vector<XX>& zl, const Vector<XX>& zu);


// Return r[i]=x[i]-c for i=1,...,n
template<class X> Vector<X> esub(const Vector<X>& x, const X& c) {
    Vector<X> r(x.size(),x.zero_element());
    for(SizeType i=0; i!=r.size(); ++i) {
        r[i]=x[i]-c;
    }
    return r;
}

// Return r[i]=x[i]*y[i] for i=1,...,n
template<class X> Vector<X> emul(const Vector<X>& x, const Vector<X>& y) {
    Vector<X> r(x.size(),x.zero_element()*y.zero_element());
    for(SizeType i=0; i!=r.size(); ++i) {
        r[i]=x[i]*y[i];
    }
    return r;
}

// Return r[i]=x[i]*y[i] for i=1,...,n
template<class X> Vector<X> ediv(const Vector<X>& x, const Vector<X>& z) {
    Vector<X> r(x.size(),x.zero_element());
    for(SizeType i=0; i!=r.size(); ++i) {
        r[i]=x[i]/z[i];
    }
    return r;
}

// Return r[i]=x[i]*y[i]+z for i=1,...,n
template<class X> Vector<X> efma(const Vector<X>& x, const Vector<X>& y, const X& z) {
    Vector<X> r(x.size(),nul(z));
    for(SizeType i=0; i!=r.size(); ++i) {
        r[i]=x[i]*y[i]+z;
    }
    return r;
}

// Return r[i]=1/y[i] for i=1,...,n
template<class X> Vector<X> erec(const Vector<X>& v) {
    Vector<X> r(v.size(),v.zero_element());
    for(SizeType i=0; i!=r.size(); ++i) {
        r[i]=rec(v[i]);
    }
    return r;
}

template<class X> Bool is_nan(Vector<X> const& v) {
    for(SizeType i=0; i!=v.size(); ++i) {
        if(is_nan(v[i].raw())) { return true; }
    }
    return false;
}

// Return r[i]=x[i]*y[i] for i=1,...,n
template<class X> Bool emul_gtr(const Vector<X>& x, const Vector<X>& z, X const& c) {
    for(SizeType i=0; i!=x.size(); ++i) {
        if (not (decide(x[i]*z[i]>c || x[i]==inf))) { return false; }
    }
    return true;
}


// Compute R=ADA^T for diagonal D
template<class XA, class XD> Matrix<ArithmeticType<XA,XD>> adat(const Matrix<XA>& A, const Vector<XD>& D)
{
    typedef ArithmeticType<XA,XD> XR;
    Matrix<XR> R(A.row_size(),A.row_size(),A.zero_element()*D.zero_element()*A.zero_element());
    ARIADNE_ASSERT(D.size()==A.column_size());
    for(SizeType i=0; i!=A.row_size(); ++i) {
        for(SizeType k=0; k!=A.column_size(); ++k) {
            XR ADik=A[i][k]*D[k];
            for(SizeType j=0; j!=A.row_size(); ++j) {
                R[i][j]+=ADik*A[j][k];
            }
        }
    }
    return R;
}

template<class X, class XX> XX compute_mu(const Vector<X>& xl, const Vector<X>& xu,
                                          const Vector<XX>& x, const Vector<XX>& zl, const Vector<XX>& zu)
{
    const SizeType n=x.size();
    XX mu=x.zero_element();
    for(SizeType i=0; i!=n; ++i) {
        if(xl[i]!=-inf) { mu += ((x[i]-xl[i])*zl[i]); }
        if(xu[i]!=+inf) { mu += ((xu[i]-x[i])*zu[i]); }
    }
    mu /= (2*n);
    return mu;
}



inline FloatDPBounds mul_val(FloatDP x1, FloatDP x2) { return FloatDPBounds(mul(down,x1,x2),mul(up,x1,x2)); }
inline Vector<FloatDP> const& cast_exact(Vector<FloatDP> const& v) { return reinterpret_cast<Vector<FloatDP>const&>(v); }
inline Matrix<FloatDP> const& cast_exact(Matrix<FloatDP> const& A) { return reinterpret_cast<Matrix<FloatDP>const&>(A); }

OutputStream& operator<<(OutputStream& os, LinearProgramStatus lps) {
    switch (lps) {
        case LinearProgramStatus::INDETERMINATE_FEASIBILITY: return os << "INDETERMINATE_FEASIBILITY";
        case LinearProgramStatus::PRIMAL_FEASIBLE: return os << "PRIMAL_FEASIBLE";
        case LinearProgramStatus::DUAL_FEASIBLE: return os << "DUAL_FEASIBLE";
        case LinearProgramStatus::PRIMAL_DUAL_FEASIBLE: return os << "PRIMAL_DUAL_FEASIBLE";
        case LinearProgramStatus::DEGENERATE_FEASIBILITY: return os << "DEGENERATE_FEASIBILITY";
        default: return os << "UNKNOWN_FEASIBILITY_STATUS";
    }
}

ValidatedKleenean InteriorPointSolver::
validate_feasibility(const Vector<X>& xl, const Vector<X>& xu,
                     const Matrix<X>& A, const Vector<X>& b,
                     const Vector<AX>& ax, const Vector<AX>& ay) const
{
    CONCLOG_SCOPE_CREATE;

    Vector<VX> x = cast_exact(ax);
    Vector<VX> y = cast_exact(ay);

    X zero=A.zero_element();

    const SizeType n=A.column_size();

    // x should be an approximate solution to Ax=b
    // Use the fact that for any x, x'=(x + A^T (AA^T)^{-1}(b-Ax)) satisfies Ax'=0
    Vector<VX> e = b-A*x;

    Matrix<VX> S = A*transpose(A);

    try {
        Vector<VX> d = transpose(A) * solve(S,e);
        x += d;
    } catch (SingularMatrixException const& err) {
        return indeterminate;
    }

    CONCLOG_PRINTLN("[x] = "<<x);
    ValidatedKleenean result=true;
    for(SizeType i=0; i!=n; ++i) {
        if(x[i].lower().raw()<=xl[i].raw() || x[i].upper().raw()>=xu[i].raw()) {
            result = indeterminate; break;
        }
    }

    if(definitely(result)) { return result; }

    // If yb - max(yA,0) xu + min(yA,0) xl > 0, then problem is infeasible
    // Evaluate lower bound for yb - max(z,0) xu + min(z,0) xl
    Vector<VX> z=transpose(A) * y;
    VX mx = zero;
    for(SizeType i=0; i!=y.size(); ++i) {
        mx += (b[i]*y[i]);
    }
    for(SizeType i=0; i!=x.size(); ++i) {
        X zil=cast_exact(z[i].lower());
        X ziu=cast_exact(z[i].upper());
        if(ziu>0) { mx -= ziu * xu[i]; }
        if(zil<0) { mx += zil * xl[i]; }
    }
    if(definitely(mx>0)) { return false; }

    return indeterminate;
}


auto
InteriorPointSolver::minimise(const Vector<X>& c,
                              const Vector<X>& xl, const Vector<X>& xu,
                              const Matrix<X>& A, const Vector<X>& b) const
    -> Tuple< AX, Vector<AX>, Vector<AX> >
{
    CONCLOG_SCOPE_CREATE;
    CONCLOG_PRINTLN("A="<<A<<", b="<<b<<", c="<<c);
    CONCLOG_PRINTLN("xl="<<xl<<", xu="<<xu);

    X zero=A.zero_element();

    const SizeType m = b.size();
    const SizeType n = c.size();
    Vector<AX> y(m,zero);
    Vector<AX> x(n,zero);
    Vector<AX> zl(n,zero);
    Vector<AX> zu(n,zero);
    for(SizeType i=0; i!=n; ++i) {
        if(xl[i]==-inf) {
            if(xu[i]==+inf) { x[i]=0.0_x; } else { x[i] = xu[i]-1.0_x; }
        } else {
            if(xu[i]==+inf) { x[i]=xl[i]+1.0_x; } else { ARIADNE_ASSERT(xl[i]<xu[i]); x[i] = (xl[i]+xu[i])/2; }
        }
        if(xl[i]==-inf) { zl[i] = 0.0_x; } else { zl[i] = 1.0_x; }
        if(xu[i]==+inf) { zu[i] = 0.0_x; } else { zu[i] = 1.0_x; }
    }

    LinearProgramStatus status = LinearProgramStatus::INDETERMINATE_FEASIBILITY;
    while(status == LinearProgramStatus::INDETERMINATE_FEASIBILITY) {
        status = this->_minimisation_step(c,xl,xu,A,b, x,y,zl,zu);
    }

    do {
        this->_minimisation_step(c,xl,xu,A,b, x,y,zl,zu);
    } while((dot(c,x)-dot(y,b)).raw()>1e-4_pr);

    // Todo: check for optimality
    return make_tuple(dot(c,x),x,y);
}



auto InteriorPointSolver::
hotstarted_minimise(const Vector<X>& c,
                    const Vector<X>& xl, const Vector<X>& xu,
                    const Matrix<X>& A, const Vector<X>& b,
                    Vector<AX>& x, Vector<AX>& y, Vector<AX>& zl, Vector<AX>& zu) const
    -> Tuple< AX, Vector<AX>, Vector<AX> >
{
    CONCLOG_SCOPE_CREATE;

    ARIADNE_ASSERT(A.column_size()==c.size());
    ARIADNE_ASSERT(A.column_size()==xl.size());
    ARIADNE_ASSERT(A.column_size()==xu.size());
    ARIADNE_ASSERT(A.row_size()==b.size());
    ARIADNE_ASSERT(A.column_size()==x.size());
    ARIADNE_ASSERT(A.row_size()==y.size());
    ARIADNE_ASSERT(A.column_size()==zl.size());
    ARIADNE_ASSERT(A.column_size()==zu.size());

    const ExactDouble maxerror=1e-3_pr;
    const CounterType maxsteps=10;

    AX cx=dot(c,x);
    AX yb=dot(y,b);
    ARIADNE_ASSERT(decide(yb<=cx));

    CONCLOG_PRINTLN("xl="<<xl<<" xu="<<xu<<" A="<<A<<" b="<<b<<" c="<<c);
    CONCLOG_PRINTLN("x="<<x<<" y="<<y<<" z="<<zl<<" zu="<<zu);

    CounterType steps=0;
    while(steps<maxsteps && decide((cx-yb)>maxerror)) {
        this->_minimisation_step(c,xl,xu,A,b, x,y,zl,zu);
        ++steps;
    }

    return make_tuple(dot(c,x),x,y);

}


ValidatedKleenean
InteriorPointSolver::
feasible(const Vector<X>& xl, const Vector<X>& xu,
         const Matrix<X>& A, const Vector<X>& b) const
{
    CONCLOG_SCOPE_CREATE;
    CONCLOG_PRINTLN("A="<<A<<", b="<<b);
    CONCLOG_PRINTLN("xl="<<xl<<", xu="<<xu);

    X zero=A.zero_element();

    const SizeType m = A.row_size();
    const SizeType n = A.column_size();
    Vector<X> c(n,zero);
    Vector<AX> y(m,zero);
    Vector<AX> x(n,zero);
    Vector<AX> zl(n,zero);
    Vector<AX> zu(n,zero);
    for(SizeType i=0; i!=n; ++i) {
        if(xl[i]==-inf) {
            if(xu[i]==+inf) { x[i]=0.0_x; } else { x[i] = xu[i]-1.0_x; }
        } else {
            if(xu[i]==+inf) { x[i]=xl[i]+1.0_x; } else { ARIADNE_ASSERT(xl[i]<=xu[i]); x[i] = (xl[i]+xu[i])/2; }
        }
        if(xl[i]==-inf) { zl[i] = 0.0_x; } else { zl[i] = 1.0_x; }
        if(xu[i]==+inf) { zu[i] = 0.0_x; } else { zu[i] = 1.0_x; }
    }
    Vector<VX> ivlx(n,zero);
    for(SizeType i=0; i!=n; ++i) {
        ivlx[i]=VX(xl[i],xu[i]);
    }

    const ExactDouble THRESHOLD = 1e-8_pr;
    CounterType step=0;
    while(step++<24) {
        LinearProgramStatus result=this->_feasibility_step(xl,xu,A,b, x,y,zl,zu);
//        std::cerr<<step<<": result="<<result<<"\n";
        if(result==LinearProgramStatus::PRIMAL_DUAL_FEASIBLE || result==LinearProgramStatus::PRIMAL_FEASIBLE) {
            ValidatedKleenean validated_feasible=this->validate_feasibility(xl,xu,A,b, x,y);
            if(definitely(validated_feasible)) { return true; }
        }
        Vector<X> yv=cast_exact(y);
        VX yb=dot(yv,b);
        // NOTE: Must compute y*A first, as A*X may give NaN.
        VX yAX = dot( transpose(A) * yv, ivlx );
        if(inconsistent(yb,yAX)) { return false; }
        if(result==LinearProgramStatus::DEGENERATE_FEASIBILITY) { CONCLOG_PRINTLN("Degenerate"); return indeterminate; }
        if(decide(compute_mu(xl,xu, x,zl,zu)<THRESHOLD) ) { CONCLOG_PRINTLN("Threshold"); return indeterminate; }
    }
    return indeterminate;
}












LinearProgramStatus InteriorPointSolver::
_minimisation_step(const Vector<X>& c, const Vector<X>& xl, const Vector<X>& xu, const Matrix<X>& A, const Vector<X>& b,
                   Vector<AX>& x, Vector<AX>& y, Vector<AX>& zl, Vector<AX>& zu) const
{
    CONCLOG_SCOPE_CREATE;
    CONCLOG_PRINTLN("x="<<x<<", y="<<y<<", zl="<<zl<<", zu="<<zu);

    typename X::PrecisionType pr;

    static const ExactDouble gamma=0.00390625_x; // 1/256
    static const ExactDouble sigma=0.125_x;
    static const ExactDouble scale=0.75_x;


    const SizeType m=A.row_size();
    const SizeType n=A.column_size();

    Vector<AX> dx(n,pr),dy(m,pr),dzl(n,pr),dzu(n,pr);
    Vector<AX> nx,ny,nzl,nzu;
    Vector<AX> rx(m,pr),ry(n,pr),rzl(n,pr),rzu(n,pr);
    Matrix<AX> S(m,m,pr);
    DiagonalMatrix<AX> Xl(xl), Xu(xu), Xa(x), Zl(zl), Zu(zu);

    AX mu = compute_mu(xl,xu, x,zl,zu) * sigma;
    mu=1.0_x;
    CONCLOG_PRINTLN("mu="<<mu);

    // rx = Ax-b; ry=yA+zl-zu-c; rzl=(x-xl).zl-mu; rzu=(xu-x).zu-mu.
    rx=A*x-b;
    ry=transpose(A)*y+(zl-zu)-c;
    for(SizeType i=0; i!=n; ++i) {
        if(xl[i]!=-inf) { rzl[i] = (x[i]-xl[i])*zl[i] - mu; } else { rzl[i]=0.0_x; }
        if(xu[i]!=+inf) { rzu[i] = (xu[i]-x[i])*zu[i] - mu; } else { rzu[i]=0.0_x; }
    }
    CONCLOG_PRINTLN("rx="<<rx<<", ry="<<ry<<", rzl="<<rzl<<", rzu="<<rzu);

    // A dx = rx;  AT dy + dzl - dzu = ry;  Zl dx + (X-Xl) dzl = rzl; -Zu dx + (Xu-X) dzu = rzu;

    // Derivation
    //   AT dy + (rzl-Zl*dx)/(X-Xl) - (rzu+Zu*dx)/(Xu-X) = ry
    //   AT dy - ( Zl/(X-Xl) + Zu/(Xu-X) ) dx = ry - rzl/(X-Xl) + rzu/(Xu-X)
    //   AT dy - Dinv dx = ryz
    //   D AT dy - dx = D ryz
    //      dx = D (AT dy - ryz)
    //   A D AT dy - A dx = A D ryz
    //   S dy - rx = A D ryz
    //    S dy = A D ryz + rx

    //   ryz = ( ry - rzl/(X-Xl) + rzu/(Xu-X) )
    //   D = 1/( Zu/(Xu-X) + Zl/(X-Xl) )
    DiagonalMatrix<AX> D(erec(ediv(zu,xu-x)+ediv(zl,x-xl)));
    Vector<AX> ryz = ry - ediv(rzl,Vector<AX>(x-xl)) + ediv(rzu,Vector<AX>(xu-x));
    S=adat(A,D.diagonal());
    CONCLOG_PRINTLN("S="<<S<<", inverse(S)="<<inverse(S));

    // dzl = (rzl - Zl dx) / (X-Xl)
    // dzu = (rzu + Zu dx) / (Xu-X)
    // dx = D (AT dy - ryz)
    // (A D AT) dy = rx + A D ryz

    try {
        dy = solve(S, Vector<AX>( rx + A * (D * ryz) ) );
    } catch(SingularMatrixException const& err) {
        return LinearProgramStatus::DEGENERATE_FEASIBILITY;
    }

//    std::cerr<<"dy="<<dy<<", is_nan(dy)="<<is_nan(dy)<<"\n";
    if(is_nan(dy)) { return LinearProgramStatus::DEGENERATE_FEASIBILITY; }
    dx = D * Vector<AX>(transpose(A)*dy - ryz);
    dzl = Vector<AX>(rzl-Zl*dx)/(Xa-Xl);
    dzu = Vector<AX>(rzu+Zu*dx)/(Xu-Xa);
    CONCLOG_PRINTLN("dx="<<dx<<" dy="<<dy<<" dzl="<<dzl<<" dzu="<<dzu);

    CONCLOG_PRINTLN_AT(1,"A*dx="<<(A*dx)<<" AT*dy+dzl-dzu="<<(transpose(A)*dy+dzl-dzu)<<" Zl*dx+(Xa-Xl)*dzl="<<(Zl*dx+(Xa-Xl)*dzl)<<" -Zu*dx+(Xu-Xa)*dzu="<<(-(Zu*dx)+(Xu-Xa)*dzu));
    CONCLOG_PRINTLN_AT(1,"A*dx-rx="<<(A*dx-rx)<<" AT*dy+dzl-dzu-ry="<<(transpose(A)*dy+dzl-dzu-ry)<<" Zl*dx+(Xa-Xl)*dzl-rzl="<<(Zl*dx+(Xa-Xl)*dzl-rzl)<<" -Zu*dx+(Xu-Xa)*dzu-rzu="<<(-(Zu*dx)+(Xu-Xa)*dzu-rzu));

    // Try to enforce feasibility or dual feasibility
    AX alphax(1.0_x,pr);
    nx=x-dx;
    while ( decide( !emul_gtr(nx-xl,zl,gamma*mu) || !emul_gtr(xu-nx,zu,gamma*mu) ) ) {
        alphax=alphax*scale;
        nx=(x-alphax*dx);
        if(decide(alphax<gamma*mu/4096)) { return LinearProgramStatus::DEGENERATE_FEASIBILITY; }
    }

    AX alphaz(1.0_x,pr);
    nzl=zl-dzl; nzu=zu-dzu;
    while ( decide( !emul_gtr(nx-xl,nzl,gamma*mu) || !emul_gtr(xu-nx,nzu,gamma*mu) ) ) {
        alphaz=alphaz*scale;
        nzl=(zl-alphaz*dzl);
        nzu=(zu-alphaz*dzu);
        if(decide(alphaz<gamma*mu/4096)) { return LinearProgramStatus::DEGENERATE_FEASIBILITY; }
    }
    ny=(y-alphaz*dy);
    CONCLOG_PRINTLN("alphax="<<alphax<<" nx="<<nx<<" alphaz="<<alphaz<<" ny="<<ny<<" nzl="<<nzl<<" nzu="<<nzu);

    x=nx; y=ny; zl=nzl; zu=nzu;
    CONCLOG_PRINTLN("cx="<<dot(c,x)<<" yb="<<dot(y,b)<<" Ax-b="<<(A*x-b)<<" yA+(zl-zu)-c="<<(transpose(A)*y+(zl-zu)-c));

    if(decide(alphax==1.0_x && alphaz==1.0_x)) { return LinearProgramStatus::PRIMAL_DUAL_FEASIBLE; }
    if(decide(alphax==1.0_x)) { return LinearProgramStatus::PRIMAL_FEASIBLE; }
    if(decide(alphaz==1.0_x)) { return LinearProgramStatus::DUAL_FEASIBLE; }
    return LinearProgramStatus::INDETERMINATE_FEASIBILITY;
}



LinearProgramStatus InteriorPointSolver::
_feasibility_step(const Vector<X>& xl, const Vector<X>& xu, const Matrix<X>& A, const Vector<X>& b,
                  Vector<AX>& x, Vector<AX>& y, Vector<AX>& zl, Vector<AX>& zu) const
{
    Vector<X> c(x.size(),X(0.0_x,dp));
    return this->_minimisation_step(c,xl,xu,A,b, x,y,zl,zu);
}


} // namespace Ariadne

