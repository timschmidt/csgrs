/***************************************************************************
 *            dynamics/first_order_pde.cpp
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

#include "first_order_pde.hpp"

#include "numeric/floatdp.hpp"
#include "numeric/floatmp.hpp"
#include "numeric/float_bounds.hpp"
#include "algebra/expansion.inl.hpp"
#include "algebra/algebra.hpp"
#include "function/function.hpp"
#include "function/formula.hpp"
#include "function/taylor_model.hpp"
#include "function/domain.hpp"


namespace Ariadne {

template<class X> decltype(auto) sup_norm(Matrix<X> const& A) {
    auto z=A.zero_element();
    auto s=mag(z);
    for (SizeType i=0; i!=A.column_size(); ++i) {
        auto r=mag(z);
        for (SizeType j=0; j!=A.column_size(); ++j) {
            r+=mag(A[i][j]);
        }
        s=max(s,r);
    }
    return s;
}

template<class IVL> decltype(mag(declval<IVL>())) mag(Box<IVL> const& bx) {
    assert(bx.dimension()>0);
    auto r=mag(bx[0]);
    for(SizeType i=1; i!=bx.dimension(); ++i) {
        r=max(r,mag(bx[i]));
    }
    return r;
}

template<class X> decltype(auto) mag(Differential<X> const& d) {
    auto r=mag(d.value());
    for (auto term : d.expansion()) {
        r=max(r,mag(term.coefficient()));
    }
    return r;
}

template<class X> decltype(auto) mag(Vector<X> const& v) {
    auto r=mag(v.zero_element());
    for (SizeType i=0; i!=v.size(); ++i) {
        r=max(r,mag(v[i]));
    }
    return r;
}

decltype(auto) mag_range(EffectiveVectorMultivariateFunction const& f, UpperBoxType const& bx) {
    return mag(apply(f,bx));
}

template<class X0, class... XS> decltype(auto) maxs(X0 const& x0, XS const& ... xs) { return max(x0,maxs(xs...)); }
template<class X0> decltype(auto) maxs(X0 const& x0) { return x0; }



template<class X> Vector<Bounds<X>> multiaffine_interpolate(Tensor<2,Vector<Bounds<X>>> const& us, Vector<X> const& x) {
    assert(x.size()==2);
    SizeType two_pow_N = us.sizes()[0]-1;
    X const& x0=x[0];
    X const& x1=x[1];
    SizeType i0 = floor(Dyadic(x0)*two_pow_N).get_si(); if(i0==us.size(0)-1) { --i0; }
    SizeType i1 = floor(Dyadic(x1)*two_pow_N).get_si(); if(i1==us.size(1)-1) { --i1; }
    Bounds<X> a0=x0*two_pow_N-i0;
    Bounds<X> a1=x1*two_pow_N-i1;
    return us[i0][i1]+a0*(us[i0+1][i1]-us[i0][i1])+a1*(us[i0][i1+1]-us[i0][i1])+a0*(us[i0+1][i1+1]-us[i0+1][i1]-us[i0][i1+1]+us[i0][i1]);
}

template<class PR>
FirstOrderPDESolution<PR>
first_order_pde(FirstOrderPDE const& pde, EffectiveVectorMultivariateFunction const& phi0, PR pr) {
    Matrix<Real>const& rA=pde.A;
    Array<Matrix<Real>>const& rBs=pde.Bs;
    Array<DiagonalMatrix<Real>>const& rDs=pde.Ds;
    Array<Matrix<Real>>const& rTs=pde.Ts;
    EffectiveVectorMultivariateFunction const& f=pde.f;

    typedef RawFloat<PR> X;

    Rational tolerance = 1/3_q;
    Dyadic tmax=Integer(1)/(two^2);

    Tensor<3,X> T({2,3,4},X(pr));

    DimensionType n = rA.row_size(); // Number of unknown functions u
    DimensionType m = rBs.size(); // Number of state variables x

    ARIADNE_ASSERT(rA.row_size()==n);  ARIADNE_ASSERT(rA.column_size()==n);
    ARIADNE_ASSERT(rBs.size()==m);
    for (DimensionType i=0; i!=m; ++i) { ARIADNE_ASSERT(rBs[i].row_size()==n);  ARIADNE_ASSERT(rBs[i].column_size()==n); }
    ARIADNE_ASSERT(f.argument_size()==m+1); ARIADNE_ASSERT(f.result_size()==n);
    ARIADNE_ASSERT(phi0.argument_size()==m); ARIADNE_ASSERT(phi0.result_size()==n);

    ARIADNE_ASSERT(m==2);


    Matrix<Bounds<X>> A(rA,pr);
    Matrix<Bounds<X>> invA=inverse(A);
    Array<Matrix<Bounds<X>>> Bs={Matrix<Bounds<X>>(rBs[0],pr),Matrix<Bounds<X>>(rBs[1],pr)};
    auto B0=Bs[0], B1=Bs[1];

    // Compute error of finite difference scheme
    auto cndA = sup_norm(A)*sup_norm(invA);
    auto max_nrm = maxs(sup_norm(A),sup_norm(B0),sup_norm(B1),sup_norm(invA*B0*invA*B0),sup_norm(invA*B1*invA*B1),sup_norm(invA*B0*invA*B1-invA*B1*invA*B0));

    auto const& phi=phi0;
    UpperBoxType bx({{0,1},{0,1}});
    Vector<Bounds<X>> bd({{0,1},{0,1}},pr);
    auto ddphi = differential(phi,bd,2);
    auto mag_ddphi = mag(ddphi);

    PositiveUpperBound<X> error_constant = 2u*sqrt(PositiveUpperBound<X>(cndA * max_nrm * mag_ddphi));
    Nat N = log2((error_constant/tolerance).get_d())+1;

    SizeType two_pow_N = std::pow(2,N);
    Positive<X> h(Dyadic(1,N),pr);

    auto courant_nrm = 1/(1/sup_norm(invA*B0)+1/sup_norm(invA*B1));
    Nat L = log2(courant_nrm.get_d())+1;
    Positive<X> tau(Dyadic(1,N+L),pr);

    Nat steps=(tmax*pow(Natural(2u),N+L)).get_d();

    UpperBound<X> error = h * error_constant;


    // Generate array of x values (0,1,...,2^N)/2^N
    //Array<X> xs(two_pow_N+1, [N,pr](SizeType i){return X(Dyadic(i,N),pr);});
    // Generate array of x values (1,3,...,2*2^N-1)/2^(N+1)
    Array<X> hxs(two_pow_N, [N,pr](SizeType i){return X(Dyadic(2*i+1,N+1),pr);});

    Bounds<X> zb(0,pr);
    Vector<Bounds<X>> zbn(n,zb);
    Vector<X> x(m,pr);

    // FIXME: Relax size of rTs
    assert(rTs.size()==2);
    Array<Matrix<Bounds<X>>> Ts(rTs.size(), [&](SizeType i){return Matrix<Bounds<X>>(rTs[i],pr);});
    Array<Matrix<Bounds<X>>> invTs(Ts.size(), [&](SizeType i){return inverse(Ts[i]);});

    Array<DiagonalMatrix<Bounds<X>>> Ds(rDs.size(), [&](SizeType i){return DiagonalMatrix<Bounds<X>>(rDs[i],pr);});

    // Initialise u values at grid points
    Tensor<2,Vector<Bounds<X>>> us({two_pow_N,two_pow_N},zbn);
    for(SizeType i0=0; i0!=two_pow_N; ++i0) {
        x[0]=hxs[i0];
        for(SizeType i1=0; i1!=two_pow_N; ++i1) {
            x[1]=hxs[i1];
            auto y=phi0(x);
            us[{i0,i1}] = y;
        }
    }

    Tensor<3,Vector<Bounds<X>>> uts({two_pow_N,two_pow_N,steps+1},zbn);
    for(SizeType i0=0; i0!=two_pow_N; ++i0) {
        for(SizeType i1=0; i1!=two_pow_N; ++i1) {
            uts[{i0,i1,0}] = us[{i0,i1}];
        }
    }

    Tensor<2,Vector<Bounds<X>>> V0s({two_pow_N,two_pow_N},zbn);
    Tensor<2,Vector<Bounds<X>>> V1s({two_pow_N,two_pow_N},zbn);
    Tensor<2,Vector<Bounds<X>>> VF0s({two_pow_N+1,two_pow_N},zbn);
    Tensor<2,Vector<Bounds<X>>> VF1s({two_pow_N,two_pow_N+1},zbn);
    Tensor<2,Vector<Bounds<X>>> UF0s({two_pow_N+1,two_pow_N},zbn);
    Tensor<2,Vector<Bounds<X>>> UF1s({two_pow_N,two_pow_N+1},zbn);
    Tensor<2,Vector<Bounds<X>>> new_us(us.sizes(),zbn);

    for(SizeType step=0; step!=steps; ++step) {

        V0s=transform(us,[&](auto u){return invTs[0]*u;});
        V1s=transform(us,[&](auto u){return invTs[1]*u;});

        // FIXME: Need to decide whether eigenvalue is nonzero
        // Compute array of points from left or right
        for(SizeType j=0; j!=n; ++j) {
            Bool use_lower = decide(Ds[0][j]>=0);
            for(SizeType i0=0; i0!=two_pow_N+1; ++i0) {
                SizeType next_i0 = use_lower ? ((i0==0)?two_pow_N-1:i0-1) : ((i0==two_pow_N)?0:i0);
                for(SizeType i1=0; i1!=two_pow_N; ++i1) {
                    VF0s[{i0,i1}][j] = V0s[{next_i0,i1}][j];
                }
            }
        }

        for(SizeType j=0; j!=n; ++j) {
            Bool use_lower = decide(Ds[1][j]>=0);
            for(SizeType i1=0; i1!=two_pow_N+1; ++i1) {
                SizeType next_i1 = use_lower ? ((i1==0)?two_pow_N-1:i1-1) : ((i1==two_pow_N)?0:i1);
                for(SizeType i0=0; i0!=two_pow_N; ++i0) {
                    VF1s[{i0,i1}][j] = V1s[{i0,next_i1}][j];
                }
            }
        }

        UF0s=transform(VF0s, [&](auto v){return Ts[0]*v;});
        UF1s=transform(VF1s, [&](auto v){return Ts[1]*v;});

        auto new_u = [&](Array<SizeType> const& is){return us[is] - tau/h*invA*(Bs[0]*(UF0s[{is[0]+1,is[1]}]-UF0s[{is[0],is[1]}])+Bs[1]*(UF1s[{is[0],is[1]+1}]-UF1s[{is[0],is[1]}]));};
        new_us=Tensor<2,Vector<Bounds<X>>>(us.sizes(),new_u);

        std::swap(us,new_us);
        for(SizeType i0=0; i0!=two_pow_N; ++i0) {
            for(SizeType i1=0; i1!=two_pow_N; ++i1) {
                uts[{i0,i1,step+1}] = us[{i0,i1}];
            }
        }

    } // main loop

    auto final_us=us;

    return FirstOrderPDESolution<PR>{h,tau,uts,error};
}

template FirstOrderPDESolution<DP>
first_order_pde(FirstOrderPDE const&, EffectiveVectorMultivariateFunction const& phi0, DP pr);

template FirstOrderPDESolution<MP>
first_order_pde(FirstOrderPDE const&, EffectiveVectorMultivariateFunction const& phi0, MP pr);

} // namespace Ariadne
