/***************************************************************************
 *            matrix.tpl.hpp
 *
 *  Copyright  2005-20  Alberto Casagrande, Pieter Collins
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

#include "algebra/vector.hpp"
#include "algebra/covector.hpp"

namespace Ariadne {

template<class X> inline X create_zero();

template<class X> Matrix<X>::Matrix(SizeType m, SizeType n, const X* p)
    : _rs(m), _cs(n), _ary(p,p+m*n) {
}


template<class X> Matrix<X>::Matrix(InitializerList<InitializerList<X>> lst)
    : _rs(lst.size()), _cs((assert(_rs!=0),assert(lst.begin()->size()!=0),lst.begin()->size())), _ary(_rs*_cs,characteristics(*lst.begin()->begin()))
{
    typename InitializerList<InitializerList<X>>::const_iterator row_iter=lst.begin();
    for(SizeType i=0; i!=this->row_size(); ++i, ++row_iter) {
        ARIADNE_PRECONDITION(row_iter->size()==this->column_size());
        typename InitializerList<X>::const_iterator col_iter=row_iter->begin();
        for(SizeType j=0; j!=this->column_size(); ++j, ++col_iter) {
            this->at(i,j)=*col_iter;
        }
    }
}

template<class X> Matrix<X> join(Matrix<X> const& A1, Matrix<X> const& A2) {
    ARIADNE_PRECONDITION(A1.column_size()==A2.column_size());
    Matrix<X> R(A1.row_size()+A2.row_size(),A1.column_size(),A1.zero_element());
    const SizeType m1=A1.row_size();
    for(SizeType i=0; i!=A1.row_size(); ++i) {
        for(SizeType j=0; j!=A1.column_size(); ++j) {
            R[i][j]=A1[i][j];
        }
    }
    for(SizeType i=0; i!=A2.row_size(); ++i) {
        for(SizeType j=0; j!=A2.column_size(); ++j) {
            R[m1+i][j]=A2[i][j];
        }
    }
    return R;
}

template<class X> Matrix<X> join(Matrix<X> const& A1, Covector<X> const& u2) {
    ARIADNE_PRECONDITION(A1.column_size()==u2.size());
    Matrix<X> R(A1.row_size()+1u,A1.column_size(),A1.zero_element());
    const SizeType m1=A1.row_size();
    for(SizeType i=0; i!=A1.row_size(); ++i) {
        for(SizeType j=0; j!=A1.column_size(); ++j) {
            R[i][j]=A1[i][j];
        }
    }
    for(SizeType j=0; j!=u2.size(); ++j) {
        R[m1][j]=u2[j];
    }
    return R;
}

template<class X> Matrix<X> join(Covector<X> const& u1, Matrix<X> const& A2) {
    ARIADNE_PRECONDITION(u1.size()==A2.column_size());
    Matrix<X> R(1u+A2.row_size(),A2.column_size(),A2.zero_element());
    for(SizeType j=0; j!=u1.size(); ++j) {
        R[0u][j]=u1[j];
    }
    for(SizeType i=0; i!=A2.row_size(); ++i) {
        for(SizeType j=0; j!=A2.column_size(); ++j) {
            R[1u+i][j]=A2[i][j];
        }
    }
    return R;
}

template<class X> Matrix<X> cojoin(Matrix<X> const& A1, Matrix<X> const& A2) {
    ARIADNE_PRECONDITION(A1.row_size()==A2.row_size());
    Matrix<X> R(A1.row_size(),A1.column_size()+A2.column_size(),A1.zero_element());
    const SizeType n1=A1.column_size();
    for(SizeType i=0; i!=A1.row_size(); ++i) {
        for(SizeType j=0; j!=A1.column_size(); ++j) {
            R[i][j]=A1[i][j];
        }
        for(SizeType j=0; j!=A2.column_size(); ++j) {
            R[i][n1+j]=A2[i][j];
        }
    }
    return R;
}

template<class X> Matrix<X> cojoin(Matrix<X> const& A1, Vector<X> const& v2) {
    ARIADNE_PRECONDITION(A1.row_size()==v2.size());
    Matrix<X> R(A1.row_size(),A1.column_size()+1u,A1.zero_element());
    const SizeType n1=A1.column_size();
    for(SizeType i=0; i!=A1.row_size(); ++i) {
        for(SizeType j=0; j!=A1.column_size(); ++j) {
            R[i][j]=A1[i][j];
        }
        R[i][n1]=v2[i];
    }
    return R;
}

template<class X> Matrix<X> cojoin(Vector<X> const& v1, Matrix<X> const& A2) {
    ARIADNE_PRECONDITION(v1.size()==A2.row_size());
    Matrix<X> R(A2.row_size(),1u+A2.column_size(),A2.zero_element());
    for(SizeType i=0; i!=A2.row_size(); ++i) {
        R[i][0]=v1[i];
        for(SizeType j=0; j!=A2.column_size(); ++j) {
            R[i][1u+j]=A2[i][j];
        }
    }
    return R;
}


#ifdef ARIADNE_OMIT
template<class X> InputStream& Matrix<X>::read(InputStream& is) {
    Matrix<X>& A=*this;
    char c;
    is >> c;
    is.putback(c);
    if(c=='[') {
        is >> c;
        /* Representation as a literal [a11,a12,...,a1n; a21,a22,...a2n; ... ; am1,am2,...,amn] */
        std::vector< std::vector<X> > v;
        X x;
        c=';';
        while(is && c==';') {
            v.push_back(std::vector<X>());
            c=',';
            while(is && c==',') {
                is >> x;
                v.back().push_back(x);
                is >> c;
            }
        }
        if(is) {
            A=Matrix<X>(v.size(),v.front().size());
            for(SizeType i=0; i!=A.row_size(); ++i) {
                if(v[i].size()!=A.column_size()) {
                    // TOOD: Add exception
                    assert(false);
                    //ARIADNE_THROW(InvalidInput,"Matrix::read(istream&)","row[0].size()="<<v[0].size()<<", row["<<i<<"].size()="<<v[i].size());
                }
                for(SizeType j=0; j!=A.column_size(); ++j) {
                    A[i][j]=v[i][j];
                }
            }
        }
    }
    else {
        // TOOD: Add exception
        assert(false);
        //ARIADNE_THROW(InvalidInput,"Matrix::read(istream&)"," separator c="<<c);
    }
    return is;
}
#endif

template<class X> decltype(mag(declval<X>())) sup_norm(const Matrix<X>& A)
{
    typedef decltype(mag(declval<X>())) R;
    R zero=mag(A.zero_element());
    R result=zero;
    for(SizeType i=0; i!=A.row_size(); ++i) {
        R row_sum=zero;
        for(SizeType j=0; j!=A.column_size(); ++j) {
            row_sum+=mag(A[i][j]);
        }
        // NOTE: The arguments must be this way round to propagate a nan row_sum
        result=max(row_sum,result);
    }
    return result;
}

template<class X> decltype(declval<X>()+mag(declval<X>())) log_norm(Matrix<X> const& A)
{
    ARIADNE_PRECONDITION(A.row_size()==A.column_size());
    typedef decltype(declval<X>()+mag(declval<X>())) R;
    R r=A.zero_element(); r=-inf;
    for(SizeType i=0; i!=A.row_size(); ++i) {
        R t=A[i][i];
        for(SizeType j=0; j!=A.column_size(); ++j) {
            if(j!=i) {
                t+=mag(A[i][j]);
            }
        }
        r=max(r,t);
    }
    return r;
}

template<class X>
Matrix<X>
lu_inverse(const Matrix<X>& M)
{
    typedef X RealType;
    ARIADNE_ASSERT_MSG(M.row_size()==M.column_size(),"A="<<M);

    SizeType m=M.row_size();
    SizeType n=M.column_size();
    Matrix<RealType> A=M;
    Matrix<RealType> B=Matrix<RealType>::identity(n,M.element_characteristics());

    // Array of row pivots. The value p[i] gives the row
    // swapped with the ith row in the ith stage.
    Array<SizeType> p(m);
    for(SizeType k=0; k!=m; ++k) { p[k]=k; }

    for(SizeType k=0; k!=std::min(m,n); ++k) {
        // Choose a pivot row
        SizeType iamax=m;
        auto amax=mig(M.zero_element());
        for(SizeType i=k; i!=m; ++i) {
            if(decide(mig(A[i][k])>amax)) {
                iamax=i;
                amax=mig(A[i][k]);
            }
        }

        if(iamax==m) {
            ARIADNE_THROW(SingularMatrixException,"lu_inverse(Matrix<"<<class_name<X>()<<"> M)","M="<<M);
        }

        // Set pivot row
        SizeType q=iamax;
        p[k]=q;

        // Swap rows of both A and B
        for(SizeType j=k; j!=n; ++j) {
            X tmp=A[k][j];
            A[k][j]=A[q][j];
            A[q][j]=tmp;
        }

        for(SizeType j=0; j!=n; ++j) {
            X tmp=B[k][j];
            B[k][j]=B[q][j];
            B[q][j]=tmp;
        }

        RealType r  = rec(A[k][k]);
        for(SizeType i=k+1; i!=n; ++i) {
            RealType s=A[i][k] * r;
            for(SizeType j=0; j!=n; ++j) {
                B[i][j] -= s * B[k][j];
            }
            for(SizeType j=k+1; j!=n; ++j) {
                A[i][j] -= s * A[k][j];
            }
            A[i][k] = 0;
        }
        for(SizeType j=0; j!=n; ++j) {
            B[k][j] *= r;
        }
        for(SizeType j=k+1; j!=n; ++j) {
            A[k][j] *= r;
        }
        A[k][k] = 1;
    }

    // Backsubstitute to find inverse
    for(SizeType k=n; k!=0; ) {
        --k;
        for(SizeType i=0; i!=k; ++i) {
            RealType s=A[i][k];
            for(SizeType j=0; j!=n; ++j) {
                B[i][j] -= s * B[k][j];
            }
            A[i][k] = 0;
        }
    }

    // No need to repivot!

    return B;
}



template<class X> Matrix<Bounds<X>> dd_solve_bounds(const Matrix<Bounds<X>>& A, const Matrix<Bounds<X>>& B);
template<class X> Matrix<Bounds<X>> gs_solve_bounds(const Matrix<Bounds<X>>& A, const Matrix<Bounds<X>>& B);

template<class X> Matrix<X> lu_solve(const Matrix<X>& A, const Matrix<X>& B);
template<class X> Matrix<X> gs_solve(const Matrix<X>& A, const Matrix<X>& B) {
    return gs_solve_bounds(A,B); }
template<class X> Matrix<X> dd_solve(const Matrix<X>& A, const Matrix<X>& B) {
    return gs_solve_bounds(A,B); }

template<class X> Matrix<X> lu_solve(const Matrix<X>& A, const Matrix<X>& B) {
    return lu_inverse(A)*B;
}

template<class X> Vector<X> lu_solve(const Matrix<X>& A, const Vector<X>& b) {
    return lu_inverse(A)*b;
}

// Find a starting solution for a diagonally dominant system
template<class X> Matrix<Bounds<X>> dd_solve_bounds(const Matrix<Bounds<X>>& A, const Matrix<Bounds<X>>& B)
{
    const SizeType n=B.row_size();
    const SizeType m=B.column_size();

    //Compute an upper bound for 1/(|aii|-sum|aij|) using outward rounding
    Vector<UpperBound<X>> c(n,UpperBound<X>(0u));
    for(SizeType i=0; i!=n; ++i) {
        LowerBound<X> rci=mig(A[i][i]);
        for(SizeType j=0; j!=n; ++j) {
            if(j!=i) {
                rci=rci-mag(A[i][j] );
            }
        }
        if(possibly(rci<=0)) {
            ARIADNE_THROW(std::runtime_error,"dd_solve(Matrix<Bounds<X>> A, Matrix<Bounds<X>> B)",
                          "Matrix A="<<A<<" is not diagonally-dominant.");
        }
        c[i]=rec(cast_positive(rci));
    }

    // Compute initial solution
    Matrix<Bounds<X>> R(n,m);
    for(SizeType i=0; i!=n; ++i) {
        Bounds<X> ci(-c[i],+c[i]);
        for(SizeType j=0; j!=m; ++j) {
            R[i][j]=B[i][j]*ci;
        }
    }

    return R;
}


template<class X> Void gs_step(const Matrix<Bounds<X>>& A, const Vector<Bounds<X>>& b, Vector<Bounds<X>>& x)
{
    // Perform Gauss-Seidel iteration
    const SizeType n=x.size();
    for(SizeType i=0; i!=n; ++i) {
        // compute R'[i][j] := (JB[i][j] - Sum{k!=j}JA[i][k]*R[k][j]) / JA[i][i]
        Bounds<X> ri=b[i];
        for(SizeType k=0; k!=n; ++k) {
            if(k!=i) {
                ri-=A[i][k]*x[k];
            }
        }
        if(definitely(A[i][i].lower()>0 || A[i][i].upper()<0)) {
            ri/=A[i][i];
            x[i]=refinement(x[i],ri);
        }
    }
}

template<class X> Matrix<Bounds<X>> gs_solve_bounds(const Matrix<Bounds<X>>& A, const Matrix<Bounds<X>>& B)
{
    ARIADNE_ASSERT(A.row_size()==A.column_size());
    ARIADNE_ASSERT(B.row_size()==A.column_size());
    const SizeType n=B.row_size();
    const SizeType m=B.column_size();

    // Precondition A and B
    Matrix<Approximation<X>> mA(A);

    Matrix<X> J=cast_exact(inverse(mA));
    Matrix<Bounds<X>> JA=J*A;
    Matrix<Bounds<X>> JB=J*B;

    //Matrix<Bounds<X>> R=dd_solve(JA,JB);
    Matrix<Bounds<X>> R=lu_solve(A,B);
    //std::cerr<<"R="<<R<<"\n";

    // Perform Gauss-Seidel iteration
    SizeType step=0;
    while(step<1) {
        for(SizeType i=0; i!=n; ++i) {
            for(SizeType j=0; j!=m; ++j) {
                // compute R'[i][j] := (JB[i][j] - Sum{k!=j}JA[i][k]*R[k][j]) / JA[i][i]
                Bounds<X> Rij=JB[i][j];
                for(SizeType k=0; k!=n; ++k) {
                    if(k!=i) {
                        Rij-=JA[i][k]*R[k][j];
                    }
                }
                if(definitely(JA[i][i].lower()>0 || JA[i][i].upper()<0)) {
                    Rij/=JA[i][i];
                    R[i][j]=refinement(R[i][j],Rij);
                }
                // FIXME: Use FloatBounds or Float here?
                //R[i][j]=FloatBounds(max(R[i][j].lower(),Rij.lower()),min(R[i][j].upper(),Rij.upper()));
            }
        }
        ++step;
        //std::cerr<<"R="<<R<<"\n";
    }

    return R;
}

template<class X> Vector<X> gs_solve(const Matrix<X>& A, const Vector<X>& b) {
    X bz=b.zero_element();
    Matrix<X> B(b.size(),1u,bz); for(SizeType i=0; i!=b.size(); ++i) { B[i][0]=b[i]; }
    Matrix<X> R=gs_solve(A,B);
    Vector<X> r(R.row_size(),R.zero_element()); for(SizeType i=0; i!=r.size(); ++i) { r[i]=R[i][0]; }
    return r;
}


template<class X> Matrix<X> gs_inverse(const Matrix<X>& A) {
    return gs_solve(A,Matrix<X>::identity(A.row_size(),A.zero_element()));
}


template<class X> Matrix<ArithmeticType<X>> inverse(const Matrix<X>& A) {
    return lu_inverse(A);
}

template<> Matrix<FloatDPBounds> inverse<FloatDP>(const Matrix<FloatDP>& A);
template<> Matrix<FloatDPBounds> inverse<FloatDPBounds>(const Matrix<FloatDPBounds>& A);


template<class X1, class X2> Matrix<ArithmeticType<X1,X2>> solve(const Matrix<X1>& A, const Matrix<X2>& B) {
    //return inverse(A)*B;
    auto Ainv=inverse(A); return Ainv*B;
}

template<> Matrix<FloatDPBounds> solve(const Matrix<FloatDPBounds>& A, const Matrix<FloatDPBounds>& B);

template<class X1, class X2> Vector<ArithmeticType<X1,X2>> solve(const Matrix<X1>& A, const Vector<X2>& b) {
    return lu_inverse(A)*b;
}



template<class X>
Matrix<X>
solve(const PLUMatrix<X>& A, const Matrix<X>& B)
{
    const PivotMatrix& P=A.P;
    const Matrix<X>& L=A.L;
    const Matrix<X>& U=A.U;

    // Only work with square matrices L, U
    ARIADNE_ASSERT(L.row_size()==L.column_size());
    ARIADNE_ASSERT(U.row_size()==U.column_size());

    // Check sizes for consistency
    ARIADNE_ASSERT(L.column_size()==U.row_size());
    ARIADNE_ASSERT(P.size()==L.row_size());
    ARIADNE_ASSERT(B.row_size()==P.size());

    // Set constants for sizes
    const SizeType m=B.column_size();
    const SizeType n=B.row_size();

    Matrix<X> R=B;

    // PivotMatrix rows of B
    for(SizeType k=0; k!=n; ++k) {
        SizeType i=P.pivot(k);
        for(SizeType j=0; j!=m; ++j) {
            X tmp=R[i][j];
            R[i][k]=R[k][j];
            R[k][j]=tmp;
        }
    }

    // Backsubstitute on L
    for(SizeType k=0; k!=n; ++k) {
        for(SizeType i=k+1; i!=n; ++i) {
            X s=L[i][k];
            for(SizeType j=0; j!=n; ++j) {
                R[i][j] -= s * R[k][j];
            }
        }
    }

    // Backsubstitute on U with row scaling
    for(SizeType k=n; k!=0; ) {
        --k;
        X s=1/U[k][k];
        for(SizeType j=0; j!=m; ++j) {
            R[k][j] *= s;
        }
        for(SizeType i=0; i!=k; ++i) {
            X t=U[i][k];
            for(SizeType j=0; j!=m; ++j) {
                R[i][j] -= t * R[k][j];
            }
        }
    }

}


template<class X> Matrix<X> operator*(PivotMatrix P, Matrix<X> A) {
    for(SizeType i=0; i!=A.row_size(); ++i) {
        SizeType const& k=P.pivot(i);
        if (k!=i) {
            for (SizeType j=0; j!=A.column_size(); ++j) {
                std::swap(A[i][j],A[k][j]);
            }
        }
    }
    return A;
}

// Compute the vector of row norms (sums of absolute values) of A
template<class X>
Vector<RowNormType<X>>
row_norms(const Matrix<X>& A)
{
    const SizeType m=A.row_size();
    const SizeType n=A.column_size();
    Vector<RowNormType<X>> r(m,A.zero_element());

    for(SizeType i=0; i!=m; ++i) {
        r[i]=A.zero_element();
        for(SizeType j=0; j!=n; ++j) {
            r[i]+=abs(A[i][j]);
        }
    }

    return r;

}


// Scale the rows of A to each have sum of absolute values less than or equal to one
// FIXME: This code is inherantly approximate and relies on changing the rounding mode
template<class X>
Void
normalise_rows(Matrix<X>& A)
{

    const SizeType m=A.row_size();
    const SizeType n=A.column_size();

    auto prev_rounding_mode=X::get_rounding_mode();
    X::set_rounding_upward();
    Array<X> row_asums(m,A.zero_element());
    for(SizeType i=0; i!=m; ++i) {
        row_asums[i]=0.0_x;
        for(SizeType j=0; j!=n; ++j) {
            row_asums[i]+=abs(A[i][j]);
        }
    }
    X::set_rounding_toward_zero();
    for(SizeType i=0; i!=m; ++i) {
        for(SizeType j=0; j!=n; ++j) {
            A[i][j]/=row_asums[i];
        }
    }
    X::set_rounding_mode(prev_rounding_mode);
}


// Returns a pivot P and matrices L and U such that L is unit lower-triangular,
// U is upper-trianguler and A=PLU.
template<class X>
Tuple< PivotMatrix, Matrix<X>, Matrix<X> >
triangular_decomposition(const Matrix<X>& A)
{
    ARIADNE_ASSERT(A.row_size()==A.column_size());

    SizeType m=A.row_size();
    SizeType n=A.column_size();
    Matrix<X> L=Matrix<X>::identity(m,A.element_characteristics());
    Matrix<X> U=A;

    // Array of row pivots. The value P[i] gives the row
    // swapped with the ith row in the ith stage.
    PivotMatrix P=PivotMatrix::identity(m);

    for(SizeType k=0; k!=std::min(m,n); ++k) {
        // Choose a pivot row
        SizeType iamax=k;
        X amax=A.zero_element();
        for(SizeType i=k; i!=m; ++i) {
            if(decide(abs(A[i][k])>amax)) {
                iamax=i;
                amax=abs(A[i][k]);
            }
        }

        // Set pivot row
        SizeType l=iamax;
        P.pivot(k)=l;

        // Swap rows of L and U
        for(SizeType j=0; j!=k; ++j) {
            X tmp=L[k][j];
            L[k][j]=L[l][j];
            L[l][j]=tmp;
        }

        for(SizeType j=k; j!=n; ++j) {
            X tmp=U[k][j];
            U[k][j]=U[l][j];
            U[l][j]=tmp;
        }

        X r  = 1/U[k][k];
        for(SizeType i=k+1; i!=m; ++i) {
            X s=U[i][k] * r;
            for(SizeType j=k+1; j!=n; ++j) {
                U[i][j] -= s * U[k][j];
            }
            U[i][k] = 0;
            L[i][k] = s;
        }

    }

    return Tuple<PivotMatrix,Matrix<X>,Matrix<X>>{P,L,U};

}

// Returns a matrix R such that A=OR where O is a square matrix with
// orthogonal rows and the sum of the absolute values of the rows of R are
// at most one.
//
// Use Householder transformation H=I-vv' where v=u/|u|
// and u=a +/- |a|e with a and e the working column of A
// and corresponding unit vector.
template<class X> Tuple< Matrix<X>, PivotMatrix>
triangular_factor(const Matrix<X>& A)
{
    X zero = A.zero_element();

    const SizeType m=A.row_size();
    const SizeType n=A.column_size();

    Matrix<X> R=A;
    PivotMatrix P=PivotMatrix::identity(n);

    // Array of column norm squares
    Array<X> cns(n);

    Vector<X> u(m);

    for(SizeType k=0; k!=std::min(m,n); ++k) {
        //std::cerr<<"k="<<k<<" R="<<R<<std::endl;

        Bool pivoting=true;
        if(pivoting) {
            // Compute column norms
            for(SizeType j=k; j!=n; ++j) {
                cns[j]=zero;
                for(SizeType i=k; i!=m; ++i) {
                    cns[j]+=sqr(R[i][j]);
                }
            }

            // Find largest column norm
            SizeType l=k; X cnsmax=cns[k];
            for(SizeType j=k+1; j!=n; ++j) {
                if(cns[j]>cnsmax) {
                    l=j; cnsmax=cns[j];
                }
            }

            // Swap columns l and k
            for(SizeType i=0; i!=m; ++i) {
                X tmp=R[i][k];
                R[i][k]=R[i][l];
                R[i][l]=tmp;
            }

            // Set pivot element to pivot column
            P.pivot(k)=l;
        }

        // Compute |a| where a is the working column
        X nrmas=zero;
        for(SizeType i=k; i!=m; ++i) {
            nrmas+=R[i][k]*R[i][k];
        }
        X nrma=sqrt(nrmas);

        // Compute u=a +/- |a|e
        for(SizeType i=k; i!=m; ++i) {
            u[i]=R[i][k];
        }
        if(u[k]>=0) { u[k]+=nrma; }
        else { u[k]-=nrma; }

        // Compute -2/u.u
        X nrmus=zero;
        for(SizeType i=k; i!=m; ++i) {
            nrmus+=sqr(u[i]);
        }
        X mtdnu=(-2)/nrmus;

        // For each column b, compute b-=2u(u.b)/u.u
        for(SizeType j=k; j!=n; ++j) {
            X udtb=zero;
            for(SizeType i=k; i!=m; ++i) {
                udtb+=u[i]*R[i][j];
            }
            X scl=udtb*mtdnu;
            for(SizeType i=k; i!=m; ++i) {
                R[i][j]+=scl*u[i];
            }
        }

        // For the kth column, set R[k][k]=-/+ |a|
        // and R[i][k]=0 for i>k
        for(SizeType i=k+1; i!=m; ++i) {
            R[i][k]=zero;
        }

    } // end of loop on working column k

    // Scale the rows of R to have sum of absolute values equal to 1
    normalise_rows(R);

    return Tuple<Matrix<X>,PivotMatrix>{R,P};

}

// Returns a matrix T such that the matrix O=AT is a square matrix with
// orthogonal rows, and such that the row absolute value sums of the inverse of
// T are at most 1. Then the image of the unit box under the matrix T is
// an over-approximation of the unit box.
template<class X> Matrix<X>
triangular_multiplier(const Matrix<X>& A)
{
    ARIADNE_ASSERT(A.row_size()<=A.column_size());
    const SizeType m=A.row_size();
    const SizeType n=A.column_size();

    Matrix<X> R; PivotMatrix P;
    make_ltuple(R,P)=triangular_factor(A);

    Matrix<X> T(n,m); for(SizeType i=0; i!=m; ++i) { T[i][i]=1.0_x; }

    for(SizeType i=0; i!=m; ++i) { assert(R[i][i]!=0.0_x); }

    for(SizeType k=m-1; k!=SizeType(-1); --k) {
        X r=1/R[k][k];
        for(SizeType i=0; i!=k; ++i) {
            X s=R[i][k]*r;
            for(SizeType j=k; j!=m; ++j) {
                T[i][j]-=s*T[k][j];
            }
        }
        for(SizeType j=k; j!=m; ++j) {
            T[k][j]*=r;
        }
    }

    for(SizeType k=m-1; k!=SizeType(-1); --k) {
        SizeType p=P.pivot(k);
        for(SizeType i=0; i!=m; ++i) {
            X tmp=T[p][i]; T[p][i]=T[k][i]; T[k][i]=tmp;
        }
    }

    return T;
}


template<class X> Matrix<X> pivot_matrix(const Array<SizeType>& pv)
{
    const SizeType n=pv.size();
    Array<SizeType> perm(n); for(SizeType i=0; i!=n; ++i) { perm[i]=i; }
    for(SizeType i=0; i!=n; ++i) {
        std::swap(perm[i],perm[pv[i]]);
    }
    Matrix<X> P(n,n);
    for(SizeType i=0; i!=n; ++i) {
        P[i][perm[i]]=1;
    }
    return P;
}

template<class X> PivotMatrix::operator Matrix<X> () const {
    return pivot_matrix<X>(this->_ary);
}

// Compute the orthogonal decomposition A=QR with or without column pivoting. The
// matrix Q is built up as a composition of elementary Householder
// transformations H=I-vv' with |v|=1. Note that inv(H)=H'=H. The vector v is
// chosen to be a multiple of the first working column of A.
template<class X> Tuple< Matrix<X>, Matrix<X>, PivotMatrix >
orthogonal_decomposition(const Matrix<X>& A, Bool allow_pivoting)
{
    X zero=A.zero_element();
    X one=zero+1;

    SizeType m=A.row_size();
    SizeType n=A.column_size();
    Matrix<X> Q(m,m,zero);
    Matrix<X> R(A);
    PivotMatrix P(n);

    Array<X> p(n,zero);
    Vector<X> u(m,zero);

    for(SizeType i=0; i!=m; ++i) {
        for(SizeType j=0; j!=m; ++j) {
            Q[i][j]=zero;
        }
        Q[i][i]=one;
    }

    for(SizeType k=0; k!=std::min(m,n); ++k) {
        //std::cerr<<"k="<<k<<" Q="<<Q<<" R="<<R<<std::flush;

        if(allow_pivoting) {
            // Find a pivot column
            SizeType pivot_column=k;
            X max_column_norm=zero;
            for(SizeType j=k; j!=n; ++j) {
                X column_norm=zero;
                for(SizeType i=k; i!=m; ++i) {
                    column_norm+=abs(R[i][j]);
                }
                if(decide(column_norm>max_column_norm)) {
                    pivot_column=j;
                    max_column_norm=column_norm;
                }
            }
            SizeType l=pivot_column;

            // Swap working column and pivot column
            for(SizeType i=0; i!=m; ++i) {
                std::swap(R[i][l],R[i][k]);
            }

            // Set pivot column in result
            P.pivot(k)=l;
        }

        // Compute |a| where a is the working column
        X nrmas=zero;
        for(SizeType i=k; i!=m; ++i) {
            nrmas+=R[i][k]*R[i][k];
        }
        X nrma=sqrt(nrmas);

        // Compute u=a +/- |a|e
        for(SizeType i=0; i!=k; ++i) {
            u[i]=zero;
        }
        for(SizeType i=k; i!=m; ++i) {
            u[i]=R[i][k];
        }
        if(decide(u[k]>=0)) { u[k]+=nrma; }
        else { u[k]-=nrma; }

        // Compute -2/u.u
        X nrmus=zero;
        for(SizeType i=k; i!=m; ++i) {
            nrmus+=sqr(u[i]);
        }
        X mtdnu=(-2)/nrmus;

        // Compute H=(1-2uu'/u'u)
        // Matrix<X> H(n,n); for(SizeType i=0; i!=n; ++i) {
        // H[i][i]=1.0_x; for(SizeType j=0; j!=n; ++j) { H[i][j]+=u[i]*u[j]*mtdnu; } }

        // For each column b of R, compute b-=2u(u.b)/u.u
        for(SizeType j=k; j!=n; ++j) {
            X udtb=zero;
            for(SizeType i=k; i!=m; ++i) {
                udtb+=u[i]*R[i][j];
            }
            X scl=udtb*mtdnu;
            for(SizeType i=k; i!=m; ++i) {
                R[i][j]+=scl*u[i];
            }
        }

        // For the kth column, set R[k][k]=-/+ |a|
        // and R[i][k]=0 for i>k
        for(SizeType i=k+1; i!=m; ++i) {
            R[i][k]=zero;
        }
        if(decide(u[k]>=0)) { R[k][k]=-nrma; } else { R[k][k]=nrma; }

        // Update Q'=QH = Q(I-2uu'/u'u)
        // For each row q, compute q-=2u(u.q)/(u.u)
        for(SizeType i=0; i!=m; ++i) {
            X qdtu=zero;
            for(SizeType j=k; j!=m; ++j) {
                qdtu+=Q[i][j]*u[j];
            }
            X scl=qdtu*mtdnu;
            for(SizeType j=k; j!=m; ++j) {
                Q[i][j]+=scl*u[j];
            }
        }

    }

    return std::make_tuple(Q,R,P);
}

namespace {
// TODO: Move these operations to VectorExpression
template<class X> inline decltype(auto) operator*(SelfType<X> const& s, MatrixColumn<Matrix<X>> v) {
    return s*Vector<X>(v); }
template<class X> inline decltype(auto) operator*(MatrixColumn<Matrix<X>> v, SelfType<X> const& s) {
    return Vector<X>(v)*s; }
template<class X> inline decltype(auto) operator/(MatrixColumn<Matrix<X>> v, SelfType<X> const& s) {
    return Vector<X>(v)/s; }
template<class X> inline MatrixColumn<Matrix<X>> operator+=(MatrixColumn<Matrix<X>> c, Vector<SelfType<X>> const& v) {
    for(SizeType i=0; i!=c.size(); ++i) { c[i]+=v[i]; } return c; }
template<class X> inline MatrixColumn<Matrix<X>> operator-=(MatrixColumn<Matrix<X>> c, Vector<SelfType<X>> const& v) {
    for(SizeType i=0; i!=c.size(); ++i) { c[i]-=v[i]; } return c; }
template<class X> inline MatrixColumn<Matrix<X>> operator*=(MatrixColumn<Matrix<X>> c, SelfType<X> const& s) {
    for(SizeType i=0; i!=c.size(); ++i) { c[i]*=s; } return c; }
template<class X> inline MatrixColumn<Matrix<X>> operator/=(MatrixColumn<Matrix<X>> c, SelfType<X> const& s) {
    for(SizeType i=0; i!=c.size(); ++i) { c[i]/=s; } return c; }
template<class X> inline decltype(auto) dot(MatrixColumn<Matrix<X>> v1, MatrixColumn<Matrix<X>> v2) {
    return dot(Vector<X>(v1),Vector<X>(v2)); }
template<class X> inline decltype(auto) two_norm(MatrixColumn<Matrix<X>> v) {
    return two_norm(Vector<X>(v)); }
template<class X> MatrixColumn<Matrix<X>> column(Matrix<X>& A, SizeType j) {
    return MatrixColumn<Matrix<X>>(A,j); }
} // namespace

template<class X>
Tuple< Matrix<X>, Matrix<X> >
gram_schmidt_orthogonalisation(const Matrix<X>& A)
{
    std::cerr<<"orthogonal_decomposition(Matrix<X>)\n";
    SizeType m=A.row_size();
    SizeType n=A.column_size();
    X z=A.zero_element();
    Matrix<X> O(m,m,z);
    Matrix<X> R(m,n,z);
    Matrix<X> T(A);

    for(SizeType i=0; i!=m; ++i) {
        for (SizeType j=0; j!=i; ++j) {
            R[j][i]=dot(column(O,j),column(T,i));
            column(T,i)-=R[j][i]*column(O,j);
        }
        R[i][i]=two_norm(column(T,i));
        column(T,i)/=R[i][i];

        column(O,i)=column(T,i);
    }
    return std::make_tuple(O,R);
}

template<class X>
Tuple< Matrix<X>, Matrix<X> >
orthogonal_decomposition(const Matrix<X>& A)
{
    SizeType m=A.row_size();
    SizeType n=A.column_size();
    X z=A.zero_element();

    Matrix<X> O=A;
    Matrix<X> R=Matrix<X>::identity(n,A.element_characteristics());

    Array<X> p(n,z);

    for(SizeType c=0; c!=std::min(m,n); ++c) {

        // Find a pivot column
        SizeType pivot_column=c;
        X max_column_norm=z;
        for(SizeType j=c; j!=n; ++j) {
            X column_norm=z;
            for(SizeType i=0; i!=m; ++i) {
                column_norm+=abs(O[i][j]);
            }
            if(decide(column_norm>max_column_norm)) {
                pivot_column=j;
                max_column_norm=column_norm;
            }
        }
        SizeType pc=pivot_column;
        pc=c;

        // Swap first column and pivot column of O, and similarly with R
        // FIXME: We do not keep track of column pivoting here.
        for(SizeType i=0; i!=m; ++i) {
            std::swap(O[i][pc],O[i][c]);
        }
        for(SizeType j=0; j!=n; ++j) {
            std::swap(R[pc][j],R[c][j]);
        }

        // Compute inner product of pivot column with remaining columns
        X pivot_norm_square=z;
        for(SizeType i=0; i!=m; ++i) {
            pivot_norm_square += sqr(O[i][c]);
        }

        X inner_product_sum=z;
        for(SizeType k=c; k!=n; ++k) {
            X inner_product=z;
            for(SizeType i=0; i!=m; ++i) {
                inner_product += O[i][c]*O[i][k];
            }
            p[k]=inner_product;
            inner_product_sum += abs(inner_product);
        }

        for(SizeType i=c; i!=n; ++i) {
            R[c][i] = p[i]/inner_product_sum;
        }

        for(SizeType k=c+1; k!=n; ++k) {
            for(SizeType i=0; i!=m; ++i) {
                O[i][k] -= O[i][c] * (R[c][k]/R[c][c]);
            }
        }
        for(SizeType i=0; i!=m; ++i) {
            O[i][c] /= R[c][c];
        }

    }

    if (m<n) {
        Matrix<X> rO=O[range(0,m)][range(0,m)];
        O=std::move(rO);
        Matrix<X> rR=R[range(0,m)][range(0,n)];
        R=std::move(rR);
    }

    return std::make_tuple(O,R);
}



template<class X> Matrix<MidpointType<X>> midpoint(Matrix<X> const& A) {
    Matrix<MidpointType<X>> R(A.row_size(),A.column_size(),midpoint(A.zero_element()));
    for(SizeType i=0; i!=A.row_size(); ++i) {
        for(SizeType j=0; j!=A.column_size(); ++j) {
            R.at(i,j)=midpoint(A.at(i,j));
        }
    }
    return R;
}

template<class X> Matrix<SingletonType<X>> cast_singleton(Matrix<X> const& A) {
    Matrix<SingletonType<X>> R(A.row_size(),A.column_size(),cast_singleton(A.zero_element()));
    for(SizeType i=0; i!=A.row_size(); ++i) {
        for(SizeType j=0; j!=A.column_size(); ++j) {
            R.at(i,j)=cast_singleton(A.at(i,j));
        }
    }
    return R;
}


template<class X> Matrix<ExactType<X>> cast_exact(const Matrix<X>& A) {
    Matrix<ExactType<X>> R(A.row_size(),A.column_size(),cast_exact(A.zero_element()));
    for(SizeType i=0; i!=A.row_size(); ++i) {
        for(SizeType j=0; j!=A.column_size(); ++j) {
            R[i][j]=cast_exact(A[i][j]);
        }
    }
    return R;
}

} // namespace Ariadne
