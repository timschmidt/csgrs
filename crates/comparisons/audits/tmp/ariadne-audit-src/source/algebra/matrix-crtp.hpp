/***************************************************************************
 *            algebra/matrix-crtp.hpp
 *
 *  Copyright  2013-20  Pieter Collins
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

/*! \file algebra/matrix-crtp.hpp
 *  \brief
 */



#ifndef ARIADNE_MATRIX_HPP
#define ARIADNE_MATRIX_HPP

#include <initializer_list>

#include "vector.hpp"

namespace Ariadne {

template<class T> using InitializerList = InitializerList<T>;

/************ Matrix *********************************************************/

template<class A> struct MatrixExpression : Object<A> { };
template<class A> struct MatrixObject : MatrixExpression<A> { };

template<class X> class Matrix;

template<class M> class MatrixRow {
    M* _Ap; SizeType _i;
  public:
    MatrixRow(M* Ap, SizeType i) : _Ap(Ap), _i(i) { }
    auto operator[](SizeType j) -> decltype(_Ap->at(_i,j)) { return _Ap->at(_i,j); }
};

template<class X1, class X2> struct MatrixMatrixProduct {
    Matrix<X1> const& _a1; Matrix<X2> const& _a2;
    MatrixMatrixProduct(Matrix<X1> const& a1, Matrix<X2> const& a2) : _a1(a1), _a2(a2) { }
};

template<class X1, class X2> struct MatrixVectorProduct : VectorExpression<MatrixVectorProduct<X1,X2>> {
    Matrix<X1> const& _A1; Vector<X2> const& _v2;
    MatrixVectorProduct(Matrix<X1> const& A1, Vector<X2> const& v2) : _A1(A1), _v2(v2) { }
    SizeType size() const { return _A1.row_size(); }
    auto operator[](SizeType i) -> decltype(declval<X1>()*declval<X2>()) {
        decltype(declval<X1>()*declval<X2>()) r=0; for(SizeType j=0; j!=_v2.size(); ++j) { r+=_A1.at(i,j)*_v2.at(j); } return r; }
};


template<class X> class Matrix : public MatrixObject<Matrix<X>> {
    X _zero;
    SizeType _rs;
    SizeType _cs;
    Array<X> _ary;
  public:
    typedef X ScalarType;
    Matrix(SizeType m, SizeType n);
    Matrix(SizeType m, SizeType n, const X& x);
    Matrix(InitializerList<InitializerList<X>> lst);
    static Matrix<X> identity(SizeType n);
    template<class M> Matrix<X>& operator=(const MatrixExpression<M>& Ae);
    template<class M> Matrix<X>& operator+=(const MatrixExpression<M>& Ae);
    SizeType row_size() const { return _rs; }
    SizeType column_size() const { return _cs; }
    Void resize(SizeType m, SizeType n) {
        if(m*n != _rs*_cs) { _ary.resize(m*n); } _rs=m; _cs=n; }
    MatrixRow<const Matrix<X>> operator[](SizeType i) const { return MatrixRow<const Matrix<X>>{this,i}; }
    MatrixRow<Matrix<X>> operator[](SizeType i) { return MatrixRow<Matrix<X>>{this,i}; }
    const X& at(SizeType i, SizeType j) const { return this->_ary[i*this->_rs+this->_cs]; }
    X& at(SizeType i, SizeType j) { return this->_ary[i*this->_rs+this->_cs]; }
    Void set(SizeType i, SizeType j, const X& c) const { this->_ary[i*this->_rs+this->_cs]=c; }
    X zero_element() const { return _zero; }
    OutputStream& _write(OutputStream& os) const;

    Matrix<X>& operator=(const MatrixMatrixProduct<X,X>& A1mulA2);
  public:
    static Matrix<X> _mul(const Matrix<X>& A1, const Matrix<X>& A2);
};

template<class X> inline Matrix<X>::Matrix(SizeType m, SizeType n)
    : _zero(), _rs(m), _cs(n), _ary(m*n) {
}

template<class X> inline Matrix<X>::Matrix(SizeType m, SizeType n, const X& x)
    : _zero(x*0), _rs(m), _cs(n), _ary(m*n,x) {
}

template<class X> inline Matrix<X> Matrix<X>::identity(SizeType n) {
    Matrix<X> A(n,n,X(0));
    for(SizeType i=0; i!=n; ++i) { A.at(i,i)=1u; }
    return A;
}

template<class X> inline OutputStream& operator<<(OutputStream& os, const Matrix<X>& A) {
    return A._write(os); }


template<class X> Matrix<X>::Matrix(InitializerList<InitializerList<X>> lst) : _rs(lst.size()), _cs(lst.begin()->size()), _ary(_rs*_cs) {
    typename InitializerList<InitializerList<X>>::ConstIterator row_iter=lst.begin();
    for(SizeType i=0; i!=this->row_size(); ++i, ++row_iter) {
        ARIADNE_PRECONDITION(row_iter->size()==this->column_size());
        typename InitializerList<X>::ConstIterator col_iter=row_iter->begin();
        for(SizeType j=0; j!=this->column_size(); ++j, ++col_iter) {
            this->at(i,j)=*col_iter;
        }
    }
}

template<class X> template<class M> Matrix<X>& Matrix<X>::operator=(const MatrixExpression<M>& Ae) {
    const M& A=Ae.upcast();
    if(this!=&A) {
        this->resize(A.row_size(),A.column_size());
        for(SizeType i=0; i!=this->row_size(); ++i) {
            for(SizeType j=0; j!=this->column_size(); ++j) {
                this->at(i,j)=A.at(i,j);
            }
        }
    }
    return *this;
}


template<class X> template<class M> Matrix<X>& Matrix<X>::operator+=(const MatrixExpression<M>& Ae) {
    Matrix<X> const& A=Ae.upcast();
    for(SizeType i=0; i!=this->row_size(); ++i) {
        for(SizeType j=0; j!=this->column_size(); ++j) {
            this->at(i,j)+=A.at(i,j);
        }
    }
    return *this;
}

template<class X0, class X1, class X2> Void _mul_assign(Matrix<X0>& A0, Matrix<X1> const& A1, Matrix<X2> const& A2) {
    for(SizeType i=0; i!=A0.row_size(); ++i) {
        for(SizeType j=0; j!=A0.column_size(); ++j) {
            A0.at(i,j)=0;
            for(SizeType k=0; k!=A1.column_size(); ++k) {
                A0.at(i,j)+=A1.at(i,k)*A2.at(k,j);
            }
        }
    }
}

template<class X> Matrix<X>& Matrix<X>::operator=(const MatrixMatrixProduct<X,X>& A1mulA2) {
    Matrix<X> const& A1=A1mulA2._A1;
    Matrix<X> const& A2=A1mulA2._A2;
    if(this==&A1 || this==&A2) {
        Matrix<X> A0(A1.row_size(),A2.column_size());
        _mul_assign(A0,A1,A2);
        *this = std::move(A0);
    } else {
        Matrix<X>& A0=*this;
        A0.resize(A1.row_size(),A2.column_size());
        _mul_assign(A0,A1,A2);
    }
    return *this;
}

template<class X1, class X2> inline MatrixMatrixProduct<X1,X2> operator*(Matrix<X1> const& A1, Matrix<X2> const& A2) {
    return MatrixMatrixProduct<X1,X2>(A1,A2);
}

template<class M1, class X2> struct MatrixScalarQuotient : MatrixExpression<MatrixScalarQuotient<M1,X2>> {
    const M1& _a1; const X2& _x2;
    MatrixScalarQuotient(const M1& a1, const X2& x2) : _a1(a1), _x2(x2) { }
    SizeType row_size() const { return _a1.row_size(); }
    SizeType column_size() const { return _a1.column_size(); }
    auto at(SizeType i, SizeType j) const -> decltype(_a1.at(i,j)/_x2) { return _a1.at(i,j)/_x2; }
};
template<class M1, class X2> inline
MatrixScalarQuotient<M1,X2> operator/(const MatrixExpression<M1>& a1, const ScalarObject<X2>& x2) {
    return MatrixScalarQuotient<M1,X2>(a1.upcast(),x2.upcast()); }
template<class M> inline
MatrixScalarQuotient<M,typename M::ScalarType> operator/(const MatrixExpression<M>& a1, typename M::ScalarType const& x2) {
    return MatrixScalarQuotient<M,typename M::ScalarType>(a1.upcast(),x2); }


template<class X1, class X2> inline MatrixVectorProduct<X1,X2> operator*(Matrix<X1> const& A1, Vector<X2> const& v2) {
    return MatrixVectorProduct<X1,X2>(A1,v2);
}

template<class X> inline Matrix<X>& operator*=(Matrix<X>& A, typename Matrix<X>::ScalarType const& s) {
    for(SizeType i=0; i!=A.row_size(); ++i) {
        for(SizeType j=0; j!=A.column_size(); ++j) {
            A.at(i,j)*=s;
        }
    }
    return A;
}


template<class X> inline auto sup_norm(const Matrix<X>& A) -> decltype(mag(declval<X>())) {
    typedef decltype(mag(declval<X>())+mag(declval<X>())) R;
    R r=0u;
    for(SizeType i=0; i!=A.row_size(); ++i) {
        R e=0u;
        for(SizeType j=0; j!=A.column_size(); ++j) {
            e+=mag(A.at(i,j));
        }
        r=max(r,e);
    }
    return r;
}

template<class X> inline auto log_norm(const Matrix<X>& A) -> decltype(declval<X>()+mag(declval<X>())) {
    typedef decltype(declval<X>()+mag(declval<X>())) R;
    R r=0u;
    for(SizeType i=0; i!=A.row_size(); ++i) {
        R e=A.at(i,i);
        for(SizeType j=0; j!=A.column_size(); ++j) {
            if(j!=i) {
                e+=mag(A.at(i,j));
            }
        }
        r=max(r,e);
    }
    return r;
}


template<class X> class SymmetricMatrix : public Matrix<X>
{
  public:
    SymmetricMatrix(SizeType n) : Matrix<X>(n,n) { }
};


} // namespace Ariadne

#endif
