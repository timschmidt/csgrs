/***************************************************************************
 *            algebra/symmetric_matrix.hpp
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

/*! \file algebra/symmetric_matrix.hpp
 *  \brief Symmetric matrices
 */

#ifndef ARIADNE_SYMMETRIC_MATRIX_HPP
#define ARIADNE_SYMMETRIC_MATRIX_HPP

#include <initializer_list>

#include "utility/metaprogramming.hpp"
#include "vector.hpp"

namespace Ariadne {

template<class T> using InitializerList = std::initializer_list<T>;

/************ Matrix *********************************************************/

template<class X> class Vector;
template<class X> class Covector;

template<class X> class Matrix;

class SingularMatrixException;

class NonSymmetricMatrixException { };



//! \ingroup LinearAlgebraSubModule
//! \brief Symmetric matrices over some type \a X.
template<class X> class SymmetricMatrix
    : public MatrixContainer<Matrix<X>>
{
    X _zero;
    SizeType _s;
    Array<X> _ary;
  public:
    typedef X ScalarType;
    typedef X ValueType;
  public:

    //!@{
    //! \name Constructors

    //! Destructor
    ~SymmetricMatrix();

    //! Construct a symmetric matrix with \a n rows and \a n columns with values initialised to \a z.
    SymmetricMatrix(SizeType n, X const& z);

    //! Construct a symmetric matrix with \a n rows and \a n columns with values initialised to zero.
    template<class... PRS> requires Constructible<X,Nat,PRS...>
    SymmetricMatrix(SizeType n, PRS... prs) : SymmetricMatrix(n,X(0u,prs...)) { }

    //! Construct a matrix using initializer lists.
    SymmetricMatrix(InitializerList<InitializerList<X>> lst);

    //!@{
    //! \name Static constructors

    //! \brief The zero matrix with \a n rows and \a n columns.
    template<class... PRS> requires Constructible<X,Nat,PRS...>
    static SymmetricMatrix<X> zero(SizeType n, PRS... prs);
    //! \brief The itentity matrix with \a n rows and \a n columns.
    template<class... PRS> requires Constructible<X,Nat,PRS...>
    static SymmetricMatrix<X> identity(SizeType n, PRS... prs);
    //!@}

    //! Explicit conversion of a normal matrix to a symmetric matrix.
    SymmetricMatrix(const Matrix<X>& A);

    //! Conversion of a symmetric matrix to a dense matrix.
    operator Matrix<X> () const;

    //! \brief The number of rows (equivalently, columns) of the matrix.
    SizeType size() const;
     //! \brief The number of rows of the matrix.
    SizeType row_size() const;
    //! \brief The number of columns of the matrix.
    SizeType column_size() const;
    //! \brief Resize to an \a m by \a n matrix.
    Void resize(SizeType n);
    //! \brief Get the value stored in the \a i<sup>th</sup> row and \a j<sup>th</sup> column.
    X& at(SizeType i, SizeType j);
    const X& at(SizeType i, SizeType j) const;
    //! \brief Get the value stored in the \a i<sup>th</sup> row and \a j<sup>th</sup> column.
    const X& get(SizeType i, SizeType j) const ;
    //! \brief Set the value stored in the \a i<sup>th</sup> row and \a j<sup>th</sup> column to \a x.
    Void set(SizeType i, SizeType j, const X& x);
    //! \brief A pointer to the first element of the data storage.
    const X* begin() const;

#ifdef DOXYGEN
    //! \brief C-style subscripting operator.
    X& operator[][](SizeType i, SizeType j);
    //! \brief C-style constant subscripting operator.
    const X& operator[][](SizeType i, SizeType j) const;
#else
    MatrixRow<const SymmetricMatrix<X>> operator[](SizeType i) const;
    MatrixRow<SymmetricMatrix<X>> operator[](SizeType i);
#endif
    //! \brief The zero element of the field/algebra of the matrix.
    X zero_element() const;
  public:
    template<class T> friend OutputStream& operator<<(OutputStream& os, SymmetricMatrix<T>const& A);
    template<class T> friend SymmetricMatrix<T> operator+(SymmetricMatrix<T> const&, SymmetricMatrix<T> const&);
    template<class T> friend SymmetricMatrix<T> operator-(SymmetricMatrix<T> const&, SymmetricMatrix<T> const&);
    template<class T> friend SymmetricMatrix<T> operator*(T const&, SymmetricMatrix<T> const&);
    template<class T> friend SymmetricMatrix<T> operator*(SymmetricMatrix<T> const&, T const&);
    template<class T> friend SymmetricMatrix<T> operator/(SymmetricMatrix<T> const&, T const&);
    template<class T> friend SymmetricMatrix<T> symmetrize(Matrix<T> const& A);
    template<class T> friend SymmetricMatrix<T> outer(Matrix<T> const& A);
    template<class T> friend SymmetricMatrix<T> AxTpTA(Matrix<T> const&, SymmetricMatrix<T> const&);
  private:
    Void _check_data_access(SizeType i, SizeType j) const;
    OutputStream& _write(OutputStream& os) const;

    friend Vector<X>& to_vector(SymmetricMatrix<X>& S) { return reinterpret_cast<Vector<X>&>(S._ary); }
    friend Vector<X> const& to_vector(SymmetricMatrix<X> const& S) { return reinterpret_cast<Vector<X>const&>(S._ary); }
  private:
    SizeType _fast_position(SizeType i, SizeType j) const { assert(i<=j); return i*(2*this->size()-i-1)/2+j; }
    SizeType _position(SizeType i, SizeType j) const { if(i<=j) { return this->_fast_position(i,j); } else { return this->_fast_position(j,i); } }
};

template<class X> Vector<X>& to_vector(SymmetricMatrix<X>& S);
template<class X> Vector<X> const& to_vector(SymmetricMatrix<X> const& S);




template<class X> SymmetricMatrix<X>::~SymmetricMatrix() {
}

template<class X> SymmetricMatrix<X>::SymmetricMatrix(SizeType n, X const& z)
    : _zero(z), _s(n), _ary(n*(n+1)/2,z)
{
}

template<class X> SymmetricMatrix<X>::SymmetricMatrix(Matrix<X> const& A)
    : _zero(A.zero_element()), _s((assert(A.row_size()==A.column_size()),A.row_size())), _ary(_s*(_s+1)/2,_zero)
{
    const SizeType n=this->_s;
    for(SizeType i=0; i!=n; ++i) {
        this->_ary[this->_fast_position(i,i)]=A[i][i];
        for(SizeType j=i+1; j!=n; ++j) {
            if(decide(A[i][j]!=A[j][i])) { throw NonSymmetricMatrixException(); }
            this->_ary[this->_fast_position(i,j)]=A[i][j];
        }
    }
}

template<class X> template<class... PRS> requires Constructible<X,Nat,PRS...>
SymmetricMatrix<X> SymmetricMatrix<X>::zero(SizeType n, PRS... prs) {
    return SymmetricMatrix<X>(n,X(0u,prs...));
}

template<class X> template<class... PRS> requires Constructible<X,Nat,PRS...>
SymmetricMatrix<X> SymmetricMatrix<X>::identity(SizeType n, PRS... prs) {
    SymmetricMatrix<X> S(n,X(0u,prs...));
    for(SizeType i=0; i!=n; ++i) {
        S[i][i]=1u;
    }
    return S;
}

template<class X> inline SizeType SymmetricMatrix<X>::size() const {
    return this->_s;
}

template<class X> inline SizeType SymmetricMatrix<X>::row_size() const {
    return this->_s;
}

template<class X> inline SizeType SymmetricMatrix<X>::column_size() const {
    return this->_s;
}

template<class X> inline Void SymmetricMatrix<X>::resize(SizeType n) {
    this->_s = n;
    this->_ary.resize(n*(n-1)/2,this->_zero);
}

template<class X> inline X& SymmetricMatrix<X>::at(SizeType i, SizeType j) {
    return this->_ary[this->_position(i,j)];
}

template<class X> inline const X& SymmetricMatrix<X>::at(SizeType i, SizeType j) const {
    return this->_ary[this->_position(i,j)];
}

template<class X> inline const X& SymmetricMatrix<X>::get(SizeType i, SizeType j) const {
    return this->_ary[this->_position(i,j)];
}

template<class X> inline Void SymmetricMatrix<X>::set(SizeType i, SizeType j, const X& x) {
    this->_ary[this->_position(i,j)]=x;
}

template<class X> inline const X* SymmetricMatrix<X>::begin() const {
    return this->_ary.begin();
}

#ifndef DOXYGEN
template<class X> inline MatrixRow<const SymmetricMatrix<X>> SymmetricMatrix<X>::operator[](SizeType i) const {
    return MatrixRow<const SymmetricMatrix<X>>(*this,i);
}
template<class X> inline MatrixRow<SymmetricMatrix<X>> SymmetricMatrix<X>::operator[](SizeType i) {
    return MatrixRow<SymmetricMatrix<X>>(*this,i);
}
#endif

template<class X> inline X SymmetricMatrix<X>::zero_element() const {
    return _zero;
}

template<class X> inline Void SymmetricMatrix<X>::_check_data_access(SizeType i, SizeType j) const {
    assert(i<this->row_size() && j<this->column_size());
}

template<class X> inline OutputStream& operator<<(OutputStream& os, SymmetricMatrix<X>const& S) {
    return S._write(os);
}



template<class X> SymmetricMatrix<X>::operator Matrix<X>() const {
    SymmetricMatrix<X> const& S=*this;
    Matrix<X> A(S.row_size(),S.column_size(),S.zero_element());
    for(SizeType i=0; i!=S.row_size(); ++i) {
        A[i][i]=S.at(i,i);
        for(SizeType j=i+1; j!=S.row_size(); ++j) {
            A[i][j]=S.at(i,j);
            A[j][i]=S.at(i,j);
        }
    }
    return A;
}


template<class X> OutputStream& SymmetricMatrix<X>::_write(OutputStream& os) const {
    return os << Matrix<X>(*this);
}

template<class X> SymmetricMatrix<X> operator+(SymmetricMatrix<X> const& S1, SymmetricMatrix<X> const& S2) {
    assert(S1.size()==S2.size());
    const SizeType n=S1.size();
    SymmetricMatrix<X> R(n,S1.zero_element()+S2.zero_element());
    for(SizeType i=0; i!=n*(n+1)/2; ++i) {
        R._ary[i]=S1._ary[i]+S2._ary[i];
    }
    return R;
}

template<class X> SymmetricMatrix<X> operator-(SymmetricMatrix<X> const& S1, SymmetricMatrix<X> const& S2) {
    assert(S1.size()==S2.size());
    const SizeType n=S1.size();
    SymmetricMatrix<X> R(n,S1.zero_element()-S2.zero_element());
    for(SizeType i=0; i!=n*(n+1)/2; ++i) {
        R._ary[i]=S1._ary[i]-S2._ary[i];
    }
    return R;
}

template<class X> SymmetricMatrix<X> operator*(X const& x1, SymmetricMatrix<X> const& S2) {
    const SizeType n=S2.size();
    SymmetricMatrix<X> R(n,x1*S2.zero_element());
    for(SizeType i=0; i!=n*(n+1)/2; ++i) {
        R._ary[i]=x1*S2._ary[i];
    }
    return R;
}

template<class X> SymmetricMatrix<X> operator*(SymmetricMatrix<X> const& S1, X const& x2) {
    const SizeType n=S1.size();
    SymmetricMatrix<X> R(n,S1.zero_element()*x2);
    for(SizeType i=0; i!=n*(n+1)/2; ++i) {
        R._ary[i]=S1._ary[i]*x2;
    }
    return R;
}

template<class X> SymmetricMatrix<X> operator/(SymmetricMatrix<X> const& S1, X const& x2) {
    const SizeType n=S1.size();
    SymmetricMatrix<X> R(n,S1.zero_element()/x2);
    for(SizeType i=0; i!=n*(n+1)/2; ++i) {
        R._ary[i]=S1._ary[i]/x2;
    }
    return R;
}

template<class X> SymmetricMatrix<X> outer(Matrix<X> const& A) {
    const SizeType m=A.row_size();
    const SizeType n=A.column_size();
    SymmetricMatrix<X> S(n,A.zero_element());
    for(SizeType i=0; i!=n; ++i) {
        for(SizeType j=i; j!=n; ++j) {
            X& Sij=S.at(i,j);
            Sij=0;
            for(SizeType k=0; k!=m; ++k) {
                Sij+=A[k][i]*A[k][j];
            }
        }
    }
    return S;
}

template<class X> SymmetricMatrix<X> outer(SymmetricMatrix<X> const& A) {
    const SizeType m=A.row_size();
    const SizeType n=A.column_size();
    SymmetricMatrix<X> S(n,A.zero_element());
    for(SizeType i=0; i!=n; ++i) {
        for(SizeType j=i; j!=n; ++j) {
            X& Sij=S.at(i,j);
            Sij=0;
            for(SizeType k=0; k!=m; ++k) {
                Sij+=A[k][i]*A[k][j];
            }
        }
    }
    return S;
}

template<class X> SymmetricMatrix<X> symmetrize(Matrix<X> const& A) {
    assert(A.row_size()==A.column_size());
    const SizeType n=A.row_size();
    SymmetricMatrix S(n,A.zero_element());
    for(SizeType i=0; i!=n; ++i) {
        S.at(i,i)=A[i][i];
        for(SizeType j=i+1; j!=n; ++j) {
            S.at(i,j)=hlf(A[i][j]+A[j][i]);
        }
    }
    return S;
}


} // namespace Ariadne

#endif
