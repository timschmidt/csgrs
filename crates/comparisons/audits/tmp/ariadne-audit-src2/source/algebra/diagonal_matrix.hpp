/***************************************************************************
 *            algebra/diagonal_matrix.hpp
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

/*! \file algebra/diagonal_matrix.hpp
 *  \brief
 */



#ifndef ARIADNE_DIAGONAL_MATRIX_HPP
#define ARIADNE_DIAGONAL_MATRIX_HPP

#include <initializer_list>

#include "vector.hpp"
#include "matrix.hpp"

namespace Ariadne {

template<class T> using InitializerList = InitializerList<T>;

template<class X> class Matrix;
template<class X> class SymmetricMatrix;


template<class X> class DiagonalMatrix;

struct DiagonalMatrixOperations {

    template<class X> friend OutputStream& operator<<(OutputStream& os, DiagonalMatrix<X> const& A) {
        return A._write(os);
    }

    template<class X> friend DiagonalMatrix<NegationType<X>> operator-(DiagonalMatrix<X> A) {
        for(SizeType i=0; i!=A.size(); ++i) { const_cast<X&>(A.at(i,i))=-A.at(i,i); }
        return A;
    }

    template<class X> friend DiagonalMatrix<ProductType<X,X>> operator+(DiagonalMatrix<X> A1, DiagonalMatrix<X> const& A2) {
        ARIADNE_PRECONDITION(A1.size()==A2.size());
        for(SizeType i=0; i!=A1.size(); ++i) { const_cast<X&>(A1.at(i,i))+=A2.at(i,i); }
        return A1;
    }

    template<class X> friend DiagonalMatrix<ProductType<X,X>> operator-(DiagonalMatrix<X> A1, DiagonalMatrix<X> const& A2) {
        ARIADNE_PRECONDITION(A1.size()==A2.size());
        for(SizeType i=0; i!=A1.size(); ++i) { const_cast<X&>(A1.at(i,i))-=A2.at(i,i); }
        return A1;
    }

    template<class X> friend DiagonalMatrix<ProductType<X,X>> operator*(X const& s1, DiagonalMatrix<X> A2) {
        for(SizeType i=0; i!=A2.size(); ++i) { const_cast<X&>(A2.at(i,i))*=s1; }
        return A2;
    }

    template<class X> friend DiagonalMatrix<ProductType<X,X>> operator*(DiagonalMatrix<X> A1, X const& s2) {
        for(SizeType i=0; i!=A1.size(); ++i) { const_cast<X&>(A1.at(i,i))*=s2; }
        return A1;
    }

    template<class X> friend DiagonalMatrix<ProductType<X,X>> operator/(DiagonalMatrix<X> A1, X const& s2) {
        for(SizeType i=0; i!=A1.size(); ++i) { const_cast<X&>(A1.at(i,i))/=s2; }
        return A1;
    }

    template<class X> friend DiagonalMatrix<ProductType<X,X>> operator*(DiagonalMatrix<X> A1, DiagonalMatrix<X> const& A2) {
        if(A1.size()!=A2.size()) { std::cerr<<"A1="<<A1<<", A2="<<A2<<"\n"; }
        ARIADNE_PRECONDITION(A1.size()==A2.size());
        for(SizeType i=0; i!=A1.size(); ++i) { const_cast<X&>(A1.at(i,i))*=A2.at(i,i); }
        return A1;
    }

    template<class X> friend DiagonalMatrix<ProductType<X,X>> operator/(DiagonalMatrix<X> A1, DiagonalMatrix<X> const& A2) {
        if(A1.size()!=A2.size()) { std::cerr<<"A1="<<A1<<", A2="<<A2<<"\n"; }
        ARIADNE_PRECONDITION(A1.size()==A2.size());
        for(SizeType i=0; i!=A1.size(); ++i) { const_cast<X&>(A1.at(i,i))/=A2.at(i,i); }
        return A1;
    }

    template<class X> friend DiagonalMatrix<X> inverse(DiagonalMatrix<X> A) {
        for(SizeType i=0; i!=A.size(); ++i) { const_cast<X&>(A.at(i,i))=rec(A.at(i,i)); }
        return A;
    }

    template<class X> friend Matrix<X> operator+(Matrix<X> A, DiagonalMatrix<X> const& D) {
        ARIADNE_PRECONDITION(A.row_size()==D.size() && A.column_size()==D.size());
        for(SizeType i=0; i!=D.size(); ++i) { A.at(i,i)+=D.at(i,i); }
        return A;
    }

    template<class X> friend Matrix<X> operator+(DiagonalMatrix<X> const& D, Matrix<X> A) {
        ARIADNE_PRECONDITION(A.row_size()==D.size() && A.column_size()==D.size());
        for(SizeType i=0; i!=D.size(); ++i) { A.at(i,i)+=D.at(i,i); }
        return A;
    }

    template<class X> friend Matrix<X> operator-(Matrix<X> A, DiagonalMatrix<X> const& D) {
        ARIADNE_PRECONDITION(A.row_size()==D.size() && A.column_size()==D.size());
        for(SizeType i=0; i!=D.size(); ++i) { A.at(i,i)-=D.at(i,i); }
        return A;
    }

    template<class X> friend Matrix<X> operator-(DiagonalMatrix<X> const& D, Matrix<X> A) {
        ARIADNE_PRECONDITION(A.row_size()==D.size() && A.column_size()==D.size());
        for(SizeType i=0; i!=A.row_size(); ++i) {
            for(SizeType j=0; j!=A.column_size(); ++j) { A.at(i,j)=-A.at(i,j); }
            A.at(i,i)+=D.at(i,i);
        }
        return A;
    }

    template<class X> friend Matrix<X> operator*(Matrix<X> A, DiagonalMatrix<X> const& D) {
        ARIADNE_PRECONDITION(A.column_size()==D.size());
        for(SizeType j=0; j!=A.column_size(); ++j) {
            for(SizeType i=0; i!=A.row_size(); ++i) {
                A.at(i,j)*=D.at(j,j);
            }
        }
        return A;
    }

    template<class X> friend Matrix<X> operator*(DiagonalMatrix<X> const& D, Matrix<X> A) {
        ARIADNE_PRECONDITION(D.size()==A.row_size())
        for(SizeType i=0; i!=A.row_size(); ++i) {
            for(SizeType j=0; j!=A.column_size(); ++j) {
                A.at(i,j)*=D.at(i,i);
            }
        }
        return A;
    }

    template<class X> friend Matrix<X> operator*(DiagonalMatrix<X> const& D, Transpose<Matrix<X>> const& A) {
        ARIADNE_PRECONDITION(D.size()==A.row_size())
        Matrix<X> R(A.row_size(),A.column_size(),(A.zero_element()*D.zero_element()*A.zero_element()));
        for(SizeType i=0; i!=A.row_size(); ++i) {
            for(SizeType j=0; j!=A.column_size(); ++j) {
                R.at(i,j)=D.at(i,i)*A.at(i,j);
            }
        }
        return R;
    }

    template<class X> friend Vector<X> operator*(DiagonalMatrix<X> const& D, Vector<X> v) {
        ARIADNE_PRECONDITION(D.size()==v.size())
        for(SizeType i=0; i!=v.size(); ++i) {
            v.at(i)*=D.at(i,i);
        }
        return v;
    }

    template<class X> friend Vector<X> operator*(Vector<X> v, DiagonalMatrix<X> const& D) {
        ARIADNE_PRECONDITION(D.size()==v.size())
        for(SizeType i=0; i!=v.size(); ++i) {
            v.at(i)*=D.at(i,i);
        }
        return v;
    }

    template<class X> friend Vector<X> operator/(Vector<X> v, DiagonalMatrix<X> const& D) {
        ARIADNE_PRECONDITION(D.size()==v.size())
        for(SizeType i=0; i!=v.size(); ++i) {
            v.at(i)/=D.at(i,i);
        }
        return v;
    }
};


//! \ingroup LinearAlgebraSubModule
//! \brief Diagonal matrices over some type \a X.
template<class X> class DiagonalMatrix
    : DiagonalMatrixOperations
{
    X _zero;
    UniformArray<X> _ary;
  public:
    template<class... PRS> requires Constructible<X,Nat,PRS...> explicit DiagonalMatrix(SizeType n, PRS...);
    explicit DiagonalMatrix(Array<X>);
    explicit DiagonalMatrix(UniformArray<X>);
    explicit DiagonalMatrix(Vector<X>);
    template<class Y, class... PRS> requires Constructible<X,Y,PRS...> explicit DiagonalMatrix(DiagonalMatrix<Y> const&, PRS...);
    SizeType size() const;
    SizeType row_size() const;
    SizeType column_size() const;
    X const& zero_element() const;
    X const& operator[](SizeType i) const;
    X const& at(SizeType i, SizeType j) const;
    X const& get(SizeType i, SizeType j) const;
    ElementReference<X>& operator[](SizeType i);
    Void set(SizeType i, SizeType j, X const& x);
    Vector<X> diagonal() const;
    operator Matrix<X>() const;
    operator SymmetricMatrix<X>() const;
  private: public:
    OutputStream& _write(OutputStream&) const;
};

template<class X> template<class... PRS> requires Constructible<X,Nat,PRS...>
DiagonalMatrix<X>::DiagonalMatrix(SizeType n, PRS... prs)
    : _zero(0u,prs...), _ary(n,_zero)
{ }

template<class X> DiagonalMatrix<X>::DiagonalMatrix(Array<X> ary)
    : _zero(create_zero(ary[0])), _ary(ary)
{ }

template<class X> DiagonalMatrix<X>::DiagonalMatrix(UniformArray<X> ary)
    : _zero(ary.default_element()), _ary(ary)
{ }

template<class X> DiagonalMatrix<X>::DiagonalMatrix(Vector<X> vec)
    : _zero(vec.zero_element()), _ary(vec.array())
{ }

template<class X> template<class Y, class... PRS> requires Constructible<X,Y,PRS...>
DiagonalMatrix<X>::DiagonalMatrix(DiagonalMatrix<Y> const& D, PRS... prs)
    : _zero(D.zero_element(),prs...), _ary(D.diagonal().array(),prs...)
{ }

template<class X> SizeType DiagonalMatrix<X>::size() const {
    return this->_ary.size();
}

template<class X> SizeType DiagonalMatrix<X>::row_size() const {
    return this->_ary.size();
}

template<class X> SizeType DiagonalMatrix<X>::column_size() const {
    return this->_ary.size();
}

template<class X> X const& DiagonalMatrix<X>::zero_element() const {
    return this->_zero;
}

template<class X> X const& DiagonalMatrix<X>::operator[](SizeType i) const {
    return this->_ary[i];
}

template<class X> ElementReference<X>& DiagonalMatrix<X>::operator[](SizeType i) {
    return this->_ary[i];
}

template<class X> X const& DiagonalMatrix<X>::at(SizeType i, SizeType j) const {
    if (i==j) { return _ary[i]; } else { return _zero; }
}

template<class X> X const& DiagonalMatrix<X>::get(SizeType i, SizeType j) const {
    if (i==j) { return _ary[i]; } else { return _zero; }
}

template<class X> Void DiagonalMatrix<X>::set(SizeType i, SizeType j, X const& x) {
    ARIADNE_PRECONDITION(i==j);
    _ary[i]=x;
}

template<class X> Vector<X> DiagonalMatrix<X>::diagonal() const {
    return Vector<X>(this->_ary);
}

template<class X> DiagonalMatrix<X>::operator Matrix<X> () const {
    Matrix<X> A(this->row_size(),this->column_size(),this->zero_element());
    for(SizeType i=0; i!=this->size(); ++i) { A[i][i]=this->_ary[i]; }
    return A;
}

template<class X> OutputStream& DiagonalMatrix<X>::_write(OutputStream& os) const {
    return os << "diag(" << this->_ary << ")";
}

} // namespace Ariadne

#endif
