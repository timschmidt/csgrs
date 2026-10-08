/***************************************************************************
 *            riccati.hpp
 *
 *  Copyright  2013-14  Pieter Collins
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

/*! \file riccati.hpp
 *  \brief Solution of algebraic Riccati equations.
 */

#include "numeric/float-user.hpp"
#include "algebra/matrix.hpp"
#include "algebra/symmetric_matrix.hpp"

namespace Ariadne {

template<class X> Matrix<X> operator*(Matrix<X> A, SymmetricMatrix<X> S) { return A*Matrix<X>(S); }
template<class X> Matrix<X> operator*(SymmetricMatrix<X> S, Matrix<X> A) { return Matrix<X>(S)*A; }
template<class X> Matrix<X> operator*(Transpose<Matrix<X>> AT, SymmetricMatrix<X> S) { return Matrix<X>(AT)*S; }

template<class T> SymmetricMatrix<T> ATXpXA(Matrix<T> const& A, SymmetricMatrix<T> const& X) {
    assert(X.size()==A.row_size());
    assert(X.size()==A.column_size());
    const SizeType n=A.column_size();
    SymmetricMatrix<T> S(n);
    for(SizeType i=0; i!=n; ++i) {
        for(SizeType j=i; j!=n; ++j) {
            T& Sij=S.at(i,j);
            Sij=0;
            for(SizeType k=0; k!=n; ++k) {
                Sij+=A[k][i]*X[k][j];
                Sij+=X[k][i]*A[k][j];
            }
        }
    }
    return std::move(S);
}

template<class T> struct Outer {
    SymmetricMatrix<T> _S;
  public:
    SymmetricMatrix<T> operator() (SymmetricMatrix<T> const& X) {
        Matrix<T> P=X; return symmetrize(P*_S*P); }
};
struct OuterProductGenerator { template<class T> Outer<T> operator[](SymmetricMatrix<T> S) const { return Outer<T>{S}; } };

const OuterProductGenerator outer_product;

template<class T> class LyapounovEquation {
    Matrix<T> _A; SymmetricMatrix<T> _CTC;
    LyapounovEquation(Matrix<T>& A, Matrix<T>& C)
        : _A(A), _CTC(outer(C)) { }

    Vector<T> operator() (Vector<T> const& v) {
        SizeType n=_CTC.size();
        SymmetricMatrix<T> X(n);
        to_vector<T>(X)=v;
        const SymmetricMatrix<T> R=ATXpXA(_A,X)+_CTC;
        return to_vector<T>(R);
    }
};

template class LyapounovEquation<Float>;

// The Riccati equation AT X + X A - X S XT + Q
// or  AT X + X A + Q - X XT + Q
template<class T> class RiccatiEquation {
    Matrix<T> _A; SymmetricMatrix<T> _S; SymmetricMatrix<T> _Q;
  public:
    RiccatiEquation(Matrix<T>& A, SymmetricMatrix<T>& S, SymmetricMatrix<T>& Q)
        : _A(A), _S(S), _Q(Q) { }


    RiccatiEquation(Matrix<T>& A, SymmetricMatrix<T>& Q)
        : _A(A), _S(SymmetricMatrix<T>::identity(Q.size())), _Q(Q) { }

    SymmetricMatrix<T> operator() (SymmetricMatrix<T> const& X) const {
        return ATXpXA(_A,X)-outer_product[_S](X)+_Q; }

    Vector<T> operator() (Vector<T> const& v) const {
        SizeType n=_Q.size();
        SymmetricMatrix<T> X(n);
        to_vector<T>(X)=v;
        SymmetricMatrix<T> R=(*this)(X);
        return to_vector<T>(R);
    }
};

template class RiccatiEquation<Float>;

}
