/***************************************************************************
 *            riccati.cc
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

#include "numeric/float-user.hpp"
#include "algebra/matrix.hpp"
#include "algebra/symmetric_matrix.hpp"

#include "riccati.hpp"

namespace Ariadne {

typedef SymmetricMatrix<FloatDPApproximation> SymmetricFloatMatrix;
typedef Matrix<Float> FloatMatrix;

template<class X> SymmetricMatrix<X> inverse(SymmetricMatrix<X> const& S) {
    return SymmetricMatrix<X>(inverse(Matrix<X>(S)));
}

template<class X> SymmetricMatrix<X>::SymmetricMatrix(InitializerList<InitializerList<X>> lst)
    : SymmetricMatrix<X>(Matrix<X>(lst))
{
}

template class SymmetricMatrix<Float>;

template<class T> SymmetricMatrix<T> ATXpXA(Matrix<T> const& A, SymmetricMatrix<T> const& X);
template SymmetricMatrix<Float> ATXpXA(Matrix<Float> const& A, SymmetricMatrix<Float> const& S);
template SymmetricMatrix<Float> operator+(SymmetricMatrix<Float> const& S1, SymmetricMatrix<Float> const& S2);

}

using namespace Ariadne;

int main() {
    // Set up and solve a the Riccati equation
    // For the optimal control problem with dx/dt = Ax+Bu, with cost dV/dt = x^TQx+u^TRu
    // the optimal control is given by solving the algebraic Riccati equation
    //    A^T P + P A - P S P + Q = 0
    // where S = B R^-1 B^T
    FloatMatrix A={{-1.00,-1.25},{0.32,1.16}};
    FloatMatrix B={{-0.22},{2.12}};
    std::cout << "\nA="<<A<<"\nB="<<B<<"\n";
    SymmetricFloatMatrix Q={{2.61,1.27},{1.27,2.95}};
    SymmetricFloatMatrix R={{0.0014}};
    std::cout << "Q="<<Q<<"\nR="<<R<<"\n";

    // Set up the Riccati function
    SymmetricFloatMatrix S=symmetrize(B*inverse(R)*transpose(B));
    std::cout << "S="<<S<<"\n\n";
    RiccatiEquation<Float> riccati(A,S,Q);

    // Evaluate on a simple matrix
    SymmetricFloatMatrix X(2);
    //X={{2,3},{3,5}};
    auto E=riccati(X);
    std::cout<<"X="<<X<<"\n";
    std::cout<<"E="<<E<<"\n";

    // Check that vectorization of the algorithm works
    FloatVector vX=to_vector(X);
    std::cout<<"vX="<<vX<<"\n";
    auto vE=riccati(vX);
    std::cout<<"vE="<<vE<<"\n";

}
