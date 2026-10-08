/***************************************************************************
 *            algebra/matrix.cpp
 *
 *  Copyright  2008-20  Alberto Casagrande, Pieter Collins
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

#include "numeric/module.hpp"

#include "config.hpp"

#include "matrix.hpp"
#include "symmetric_matrix.hpp"

#include "utility/exceptions.hpp"
#include "numeric/floats.hpp"
#include "numeric/dyadic.hpp"
#include "numeric/rational.hpp"
#include "numeric/casts.hpp"
#include "algebra/vector.hpp"
#include "algebra/covector.hpp"

#include "matrix.tpl.hpp"

namespace Ariadne {

OutputStream& operator<<(OutputStream& os, const PivotMatrix& pv) {
    return os << "PivotMatrix(" << static_cast< Matrix<Int> >(pv) << ")";
}

template<> Matrix<FloatDPBounds> inverse<>(const Matrix<FloatDP>& A) {
    return lu_inverse(Matrix<FloatDPBounds>(A));
}

template<> Matrix<FloatDPBounds> inverse<FloatDPBounds>(const Matrix<FloatDPBounds>& A) {
    try {
        return lu_inverse(A);
    } catch(const DivideByZeroException& e) {
        ARIADNE_THROW(SingularMatrixException,"inverse(Matrix<"<<class_name<FloatDPBounds>()<<"> A)","A="<<A);
    }
}

template<> Matrix<FloatDPBounds> solve(const Matrix<FloatDPBounds>& A, const Matrix<FloatDPBounds>& B) {
    return gs_solve(A,B);
}


template class Matrix<RoundedFloatDP>;
template Matrix<RoundedFloatDP> inverse(const Matrix<RoundedFloatDP>&);
template Vector<RoundedFloatDP> solve(const Matrix<RoundedFloatDP>&, const Vector<RoundedFloatDP>&);
template Void normalise_rows(Matrix<RoundedFloatDP>&);


template class Matrix<FloatDPApproximation>;
template Matrix<FloatDPApproximation> inverse(const Matrix<FloatDPApproximation>&);
template Matrix<FloatDPApproximation> solve(const Matrix<FloatDPApproximation>&, const Matrix<FloatDPApproximation>&);
template Vector<FloatDPApproximation> solve(const Matrix<FloatDPApproximation>&, const Vector<FloatDPApproximation>&);
template Tuple<PivotMatrix,Matrix<FloatDPApproximation>,Matrix<FloatDPApproximation>> triangular_decomposition(Matrix<FloatDPApproximation> const&);
template Tuple<Matrix<FloatDPApproximation>,Matrix<FloatDPApproximation>> orthogonal_decomposition(Matrix<FloatDPApproximation> const&);
template Tuple<Matrix<FloatDPApproximation>,Matrix<FloatDPApproximation>> gram_schmidt_orthogonalisation(Matrix<FloatDPApproximation> const&);
template Tuple<Matrix<FloatDPApproximation>,Matrix<FloatDPApproximation>,PivotMatrix> orthogonal_decomposition(Matrix<FloatDPApproximation> const&, Bool);
template Vector<FloatDPApproximation> row_norms(Matrix<FloatDPApproximation> const&);
template Matrix<FloatDPApproximation> operator*(PivotMatrix, Matrix<FloatDPApproximation>);

template class Matrix<FloatDPBounds>;
// template Matrix<FloatDPBounds> inverse(const Matrix<FloatDPBounds>&);
template Matrix<FloatDPBounds> lu_inverse(const Matrix<FloatDPBounds>&);
template Matrix<FloatDPBounds> gs_inverse(const Matrix<FloatDPBounds>&);
template Matrix<FloatDPBounds> lu_solve(const Matrix<FloatDPBounds>&, const Matrix<FloatDPBounds>&);
template Vector<FloatDPBounds> solve(const Matrix<FloatDPBounds>&, const Vector<FloatDPBounds>&);
template Vector<FloatDPBounds> lu_solve(const Matrix<FloatDPBounds>&, const Vector<FloatDPBounds>&);
template Vector<FloatDPBounds> gs_solve(const Matrix<FloatDPBounds>&, const Vector<FloatDPBounds>&);
template Matrix<FloatDPBounds> gs_solve(const Matrix<FloatDPBounds>&, const Matrix<FloatDPBounds>&);
template Matrix<MidpointType<FloatDPBounds>> midpoint(Matrix<FloatDPBounds> const&);
template Tuple<PivotMatrix,Matrix<FloatDPBounds>,Matrix<FloatDPBounds>> triangular_decomposition(Matrix<FloatDPBounds> const&);
template Tuple<Matrix<FloatDPBounds>,Matrix<FloatDPBounds>> orthogonal_decomposition(Matrix<FloatDPBounds> const&);
template Tuple<Matrix<FloatDPBounds>,Matrix<FloatDPBounds>> gram_schmidt_orthogonalisation(Matrix<FloatDPBounds> const&);
template Matrix<FloatDPBounds> operator*(PivotMatrix, Matrix<FloatDPBounds>);

template class Matrix<FloatDP>;
template Matrix<ExactType<FloatDPApproximation>> cast_exact(const Matrix<FloatDPApproximation>&);


template class Matrix<FloatMPApproximation>;
template Matrix<FloatMPApproximation> inverse(const Matrix<FloatMPApproximation>&);
template Matrix<FloatMPApproximation> solve(const Matrix<FloatMPApproximation>&, const Matrix<FloatMPApproximation>&);
template Vector<FloatMPApproximation> solve(const Matrix<FloatMPApproximation>&, const Vector<FloatMPApproximation>&);
template Tuple<PivotMatrix,Matrix<FloatMPApproximation>,Matrix<FloatMPApproximation>> triangular_decomposition(Matrix<FloatMPApproximation> const&);
template Tuple<Matrix<FloatMPApproximation>,Matrix<FloatMPApproximation>> orthogonal_decomposition(Matrix<FloatMPApproximation> const&);
template Tuple<Matrix<FloatMPApproximation>,Matrix<FloatMPApproximation>> gram_schmidt_orthogonalisation(Matrix<FloatMPApproximation> const&);
template Tuple<Matrix<FloatMPApproximation>,Matrix<FloatMPApproximation>,PivotMatrix> orthogonal_decomposition(Matrix<FloatMPApproximation> const&, Bool);
template Vector<FloatMPApproximation> row_norms(Matrix<FloatMPApproximation> const&);
template Matrix<FloatMPApproximation> operator*(PivotMatrix, Matrix<FloatMPApproximation>);

template class Matrix<FloatMPBounds>;
template Matrix<FloatMPBounds> inverse(const Matrix<FloatMPBounds>&);
template Matrix<FloatMPBounds> lu_inverse(const Matrix<FloatMPBounds>&);
template Matrix<FloatMPBounds> gs_inverse(const Matrix<FloatMPBounds>&);
template Matrix<FloatMPBounds> lu_solve(const Matrix<FloatMPBounds>&, const Matrix<FloatMPBounds>&);
template Vector<FloatMPBounds> solve(const Matrix<FloatMPBounds>&, const Vector<FloatMPBounds>&);
template Vector<FloatMPBounds> lu_solve(const Matrix<FloatMPBounds>&, const Vector<FloatMPBounds>&);
template Vector<FloatMPBounds> gs_solve(const Matrix<FloatMPBounds>&, const Vector<FloatMPBounds>&);
template Matrix<FloatMPBounds> gs_solve(const Matrix<FloatMPBounds>&, const Matrix<FloatMPBounds>&);
template Matrix<MidpointType<FloatMPBounds>> midpoint(Matrix<FloatMPBounds> const&);
template Tuple<PivotMatrix,Matrix<FloatMPBounds>,Matrix<FloatMPBounds>> triangular_decomposition(Matrix<FloatMPBounds> const&);
template Tuple<Matrix<FloatMPBounds>,Matrix<FloatMPBounds>> orthogonal_decomposition(Matrix<FloatMPBounds> const&);
template Tuple<Matrix<FloatMPBounds>,Matrix<FloatMPBounds>> gram_schmidt_orthogonalisation(Matrix<FloatMPBounds> const&);
template Matrix<FloatMPBounds> operator*(PivotMatrix, Matrix<FloatMPBounds>);

template class Matrix<FloatMP>;
template Matrix<ExactType<FloatMPApproximation>> cast_exact(const Matrix<FloatMPApproximation>&);


template class Matrix<Real>;

template PositiveFloatDPUpperBound sup_norm(const Matrix<FloatDPBounds>& A);
template FloatDPUpperBound log_norm(const Matrix<FloatDPBounds>& A);

template class Matrix<Dyadic>;

template class Matrix<Rational>;
template Matrix<Rational> inverse(const Matrix<Rational>&);
template Matrix<Rational> solve(const Matrix<Rational>&, const Matrix<Rational>&);
template Vector<Rational> solve(const Matrix<Rational>&, const Vector<Rational>&);
Rational midpoint(Rational);
template<> Matrix<Rational> midpoint(Matrix<Rational> const& A) { return A; }

template class SymmetricMatrix<FloatDPApproximation>;

} // namespace Ariadne

#include "geometry/interval.hpp"

namespace Ariadne {
template class Matrix<FloatDPUpperInterval>;
template Matrix<SingletonType<FloatDPUpperInterval>> cast_singleton(Matrix<FloatDPUpperInterval> const&);
template Matrix<MidpointType<FloatDPUpperInterval>> midpoint(Matrix<FloatDPUpperInterval> const&);
template Matrix<FloatDPUpperInterval> inverse(const Matrix<FloatDPUpperInterval>&);
template Vector<FloatDPUpperInterval> solve(const Matrix<FloatDPUpperInterval>&, const Vector<FloatDPUpperInterval>&);

template class Matrix<FloatMPUpperInterval>;
} // namespace Ariadne

