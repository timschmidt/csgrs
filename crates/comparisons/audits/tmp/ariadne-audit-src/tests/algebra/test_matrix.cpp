/***************************************************************************
 *            test_matrix.cpp
 *
 *  Copyright  2006-20  Pieter Collins, Alberto Casagrande
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

#define NO_CBLAS

#include <iostream>
#include <fstream>

#include "config.hpp"

#include "../test.hpp"

#include "numeric/numeric.hpp"
#include "algebra/vector.hpp"
#include "algebra/matrix.hpp"
#include "algebra/covector.hpp"

namespace Ariadne {
typedef Matrix<FloatDPApproximation> FloatApproximationMatrix;
}

using namespace std;
using namespace Ariadne;


class TestMatrix {
    DoublePrecision pr;
  public:
    Void test();
  private:
    Void test_concept();
    Void test_project();
    Void test_factorisations();
    Void test_misc();
};

Void
TestMatrix::test()
{
    ARIADNE_TEST_CALL(test_project());
    ARIADNE_TEST_CALL(test_misc());
}

Void
TestMatrix::test_concept()
{
    FloatDPApproximation fx(1,dp);
    FloatDPBounds ix(1,dp);
    FloatDP ex(1,dp);
    Vector<FloatDPApproximation> fv;
    Vector<FloatDPBounds> iv;
    Vector<FloatDP> ev;
    Matrix<FloatDPApproximation> fA(0,0,dp);
    Matrix<FloatDPBounds> iA(0,0,dp);
    Matrix<FloatDP> eA(0,0,dp);

    fv=fv+fv; iv=ev+ev; iv=ev+iv; iv=iv+ev; iv=iv+iv;
    fv=fv-fv; iv=ev-ev; iv=ev-iv; iv=iv-ev; iv=iv-iv;
    fv=fx*fv; iv=ex*ev; iv=ex*iv; iv=ix*ev; iv=ix*iv;
    fv=fv*fx; iv=ev*ex; iv=ev*ix; iv=iv*ex; iv=iv*ix;
    fv=fv/fx; iv=ev/ex; iv=ev/ix; iv=iv/ex; iv=iv/ix;

    fA=fA+fA; iA=eA+eA; iA=eA+iA; iA=iA+eA; iA=iA+iA;
    fA=fA-fA; iA=eA-eA; iA=eA-iA; iA=iA-eA; iA=iA-iA;

    fA=fx*fA; iA=ex*eA; iA=ex*iA; iA=ix*eA; iA=ix*iA;
    fA=fA*fx; iA=eA*ex; iA=eA*ix; iA=iA*ex; iA=iA*ix;
    //fv=fA*fv; iv=eA*ev; iv=eA*iv; iv=iA*ev; iv=iA*iv;
    //fA=fA*fA; iA=eA*eA; iA=eA*iA; iA=iA*eA; iA=iA*iA;

    fv=fA*fv; iv=eA*ev; iv=eA*iv; iv=iA*ev; iv=iA*iv;
    fA=fA*fA; iA=eA*eA; iA=eA*iA; iA=iA*eA; iA=iA*iA;
}

Void
TestMatrix::test_project()
{
    typedef FloatDP X;
    ARIADNE_TEST_CONSTRUCT(Matrix<X>,A,({{11,12,13,14,15},{21,22,23,24,25},{31,32,33,34,35}},pr));
    ARIADNE_TEST_CONSTRUCT(Matrix<X>,B,({{120,130,140},{220,230,240}},pr));

    ARIADNE_TEST_ASSIGN_CONSTRUCT(Vector<X>,v,A[range(0,2)][1]);
    ARIADNE_TEST_EQUAL(v[1],A[1][1]);
    ARIADNE_TEST_ASSIGN_CONSTRUCT(Covector<X>,u,A[2][range(1,4)]);
    ARIADNE_TEST_EQUAL(u[2],A[2][3]);

    ARIADNE_TEST_EQUALS(A[range(0,2)][range(1,4)].row_size(),2);
    ARIADNE_TEST_EQUALS(A[range(0,2)][range(1,4)].column_size(),3);
    ARIADNE_TEST_CONSTRUCT(MatrixRange<Matrix<X>>,AR,(A[range(0,2)][range(1,4)]));
    ARIADNE_TEST_CONSTRUCT(Matrix<X>,ACR,(A[range(0,2)][range(1,4)]));

    ARIADNE_TEST_EQUAL(AR[1][2],A[1][3]);
    ARIADNE_TEST_EQUAL(ACR[1][2],A[1][3]);
    ARIADNE_TEST_EXECUTE(AR=B);
    ARIADNE_TEST_PRINT(A);
    ARIADNE_TEST_EQUALS(A[range(0,2)][range(1,4)],B);
//    ARIADNE_TEST_EQUALS(AR,B);
//    ARIADNE_TEST_EQUALS(ACR,B);

}


Void
TestMatrix::test_factorisations()
{
    // Set all diagonal elements to 0.
    auto set_diagonal_to_zero = [](FloatApproximationMatrix A){ for (SizeType i=0; i!=min(A.row_size(),A.column_size()); ++i) { A[i][i]=0; } return A; };

    {
        // LU test_factorisation
        auto A = FloatApproximationMatrix({{1.0_x,2.0_x,3.0_x},{4.0_x,5.0_x,6.0_x},{7.0_x,8.0_x,10.0_x}},pr);
        auto PLU = triangular_decomposition(A); auto P=std::get<0>(PLU); auto L=std::get<1>(PLU); auto U=std::get<2>(PLU);
        ARIADNE_TEST_COMPARE(norm(L*U-P*A),<,1e-12);
    }

    {
        // QR decomposition
        auto I = FloatApproximationMatrix::identity(3,dp);
        auto A = FloatApproximationMatrix({{1.0_x,2.0_x,3.0_x},{4.0_x,5.0_x,6.0_x},{7.0_x,8.0_x,10.0_x}},pr);
        auto QR = gram_schmidt_orthogonalisation(A); auto Q=std::get<0>(QR); auto R=std::get<1>(QR);
        ARIADNE_TEST_COMPARE(norm(Q*R-A),<,1e-12);
        ARIADNE_TEST_COMPARE(norm(transpose(Q)*Q-I),<,1e-12);
    }

    {
        // Orthogonal decomposition
        auto A1 = FloatApproximationMatrix({{1,7,4},{4,-2,5},{-5,-5,-3}},pr);
        auto OR1 = orthogonal_decomposition(A1); auto O1=std::get<0>(OR1); auto R1=std::get<1>(OR1);
        ARIADNE_TEST_COMPARE(norm(O1*R1-A1),<,1e-12);
        ARIADNE_TEST_COMPARE(norm(set_diagonal_to_zero(transpose(O1)*O1)),<,1e-12);
        ARIADNE_TEST_COMPARE(norm(R1),<=,1.0+1e-12);

        auto A2 = FloatApproximationMatrix({{9,-5,6},{-7,-1,-1},{8,-5,5},{-5,5,2}},pr);
        auto OR2 = orthogonal_decomposition(A2); auto O2=std::get<0>(OR2); auto R2=std::get<1>(OR2);
        ARIADNE_TEST_EQUALS(A2.column_size(),O2.column_size());
        ARIADNE_TEST_COMPARE(norm(O2*R2-A2),<,1e-12);
        ARIADNE_TEST_COMPARE(norm(set_diagonal_to_zero(transpose(O2)*O2)),<,1e-12);
        ARIADNE_TEST_COMPARE(norm(R2),<=,1.0+1e-12);

        auto A3 = FloatApproximationMatrix({{-2,-7,1,5},{-6,-5,-7,7},{3,2,0,8}},dp);
        auto OR3 = orthogonal_decomposition(A3); auto O3=std::get<0>(OR3); auto R3=std::get<1>(OR3);
        ARIADNE_TEST_EQUALS(A3.row_size(),O3.column_size());
        ARIADNE_TEST_COMPARE(norm(O3*R3-A3),<,1e-12);
        ARIADNE_TEST_COMPARE(norm(set_diagonal_to_zero(transpose(O3)*O3)),<,1e-12);
        ARIADNE_TEST_COMPARE(norm(R3),<=,1.0+1e-12);
    }

}

Void
TestMatrix::test_misc()
{
    Array<FloatDPApproximation> Aary={{-1.0,3.0,1.0, -1.0,1.0,2.0, 2.0,1.0,1.0},pr};
    Array<FloatDPBounds> iAary={{-1.0_x,3.0_x, -1.0_x,1.0_x},pr};
    FloatDPApproximation* Aptr=Aary.begin();

    Matrix<FloatDPApproximation> A0(0,0,pr);
    ARIADNE_TEST_PRINT(A0);
    Matrix<FloatDPApproximation> A1(3,2,pr);
    ARIADNE_TEST_PRINT(A1);
    Matrix<FloatDPApproximation> A2(3,3,Aary.begin());
    ARIADNE_TEST_PRINT(A2);
    Matrix<FloatDPApproximation> A3({{-1.0,3.0,1.0}, {-1.0,1.0,2.0}, {2.0,1.0,1.0}},pr);
    ARIADNE_TEST_PRINT(A3);

    for(SizeType i=0; i!=A2.row_size(); ++i) {
        for(SizeType j=0; j!=A2.column_size(); ++j) {
            ARIADNE_TEST_EQUAL(A2[i][j],A2.get(i,j));
            ARIADNE_TEST_EQUALS(A2[i][j],Aptr[i*A2.column_size()+j]);
        }
    }

    ARIADNE_TEST_EQUAL(A2,A3);

    A1[0][0]=1.0;
    ARIADNE_TEST_EQUALS(A1[0][0],1.0);
    A1.set(0,1,3.0);
    ARIADNE_TEST_EQUALS(A1[0][1],3.0);

    A1.at(1,0)=2.0;
    ARIADNE_TEST_EQUALS(A1[1][0],2.0);

    A0=Matrix<FloatDPApproximation>::zero(2,3,dp);
    ARIADNE_TEST_PRINT(A0);
    A1=Matrix<FloatDPApproximation>::identity(4,dp);
    ARIADNE_TEST_PRINT(A1);

    ARIADNE_TEST_EQUALS(+FloatApproximationMatrix({{1.0_x,2.0_x},{3.0_x,4.0_x}},pr),FloatApproximationMatrix({{1.0_x,2.0_x},{3.0_x,4.0_x}},pr));
    ARIADNE_TEST_EQUALS(-FloatApproximationMatrix({{1.0_x,2.0_x},{3.0_x,4.0_x}},pr),FloatApproximationMatrix({{-1.0_x,-2.0_x},{-3.0_x,-4.0_x}},pr));
    ARIADNE_TEST_EQUALS(FloatApproximationMatrix({{1.0_x,2.0_x},{3.0_x,4.0_x}},pr)+FloatApproximationMatrix({{5.0_x,7.0_x},{8.0_x,6.0_x}},pr),FloatApproximationMatrix({{6.0_x,9.0_x},{11.0_x,10.0_x}},pr));
    ARIADNE_TEST_EQUALS(FloatApproximationMatrix({{1.0_x,2.0_x},{3.0_x,4.0_x}},pr)-FloatApproximationMatrix({{5.0_x,7.0_x},{8.0_x,6.0_x}},pr),FloatApproximationMatrix({{-4.0_x,-5.0_x},{-5.0_x,-2.0_x}},pr));
    ARIADNE_TEST_EQUALS(FloatApproximationMatrix({{1.0_x,2.0_x},{3.0_x,4.0_x}},pr)*FloatApproximationMatrix({{5.0_x,7.0_x},{8.0_x,6.0_x}},pr),FloatApproximationMatrix({{21.0_x,19.0_x},{47.0_x,45.0_x}},pr));

    // Transpose operations
    ARIADNE_TEST_EQUALS(FloatApproximationMatrix(transpose(FloatApproximationMatrix({{1.0_x,2.0_x,3.0_x},{4.0_x,5.0_x,6.0_x}},pr))),FloatApproximationMatrix({{1.0_x,4.0_x},{2.0_x,5.0_x},{3.0_x,6.0_x}},pr));
    ARIADNE_TEST_EQUALS(transpose(FloatApproximationMatrix({{1.0_x,2.0_x},{3.0_x,4.0_x}},pr))*FloatApproximationMatrix({{5.0_x,7.0_x},{8.0_x,6.0_x}},pr),FloatApproximationMatrix({{1.0_x,3.0_x},{2.0_x,4.0_x}},pr)*FloatApproximationMatrix({{5.0_x,7.0_x},{8.0_x,6.0_x}},pr));
    ARIADNE_TEST_EQUALS(FloatApproximationMatrix({{1.0_x,2.0_x},{3.0_x,4.0_x}},pr)*transpose(FloatApproximationMatrix({{5.0_x,7.0_x},{8.0_x,6.0_x}},pr)),FloatApproximationMatrix({{1.0_x,2.0_x},{3.0_x,4.0_x}},pr)*FloatApproximationMatrix({{5.0_x,8.0_x},{7.0_x,6.0_x}},pr));
    ARIADNE_TEST_EQUALS(transpose(FloatApproximationMatrix({{1.0_x,2.0_x,3.0_x},{4.0_x,5.0_x,6.0_x}},pr))*FloatDPApproximationVector({5.0_x,7.0_x},pr),FloatApproximationMatrix({{1.0_x,4.0_x},{2.0_x,5.0_x},{3.0_x,6.0_x}},pr)*FloatDPApproximationVector({5.0_x,7.0_x},pr));

}


Int main() {
    TestMatrix().test();
    return ARIADNE_TEST_FAILURES;
}

