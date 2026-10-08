/***************************************************************************
 *            test_vector.cpp
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

#include <iostream>
#include <fstream>
#include <cassert>

#include "config.hpp"
#include "numeric/numeric.hpp"
#include "numeric/floats.hpp"
#include "algebra/vector.hpp"

#include "../test.hpp"

using namespace std;
using namespace Ariadne;

class TestVector {
  public:
    Void test();
  private:
    Void test_concept();
    Void test_constructors();
    Void test_comparisons();
    Void test_arithmetic();
    Void test_misc();
};

Void
TestVector::test()
{
    ARIADNE_TEST_CALL(test_constructors());
    ARIADNE_TEST_CALL(test_comparisons());
    ARIADNE_TEST_CALL(test_arithmetic());
    ARIADNE_TEST_CALL(test_misc());
}

Void
TestVector::test_concept()
{
    FloatDPApproximation ax(1,dp);
    FloatDPBounds ix(1,dp);
    FloatDP ex(1,dp);
    Vector<FloatDPApproximation> av;
    Vector<FloatDPBounds> iv;
    Vector<FloatDP> ev;

    iv=Vector<FloatDPBounds>(ev);

    av=av+av;
    iv=ev+ev;
    iv=ev+iv;
    iv=iv+ev;
    iv=iv+iv;
    av=av-av; iv=ev-ev; iv=ev-iv; iv=iv-ev; iv=iv-iv;

    av=ax*av; iv=ex*ev; iv=ex*iv; iv=ix*ev; iv=ix*iv;
    av=av*ax; iv=ev*ex; iv=ev*ix; iv=iv*ex; iv=iv*ix;
    av=av/ax; iv=ev/ex; iv=ev/ix; iv=iv/ex; iv=iv/ix;
}


Void
TestVector::test_constructors()
{
    ARIADNE_TEST_CONSTRUCT( Vector<RoundedFloatDP>, v0, (3,dp) );
    ARIADNE_TEST_ASSERT( v0.size()==3 );
    ARIADNE_TEST_ASSERT( v0[0]==0.0_x );
    ARIADNE_TEST_ASSERT( v0[1]==0.0_x );
    ARIADNE_TEST_ASSERT( v0[2]==0.0_x );

    ARIADNE_TEST_CONSTRUCT( Vector<RoundedFloatDP>, v1, ({3.25_x,-0.75_x,0.0_x,1.375_x},dp) );
    ARIADNE_TEST_ASSERT( v1.size()==4 );
    ARIADNE_TEST_ASSERT( v1[0]==3.25_x );
    ARIADNE_TEST_ASSERT( v1[1]==-0.75_x );
    ARIADNE_TEST_ASSERT( v1[2]==0.0_x );
    ARIADNE_TEST_ASSERT( v1[3]==1.375_x );

    ARIADNE_TEST_CONSTRUCT( Vector<FloatDPApproximation>, va, ({3.25,-0.75,0.0,1.375},dp) );
    ARIADNE_TEST_CONSTRUCT( Vector<FloatDPBounds>, vb, ({3.25_x,-0.75_x,0.0_x,1.375_x},dp) );
    ARIADNE_TEST_CONSTRUCT( Vector<FloatDP>, vx, ({3.25_x,-0.75_x,0.0_x,1.375_x},dp) );
}


Void
TestVector::test_comparisons()
{
    Vector<RoundedFloatDP> v(2,dp); v[0]=1.25_x; v[1]=-1.375_x;
    ARIADNE_TEST_PRINT( v );
    ARIADNE_TEST_COMPARE( v , ==, Vector<RoundedFloatDP>({1.25_x,-1.375_x},dp) );
    ARIADNE_TEST_COMPARE( v , !=, Vector<RoundedFloatDP>({1.25_x,-1.625_x},dp) );
}


Void
TestVector::test_arithmetic()
{
    ARIADNE_TEST_EQUAL( + Vector<RoundedFloatDP>({2.0_x,-3.0_x,5.0_x},dp) , Vector<RoundedFloatDP>({2.0_x,-3.0_x,5.0_x},dp) );
    ARIADNE_TEST_EQUAL( - Vector<RoundedFloatDP>({2.0_x,-3.0_x,5.0_x},dp) , Vector<RoundedFloatDP>({-2.0_x,+3.0_x,-5.0_x},dp) );
    ARIADNE_TEST_EQUAL( Vector<RoundedFloatDP>({2.0_x,-3.0_x,5.0_x},dp) + Vector<RoundedFloatDP>({1.25_x,2.75_x,-3.5_x},dp), Vector<RoundedFloatDP>({3.25_x,-0.25_x,1.5_x},dp) );
    ARIADNE_TEST_EQUAL( Vector<RoundedFloatDP>({2.0_x,-3.0_x,5.0_x},dp) - Vector<RoundedFloatDP>({1.25_x,2.75_x,-3.5_x},dp), Vector<RoundedFloatDP>({0.75_x,-5.75_x,8.5_x},dp) );
    ARIADNE_TEST_EQUAL( RoundedFloatDP(4.0_x,dp) * Vector<RoundedFloatDP>({2.0_x,-3.0_x,5.0_x},dp), Vector<RoundedFloatDP>({8.0_x,-12.0_x,20.0_x},dp) );
    ARIADNE_TEST_EQUAL( Vector<RoundedFloatDP>({2.0_x,-3.0_x,5.0_x},dp) * RoundedFloatDP(4.0_x,dp), Vector<RoundedFloatDP>({8.0_x,-12.0_x,20.0_x},dp) );
    ARIADNE_TEST_EQUAL( Vector<RoundedFloatDP>({2.0_x,-3.0_x,5.0_x},dp) / RoundedFloatDP(4.0_x,dp), Vector<RoundedFloatDP>({0.5_x,-0.75_x,1.25_x},dp) );

    ARIADNE_TEST_EQUAL( sup_norm(Vector<RoundedFloatDP>({2.0_x,-3.0_x,1.0_x},dp)), RoundedFloatDP(3.0_x,dp) )
}


Void
TestVector::test_misc()
{
    DoublePrecision pr;
    Array<FloatDPApproximation> vary={{-4.0_x,3.0_x,1.0_x},pr};
    FloatDPApproximation x={1.5_x,pr};

    Vector<FloatDPApproximation> v0;
    cout << "v0.size()=" << v0.size() << endl;
    cout << "v0=" << flush; cout << v0 << endl;
    ARIADNE_TEST_NOTIFY("Constructor Vector<X>(SizeType, const X*) is unsafe and has been removed.");
    Array<FloatDPApproximation> a1(vary.begin(),vary.end());
    Vector<FloatDPApproximation> v1(a1);
    cout << "v1=" << v1 << endl;
    Vector<FloatDPApproximation> v2=Vector<FloatDPApproximation>({2.375_x,4.25_x,-1.25_x},pr);
    cout << "v2=" << v2 << endl;
    cout << "norm(v1)=" << norm(v1) << "  norm(v2)=" << norm(v2) << endl;
    ARIADNE_TEST_EQUAL(norm(v1).raw(),4);
    ARIADNE_TEST_EQUAL(norm(v2).raw(),4.25_x);

    Vector<FloatDPApproximation> v3(1,pr);
    cout << "v3=" << v3 << endl;
    Vector<FloatDPApproximation> v4=v2;
    cout << "v4=" << v4 << endl;
    Vector<FloatDPApproximation> v5({-4.0_x,3.0_x,1.0_x},pr);
    cout << "v5=" << v5 << endl;
    ARIADNE_TEST_EQUAL(v1,v5);
    cout << endl;

    Vector<FloatDPApproximation> vf0;
    v1=Vector<FloatDPApproximation>({0.25_x,-1.5_x},pr);
    v2=Vector<FloatDPApproximation>({-0.5_x,2.25_x},pr);
    vf0=-v1;
    cout << vf0 << " = -" << v1 << endl;
    vf0=Vector<FloatDPApproximation>(v1)+v2;
    cout << vf0 << " = " << v1 << " + " << v2 << endl;
    vf0=Vector<FloatDPApproximation>(v1)-v2;
    cout << vf0 << " = " << v1 << " - " << v2 << endl;
    vf0=x*Vector<FloatDPApproximation>(v2);
    cout << vf0 << " = " << x << " * " << v2 << endl;
    vf0=Vector<FloatDPApproximation>(v1)*x;
    cout << vf0 << " = " << v1 << " * " << x << endl;
    vf0=Vector<FloatDPApproximation>(v1)/x;
    cout << vf0 << " = " << v1 << " / " << x << endl;
    cout << endl;

    Vector< FloatDPBounds > iv1=Vector<FloatDPBounds>({{0.984375_x,1.015625_x},{2.25_x,2.375_x},{4.0_x,4.375_x},{-0.03125_x,0.015625_x}},pr);
    cout << "iv1=" << iv1 << endl;
    cout << "norm(iv1)=" << norm(iv1) << endl;
    cout << "norm(iv1).upper()=" << norm(iv1).upper() << endl;

    Vector< FloatDPBounds > iv2=Vector<FloatDPBounds>({{-1.0_x,1.0_x},{-1.0_x,1.0_x}},pr);
    cout << "iv2=" << iv2 << endl;
    Vector< FloatDPBounds > iv3(3,pr);
    cout << "iv3=" << iv3 << endl;
    iv3=Vector<FloatDPBounds>({{4.25_x,4.25_x},{2.375_x,2.375_x}},pr);
    cout << "iv3=" << iv3 << endl;
    FloatDPBounds ix=FloatDPBounds(-2,1,pr);

    Vector< FloatDPBounds > iv0;
    cout << "iv0=" << iv0 << endl;
    iv1=iv0;
    cout << "iv1=" << iv1 << endl;
    iv1=iv2;
    cout << "iv1=" << iv1 << endl;
    cout << endl;

    FloatDPBounds ix2=iv2[0];
    FloatDPBounds ix3=iv3[0];
    FloatDPBounds ix1=ix2+ix3;
    ix1=ix2+ix3;

    cout << "iv2=" << iv2 << ", iv3=" << iv3 << endl;
    iv1=iv2+iv3;
    cout << iv1 << " = " << iv2 << " + " << iv3 << endl;
    iv1=iv2-iv3;
    cout << iv1 << " = " << iv2 << " - " << iv3 << endl;
    iv1=ix*iv3;
    cout << iv1 << " = " << ix << " * " << iv3 << endl;
    iv1=iv2*ix;
    cout << iv1 << " = " << iv2 << " * " << ix << endl;
    ix=FloatDPBounds(1,2,dp);
    iv1=iv2/ix;
    cout << iv1 << " = " << iv2 << " / " << ix << endl;
    cout << endl;

    Vector<FloatDP> ev1(reinterpret_cast<Vector<FloatDP>const&>(v1));
    FloatDP ex(reinterpret_cast<FloatDP const&>(x));
    iv0=iv1+ev1;
    cout << iv0 << " = " << iv1 << " + " << ev1 << endl;
    iv0=ev1+iv1;
    cout << iv0 << " = " << ev1 << " + " << iv1 << endl;
    iv0=iv1-ev1;
    cout << iv0 << " = " << iv1 << " - " << ev1 << endl;
    iv0=ev1-iv1;
    cout << iv0 << " = " << ev1 << " - " << iv1 << endl;
    iv0=ex*iv1;
    cout << iv0 << " = " << ex << " * " << iv1 << endl;
    iv0=ix*ev1;
    cout << iv0 << " = " << ix << " * " << ev1 << endl;
    iv0=iv1*ex;
    cout << iv0 << " = " << iv1 << " * " << ex << endl;
    iv0=ev1*ix;
    cout << iv0 << " = " << ev1 << " * " << ix << endl;
    iv0=iv1/ex;
    cout << iv0 << " = " << iv1 << " / " << ex << endl;
    iv0=ev1/ix;
    cout << iv0 << " = " << ev1 << " / " << ix << endl;

    iv0=ev1;
    iv0/=ix;
    iv0=Vector<FloatDPBounds>({2,1},pr);
    iv1=Vector<FloatDPBounds>({0,1},pr);
    /*
      ARIADNE_TEST_ASSERT( (iv0+=Vector<FloatDPBounds>("[0,1]")) == Vector<FloatDPBounds>("[2,2]") );
      ARIADNE_TEST_ASSERT( (iv0-=Vector<FloatDPBounds>("[0,1]")) == Vector<FloatDPBounds>("[2,1]") );
      ARIADNE_TEST_ASSERT( (iv0*=2) == Vector<FloatDPBounds>("[4,2]") );
      ARIADNE_TEST_ASSERT( (iv0/=4) == Vector<FloatDPBounds>("[1,0.5_x]") );
    */

    /*
      cout << "test_vector_slice" << endl;
      v1=Vector<FloatDPApproximation>("[-1.25_x,0.75_x,-0.5_x,-4.25_x,2.375_x]");
      cout << v1 << endl;
      VectorSlice<FloatDPApproximation> vs1(2,v1.begin()+2,2);
      cout << vs1 << endl;
      VectorSlice<FloatDPApproximation> vs2(2,v1.begin(),3);
      cout << vs2 << endl;

      iv1=vs1+vs2;
      cout << iv1 << endl;
      iv1=vs1-vs2;
      cout << iv1 << endl;
      iv1=x*vs1;
      cout << iv1 << endl;
      iv1=vs1*x;
      cout << iv1 << endl;
      iv1=vs1/x;
      cout << iv1 << endl;
    */

}


Int main() {
    TestVector().test();

    return ARIADNE_TEST_FAILURES;
}

