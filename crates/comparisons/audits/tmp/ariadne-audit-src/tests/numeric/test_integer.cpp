/***************************************************************************
 *            test_integer.cpp
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

#include "config.hpp"
#include "numeric/integer.hpp"
#include "foundations/logical.hpp"
#include "utility/string.hpp"

#include <iostream>
#include <iomanip>

#include "../test.hpp"

using namespace std;
using namespace Ariadne;


class TestInteger
{
  public:
    void test();
  private:
    void test_concept();
    void test_literal();
    void test_constructors();
    void test_arithmetic();
    void test_comparisons();
};

void TestInteger::test()
{
    ARIADNE_TEST_CALL(test_constructors());
    ARIADNE_TEST_CALL(test_comparisons());
    ARIADNE_TEST_CALL(test_literal());
    ARIADNE_TEST_CALL(test_arithmetic());
}

void TestInteger::test_concept() {
    unsigned int m=1; unsigned long int lm=1; int n=-2; long int ln=-2;
    Integer z,z2; Boolean b;

    z=Integer(); z=Integer(m); z=Integer(lm); z=Integer(n); z=Integer(ln); z=Integer(z);
    z2=Integer();
    z=m; z=lm; z=n; z=ln; z=z2;

    z=+z; z=-z;
    z=z+z; z=z-z; z=z*z;
    z=z+n; z=z-n; z=z*n;
    z=n+z; z=n-z; z=n*z;

    z=1_z; z=-1_z;

    b=(z==z); b=(z!=z); b=(z<=z); b=(z>=z); b=(z<z); b=(z>z);
    b=(z==n); b=(z!=n); b=(z<=n); b=(z>=n); b=(z<n); b=(z>n);
    b=(n==z); b=(n!=z); b=(n<=z); b=(n>=z); b=(n<z); b=(n>z);
}


void TestInteger::test_literal() {
    ARIADNE_TEST_CONSTRUCT(Integer,z,(3_z));
    ARIADNE_TEST_EQUALS(z,3);
    ARIADNE_TEST_EQUALS(z,Integer(3));
    ARIADNE_TEST_EQUALS(100000_z,Integer(100000));
    ARIADNE_TEST_EQUAL(1000000000000_z,sqr(Integer(1000000)));
    ARIADNE_TEST_EQUALS(1000000000000_z-sqr(Integer(1000000)),0);
    ARIADNE_TEST_EQUALS(4294967295_z,4294967295u);
    ARIADNE_TEST_EQUALS(-2147483647_z,-2147483647);
    ARIADNE_TEST_EQUALS(4294967295_z,Integer(4294967295u));
    ARIADNE_TEST_EQUALS(4611686016279904256_z,Integer(2147483647)*2147483647+2147483647);
}

void TestInteger::test_constructors() {
    int m=2147483647;
    unsigned int um=2147483647u;
    unsigned long int ulm=um;
    unsigned long long int ullm=um;
    int n=-2147483647;
    long int ln=n;
    long long int lln=n;
    String sn="-2147483647";
    String sz="314159265358979323846264";

    ARIADNE_TEST_ASSERT((not Constructible<Integer,float>));
    ARIADNE_TEST_ASSERT((not Constructible<Integer,double>));

    ARIADNE_TEST_CONSTRUCT(Integer,zum,(um));
    ARIADNE_TEST_EQUALS(zum.get_si(),m);
    ARIADNE_TEST_CONSTRUCT(Integer,zulm,(ulm));
    ARIADNE_TEST_EQUALS(zulm.get_si(),m);
    ARIADNE_TEST_CONSTRUCT(Integer,zullm,(ullm));
    ARIADNE_TEST_EQUALS(zullm.get_si(),m);
    ARIADNE_TEST_CONSTRUCT(Integer,zn,(n));
    ARIADNE_TEST_EQUALS(zn.get_si(),n);
    ARIADNE_TEST_CONSTRUCT(Integer,zln,(ln));
    ARIADNE_TEST_EQUALS(zln.get_si(),n);
    ARIADNE_TEST_CONSTRUCT(Integer,zlln,(lln));
    ARIADNE_TEST_EQUALS(zlln.get_si(),n);
    ARIADNE_TEST_CONSTRUCT(Integer,zsn,(sn));
    ARIADNE_TEST_EQUALS(zsn.get_si(),n);
    ARIADNE_TEST_CONSTRUCT(Integer,zsz,(sz));
    ARIADNE_TEST_EQUALS(zsz,Integer(314159265358)*Integer(1000000000000)+Integer(979323846264));

    ARIADNE_TEST_CONSTRUCT(Integer,z1,(0));
    ARIADNE_TEST_EQUALS(z1.get_si(),0);
    ARIADNE_TEST_CONSTRUCT(Integer,z2,(-3));
    ARIADNE_TEST_EQUALS(z2.get_si(),-3);
    ARIADNE_TEST_CONSTRUCT(Integer,z3,(ullm*ullm+ullm));
    ARIADNE_TEST_EQUALS(z3,zum*zum+zum);
    ARIADNE_TEST_CONSTRUCT(Integer,z4,(lln*lln+lln));
    ARIADNE_TEST_EQUALS(z4,zn*zn+zn);
}

void TestInteger::test_arithmetic() {
    ARIADNE_TEST_EQUALS(+Integer(-5),-5);
    ARIADNE_TEST_EQUALS(-Integer(-5), 5);
    ARIADNE_TEST_EQUALS(Integer(-5)+Integer(2),-3);
    ARIADNE_TEST_EQUALS(Integer(-5)-Integer(2),-7);
    ARIADNE_TEST_EQUALS(Integer(-5)*Integer(2),-10);

    ARIADNE_TEST_EQUALS(pos(Integer(-5)),-5);
    ARIADNE_TEST_EQUALS(neg(Integer(-5)), 5);
    ARIADNE_TEST_EQUALS(sqr(Integer(-5)),25);
    ARIADNE_TEST_EQUALS(pow(Integer(-5),3u),-125);


    ARIADNE_TEST_EQUALS((Integer)max(Integer(5),Integer(3)),5);
    ARIADNE_TEST_EQUALS((Integer)max(Integer(-5),Integer(-3)),-3);
    ARIADNE_TEST_EQUALS((Integer)min(Integer(5),Integer(3)),3);
    ARIADNE_TEST_EQUALS((Integer)min(Integer(-5),Integer(-3)),-5);
    ARIADNE_TEST_EQUALS(abs(Integer(-5)),5);
    ARIADNE_TEST_EQUALS(abs(Integer( 0)),0);
    ARIADNE_TEST_EQUALS(abs(Integer(+5)),5);
}

void TestInteger::test_comparisons() {
    ARIADNE_TEST_COMPARE(Integer(3),==,3);
    ARIADNE_TEST_COMPARE(3,==,Integer(3));

    ARIADNE_TEST_COMPARE(Integer(2),==,Integer(2));
    ARIADNE_TEST_COMPARE(Integer(0),==,Integer(-0));
    ARIADNE_TEST_COMPARE(Integer(2),!=,Integer(-2));
    ARIADNE_TEST_COMPARE(Integer(2),!=,Integer(-3));
    ARIADNE_TEST_COMPARE(Integer(2),<=,Integer(23));
    ARIADNE_TEST_COMPARE(Integer(2),<=,Integer(3));
    ARIADNE_TEST_COMPARE(Integer(2),>=,Integer(2));
    ARIADNE_TEST_COMPARE(Integer(2),>=,Integer(-3));
    ARIADNE_TEST_COMPARE(Integer(2),< ,Integer(3));
    ARIADNE_TEST_COMPARE(Integer(2),> ,Integer(-3));
}

int main() {
    ARIADNE_TEST_CLASS(Integer,TestInteger());

    return ARIADNE_TEST_FAILURES;
}
