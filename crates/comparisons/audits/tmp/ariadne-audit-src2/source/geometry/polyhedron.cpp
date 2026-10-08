/***************************************************************************
 *            geometry/polyhedron.cpp
 *
 *  Copyright  2006-20  Alberto Casagrande, Pieter Collins
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

#include "function/functional.hpp"
#include "config.hpp"

#include <iostream>
#include <string>
#include <sstream>

#include <list>
#include <set>
#include <vector>
#include <valarray>

#include "geometry/polyhedron.hpp"

#include "utility/macros.hpp"
#include "utility/exceptions.hpp"

#include "numeric/numeric.hpp"

#include "algebra/vector.hpp"
#include "algebra/matrix.hpp"

#include "function/function.hpp"

#include "geometry/box.hpp"
#include "geometry/polytope.hpp"



namespace Ariadne {


extern Int global_verbosity;
Int verbosity=global_verbosity;


Void ddconv(std::vector< Vector<FloatDP> >&, const std::vector< Vector<FloatDP> >&);

Polyhedron polyhedron(const ExactBoxType& bx);
Polyhedron polyhedron(const Polytope& p);
Polytope polytope(const Polyhedron& p);



Polyhedron::Polyhedron()
    : _A(), _b()
{
}


Polyhedron::Polyhedron(DimensionType d)
    : _A(0,d), _b(0)
{
}


Polyhedron::Polyhedron(const Matrix<FloatDP>& A, const Vector<FloatDP>& b)
    : _A(A), _b(b)
{
    ARIADNE_ASSERT_MSG(A.column_size()==b.size(),"Invalid sizes of A="<<A<<" and b="<<b<<" for polyhedral constraints");
}




Polyhedron::Polyhedron(const ExactBoxType& bx)
    : _A(bx.dimension()*2,bx.dimension()), _b(bx.dimension()*2)
{
    const DimensionType n=bx.dimension();
    for(SizeType i=0; i!=n; ++i) {
        _A[i][i]=-1;
        _b[i]=-bx[i].lower();
        _A[i+n][i]=+1;
        _b[i+n]=+bx[i].upper();
    }
}

Polyhedron::Polyhedron(const Polytope& p)
{
    ARIADNE_NOT_IMPLEMENTED;
}

Polyhedron* Polyhedron::clone() const
{
    return new Polyhedron(*this);
}



Matrix<FloatDP>
Polyhedron::A() const
{
    return this->_A;
}

Vector<FloatDP>
Polyhedron::b() const
{
    return this->_b;
}



DimensionType
Polyhedron::dimension() const
{
    return this->_A.column_size();
}

ValidatedKleenean
Polyhedron::empty() const
{
    ARIADNE_NOT_IMPLEMENTED;
}


ValidatedKleenean
Polyhedron::is_bounded() const
{
    ARIADNE_NOT_IMPLEMENTED;
}


Polyhedron
Polyhedron::halfspace(SizeType i) const
{
    ARIADNE_NOT_IMPLEMENTED;
}



ValidatedKleenean Polyhedron::overlaps(const ExactBoxType& bx) const {
    ARIADNE_NOT_IMPLEMENTED;
}

ValidatedKleenean Polyhedron::covers(const ExactBoxType& bx) const {
    ARIADNE_NOT_IMPLEMENTED;
}

ValidatedKleenean Polyhedron::separated(const ExactBoxType& bx) const {
    ARIADNE_NOT_IMPLEMENTED;
}



Polyhedron
intersection(const Polyhedron& plhd1, const Polyhedron& plhd2)
{
    ARIADNE_ASSERT(plhd1.dimension()==plhd2.dimension());
    DimensionType d=plhd1.dimension();
    SizeType nc1=plhd1.number_of_constraints();
    SizeType nc2=plhd2.number_of_constraints();
    Matrix<FloatDP> A(nc1+nc2,d);
    project(A,range(0,nc1),range(0,d)) = plhd1.A();
    project(A,range(nc1,nc1+nc2),range(0,d)) = plhd1.A();
    Vector<FloatDP> b=join(plhd1.b(),plhd2.b());
    return Polyhedron(A,b);
}





Polyhedron
polyhedron(const ExactBoxType& bx)
{
    return Polyhedron(bx);
}

Polyhedron
polyhedron(const Polytope& pltp)
{
    ARIADNE_NOT_IMPLEMENTED;
}



Polytope
polytope(const Polyhedron& pltp)
{
    ARIADNE_NOT_IMPLEMENTED;
}







OutputStream&
Polyhedron::_write(OutputStream& os) const
{
    //return os << "Polyhedron( A=" << this->A() << ", b=" << this->b() << " )";
    const Matrix<FloatDP> A=this->A();
    const Vector<FloatDP> b=this->b();
    os << "Polyhedron( constraints=";
    DimensionType d=this->dimension();
    SizeType nc=this->number_of_constraints();
    for(SizeType i=0; i!=nc; ++i) {
        os << ( i==0 ? "[" : "," );
        for(SizeType j=0; j!=d; ++j) {
            os << ( j==0 ? "(" : ",");
            os << A[i][j];
        }
        os << ":" << b[i] << ")";
    }
    os << "] )";
    return os;
}

InputStream&
operator>>(InputStream& is, Polyhedron& p)
{
    std::vector< std::vector<FloatDP> > Alst;
    std::vector< FloatDP > Blst;

    std::vector<FloatDP> a;
    FloatDP b;

    char c;
    is >> c;
    ARIADNE_ASSERT(c=='[');

    c=is.peek();
    while(c=='[') {
        // Read constraint ax<=b in form [a_1,a_2,...,a_n;b];
        read_sequence(is,a,'[',';',',');
        is >> b;
        is >> c;
        ARIADNE_ASSERT(c==']');
        Alst.push_back(a);
        Blst.push_back(b);
    }

    SizeType m=Alst.size();
    SizeType n=Alst[0].size();
    Matrix<FloatDP> A(m,n);
    Vector<FloatDP> B(m);
    for(SizeType i=0; i!=m; ++i) {
        for(SizeType j=0; j!=n; ++j) {
            A[i][j]=Alst[i][j];
        }
        B[i]=Blst[i];
    }

    p=Polyhedron(A,B);

    return is;
}




} // namespace Ariadne

