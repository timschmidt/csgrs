/***************************************************************************
 *            linear_algebra_submodule.hpp
 *
 *  Copyright  2008-22  Pieter Collins
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

#include "pybind11.hpp"

#include "config.hpp"

#include "algebra/vector.hpp"
#include "algebra/covector.hpp"
#include "algebra/matrix.hpp"

using namespace Ariadne;

namespace Ariadne {

template<class X>
X __vgetitem__(const Vector<X>& v, Nat i)
{
    if(i<0) { i+=static_cast<Nat>(v.size()); }
    ARIADNE_ASSERT_MSG(0<=i && Nat(i)<v.size(),"v="<<v<<" i="<<i);
    return v[static_cast<Nat>(i)];
}


template<class X>
Vector<X> __vgetslice__(const Vector<X>& v, Nat start, Nat stop)
{
    if(start<0) { start+=static_cast<Nat>(v.size()); }
    if(stop<0) { stop+=static_cast<Nat>(v.size()); }
    ARIADNE_ASSERT(0<=start && start<=stop && Nat(stop)<=v.size());
    return project(v,range(static_cast<Nat>(start),static_cast<Nat>(stop)));
}


template<class X>
Void __vsetitem__(Vector<X>& v, Nat i, const X& x)
{
    if(i<0) { i+=static_cast<Nat>(v.size()); }
    ARIADNE_ASSERT(0<=i && Nat(i)<v.size());
    v[static_cast<Nat>(i)]=x;
}


template<class X>
X __cvgetitem__(const Covector<X>& u, Nat i)
{
    if(i<0) { i+=static_cast<Nat>(u.size()); }
    ARIADNE_ASSERT_MSG(0<=i && Nat(i)<u.size(),"v="<<u<<" i="<<i);
    return u[static_cast<Nat>(i)];
}

template<class X>
Void __cvsetitem__(Covector<X>& u, Nat i, const X& x)
{
    if(i<0) { i+=static_cast<Nat>(u.size()); }
    ARIADNE_ASSERT(0<=i && Nat(i)<u.size());
    u[static_cast<Nat>(i)]=x;
}

template<class X>
X __mgetitem__(const Matrix<X>& A, const std::tuple<Nat,Nat>& ind)
{
    Nat i=std::get<0>(ind);
    Nat j=std::get<1>(ind);
    return A[i][j];
}

template<class X>
Void __msetitem__(Matrix<X>& A, const std::tuple<Nat,Nat>& ind, const X& x)
{
    Nat i=std::get<0>(ind);
    Nat j=std::get<1>(ind);
    A[i][j]=x;
}

} // namespace Ariadne

