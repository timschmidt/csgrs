/***************************************************************************
 *            geometry/box.cpp
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

/*! \file geometry/box.cpp
 *  \brief
 */


#include "numeric/module.hpp"
#include "function/function.hpp"
#include "function/taylor_model.hpp"

#include "box.hpp"
#include "box.tpl.hpp"

#include "function/formula.hpp"

#include "algebra/algebra.hpp"

#include "io/geometry2d.hpp"

namespace Ariadne {

typedef FloatDPBounds ValidatedNumericType;
typedef Interval<FloatDPUpperBound> UpperIntervalType;

UpperIntervalType apply(ValidatedScalarUnivariateFunction const& f, UpperIntervalType const& ivl) {
    if (definitely(ivl.is_empty())) { return UpperIntervalType::empty_interval(); }
    return static_cast<UpperIntervalType>(f(reinterpret_cast<ValidatedNumericType const&>(ivl))); }
UpperBoxType apply(ValidatedVectorUnivariateFunction const& f, UpperIntervalType const& ivl) {
    if (definitely(ivl.is_empty())) { return UpperBoxType(f.result_size(),UpperIntervalType::empty_interval()); }
    return static_cast<UpperBoxType>(f(reinterpret_cast<ValidatedNumericType const&>(ivl))); }
UpperIntervalType apply(ValidatedScalarMultivariateFunction const& f, UpperBoxType const& bx) {
    if (definitely(bx.is_empty())) { return UpperIntervalType::empty_interval(); }
    return static_cast<UpperIntervalType>(f(reinterpret_cast<Vector<ValidatedNumericType>const&>(bx))); }
UpperBoxType apply(ValidatedVectorMultivariateFunction const& f, UpperBoxType const& bx) {
    if (definitely(bx.is_empty())) { return UpperBoxType(f.result_size(),UpperIntervalType::empty_interval()); }
    return static_cast<UpperBoxType>(f(reinterpret_cast<Vector<ValidatedNumericType>const&>(bx))); }

UpperIntervalType image(UpperIntervalType const& ivl, ValidatedScalarUnivariateFunction const& f) {
    return apply(f,ivl); }
UpperBoxType image(UpperIntervalType const& ivl, ValidatedVectorUnivariateFunction const& f) {
    return apply(f,ivl); }
UpperIntervalType image(UpperBoxType const& bx, ValidatedScalarMultivariateFunction const& f) {
    return apply(f,bx); }
UpperBoxType image(UpperBoxType const& bx, ValidatedVectorMultivariateFunction const& f) {
    return apply(f,bx); }

template class Box<Interval<Real>>;
template class Box<Interval<FloatDP>>;
template class Box<Interval<FloatDPUpperBound>>;
template class Box<Interval<FloatDPLowerBound>>;
template class Box<Interval<FloatDPApproximation>>;

Void draw(CanvasInterface& c, Projection2d const& p, ApproximateBoxType const& bx) {
    auto const& xl = numeric_cast<double>(bx[p.x_coordinate()].lower_bound());
    auto const& xu = numeric_cast<double>(bx[p.x_coordinate()].upper_bound());
    auto const& yl = numeric_cast<double>(bx[p.y_coordinate()].lower_bound());
    auto const& yu = numeric_cast<double>(bx[p.y_coordinate()].upper_bound());
    List<Point2d> boundary = {Point2d(xl,yl),Point2d(xu,yl),Point2d(xu,yu),Point2d(xl,yu)};
    c.fill_boundary(boundary);
}

ExactBoxType make_box(const String& str)
{
    // Representation as a literal
    //   "[a1,b1]x[a2,b2]x...x[an,bn]"

    StringStream ss(str);
    std::vector<ExactIntervalType> vec;
    ExactIntervalType ivl;
    char c;

    c='x';
    while(c=='x') {
        ss >> ivl;
        vec.push_back(ivl);
        c=' ';
        while( ss && c==' ') {
            ss >> c;
        }
    }
    if(ss) {
        ss.putback(c);
    }

    ExactBoxType bx(vec.size());
    for(SizeType i=0; i!=bx.dimension(); ++i) {
        bx[i]=vec[i];
    }
    return bx;
}


template<class BX> using VertexType = typename BX::VertexType;

template<class BX> void make_vertices_down(const BX& bx, SizeType i, SizeType n, VertexType<BX>& pt, std::vector<VertexType<BX>>& v);

template<class BX> void make_vertices_up(const BX& bx, SizeType i, SizeType n, VertexType<BX>& pt, std::vector<VertexType<BX>>& v) {
    ARIADNE_ASSERT(i <= n);
    if(i == n) {    // base case: we are at the last dimension of the box
        pt[i] = bx[i].lower_bound();
        v.push_back(pt);
        pt[i] = bx[i].upper_bound();
        v.push_back(pt);
    } else {        // recursive case: we are still scanning dimensions
        pt[i] = bx[i].lower_bound();
        make_vertices_up(bx, i+1, n, pt, v);
        pt[i] = bx[i].upper_bound();
        make_vertices_down(bx, i+1, n, pt, v);
    }
}

template<class BX> void make_vertices_down(const BX& bx, SizeType i, SizeType n, VertexType<BX>& pt, std::vector<VertexType<BX>>& v) {
    ARIADNE_ASSERT(i <= n);
    if(i == n) {    // base case: we are at the last dimension of the box
        pt[i] = bx[i].upper_bound();
        v.push_back(pt);
        pt[i] = bx[i].lower_bound();
        v.push_back(pt);
    } else {        // recursive case: we are still scanning dimensions
        pt[i] = bx[i].upper_bound();
        make_vertices_up(bx, i+1, n, pt, v);
        pt[i] = bx[i].lower_bound();
        make_vertices_down(bx, i+1, n, pt, v);
    }
}

template<class I> List<typename Box<I>::VertexType> Box<I>::vertices() const {
    std::vector<VertexType> v;
    SizeType n = this->dimension();
    if(n > 0) {
        VertexType pt=this->lower_bounds();
        make_vertices_up(*this, 0, n-1, pt, v);
    }
    return v;
}

template List<typename FloatDPExactBox::VertexType> Box<FloatDPExactInterval>::vertices() const;

/*
FloatDPLowerBox under_approximation(const RealBox& rbx) {
    FloatDPLowerBox bx(rbx.size());
    for(SizeType i=0; i!=bx.size(); ++i) {
        bx[i]=under_approximation(rbx[i]);
    }
    return bx;
}

FloatDPUpperBox over_approximation(const Box& rbx) {
    FloatDPUpperBox bx(rbx.size(),FloatDPUpperInterval(-inf,+inf));
    for(SizeType i=0; i!=bx.size(); ++i) {
        bx[i]=over_approximation(rbx[i]);
    }
    return bx;
}

FloatDPApproximationBox approximation(const RealBox& rbx) {
    FloatDPApproximationBox bx(rbx.size(),FloatDPApproximationInterval(-inf,+inf));
    for(SizeType i=0; i!=bx.size(); ++i) {
        bx[i]=approximation(rbx[i]);
    }
    return bx;
}
*/


/*
Vector<FloatBounds> cast_singleton(const Vector<FloatDPUpperInterval>& bx) {
    Vector<FloatBounds> r(bx.size());
    for(SizeType i=0; i!=r.size(); ++i) { r[i]=cast_singleton(bx[i]); }
    return r;
}

template class Box<RealInterval>;
template class Box<FloatDPExactInterval>;
template class Box<FloatDPUpperInterval>;
template class Box<FloatDPApproximationInterval>;
*/

} //namespace Ariadne
