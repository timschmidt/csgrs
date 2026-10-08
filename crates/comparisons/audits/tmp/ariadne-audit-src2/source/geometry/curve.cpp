/***************************************************************************
 *            geometry/curve.cpp
 *
 *  Copyright  2007-20  Pieter Collins
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
#include "function/taylor_model.hpp"
#include "function/formula.hpp"

#include "config.hpp"

#include <cassert>

#include "algebra/vector.hpp"
#include "algebra/matrix.hpp"
#include "algebra/differential.hpp"
#include "algebra/algebra.hpp"
#include "geometry/point.hpp"
#include "geometry/box.hpp"

#include "geometry/curve.hpp"
#include "symbolic/space.hpp"

#include "io/figure.hpp"

namespace Ariadne {

template<class F> inline const Point<F>& cast_exact(const Point<Approximation<F>>& t) {
    return reinterpret_cast<const Point<F>&>(t); }

template<> double numeric_cast(const CheckedAssignable<Approximation<FloatDP>>& a) { return a.get_d(); }
template<> double numeric_cast(const CheckedAssignable<Approximation<FloatMP>>& a) { return a.get_d(); }


Curve::Curve(const EffectiveVectorUnivariateFunction& f)
    : _function(f)
{
    ARIADNE_ASSERT(this->_function.argument_size()==1);
}


Curve::Curve(const Curve& c)
    : _function(c._function)
{
}


Curve*
Curve::clone() const
{
    return new Curve(*this);
}


DimensionType
Curve::dimension() const
{
    return this->_function.result_size();
}


DegreeType
Curve::smoothness() const
{
    return SMOOTH;
}



Curve::PointType
Curve::value(const ParameterType& s) const
{
    Vector<FloatDPApproximation> fv=this->_function.evaluate(s);
    return PointType(fv);
}


Curve::TangentVectorType
Curve::tangent(const ParameterType& s) const
{
    return Ariadne::tangent(this->_function,s);
}



OutputStream&
Curve::_write(OutputStream& os) const
{
    return os << "Curve( function=" << this->_function << " )";
}



InterpolatedCurve*
InterpolatedCurve::clone() const
{
    return new InterpolatedCurve(*this);
}

Void
InterpolatedCurve::insert(const ParameterType& s, const PointType& pt) {
    if(!this->_points.empty()) { ARIADNE_ASSERT(pt.size()==this->dimension()); }
    this->_points.insert(Pair< ParameterType, PointType >(s,pt));
}

Void
InterpolatedCurve::insert(const Dyadic& s, const PointType& pt) {
    decltype(auto) spr = this->_points.begin()->first.precision();
    this->insert(ParameterType(s,spr),pt);
}

Void
InterpolatedCurve::insert(const RawFloatDP& s, const Vector<RawFloatDP>& pt) {
    if(!this->_points.empty()) { ARIADNE_ASSERT(pt.size()==this->dimension()); }
    this->insert(ParameterType(s),PointType(reinterpret_cast<Vector<FloatDP>const&>(pt)));
}

UpperBoxType
InterpolatedCurve::bounding_box() const
{
    auto pt=cast_exact(this->_points.begin()->second);
    ExactBoxType bx(pt);
    for(ConstIterator iter=this->_points.begin(); iter!=this->_points.end(); ++iter) {
        pt=cast_exact(iter->second);
        bx=hull(bx,pt);
    }
    return bx;
}

Void
InterpolatedCurve::draw(CanvasInterface& c, const Projection2d& p) const
{
    Nat xi=p.x_coordinate(); Nat yi=p.y_coordinate();
    ConstIterator iter=this->begin();
    auto pt=join(iter->second,iter->first);
    c.move_to(pt[xi],pt[yi]);
    while(iter!=this->end()) {
        ++iter;
        pt=join(iter->second,iter->first);
        c.line_to(pt[xi],pt[yi]);
    }
    if( decide(this->begin()->second==(this->end())->second) ) {
        c.fill();
    } else {
        c.stroke();
    }
}

OutputStream&
InterpolatedCurve::_write(OutputStream& os) const
{
    return os << (*this);
}

LabelledInterpolatedCurve::LabelledInterpolatedCurve(const PointType& pt)
            : InterpolatedCurve(CurveInterface::PointType(pt)), _state_variables(pt.state_variables()) { }

RealSpace
LabelledInterpolatedCurve::state_space() const {
    return RealSpace(this->_state_variables);
}

LabelledInterpolatedCurve* LabelledInterpolatedCurve::clone() const {
    return new LabelledInterpolatedCurve(*this);
}

Void
LabelledInterpolatedCurve::draw(CanvasInterface& c, const Variables2d& v) const
{
    Projection2d proj=projection(this->state_space(),v);
    this->draw(c,proj);
}

} // namespace Ariadne
