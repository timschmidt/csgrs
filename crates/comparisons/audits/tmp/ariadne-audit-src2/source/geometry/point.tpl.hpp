/***************************************************************************
 *            point.tpl.hpp
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

#include <cstdarg>
#include <sstream>
#include <string>
#include <vector>

#include "geometry/point.hpp"
#include "geometry/box.hpp"
#include "helper/stlio.hpp"
#include "symbolic/space.hpp"
#include "symbolic/assignment.hpp"
#include "io/figure.hpp"

namespace Ariadne {

template<class X> Point<X>::Point(InitializerList<X> lst)
    : Vector<X>(lst)
{
}

template<class X> Point<X>* Point<X>::clone() const {
    return new Point<X>(*this);
}

template<class X> BoundingBoxType Point<X>::bounding_box() const {
    typedef decltype(declval<X>()+declval<FloatDPError>()) U;
    FloatDPError eps(FloatDP::eps(dp));
    ExactBoxType r(this->dimension());
    UpperIntervalType v;
    for(SizeType i=0; i!=this->dimension(); ++i) {
        X const& pti=(*this)[i];
        r[i]=cast_exact_interval(Interval<U>(pti-eps,pti+eps)); }
    return r;
}

template<class X> Void Point<X>::draw(CanvasInterface& canv, const Projection2d& proj) const {
    canv.dot(numeric_cast<double>((*this)[proj.x_coordinate()]),numeric_cast<double>((*this)[proj.y_coordinate()]));
    canv.stroke();
}


template<class X> LabelledPoint<X>::LabelledPoint(RealSpace const& state_space, Point<X> const& pt)
            : Point<X>(pt), _state_variables(state_space.variable_names()) { }

template<class X> LabelledPoint<X>::LabelledPoint(List<Assignment<RealVariable,X>> const& x)
        : Point<X>(x.size(),[&x](SizeType i){return x[i].right_hand_side();}), _state_variables(RealSpace(left_hand_sides(x)).variable_names()) { }

template<class X> LabelledPoint<X>::LabelledPoint(InitializerList<Assignment<RealVariable,X>> const& x)
        : LabelledPoint<X>(List<Assignment<RealVariable,X>>(x)) { }

template<class X> LabelledPoint<X>* LabelledPoint<X>::clone() const {
    return new LabelledPoint(*this);
}

template<class X> RealSpace LabelledPoint<X>::state_space() const {
    return RealSpace(this->_state_variables);
}

template<class X> Void LabelledPoint<X>::draw(CanvasInterface& canvas, const Variables2d& axes) const {
    Projection2d proj=projection(this->state_space(),axes);
    this->draw(canvas,proj);
}

} //namespace Ariadne
