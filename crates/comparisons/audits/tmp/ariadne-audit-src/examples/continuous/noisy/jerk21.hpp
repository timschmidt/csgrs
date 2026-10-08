/***************************************************************************
 *            jerk21.hpp
 *
 *  Copyright  2008-18 Luca Geretti
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

#include "ariadne.hpp"

using namespace Ariadne;


inline Tuple<String,DottedRealAssignments,RealVariablesBox,RealVariablesBox,Real,double> J21()
{
    RealVariable x("x"), y("y"), z("z"), u("u");
    DottedRealAssignments dynamics={dot(x)=y,dot(y)=z,dot(z)=-pow(z,3)-y*pow(x,2)-u*x};
    RealVariablesBox inputs={0.249_dec<=u<=0.251_dec};

    Real e=1/1024_q;
    RealVariablesBox initial={{0.25_dec-e<=x<=0.25_dec+e},{-e<=y<=e},{-e<=z<=e}};

    Real evolution_time=10;
    double step=1.0/16;

    return make_tuple("J21",dynamics,inputs,initial,evolution_time,step);
}
