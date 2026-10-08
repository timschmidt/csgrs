/***************************************************************************
 *            fitzhugh-nagumo.hpp
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


inline Tuple<String,DottedRealAssignments,RealVariablesBox,RealVariablesBox,Real,double> FN()
{
    RealVariable x("x"), y("y"), u1("u1"), u2("u2");
    DottedRealAssignments dynamics={dot(x)=x-pow(x,3)-y+u1,dot(y)=x+u2-0.8_dec*y};
    RealVariablesBox inputs={0.874_dec<=u1<=0.876_dec,0.699_dec<=u2<=0.701_dec};

    Real e=1/100_q;
    RealVariablesBox initial={{-1-e<=x<=-1+e},{1-e<=y<=1+e}};

    Real evolution_time=10;
    double step=1.0/20;

    return make_tuple("FN",dynamics,inputs,initial,evolution_time,step);
}
