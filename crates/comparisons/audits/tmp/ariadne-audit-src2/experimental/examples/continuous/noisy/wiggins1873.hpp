/***************************************************************************
 *            wiggins1873.hpp
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


inline Tuple<String,DottedRealAssignments,RealVariablesBox,RealVariablesBox,Real,double> W18()
{
    RealVariable x("x"), y("y"), u1("u1"), u2("u2");
    DottedRealAssignments dynamics={dot(x)=-x+2*y+pow(x,2)*y+pow(x,4)*pow(y,5)+u1,
                                         dot(y)=-y-pow(x,4)*pow(y,6)+pow(x,8)*pow(y,9)+u2};
    RealVariablesBox inputs={-2/100_q<=u1<=2/100_q,-2/100_q<=u2<=2/100_q};

    auto e=1/10000000_q;
    RealVariablesBox initial={{1/3_q-e<=x<=1/3_q+e},{1/3_q-e<=y<=1/3_q+e}};

    Real evolution_time=9;
    double step=1.0/16;

    return make_tuple("W18",dynamics,inputs,initial,evolution_time,step);
}
