/***************************************************************************
 *            order7.hpp
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

inline Tuple<String,DottedRealAssignments,RealVariablesBox,RealVariablesBox,Real,double> O7()
{
    RealVariable x("x"), y("y"), u("u");
    DottedRealAssignments dynamics={dot(x)=-42*pow(x,7)+68*pow(x,6)*y-46*pow(x,5)*y+256*pow(x,4)*y+156*pow(x,3)*y+50*pow(x,2)*y+20*x*pow(y,6)-8*pow(y,7),
                                         dot(y)=y*(1110*pow(x,6)-220*pow(x,5)*y-3182*pow(x,4)*y+478*pow(x,3)*pow(y,3)+487*pow(x,2)*pow(y,4)-102*x*pow(y,5)-12*pow(y,6))+u};
    RealVariablesBox inputs={-1/100_q<=u<=1/100_q};

    auto e=1/10_q;
    RealVariablesBox initial={{-1-e<=x<=-1+e},{1-e<=y<=1+e}};

    Real evolution_time=5;
    double step=1.0/32;

    return make_tuple("O7",dynamics,inputs,initial,evolution_time,step);
}
