/***************************************************************************
 *            lorenz-attractor.hpp
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


inline Tuple<String,DottedRealAssignments,RealVariablesBox,RealVariablesBox,Real,double> LA()
{
    RealVariable x("x"), y("y"), z("z"), rho("rho");
    RealConstant sigma("sigma",10);
    RealConstant beta("beta",8/3_q);
    DottedRealAssignments dynamics={dot(x)=sigma*(y-x),
                                         dot(y)=x*(rho - z) - y,
                                         dot(z)=x*y - beta*z};
    RealVariablesBox inputs={28-1/100_q<=rho<=28+1/100_q};

    Real e=1/1024_q;
    RealVariablesBox initial={1-e<=x<=1+e,1-e<=y<=1+e,1-e<=z<=1+e};

    Real evolution_time=1;
    double step=1.0/256;

    return make_tuple("LA",dynamics,inputs,initial,evolution_time,step);
}
