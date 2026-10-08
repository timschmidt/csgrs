/***************************************************************************
 *            ross-mcdonald.hpp
 *
 *  Copyright  2008-21 Luca Geretti
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

inline Tuple<String,DottedRealAssignments,RealVariablesBox,RealVariablesBox,Real,double> RM()
{
    RealVariable x("x"), y("y"), a("a"), b("b"), r("r");
    RealConstant m("m",2);
    DottedRealAssignments dynamics={dot(x)=a*y*(1-x)-r*x, dot(y)=b*x*(1-y)-m*y};

    auto ca=3_dec;
    auto cb=8_dec;
    auto cr=2_dec;
    auto da=0.02_dec;
    auto db=0.02_dec;
    auto dr=0.02_dec;
    RealVariablesBox inputs={{ca-da<=a<=ca+da},{cb-db<=b<=cb+db},{cr-dr<=r<=cr+dr}};

    RealVariablesBox initial={{x==0.95_dec},{y==0.95_dec}};

    Real evolution_time=2;
    double step=1.0/32;

    return make_tuple("RM",dynamics,inputs,initial,evolution_time,step);
}
