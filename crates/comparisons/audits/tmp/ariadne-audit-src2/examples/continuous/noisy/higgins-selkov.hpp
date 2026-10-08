/***************************************************************************
 *            higgins-selkov.hpp
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


inline Tuple<String,DottedRealAssignments,RealVariablesBox,RealVariablesBox,Real,double> HS()
{
    RealVariable S("S"), P("P"), v0("v0"), k1("k1"), k2("k2");
    DottedRealAssignments dynamics={dot(S)=v0-S*k1*pow(P,2),dot(P)=S*k1*pow(P,2)-k2*P};
    RealVariablesBox inputs={0.9998_dec<=v0<=1.0002_dec,0.9998_dec<=k1<=1.0002_dec,0.99981_dec<=k2<=1.00021_dec};

    Real e=1/100_q;
    RealVariablesBox initial={{2-e<=S<=2+e},{1-e<=P<=1+e}};

    Real evolution_time=10;
    double step=1.0/50;

    return make_tuple("HS",dynamics,inputs,initial,evolution_time,step);
}
