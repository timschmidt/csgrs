/***************************************************************************
 *            harmonic-oscillator.hpp
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


inline Tuple<String,DottedRealAssignments,RealVariablesBox,RealVariablesBox,Real,double> HO()
{
    RealVariable x("x"), y("y"), u("u");
    DottedRealAssignments dynamics={dot(x)=y+u,dot(y)=-x};
    RealVariablesBox inputs={-4/100_q<=u<=4/100_q};

    Real e=1/10000000_q;
    RealVariablesBox initial={{-e<=x<=e},{-e<=y<=e}};

    Real evolution_time=3.141592_dec;
    double step = 1.0/64;

    return make_tuple("HO",dynamics,inputs,initial,evolution_time,step);
}
