/***************************************************************************
 *            chemical-reactor.hpp
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


inline Tuple<String,DottedRealAssignments,RealVariablesBox,RealVariablesBox,Real,double> CR()
{
    RealVariable xA("xA"), xB("xB"), xC("xC"), xD("xD"), u1("u1"), u2("u2"), u3("u3");
    DottedRealAssignments dynamics={dot(xA)=-u3*xA*xB-0.4_dec*xA*xC+0.05_dec*u1-0.1_dec*xA,
                                    dot(xB)=-u3*xA*xB+0.05_dec*u2-0.1_dec*xB,
                                    dot(xC)=u3*xA*xB-0.4_dec*xA*xC-0.1_dec*xC,
                                    dot(xD)=0.4_dec*xA*xC-0.1_dec*xD};
    RealVariablesBox inputs={0.999_dec<=u1<=1.001_dec,0.899_dec<=u2<=0.901_dec,29.8_dec<=u3<=30.2_dec};

    Real e=1/1000000_q;
    RealVariablesBox initial={{0<=xA<=e},{0<=xB<=e},{0<=xC<=e},{0<=xD<=e}};

    Real evolution_time=10;
    double step=1.0/16;

    return make_tuple("CR",dynamics,inputs,initial,evolution_time,step);
}
