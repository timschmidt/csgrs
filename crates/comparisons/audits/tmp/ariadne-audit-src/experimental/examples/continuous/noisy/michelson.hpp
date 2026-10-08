/***************************************************************************
 *            michelson.hpp
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


inline Tuple<String,DottedRealAssignments,RealVariablesBox,RealVariablesBox,Real,double> MI()
{
    RealVariable x("x"), y("y"), z("z"), c("c");
    DottedRealAssignments dynamics={dot(x)=y,
                                    dot(y)=z,
                                    dot(z)=c-y-sqr(x)/2};

    Real ei=0.0_dec;
    RealVariablesBox inputs={1-ei<=c<=1+ei};

    Real es=0.0_dec;
    RealVariablesBox initial={{0-es<=x<=0+es},{0.5_dec-es<=y<=0.5_dec+es},{0-es<=z<=0+es}};

    Real evolution_time=9.5_dec;
    double step=1.0/64;

    return make_tuple("MI",dynamics,inputs,initial,evolution_time,step);
}
