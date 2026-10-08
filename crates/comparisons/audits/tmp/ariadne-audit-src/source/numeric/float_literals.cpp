/***************************************************************************
 *            numeric/float_literals.cpp
 *
 *  Copyright  2008-20  Pieter Collins
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

#include "float_error.hpp"
#include "float_ball.hpp"
#include "float_lower_bound.hpp"
#include "float_upper_bound.hpp"
#include "float_approximation.hpp"

namespace Ariadne {

FloatError<DoublePrecision> operator"" _error(long double lx) {
    double x=lx;
    assert(x==lx);
    return FloatError<DoublePrecision>(FloatDP(cast_exact(x),dp));
}


Float<DoublePrecision> operator"" _exact(long double lx) {
    double x=lx;
    assert(x==lx);
    return Float<DoublePrecision>(ExactDouble(x),dp);
}

FloatBall<DoublePrecision> operator"" _near(long double lx) {
    volatile double x=lx;
    volatile long double le=std::abs((long double)x-lx);
    volatile double e=le;
    while(e<le) { e=e*(1+std::numeric_limits<double>::epsilon()); }

    return FloatBall<DoublePrecision>(FloatDP(cast_exact(x),dp),FloatDP(cast_exact(e),dp));
}


FloatUpperBound<DoublePrecision> operator"" _upper(long double lx) {
    static const double eps = std::numeric_limits<double>::epsilon();
    static const double min = std::numeric_limits<double>::min();
    double x=lx;
    if(x<lx) { x+=min; }

    while (x<lx) { x+=std::abs(x)*eps; }

    return FloatUpperBound<DoublePrecision>(FloatDP(cast_exact(x),dp));
}


FloatLowerBound<DoublePrecision> operator"" _lower(long double lx) {
    static const double eps = std::numeric_limits<double>::epsilon();
    static const double min = std::numeric_limits<double>::min();
    double x=lx;
    if(x>lx) { x-=min; }


    while (x>lx) { x-=std::abs(x)*eps; }

    return FloatLowerBound<DoublePrecision>(FloatDP(cast_exact(x),dp));
}


FloatApproximation<DoublePrecision> operator"" _approx(long double lx) {
    double x=lx;
    return FloatApproximation<DoublePrecision>(FloatDP(cast_exact(x),dp));
}

} // namespace Ariadne
