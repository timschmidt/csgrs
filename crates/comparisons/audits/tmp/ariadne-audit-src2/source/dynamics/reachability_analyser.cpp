/***************************************************************************
 *            dynamics/reachability_analyser.cpp
 *
 *  Copyright  2006-20  Alberto Casagrande, Pieter Collins, Davide Bresolin
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

#include "function/functional.hpp"
#include "config.hpp"

#include <string>
#include <sstream>
#include <algorithm>

#include <list>
#include <set>
#include <vector>
#include <valarray>


#include "utility/exceptions.hpp"

#include "numeric/numeric.hpp"

#include "algebra/vector.hpp"
#include "algebra/matrix.hpp"

#include "geometry/box.hpp"
#include "geometry/list_set.hpp"
#include "geometry/grid_paving.hpp"

#include "solvers/integrator.hpp"
#include "solvers/solver.hpp"
#include "geometry/function_set.hpp"

#include "dynamics/vector_field.hpp"

#include "dynamics/orbit.hpp"
#include "dynamics/iterated_map_evolver.hpp"
#include "dynamics/vector_field_evolver.hpp"
#include "dynamics/reachability_analyser.hpp"

#include "conclog/logging.hpp"
#include "io/figure.hpp"
#include "solvers/linear_programming.hpp"

#include "dynamics/reachability_analyser.tpl.hpp"

using namespace ConcLog;

namespace Ariadne {

template class ReachabilityAnalyser<IteratedMap>;
template class ReachabilityAnalyserConfiguration<IteratedMap>;

template class ReachabilityAnalyser<VectorField>;
template class ReachabilityAnalyserConfiguration<VectorField>;

OutputStream& operator<<(OutputStream& os, const ChainOverspillPolicy& policy)
{
    switch(policy) {
        case ChainOverspillPolicy::IGNORE: os<<"ignore"; break;
        case ChainOverspillPolicy::WARNING: os<<"warning"; break;
        case ChainOverspillPolicy::ERROR: os<<"error"; break;
        default: abort();
    } return os;
}

} // namespace Ariadne
