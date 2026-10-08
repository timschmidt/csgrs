/***************************************************************************
 *            hybrid/hybrid_reachability_analyser.cpp
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

#include "dynamics/reachability_analyser.tpl.hpp"

#include "hybrid/hybrid_automaton_interface.hpp"

#include "hybrid/hybrid_time.hpp"
#include "hybrid/hybrid_space.hpp"
#include "hybrid/hybrid_orbit.hpp"
#include "hybrid/hybrid_set.hpp"
#include "hybrid/hybrid_paving.hpp"
#include "hybrid/hybrid_evolver.hpp"
#include "hybrid/hybrid_reachability_analyser.hpp"


#include "conclog/logging.hpp"
#include "io/figure.hpp"
#include "solvers/linear_programming.hpp"

#include "hybrid/hybrid_graphics.hpp"

using namespace ConcLog;

namespace Ariadne {

inline Real operator-(Real const& r1, Rational const& q2) {
    return r1-Real(q2);
}

inline FloatDPBounds operator*(Integer n1,  const FloatDP& x2) {
    ARIADNE_ASSERT(n1==n1.get_si());
    FloatDP x1((Int)n1.get_si(),dp);
    return x1*x2;
}

inline Natural compute_time_steps(HybridTime time, HybridTime lock_to_grid_time) {
    return cast_positive(round(time.continuous_time()/lock_to_grid_time.continuous_time()));
}

inline String itoa(Nat m) {
    std::stringstream ss; if(m<10) { ss<<"0"; } ss<<m; return ss.str();
}

inline const HybridExactBoxes& cast_exact(const HybridExactBoxes& boxes) {
    return boxes;
}

} // namespace Ariadne

namespace Ariadne {

template class ReachabilityAnalyser<HybridAutomatonInterface>;

HybridReachabilityAnalyser::
HybridReachabilityAnalyser(const HybridEvolverInterface& evolver)
    : ReachabilityAnalyser<HybridAutomatonInterface>(evolver)
{
}


HybridReachabilityAnalyser*
HybridReachabilityAnalyser::
clone() const
{
    return new HybridReachabilityAnalyser(*this);
}



HybridReachabilityAnalyserConfiguration::ReachabilityAnalyserConfiguration(ReachabilityAnalyser<HybridAutomatonInterface>& analyser)
    : _analyser(analyser)
{
    set_transient_time(0.0);
    set_transient_steps(0);
    set_lock_to_grid_time(1.0);
    set_lock_to_grid_steps(1);
    set_maximum_grid_fineness(3);
    set_maximum_grid_extent(16);
    set_grid(std::shared_ptr<HybridGrid>(new HybridGrid(_analyser.system().state_space(),SimpleHybridScalings())));
    set_outer_overspill_policy(ChainOverspillPolicy::ERROR);
}


OutputStream&
HybridReachabilityAnalyserConfiguration::_write(OutputStream& os) const
{
    os << "HybridReachabilityAnalyserSettings"
       << "(\n  transient_time=" << transient_time()
       << ",\n  transient_steps=" << transient_steps()
       << ",\n  lock_to_grid_steps=" << lock_to_grid_steps()
       << ",\n  lock_to_grid_time=" << lock_to_grid_time()
       << ",\n  maximum_grid_fineness=" << maximum_grid_fineness()
       << ",\n  maximum_grid_extent=" << maximum_grid_extent()
       << ",\n  bounding_domain=" << (bounding_domain_ptr() ? "available" : "none")
       << ",\n  grid=" << grid()
       << ",\n  outer_overspill_policy=" << outer_overspill_policy()
       << "\n)\n";
    return os;
}


Void
HybridReachabilityAnalyserConfiguration::set_bounding_domain_ptr(const std::shared_ptr<HybridExactBoxes> value)
{
 //   ARIADNE_ASSERT_MSG(possibly(value_ptr->space() == _analyser.system().state_space()),
 //           "The bounding domain to set has a different hybrid space than the system.");

    _bounding_domain_ptr = value;
}


Void
HybridReachabilityAnalyserConfiguration::set_grid(const std::shared_ptr<HybridGrid> value_ptr)
{
    ARIADNE_ASSERT_MSG(possibly(value_ptr->space() == _analyser.system().state_space()),
            "The grid to set has a different hybrid space than the system.");

    _grid_ptr = value_ptr;
}

} // namespace Ariadne
