/***************************************************************************
 *            hybrid/hybrid_reachability_analyser_interface.hpp
 *
 *  Copyright  2011-20  Luca Geretti
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

/*! \file hybrid/hybrid_reachability_analyser_interface.hpp
 *  \brief Interface for analysis in the hybrid case.
 */

#ifndef ARIADNE_HYBRID_REACHABILITY_ANALYSER_INTERFACE_HPP
#define ARIADNE_HYBRID_REACHABILITY_ANALYSER_INTERFACE_HPP


#include "hybrid/hybrid_set_interface.hpp"
#include "dynamics/evolver_interface.hpp"
#include "dynamics/reachability_analyser_interface.hpp"

#include "conclog/logging.hpp"

namespace Ariadne {

class HybridAutomatonInterface;
class HybridGridTreePaving;

//! \ingroup AnalysisModule
//! \ingroup HybridDynamicsSubModule
//! \brief A class for performing reachability analysis on a hybrid system.
class HybridReachabilityAnalyserInterface
    : public virtual ReachabilityAnalyserInterface<HybridAutomatonInterface>
{
  public:
    typedef HybridAutomatonInterface SystemType;
    typedef SystemType::StateSpaceType StateSpaceType;
    typedef SystemType::TimeType TimeType;
    typedef EffectiveHybridOvertSetInterface OvertSetInterfaceType;
    typedef EffectiveHybridCompactSetInterface CompactSetInterfaceType;
    typedef EffectiveHybridLocatedSetInterface LocatedSetInterfaceType;
    typedef EffectiveHybridRegularSetInterface RegularSetInterfaceType;
    typedef HybridGridTreePaving SetApproximationType;

  public:
    //!@{
    //! \name Constructors and destructors
    /*! \brief Virtual destructor */
    virtual ~HybridReachabilityAnalyserInterface() = default;

    /*! \brief Make a dynamically-allocated copy. */
    virtual HybridReachabilityAnalyserInterface* clone() const = 0;
    //!@}

};


//! \relates HybridReachabilityAnalyserInterface
//! \brief Factory for hybrid reachability analyser interface classes.
class HybridReachabilityAnalyserFactoryInterface
    : public ReachabilityAnalyserFactoryInterface<HybridAutomatonInterface>
{
  public:

    //! \brief Create a copy of the factory.
    virtual HybridReachabilityAnalyserFactoryInterface* clone() const = 0;
    //! \brief Create a hybrid reachability analyser interface object around a \a system.
    virtual HybridReachabilityAnalyserInterface* create(const HybridAutomatonInterface& system) const = 0;
};


} // namespace Ariadne


#endif // ARIADNE_HYBRID_REACHABILITY_ANALYSER_INTERFACE_HPP
