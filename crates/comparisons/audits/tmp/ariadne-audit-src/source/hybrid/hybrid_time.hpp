/***************************************************************************
 *            hybrid/hybrid_time.hpp
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

/*! \file hybrid/hybrid_time.hpp
 *  \brief Hybrid times
 */

#ifndef ARIADNE_HYBRID_TIME_HPP
#define ARIADNE_HYBRID_TIME_HPP

#include "numeric/numeric.hpp"
#include "hybrid/discrete_event.hpp"

namespace Ariadne {

class DiscreteEvent;

//! \ingroup AnalysisModule
//! \ingroup HybridDynamicsSubModule
//! \brief A value in a hybrid time domain, being a pair comprising a real \a continuous_time
//! and an integer \a discrete_time.
//!
//! When a %HybridTime is used to define a bound on a hybrid evolution, the evolution should
//! stop when <em>either</em> the continuous time or the discrete time reaches the bounding
//! value. This is to ensure that the evolution time is finite; in particular, that no
//! Zeno behaviour occurs.
class HybridTime
{
  public:
    //! \brief The type used for continuous (real, physical) time.
    typedef Real ContinuousTimeType;
    //! \brief The type used for discrete time (steps).
    typedef Integer DiscreteTimeType;

    //! \brief The continuous (real, physical) time.
    const ContinuousTimeType& continuous_time() const { return this->_continuous_time; }
    //! \brief The number of discrete steps taken.
    const DiscreteTimeType& discrete_time() const { return this->_discrete_time; }
  private: public:
    //! \brief The continuous (real, physical) time.
    ContinuousTimeType _continuous_time;
    //! \brief The number of discrete steps taken.
    DiscreteTimeType _discrete_time;
  public:
    explicit HybridTime()
      : _continuous_time(0), _discrete_time(-1) { }
    explicit HybridTime(Real t)
      : _continuous_time(t), _discrete_time(-1) { }
    //! \brief Create the hybrid time moment with continuous-time \a t and discrete-time steps \a n.
    HybridTime(Real t, Integer n)
      : _continuous_time(t), _discrete_time(n) { }
    HybridTime(FloatDP t, Integer n)
      : _continuous_time(t), _discrete_time(n) { }
    HybridTime(ExactDouble t, Int n)
      : _continuous_time(t), _discrete_time(n) { }
    HybridTime(ExactDouble t, Nat m)
      : _continuous_time(t), _discrete_time(m) { }
    HybridTime(ExactDouble t, double n) = delete;
    //! \brief The arguments for the HybridTime constructor must be ordered continuous-time, then discrete-time.
    HybridTime(Integer n, double t) = delete;

    friend HybridTime operator*(const Integer& c, const HybridTime& ht) {
        return HybridTime(c*ht._continuous_time,c*ht._discrete_time);
    }

    friend HybridTime operator-(const HybridTime& ht1, const HybridTime& ht2) {
        return HybridTime(ht1._continuous_time-ht2._continuous_time,max(ht1._discrete_time-ht2._discrete_time,Integer(0)));
    }

    friend Bool same(const HybridTime& ht1, const HybridTime& ht2) {
        return same(ht1._continuous_time,ht2._continuous_time) &&
            ht1._discrete_time==ht2._discrete_time;
    }

    friend NegatedSierpinskian operator==(const HybridTime& ht1, const HybridTime& ht2) {
        return ht1._continuous_time==ht2._continuous_time &&
            ht1._discrete_time==ht2._discrete_time;
    }

    friend Sierpinskian operator!=(const HybridTime& ht1, const HybridTime& ht2) {
        return ht1._continuous_time!=ht2._continuous_time ||
            ht1._discrete_time!=ht2._discrete_time;
    }

    friend Kleenean operator<=(const HybridTime& ht1, const HybridTime& ht2) {
        return Kleenean(ht1._continuous_time<=ht2._continuous_time) &&
            Boolean(ht1._discrete_time<=ht2._discrete_time);
    }

    friend Kleenean operator<(const HybridTime& ht1, const HybridTime& ht2) {
        return Kleenean(ht1._continuous_time< ht2._continuous_time) &&
            Boolean(ht1._discrete_time<ht2._discrete_time);
    }

    friend Kleenean operator>(const HybridTime& ht1, const HybridTime& ht2) {
        return Kleenean(ht1._continuous_time> ht2._continuous_time) &&
            Boolean(ht1._discrete_time>ht2._discrete_time);
    }

    friend Kleenean operator>(const HybridTime& ht1, const ContinuousTimeType& ct2) {
        return Kleenean(ht1._continuous_time> ct2);
    }

    friend OutputStream& operator<<(OutputStream& os, const HybridTime& ht) {
        return os << "(time="<<ht._continuous_time<<", steps="<<ht._discrete_time<<")";
    }
};

class HybridTerminationCriterion
{
  public:
    typedef HybridTime::ContinuousTimeType ContinuousTimeType;
    typedef HybridTime::DiscreteTimeType DiscreteTimeType;
  private:
    ContinuousTimeType _maximum_time;
    DiscreteTimeType _maximum_steps;
    Set<DiscreteEvent> _terminating_events;
  public:
    HybridTerminationCriterion(ContinuousTimeType tmax, DiscreteTimeType nmax, Set<DiscreteEvent> evnts)
            : _maximum_time(tmax), _maximum_steps(nmax), _terminating_events(evnts) { } //!< .
    HybridTerminationCriterion(ContinuousTimeType tmax, DiscreteTimeType nmax)
            : HybridTerminationCriterion(tmax,nmax,Set<DiscreteEvent>()) { } //!< .
    HybridTerminationCriterion(const HybridTime& maximum_time)
            : HybridTerminationCriterion(maximum_time.continuous_time(),maximum_time.discrete_time()) { } //!< .
    //! \brief The maximum continuous (real, physical) time.
    const ContinuousTimeType& maximum_time() const { return this->_maximum_time; }
    //! \brief The maximum number of discrete steps taken.
    const DiscreteTimeType& maximum_steps() const { return this->_maximum_steps; }
    //! \brief The maximum number of discrete steps taken.
    const Set<DiscreteEvent>& terminating_events() const { return this->_terminating_events; }
};
inline OutputStream& operator<<(OutputStream& os, const HybridTerminationCriterion& termination) {
    return os << "HybridTerminationCriterion( maximum_time=" << termination.maximum_time()
              << ", maximum_steps="<<termination.maximum_steps()
              << ", terminating_events="<<termination.terminating_events() << " )";
}

} // namespace Ariadne

#endif // ARIADNE_HYBRID_TIME_HPP
