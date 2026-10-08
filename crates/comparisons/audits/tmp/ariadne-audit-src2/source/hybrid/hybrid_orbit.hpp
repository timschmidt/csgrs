/***************************************************************************
 *            hybrid/hybrid_orbit.hpp
 *
 *  Copyright  2007-20  Pieter Collins
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

/*! \file hybrid/hybrid_orbit.hpp
 *  \brief Orbits of hybrid dynamical systems
 */

#ifndef ARIADNE_HYBRID_ORBIT_HPP
#define ARIADNE_HYBRID_ORBIT_HPP

#include <utility>
#include <iostream>
#include <vector>
#include <map>
#include <memory>

#include "numeric/numeric.hpp"
#include "io/graphics_interface.hpp"
#include "geometry/function_set.hpp"
#include "hybrid/hybrid_set.hpp"
#include "hybrid/hybrid_enclosure.hpp"

#include "dynamics/orbit.hpp"

namespace Ariadne {

typedef FloatDP Time;

template<class ES> class Orbit;

template<class BS> class ListSet;
template<class X> class Point;

class Grid;
class GridCell;
class GridTreePaving;
class HybridGrid;
class HybridGridCell;
class HybridGridTreePaving;
class HybridTime;

class HybridAutomatonInterface;

class DiscreteLocation;

typedef HybridBasicSet<InterpolatedCurve> HybridInterpolatedCurve;

template<class ES> OutputStream& operator<<(OutputStream&, const Orbit<ES>&);

template<>
class Orbit<HybridApproximatePoint>
    : public HybridDrawableInterface
{
  public:
    Orbit(const HybridApproximatePoint& hpt);
    Orbit(List<HybridInterpolatedCurve> crvs);
    Void insert(HybridTime ht, const HybridApproximatePoint& hpt);
    SizeType size() const;
    const InterpolatedCurve& curve(Nat m) const;
    const List<HybridInterpolatedCurve>& curves() const { return *this->_curves_ptr; }
    Void draw(CanvasInterface& c, const Set<DiscreteLocation>& l, const Variables2d& v) const;
  private:
    SharedPointer<List<HybridInterpolatedCurve> > _curves_ptr;
};

template<>
class Orbit<Vector<HybridApproximatePoint>>
  : public HybridDrawableInterface
{
  public:
    Orbit(const Vector<HybridApproximatePoint>& vhpt);
    Void insert(HybridTime ht, const HybridApproximatePoint& hpt, SizeType setNumber);
    SizeType size(Nat setNumber);
    const InterpolatedCurve& curve(Nat setNumber, Nat step);
    const List<HybridInterpolatedCurve>& curves(Nat setNumber) { return this->_curves_ptr->at(setNumber); }
    Void draw(CanvasInterface& c, const Set<DiscreteLocation>& l, const Variables2d& v) const;
  private:
    Vector<List<HybridInterpolatedCurve> >  *_curves_ptr;
};

Orbit<HybridApproximatePoint> extend_auxiliary(Orbit<HybridApproximatePoint> const& horb, HybridAutomatonInterface const& ha);

template<>
OutputStream&
operator<<(OutputStream& os, const Orbit< HybridApproximatePoint >& orb);

template<>
class Orbit<HybridStorage>
    : public HybridDrawableInterface
{
    struct Data;
  public:
    typedef HybridStorage EnclosureListType;

    Orbit(const HybridStorage& initial_set);
    Orbit(const HybridStorage& initial_set, const HybridStorage& reach_set,
          const HybridStorage& intermediate_set, const HybridStorage& final_set);

    HybridStorage const& initial() const;
    HybridStorage const& reach() const;
    HybridStorage const& intermediate() const;
    HybridStorage const& final() const;

    Void draw(CanvasInterface& c, const Set<DiscreteLocation>& l, const Variables2d& v) const;
  private:
    SharedPointer<Data> _data;
};


template<>
class Orbit<HybridEnclosure>
    : public HybridDrawableInterface
{
  public:
    typedef HybridEnclosure EnclosureType;
    typedef ListSet<HybridEnclosure> EnclosureListType;

    Orbit() { }
    Orbit(const EnclosureType& set) : _initial(set) { }
    Void adjoin_reach(const EnclosureType& set) { this->_reach.adjoin(set); }
    Void adjoin_intermediate(const EnclosureType& set) { this->_intermediate.adjoin(set); }
    Void adjoin_final(const EnclosureType& set) { this->_final.adjoin(set); }

    Void adjoin_reach(const EnclosureListType& set) { this->_reach.adjoin(set); }
    Void adjoin_intermediate(const EnclosureListType& set) { this->_intermediate.adjoin(set); }
    Void adjoin_final(const EnclosureListType& set) { this->_final.adjoin(set); }

    EnclosureType const& initial() const { return this->_initial; }
    EnclosureListType const& reach() const { return this->_reach; }
    EnclosureListType const& intermediate() const { return this->_intermediate; }
    EnclosureListType const& final() const { return this->_final; }

    Void draw(CanvasInterface& c, const Set<DiscreteLocation>& l, const Variables2d& v) const;
  private:
    EnclosureType _initial;
    EnclosureListType _reach;
    EnclosureListType _intermediate;
    EnclosureListType _final;
};

template<>
OutputStream&
operator<<(OutputStream& os, const Orbit< HybridEnclosure >& orb);


} // namespace Ariadne

#endif // ARIADNE_HYBRID_ORBIT_HPP
