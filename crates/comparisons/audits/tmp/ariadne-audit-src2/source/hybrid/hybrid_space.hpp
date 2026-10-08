/***************************************************************************
 *            hybrid/hybrid_space.hpp
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

/*! \file hybrid/hybrid_space.hpp
 *  \brief Sets in hybrid spaces.
 */

#ifndef ARIADNE_HYBRID_SPACE_HPP
#define ARIADNE_HYBRID_SPACE_HPP

#include <map>

#include "utility/container.hpp"
#include "helper/stlio.hpp"
#include "symbolic/space.hpp"
#include "hybrid/discrete_location.hpp"
#include "hybrid/hybrid_set_interface.hpp"


namespace Ariadne {

class LocationException : public std::runtime_error {
  public:
    LocationException(const StringType& what) : std::runtime_error(what) { }
};

class HybridGridTreePaving;

class HybridSpaceInterface
{
  public:
    virtual ~HybridSpaceInterface() = default;
    virtual Bool has_location(const DiscreteLocation& q) const  = 0;
    virtual RealSpace operator[](const DiscreteLocation& q) const = 0;
  public:
    virtual HybridSpaceInterface* clone() const = 0;
    virtual OutputStream& _write(OutputStream& os) const = 0;
    virtual ValidatedKleenean operator==(const HybridSpaceInterface& other) const = 0;
  public:
    friend OutputStream& operator<<(OutputStream& os, const HybridSpaceInterface& hsp) { return hsp._write(os); }
};

//! \ingroup HybridModule
//! \brief A hybrid space \f$\bigsqcup_{q\in Q} \R^{d_q}\f$ with discrete states \f$Q\f$.
class HybridSpace
{
  public:
    //! \brief The canonical type used for bounding sets in the space.
    typedef HybridExactBoxes BoundingDomainType;
    //! \brief The interface satisified by bounded sets in the space.
    typedef EffectiveHybridBoundedSetInterface BoundedSetInterfaceType;
    //! \brief The interface satisified by overt sets in the space.
    typedef EffectiveHybridOvertSetInterface OvertSetInterfaceType;
    //! \brief The interface satisified by over sets in the space.
    typedef EffectiveHybridOpenSetInterface OpenSetInterfaceType;
    //! \brief The interface satisified by closed sets in the space.
    typedef EffectiveHybridClosedSetInterface ClosedSetInterfaceType;
    //! \brief The interface satisified by compact sets in the space.
    typedef EffectiveHybridCompactSetInterface CompactSetInterfaceType;
    //! \brief The interface satisified by regular sets in the space.
    typedef EffectiveHybridRegularSetInterface RegularSetInterfaceType;
    //! \brief The interface satisified by located sets in the space.
    typedef EffectiveHybridLocatedSetInterface LocatedSetInterfaceType;
    //! \brief The interface satisified by located sets in the space.
    typedef EffectiveHybridRegularLocatedSetInterface RegularLocatedSetInterfaceType;
    //! \brief The type of approximations to sets in the space.
    typedef HybridGridTreePaving SetApproximationType;
  public:
    HybridSpace(const HybridSpaceInterface& hspc) : _ptr(hspc.clone()) { }
    HybridSpace(const HybridSpaceInterface* hspc_ptr) : _ptr(hspc_ptr) { }

    Bool has_location(const DiscreteLocation& q) const { return this->_ptr->has_location(q); }
    RealSpace operator[](const DiscreteLocation& q) const { return this->_ptr->operator[](q); }

    ValidatedKleenean operator==(const HybridSpace& other) const { return this->_ptr->operator==(other); }

    operator const HybridSpaceInterface& () const { return *_ptr; }

    friend OutputStream& operator<<(OutputStream& os, const HybridSpace& hsp) { return os << *hsp._ptr; }
  private:
    std::shared_ptr<const HybridSpaceInterface> _ptr;
};


//! \ingroup HybridModule
//! \brief A hybrid space \f$\bigsqcup_{q\in Q} \R^{d_q}\f$ with discrete states \f$Q\f$.
class MonolithicHybridSpace
    : public HybridSpaceInterface
{
  public:
    typedef Map<DiscreteLocation, RealSpace >::ConstIterator ConstIterator;

    MonolithicHybridSpace() : _locations() { }

    explicit MonolithicHybridSpace(const Map< DiscreteLocation, RealSpace >& locations) : _locations(locations) { }

    MonolithicHybridSpace* clone() const { return new MonolithicHybridSpace(*this); }

    Void new_location(const DiscreteLocation& q, const RealSpace& spc) { this->_locations.insert(q,spc); }

    Bool has_location(const DiscreteLocation& q) const { return _locations.has_key(q); }

    ValidatedKleenean operator==(const HybridSpaceInterface& other) const {
        for (ConstIterator iter=this->_locations.begin(); iter!=this->_locations.end(); ++iter) {
            if (!other.has_location(iter->first)) return false;
            if (other[iter->first] != iter->second) return false;
        }
        return true;
    }

    RealSpace operator[](const DiscreteLocation& q) const {
        ConstIterator iter = this->_locations.find(q);
        if(iter==this->_locations.end()) {
            ARIADNE_THROW(LocationException,"HybridSpace[DiscreteLocation q]","Space has no location "<<q); }
        return iter->second; }

    Set<DiscreteLocation> locations() const { return this->_locations.keys(); }
    ConstIterator begin() const { return this->_locations.begin(); }
    ConstIterator end() const { return this->_locations.end(); }

    OutputStream& _write(OutputStream& os) const { return os << "HybridSpace( " << this->_locations << " )"; }
  private:
    Map< DiscreteLocation, RealSpace > _locations;
};



}

#endif // ARIADNE_HYBRID_SPACE_HPP

