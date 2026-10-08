/***************************************************************************
 *            hybrid/hybrid_storage.hpp
 *
 *  Copyright  2009-20  Pieter Collins
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

/*! \file hybrid/hybrid_storage.hpp
 *  \brief Storage sets for hybrid systems
 */

#ifndef ARIADNE_HYBRID_STORAGE_HPP
#define ARIADNE_HYBRID_STORAGE_HPP

#include <string>
#include <vector>
#include <list>
#include <iostream>

#include "conclog/logging.hpp"
#include "utility/declarations.hpp"
#include "utility/pointer.hpp"
#include "utility/container.hpp"

#include "hybrid/hybrid_set.decl.hpp"
#include "hybrid/discrete_location.hpp"
#include "hybrid/discrete_event.hpp"
#include "hybrid/hybrid_graphics_interface.hpp"
#include "hybrid/hybrid_automaton_interface.hpp"

#include "geometry/box.hpp"
#include "dynamics/storage.hpp"

using namespace ConcLog;

namespace Ariadne {

template<class X> class Vector;

class Storage;
class GridTreePaving;

// FIXME: This class should not be necessary, as indexing HybridStorage should give a reference to a Euclidean/Labelled storage
class HybridStorageElementReference {
    HybridStorage* _hstorage; DiscreteLocation const& _loc;
  public:
    HybridStorageElementReference(HybridStorage* hstorage, DiscreteLocation const& loc) : _hstorage(hstorage), _loc(loc) { }
    inline operator const LabelledStorage () const;
    inline Void clear();
};

//! \ingroup HybridSetSubModule
//! \brief A class representing global reach or evolve set for a hybrid system.
class HybridStorage
    : public HybridDrawableInterface
{
    HybridGridTreePaving  _state_set;
    SharedPointer<HybridAutomatonInterface const> _system_ptr;
  public:
  public:
    typedef HybridGrid GridType;
    typedef HybridGridTreePaving PavingType;

    HybridStorage(HybridGrid const& hg, HybridAutomatonInterface const& sys) : _state_set(hg), _system_ptr(sys.clone()) { }
    HybridStorage(HybridGridTreePaving const& hgtp, HybridAutomatonInterface const& sys) : _state_set(hgtp), _system_ptr(sys.clone()) { }

    HybridAutomatonInterface const& system() const { return *this->_system_ptr; }
    HybridAutomatonInterface const& auxiliary_data() const { return *this->_system_ptr; }
    LabelledMapping auxiliary_mapping(DiscreteLocation const& loc) const {
        return LabelledMapping(this->_system_ptr->continuous_state_space(loc),this->_system_ptr->auxiliary_function(loc),this->_system_ptr->continuous_auxiliary_space(loc)); }

    HybridGrid grid() const { return this->_state_set.grid(); }
    HybridGridTreePaving& state_set() {
        return this->_state_set; }
    HybridGridTreePaving const& state_set() const {
        return this->_state_set; }
    HybridGridTreePaving state_auxilary_set() const {
        return extend_auxiliary(this->_state_set,*this->_system_ptr); }

    Bool is_empty() const { return this->_state_set.is_empty(); }
    SizeType size() const { return this->_state_set.size(); }
    HybridGridTreePaving::ConstIterator begin() const { return this->_state_set.begin(); }
    HybridGridTreePaving::ConstIterator end() const { return this->_state_set.end(); }

    Void clear() { this->_state_set.clear(); }
    Void adjoin(HybridStorage const& other) { return this->_state_set.adjoin(other._state_set); }
    Void remove(HybridStorage const& other) { return this->_state_set.remove(other._state_set); }
    Void restrict(HybridStorage const& other) { return this->_state_set.restrict(other._state_set); }

    friend Bool subset(HybridStorage const& set1, HybridStorage const& set2) {
        return subset(set1._state_set,set2._state_set); }
    friend Bool intersect(HybridStorage const& set1, HybridStorage const& set2) {
        return intersect(set1._state_set,set2._state_set); }

    Void adjoin_lower_approximation(EffectiveHybridOvertSetInterface const& set, Nat extent, Nat fineness) {
        this->_state_set.adjoin_lower_approximation(set,extent,fineness); }
    Void adjoin_outer_approximation(EffectiveHybridCompactSetInterface const& set, Nat fineness) {
        this->_state_set.adjoin_outer_approximation(set,fineness); }
    Void adjoin_outer_approximation(HybridExactBoxes const& bx, Nat fineness) {
        this->_state_set.adjoin_outer_approximation(bx,fineness); }

    Void restrict_to_extent(Nat level) { this->_state_set.restrict_to_extent(level); }
    Void mince(Nat fineness) { this->_state_set.mince(fineness); }
    Void recombine() { this->_state_set.recombine(); }

    const LabelledStorage operator[] (DiscreteLocation const& loc) const;
    HybridStorageElementReference operator[] (DiscreteLocation const& loc) {
        return HybridStorageElementReference{this,loc}; }

    virtual Void draw(CanvasInterface& cnv, const Set<DiscreteLocation>& locs, const Variables2d& vars) const override {
        if (this->_system_ptr!=nullptr) {
            this->state_auxilary_set().draw(cnv,locs,vars);
        } else {
            this->state_set().draw(cnv,locs,vars);
        }
    }

    friend OutputStream& operator<<(OutputStream& os, HybridStorage const& set) {
        return os << "HybridStorage(" << set._state_set << ", system_ptr=" << set._system_ptr << ", system.name()=" << set._system_ptr->name() << ")"; }

    friend class HybridStorageElementReference;
};

//HybridStorageElementReference::operator const Storage () const { return (*_hstorage)[_loc]; }
HybridStorageElementReference::operator const LabelledStorage () const {
    return LabelledStorage(_hstorage->_state_set[_loc],_hstorage->auxiliary_mapping(_loc)); }
Void HybridStorageElementReference::clear() { return _hstorage->state_set()[_loc].clear(); }

} // namespace Ariadne

#endif // ARIADNE_HYBRID_STORAGE_HPP
