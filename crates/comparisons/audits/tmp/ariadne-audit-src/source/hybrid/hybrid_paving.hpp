/***************************************************************************
 *            hybrid/hybrid_paving.hpp
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

/*! \file hybrid/hybrid_paving.hpp
 *  \brief Hybrid extension of a paving.
 */

#ifndef ARIADNE_HYBRID_PAVING_HPP
#define ARIADNE_HYBRID_PAVING_HPP

#include <map>


#include <memory>

#include "utility/macros.hpp"
#include "helper/stlio.hpp"
#include "utility/declarations.hpp"
#include "utility/container.hpp"
#include "geometry/function_set.hpp"
#include "geometry/list_set.hpp"
#include "geometry/grid_paving.hpp"
#include "geometry/curve.hpp"

#include "symbolic/expression_set.hpp"

#include "hybrid/hybrid_set.decl.hpp"
#include "hybrid/hybrid_set_interface.hpp"
#include "hybrid/hybrid_expression_set.hpp"
#include "hybrid/hybrid_space.hpp"
#include "hybrid/hybrid_grid.hpp"
#include "hybrid/hybrid_set.hpp"
#include "geometry/point.hpp"
#include "geometry/box.hpp"

#include "hybrid/hybrid_graphics_interface.hpp"

namespace Ariadne {


//! \ingroup HybridSetSubModule
//! A set comprising a %GridTreeSet in each location.
class HybridGridTreePaving
    : public HybridDrawableInterface
{
  public:
    HybridGrid _hgrid;
    Map<DiscreteLocation,GridTreePaving> _map;
  public:
    typedef HybridGrid GridType;
    typedef Map<DiscreteLocation,GridTreePaving>::Iterator LocationsIterator;
    typedef Map<DiscreteLocation,GridTreePaving>::ConstIterator LocationsConstIterator;
    typedef HybridSpaceSetConstIterator<GridTreePaving,HybridGridCell> ConstIterator;
  public:
    //!
    LocationsIterator locations_begin() { return this->_map.begin(); }
    //!
    LocationsIterator locations_end() { return this->_map.end(); }
    //!
    LocationsConstIterator locations_begin() const { return this->_map.begin(); }
    //!
    LocationsConstIterator locations_end() const { return this->_map.end(); }
    //!
    ConstIterator begin() const;
    //!
    ConstIterator end() const;
  public:
    //! Construct from a hybrid grid.
    HybridGridTreePaving(const HybridGrid& hgrid) : _hgrid(hgrid), _map() { }

    //! The hybrid grid.
    HybridGrid grid() const { return this->_hgrid; }

    //! Test if \a q is a location of the set i.e. corresponds to a valid location of the underlying hybrid space.
    Bool has_location(DiscreteLocation q) const { return _hgrid.has_location(q); }
    //! Test if \a q is a nontrivial location of the set i.e. contained in the map of <DiscreteLocation,GridTreeSet> pairs.
    Bool nontrivial_location(DiscreteLocation q) const { return _map.has_key(q); }
    //! The continuous state space corresponding to location \a q.
    RealSpace space(DiscreteLocation q) const { return _hgrid.space(q); }
    //! The continuous state space corresponding to location \a q.
    const GridTreePaving& euclidean_set(DiscreteLocation q) const { return _map[q]; }
    //! The continuous state space corresponding to location \a q.
    const GridTreePaving& euclidean_set(DiscreteLocation q, const RealSpace& s) const {
        ARIADNE_ASSERT_MSG(s==this->space(q),"Variable ordering in HybridGridTreeSet location "<<q<<" is "<<this->space(q)<<", "
                                             "which does not match requested ordering "<<q);
        return _map[q]; }

    //!
//    Void insert(DiscreteLocation q, const GridTreeSet& gts) {
//        this->_map.insert(q,gts); }
//    Void insert(Pair<DiscreteLocation,GridTreeSet>& qgts) {
//        this->_map.insert(qgts); }

    //!
//    Void adjoin(DiscreteLocation q, const GridCell& c) {
//        this->_provide_location(q).adjoin(c); }

    Void clear();
    Void adjoin(const HybridGridCell& hgc);
    Void adjoin(const ListSet<HybridGridCell>& hgcls);
    Void adjoin(const HybridGridTreePaving& hgts);
    Void remove(const HybridGridTreePaving& hgts);
    Void restrict(const HybridGridTreePaving& hgts);
    Void restrict_to_extent(Nat extent);
    Void adjoin_inner_approximation(const HybridExactBoxes& hbxs, const Nat fineness);
    Void adjoin_inner_approximation(const EffectiveHybridSetInterface& hs, const Nat fineness);
    Void adjoin_lower_approximation(const EffectiveHybridOvertSetInterface& hs, const Nat extent, const Nat fineness);
    Void adjoin_outer_approximation(const EffectiveHybridCompactSetInterface& hs, const Nat fineness);
    Void adjoin_outer_approximation(const HybridExactBoxes& hbxs, const Nat fineness);
    template<class S> Void adjoin_outer_approximation(DiscreteLocation q, const S& s);

    GridTreePaving& operator[](DiscreteLocation q);
    const GridTreePaving& operator[](DiscreteLocation q) const ;
    Bool is_empty() const;
    SizeType size() const;
    HybridListSet<ExactBoxType> boxes() const;
    Void mince(Nat fineness);
    Void recombine();

    friend Bool subset(const HybridGridTreePaving& hgts1, const HybridGridTreePaving& hgts2);
    friend Bool intersect(const HybridGridTreePaving& hgts1, const HybridGridTreePaving& hgts2);
  public:
    //!@{ \name HybridSetInterface methods
    HybridGridTreePaving* clone() const { return new HybridGridTreePaving(*this); }
    HybridSpace space() const { return this->grid().space(); }
    ValidatedLowerKleenean separated(const HybridExactBox& hbx) const;
    ValidatedLowerKleenean overlaps(const HybridExactBox& hbx) const;
    ValidatedLowerKleenean covers(const HybridExactBox& hbx) const;
    ValidatedLowerKleenean inside(const HybridExactBoxes& hbx) const ;
    HybridUpperBoxes bounding_box() const;
    OutputStream& _write(OutputStream& os) const;
    Void draw(CanvasInterface& c, const Set<DiscreteLocation>& l, const Variables2d&v) const;
    //!@}
  public:
    friend OutputStream& operator<<(OutputStream& os, const HybridGridTreePaving& hgts) {
        return os << "HybridGridTreeSet(" << hgts._map << ")"; }
  private:
    GridTreePaving& _provide_location(const DiscreteLocation& q);
};

template<class S> Void HybridGridTreePaving::adjoin_outer_approximation(DiscreteLocation q, const S& s) {
    this->_provide_location(q).adjoin_outer_approximation(s); }

inline HybridGridTreePaving inner_approximation(const EffectiveHybridSetInterface& set, HybridGrid const& grid, const Nat fineness) {
    HybridGridTreePaving paving(grid); paving.adjoin_inner_approximation(set,fineness); return paving; }


class HybridAutomatonInterface;

HybridGridTreePaving extend_auxiliary(HybridGridTreePaving const& hgtp, HybridAutomatonInterface const& ha);


} // namespace Ariadne

#endif // ARIADNE_HYBRID_PAVING_HPP
