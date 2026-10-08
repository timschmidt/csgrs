/***************************************************************************
 *            hybrid/hybrid_set.hpp
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

/*! \file hybrid/hybrid_set.hpp
 *  \brief Sets in hybrid spaces.
 */

#ifndef ARIADNE_HYBRID_SET_HPP
#define ARIADNE_HYBRID_SET_HPP

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
#include "geometry/point.hpp"
#include "geometry/box.hpp"

#include "hybrid/hybrid_graphics_interface.hpp"

namespace Ariadne {


//! \ingroup HybridSetSubModule
//! \brief A hybrid set defined in a single location obtained from a Euclidean set by naming variables.
template<class EBS>
class HybridBasicSet
    : public Pair<DiscreteLocation,LabelledSet<EBS>>
{
    typedef Pair<DiscreteLocation,LabelledSet<EBS>> Base;
  public:
    //! \brief The type of the Euclidean set used to describe the hybrid set.
    typedef EBS ContinuousSetType;

    HybridBasicSet() : Base(DiscreteLocation(),LabelledSet<EBS>(RealSpace(),EBS())) { }
    //! \brief Construct a set in location \a loc, with variables ordered by \a spc, defined by Euclidean set \a ebs.
    HybridBasicSet(const DiscreteLocation& loc, const RealSpace& spc, const ContinuousSetType& ebs) : Base(loc,LabelledSet<EBS>(spc,ebs)) { }
    HybridBasicSet(const DiscreteLocation& loc, const LabelledSet<EBS>& exbs) : Base(loc,exbs) { }
    //! \brief The location the set is contained in.
    const DiscreteLocation& location() const { return this->Base::first; }
    //! \brief A continuous set in terms of named variables in the discrete location.
    const Set<RealVariable> variables() const { return this->Base::second.space().variables(); }
    //! \brief A continuous set in terms of named variables in the discrete location.
    const LabelledSet<EBS>& continuous_set() const { return this->Base::second; }
    //! \brief The ordering of variables used to define the set.
    const RealSpace& space() const { return this->Base::second.space(); }
    //! \brief The continuous Euclidean subset.
    const ContinuousSetType& euclidean_set() const { return this->Base::second.euclidean_set(); }
    ContinuousSetType& euclidean_set() { return this->Base::second.euclidean_set(); }

    //! \brief The dimension of the continuous Euclidean subset.
    DimensionType dimension() const { return this->euclidean_set().dimension(); }

    //! \brief Test if the set is inside a union of hybrid boxes, at most one for each location.
    template<class IVL> decltype(auto) inside(HybridBoxes<IVL> const& hbxs) const {
        return (hbxs.has_location(this->location()))
            ? this->euclidean_set().inside(hbxs.euclidean_set(this->location(),this->space())) : this->euclidean_set().is_empty(); }
    //! \brief Test if the set is inside a hybrid box. Returns true if the set is empty when the locations do not match.
    template<class IVL> decltype(auto) inside(HybridBox<IVL> const& hbx) const {
        return (this->location()==hbx.location()) ? this->euclidean_set().inside(hbx.euclidean_set(this->space())) : this->euclidean_set().is_empty(); }
    //! \brief Test if the set is separated from a hybrid box. i.e. the closures are disjoint.
    template<class IVL> decltype(auto) separated(HybridBox<IVL> const& hbx) const {
        return (this->location()!=hbx.location()) or this->euclidean_set().separated(hbx.euclidean_set(this->space())); }
    //! \brief Test if the set intersects the interior of a hybrid box.
    template<class IVL> decltype(auto) overlaps(HybridBox<IVL> const& hbx) const {
        return (this->location()==hbx.location()) and this->euclidean_set().overlaps(hbx.euclidean_set(this->space())); }
    //! \brief Test if interior of the set is a superset of a hybrid box. Returns true if the box is empty.
    template<class IVL> decltype(auto) covers(HybridBox<IVL> const& hbx) const {
        return (this->location()==hbx.location()) ? this->euclidean_set().covers(hbx.euclidean_set(this->space())) : hbx.continuous_set().is_empty(); }
    //! \brief Test if the set is empty.
    template<class IVL> decltype(auto) is_empty() const {
        return this->euclidean_set().is_empty(); }

    //! \brief A bounding box for the continuous set in the given location.
    HybridUpperBox bounding_box() const;
    //! \brief A singleton list of the bounding box.
    HybridUpperBoxes bounding_boxes() const;

    //! \brief Adjoin an outer approximation of the set to \a paving using a given \a fineness of subdividing the paving cells.
    Void adjoin_outer_approximation_to(HybridGridTreePaving& paving, Nat fineness) const;

    //! \brief Draw to a canvas. Only draws if the set of locations \a q is empty, or contains the set's actual location.
    Void draw(CanvasInterface& c, const Set<DiscreteLocation>& q, const Variables2d& v) const;

    //! \brief Write to an output stream.
    friend OutputStream& operator<<(OutputStream& os, const HybridBasicSet<EBS>& hbs) {
        return os << "Hybrid" << class_name<EBS>() << "( " << hbs.location() << ", " << hbs.space() << ", " << hbs.euclidean_set() << " )";
    }

};

// NOTE: Must be out-of-function for case that EBS has no EqualityType
template<class EBS> EqualsType<EBS> operator==(const HybridBasicSet<EBS>& hset1, const HybridBasicSet<EBS>& hset2) {
    if(hset1.location()==hset2.location()) {
        ARIADNE_ASSERT(hset1.space()==hset2.space());
        return hset1.continuous_set() == hset2.continuous_set();
    } else {
        return false;
    }
}

//! \ingroup HybridSetSubModule
//! \brief A hybrid set defined in a single location obtained from a Euclidean set by naming variables.
template<class EDS>
class HybridDenotableSet
    : public Map<DiscreteLocation,LabelledSet<EDS>>
{
    typedef Map<DiscreteLocation,LabelledSet<EDS>> Base;
  public:
    typedef EDS ContinuousSetType;

    //! \brief Set the continuous state set in location \a loc to \a vbx.
    Void insert(const DiscreteLocation& loc, const LabelledSet<EDS>& eset) {
        this->Map<DiscreteLocation,LabelledSet<EDS>>::insert(loc,eset); }
    //! \brief Set the continuous state set in location \a loc to box \a bx using \a spc to order the variables.
    Void insert(const DiscreteLocation& loc, const RealSpace& spc, const EDS& set) {
        this->insert(loc,LabelledSet<EDS>(spc,set)); }

    //! \brief The set of discrete locations in which the set is nontrivial.
    Set<DiscreteLocation> locations() const { return this->keys(); }
    //! \brief The ordering of variables used to define the Euclidean box in location \a loc.
    RealSpace const& space(const DiscreteLocation& loc) const { return this->operator[](loc).space(); }
    //! \brief The Euclidean box in location \a loc.
    EDS const& continuous_set(const DiscreteLocation& loc) const { return this->operator[](loc).continuous_set(); }

    //! \brief The subset of \f$\mathbb{R}^n\f$ obtained by restricting to location \a loc and ordering the variables as defined by \a spc.
    EDS euclidean_set(const DiscreteLocation& loc, const RealSpace& spc) const {
        return this->operator[](loc).euclidean_set(spc); }

    bool has_location(DiscreteLocation const& loc) const { return this->_esets.has_label(loc); }

    friend OutputStream& operator<<(OutputStream& os, const HybridDenotableSet<EDS>& hds) {
        return os << "Hybrid" << class_name<EDS>() << static_cast<Base const&>(hds);
    }
};

//! \ingroup HybridSetSubModule
//! \brief A point in a location of a hybrid space.
template<class X> class HybridPoint
    : public HybridBasicSet<Point<X>>
{
  public:
    HybridPoint() : HybridBasicSet<Point<X>>() { }
    HybridPoint(const DiscreteLocation& q, const RealSpace& spc, const Point<X>& pt) : HybridBasicSet<Point<X>>(q,spc,pt) { }
    HybridPoint(const DiscreteLocation& q, const Map<RealVariable,X>& val)
        : HybridBasicSet<Point<X>>(q,LabelledSet<Point<X>>(VariablesPoint<X>(val))) { }
    HybridPoint(const DiscreteLocation& q, const List<Assignment<RealVariable,X>>& val);
    HybridPoint(const DiscreteLocation& q, const InitializerList<Assignment<RealVariable,X>>& val);

    template<class XX> requires Convertible<XX,X> HybridPoint(HybridPoint<XX> hpt)
        : HybridPoint(hpt.location(),hpt.space(),Point<X>(hpt.euclidean_set())) { }
    template<class Y, class PR> requires Constructible<X,Y,PR> HybridPoint(HybridPoint<Y> hpt, PR pr)
        : HybridPoint(hpt.location(),hpt.space(),Point<X>(hpt.euclidean_set(),pr)) { }

    Point<X>& point() { return this->euclidean_set(); }
    const Point<X>& point() const { return this->euclidean_set(); }
    Map<RealVariable,X> values() const;
};


//! \ingroup HybridSetSubModule
//! \brief A box in a location of a hybrid space.
//! \details Primarily used as a basic set against which abstract set properties can be tested.
template<class IVL> class HybridBox
    : public HybridBasicSet<Box<IVL>>
    , public virtual HybridDrawableInterface
{
    typedef typename IVL::UpperBoundType UB;
  public:
    explicit HybridBox(const DiscreteLocation& loc, const VariablesBox<IVL>& bx)
        : HybridBox(loc,bx.operator LabelledSet<Box<IVL>>()) { }
    HybridBox(const DiscreteLocation& loc, const List<VariableInterval<UB>>& bnds)
        : HybridBox(loc,_make_box(bnds)) { }
    HybridBox(const DiscreteLocation& loc, const InitializerList<VariableInterval<UB>>& bnds)
        : HybridBox(loc,List<VariableInterval<UB>>(bnds)) { }
    HybridBox(const DiscreteLocation& loc, const RealSpace& spc, const Box<IVL>& bx)
        : HybridBasicSet<Box<IVL>>(loc,spc,bx) { }
    HybridBox(const DiscreteLocation& loc, const LabelledSet<Box<IVL>>& ebx)
        : HybridBasicSet<Box<IVL>>(loc,ebx.space(),ebx.euclidean_set()) { }

    Box<IVL> euclidean_set() const {
        return this->HybridBasicSet<Box<IVL>>::euclidean_set(); }

    //! \brief The subset of \f$\mathbb{R}^n\f$ obtained by restricting to location \a loc and ordering the variables as defined by \a spc.
    Box<IVL> euclidean_set(const RealSpace& spc) const {
        if(spc==this->space()) { return this->euclidean_set(); }
        else { return VariablesBox<IVL>(this->space(),this->euclidean_set() ).euclidean_set(spc); }
    }

    IVL operator[](const RealVariable& v) const { return VariablesBox<IVL>(this->space(),this->euclidean_set())[v]; }

    virtual Void draw(CanvasInterface& c, const Set<DiscreteLocation>& q, const Variables2d& v) const override;
  private:
    static LabelledSet<Box<IVL>> _make_box(List<VariableInterval<UB>> const& bnds) {
        RealSpace spc; Box<IVL> bx(bnds.size());
        for(SizeType i=0; i!=bnds.size(); ++i) {
            spc.append(bnds[i].variable());
            bx[i]=bnds[i].interval();
        }
        return LabelledSet<Box<IVL>>(spc,bx);
    }
};

//! \ingroup HybridSetSubModule
//! \brief A collection of boxes, one in each location of a hybrid space.
//! \details Primarily used to represent bounds for a compact hybrid set.
template<class IVL> class HybridBoxes
    : public virtual HybridDrawableInterface
    , public HybridDenotableSet<Box<IVL>>
{
    typedef HybridDenotableSet<Box<IVL>> Base;
  public:
    //! \brief Set the continuous state set in location \a loc to \a vbx.
    Void insert(const HybridBox<IVL>& hbx) {
        this->Base::insert(hbx.location(),LabelledSet<Box<IVL>>(hbx.space(),hbx.euclidean_set())); }
    //! \brief Set the continuous state set in location \a loc to \a vbx.
    Void insert(const DiscreteLocation& loc, const LabelledSet<Box<IVL>>& bx) {
        this->Base::insert(loc,bx); }
    //! \brief Set the continuous state set in location \a loc to box \a bx using \a spc to order the variables.
    Void insert(const DiscreteLocation& loc, const RealSpace& spc, const Box<IVL>& bx) {
        this->Base::insert(loc,LabelledSet<Box<IVL>>(spc,bx)); }

    bool has_location(DiscreteLocation const& loc) const { return this->Base::has_key(loc); }

    //! \brief The set of discrete locations in which the set is nontrivial.
    Set<DiscreteLocation> locations() const { return this->Base::keys(); }
    //! \brief The ordering of variables used to define the Euclidean box in location \a loc.
    RealSpace const& space(const DiscreteLocation& loc) const {
        return this->Base::operator[](loc).space(); }
    Box<IVL> const& euclidean_set(const DiscreteLocation& loc) const {
        return this->Base::operator[](loc).euclidean_set(); }

    //! \brief The subset of \f$\mathbb{R}^V\f$ obtained by restricting to location \a loc.
    LabelledSet<Box<IVL>> const& continuous_set(const DiscreteLocation& loc) const {
        return this->Base::operator[](loc); }
    //! \brief The box in Euclidean space \f$\mathbb{R}^n\f$ obtained by restricting to location \a loc and ordering the variables as defined by \a spc.
    Box<IVL> const euclidean_set(const DiscreteLocation& loc, const RealSpace& spc) const {
        return this->Base::operator[](loc).euclidean_set(spc); }

    virtual Void draw(CanvasInterface&, const Set<DiscreteLocation>&, const Variables2d&) const override;
};

//! \ingroup LabelledSetSubModule
//! \ingroup HybridSetSubModule
//! \brief A hybrid set defined by the intersection of a box and a constraint system in each location.
class HybridValidatedConstrainedImageSet
    : public virtual ValidatedHybridLocatedSetInterface
    , public virtual HybridDrawableInterface
    , public HybridBasicSet<ValidatedConstrainedImageSet>
{
    typedef HybridBasicSet<ValidatedConstrainedImageSet> Base;
  public:
    using HybridBasicSet<ValidatedConstrainedImageSet>::HybridBasicSet;

    virtual HybridValidatedConstrainedImageSet* clone() const override {
        return new HybridValidatedConstrainedImageSet(*this); }

    virtual Set<RealVariable> variables(DiscreteLocation loc) const override {
        ARIADNE_ASSERT(loc==this->location()); return this->Base::variables(); }
    virtual Set<DiscreteLocation> locations() const override {
        return {this->Base::location()}; }

    virtual ValidatedLowerKleenean overlaps(const HybridExactBoxType& hbx) const override {
        return this->Base::overlaps(hbx); }
    inline ValidatedLowerKleenean inside(const HybridExactBoxesType& hbxs) const override {
        return this->Base::inside(hbxs); }
    virtual ValidatedLowerKleenean separated(const HybridExactBoxType& hbx) const override {
        return this->Base::separated(hbx); }
    virtual HybridUpperBoxes bounding_box() const override;

    virtual OutputStream& _write(OutputStream& os) const override {
        return os << static_cast<const Base&>(*this); }
    virtual Void draw(CanvasInterface& cnvs, const Set<DiscreteLocation>& locs, const Variables2d& vars) const override {
        return this->Base::draw(cnvs,locs,vars); }

    friend OutputStream& operator<<(OutputStream& os, const HybridValidatedConstrainedImageSet& hset) { return os << static_cast<const Base&>(hset); }
  protected:
    virtual ValidatedConstrainedImageSet* _euclidean_set(DiscreteLocation loc, RealSpace spc) const override {
        ARIADNE_ASSERT(loc==this->location());
        ARIADNE_ASSERT(spc==this->space());
        return new ValidatedConstrainedImageSet(this->Base::euclidean_set()); }
};

template<class EBS> HybridUpperBox HybridBasicSet<EBS>::bounding_box() const {
    return HybridUpperBox(this->location(),this->space(),this->euclidean_set().bounding_box());
}

template<class EBS> HybridUpperBoxes HybridBasicSet<EBS>::bounding_boxes() const {
    HybridUpperBoxes res; res.insert(this->location(),this->space(),this->euclidean_set().bounding_box()); return res;
}


template<class ES>
class HybridListSetConstIterator
    : public IteratorFacade<
                HybridListSetConstIterator<ES>,
                HybridBasicSet<ES>,
                ForwardTraversalTag,
                HybridBasicSet<ES> const&
             >

{
  public:
    typedef HybridBasicSet<ES> const& Reference;
  public:
    HybridListSetConstIterator(const Map<DiscreteLocation,Pair<RealSpace,ListSet<ES> > >& map, Bool);
    Bool equal(const HybridListSetConstIterator<ES>&) const;
    const HybridBasicSet<ES>& dereference() const;
    Void increment();
  private:
    Void _increment_loc();
  private:
    typedef typename Map< DiscreteLocation,Pair<RealSpace,ListSet<ES>>>::ConstIterator LocationsIterator;
    typedef typename ListSet<ES>::ConstIterator BasicSetIterator;
    LocationsIterator _loc_begin;
    LocationsIterator _loc_end;
    LocationsIterator _loc_iter;
    BasicSetIterator _bs_iter;
    mutable HybridBasicSet<ES> hybrid_set;
};


//! \ingroup HybridSetSubModule
//! A set comprising a %ListSet in each location.
template<class ES>
class HybridListSet
{
    Map<DiscreteLocation, Pair< RealSpace, ListSet<ES> > > _locations;
  public:
    typedef typename Map<DiscreteLocation,Pair< RealSpace, ListSet<ES> > >::ConstIterator LocationsConstIterator;
    typedef typename Map<DiscreteLocation,Pair< RealSpace, ListSet<ES> > >::Iterator LocationsIterator;
    typedef HybridListSetConstIterator<ES> ConstIterator;

    HybridListSet() { }
    HybridListSet(const HybridBasicSet<ES>& hes) { this->adjoin(hes); }

    ConstIterator begin() const {
        return ConstIterator(this->_locations,false); }
    ConstIterator end() const {
        return ConstIterator(this->_locations,true); }

    LocationsConstIterator locations_begin() const {
        return this->_locations.begin(); }
    LocationsConstIterator locations_end() const {
        return this->_locations.end(); }

    //! \brief Returns the number of basic hybrid sets forming this object.
    SizeType size() const {
        SizeType s = 0;
        for(LocationsConstIterator _loc_iter=this->locations_begin();
            _loc_iter!=this->locations_end(); ++_loc_iter) {
            s += _loc_iter->second.second.size();
        }
        return s;
    }

    const ListSet<ES>& operator[](const DiscreteLocation& q) const {
        ARIADNE_ASSERT_MSG(this->_locations.find(q)!=this->locations_end(),(*this)<<" has no location "<<q);
        return this->_locations.find(q)->second->second; }

    Void insert(const DiscreteLocation& loc, const RealSpace& spc) {
        this->_locations.insert(loc, make_pair(spc,ListSet<ES>())); }
    Void adjoin(const DiscreteLocation& loc, const RealSpace& spc, const ES& es) {
        LocationsIterator loc_iter=this->_locations.find(loc);
        if(loc_iter!=this->_locations.end()) {
            ARIADNE_ASSERT_MSG(loc_iter->second.first==spc,
                               "Space "<<loc_iter->second.first<<" of location "<<loc_iter->first<<" differs from "<<spc); }
        else { this->insert(loc,spc); loc_iter=this->_locations.find(loc); }
        loc_iter->second.second.adjoin(es); }
    Void adjoin(const DiscreteLocation& loc, const ES& es) {
        ARIADNE_ASSERT_MSG(this->_locations.find(loc)!=this->_locations.end(),(*this)<<" has no location "<<loc);
        this->_locations[loc].second.adjoin(es); }
    Void adjoin(const HybridBasicSet<ES>& hes) {
        this->adjoin(hes.location(),hes.space(),hes.euclidean_set()); }
    Void adjoin(const HybridListSet<ES>& hls) {
        for(LocationsConstIterator _loc_iter=hls.locations_begin();
            _loc_iter!=hls.locations_end(); ++_loc_iter) {
            (*this)[_loc_iter->first].adjoin(_loc_iter->second); } }

    HybridUpperBoxes bounding_boxes() const {
        HybridUpperBoxes result;
        for(LocationsConstIterator _loc_iter=this->locations_begin();
            _loc_iter!=this->locations_end(); ++_loc_iter) {
            result[_loc_iter->first]=_loc_iter->second.bounding_boxes(); }
        return result; }

    friend OutputStream& operator<<(OutputStream& os, const HybridListSet<ES>& hls) {
        return os << "HybridListSet" << hls._locations; }
};

template<class ES>
class ListSet<HybridBasicSet<ES>>
    : public HybridListSet<ES>
{
  public:
    using HybridListSet<ES>::HybridListSet;
};

template<class ES> inline
HybridListSetConstIterator<ES>::
HybridListSetConstIterator(const Map<DiscreteLocation,Pair<RealSpace,ListSet<ES>>>& map, Bool end)
    : _loc_begin(map.begin()),
      _loc_end(map.end()),
      _loc_iter(end?_loc_end:_loc_begin)
{
    if(_loc_iter!=_loc_end) {
        _bs_iter=_loc_iter->second.second.begin();
        this->_increment_loc();
    }
}


template<class ES> inline
Bool
HybridListSetConstIterator<ES>::equal(const HybridListSetConstIterator<ES>& other) const
{
    return this->_loc_iter==other._loc_iter && (this->_loc_iter==this->_loc_end || this->_bs_iter==other._bs_iter);
}


template<class ES> inline
HybridBasicSet<ES> const&
HybridListSetConstIterator<ES>::dereference() const
{
    this->hybrid_set=HybridBasicSet<ES>(_loc_iter->first,_loc_iter->second.first,*this->_bs_iter);
    return this->hybrid_set;
}


template<class ES> inline
Void
HybridListSetConstIterator<ES>::increment()
{
    ++this->_bs_iter;
    this->_increment_loc();
}

template<class ES> inline
Void
HybridListSetConstIterator<ES>::_increment_loc()
{
    while(_bs_iter==_loc_iter->second.second.end()) {
        ++_loc_iter;
        if(_loc_iter==_loc_end) { return; }
        _bs_iter=_loc_iter->second.second.begin();
    }
}





//! \ingroup HybridSetModule
//! \brief A cell associated with a grid in a hybrid space.
class HybridGridCell
    : public HybridBasicSet<GridCell>
{
  public:
    HybridGridCell() : HybridBasicSet<GridCell>() { }
    HybridGridCell(DiscreteLocation q, const RealSpace& s, const GridCell& gc) : HybridBasicSet<GridCell>(q,s,gc) { }
    HybridExactBox box() const { return HybridExactBox(this->location(),this->space(),this->euclidean_set().box()); }
};



template<class DS, class HBS>
class HybridSpaceSetConstIterator
    : public IteratorFacade<HybridSpaceSetConstIterator<DS,HBS>,
                            const HBS,
                            ForwardTraversalTag,
                            HBS const&
                           >

{
  public:
    typedef HBS const& Reference;
  public:
    HybridSpaceSetConstIterator(const Map<DiscreteLocation,DS>&, const HybridSpace& hspc, Bool);
    Bool equal(const HybridSpaceSetConstIterator<DS,HBS>&) const;
    const HBS& dereference() const;
    Void increment();
  private:
    Void increment_loc();
  private:
    typename Map<DiscreteLocation,DS>::ConstIterator _loc_begin;
    typename Map<DiscreteLocation,DS>::ConstIterator _loc_end;
    typename Map<DiscreteLocation,DS>::ConstIterator _loc_iter;
    typename DS::ConstIterator _bs_iter;
    HybridSpace hspc;
    mutable HBS hybrid_set;
};


template<class DS, class HBS> inline
Bool
HybridSpaceSetConstIterator<DS,HBS>::equal(const HybridSpaceSetConstIterator<DS,HBS>& other) const
{
    return this->_loc_iter==other._loc_iter && (this->_loc_iter==this->_loc_end || this->_bs_iter==other._bs_iter);
}


template<class DS, class HBS> inline
HBS const&
HybridSpaceSetConstIterator<DS,HBS>::dereference() const
{
    this->hybrid_set=HBS(_loc_iter->first,this->hspc[_loc_iter->first],*this->_bs_iter);
    return this->hybrid_set;
}


template<class DS, class HBS> inline
Void
HybridSpaceSetConstIterator<DS,HBS>::increment()
{
    ++this->_bs_iter;
    this->increment_loc();
}


} // namespace Ariadne

#endif // ARIADNE_HYBRID_SET_HPP
