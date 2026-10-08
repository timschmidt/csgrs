/***************************************************************************
 *            geometry/paving_interface.hpp
 *
 *  Copyright  2011-20  Pieter Collins
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

/*! \file geometry/paving_interface.hpp
 *  \brief Storage sets based on pavings and covers
 */

#ifndef ARIADNE_PAVING_INTERFACE_HPP
#define ARIADNE_PAVING_INTERFACE_HPP

#include <iosfwd>
#include "utility/declarations.hpp"
#include "geometry/grid_cell.hpp"
#include "geometry/set_interface.hpp"
#include "io/graphics_interface.hpp"

namespace Ariadne {


class Grid;
class GridCell;

#define ARIADNE_ABSTRACT_METHOD { throw std::runtime_error(StringType("ERROR: Unimplemented abstract method ")+__PRETTY_FUNCTION__); }

// A continuous predicate taking values in a three-valued logic.
// NOTE: Corresponds to your SetCheckerInterface; I think that this name is better.
template<class BS> class PredicateInterface {
  public:
    virtual ValidatedKleenean check(const BS&) = 0;
};

template<class T> class ForwardConstantIteratorHandle;

template<class T> class ForwardConstantIteratorInterface
{
    friend class ForwardConstantIteratorHandle<T>;
  public:
    virtual ~ForwardConstantIteratorInterface() = default;
  private:
    virtual ForwardConstantIteratorInterface<T>* clone() const = 0;
    virtual Void increment() = 0;
    virtual const T& dereference() const = 0;
    virtual Bool equals(const ForwardConstantIteratorInterface<T>&) const = 0;
    virtual OutputStream& _write(OutputStream&) const = 0;
    friend OutputStream& operator<<(OutputStream& os, const ForwardConstantIteratorInterface<T>& self) { self._write(os); return os; }
};

//! \brief A generic forward Iterator through constant data.
//! \details Since the Iterator is through constant data, the data may be safely be returned by value rather than by reference,
//! though this is not strictly standards-conforming, and means that operator->() cannot be provided.
template<class T> class ForwardConstantIteratorHandle
{
    ForwardConstantIteratorInterface<T>* _ptr;
  public:
    ~ForwardConstantIteratorHandle() { delete _ptr; }
    ForwardConstantIteratorHandle(ForwardConstantIteratorInterface<T>* p) : _ptr(p) { }
    ForwardConstantIteratorHandle(const ForwardConstantIteratorInterface<T>& p) : _ptr(p.clone()) { }
    ForwardConstantIteratorHandle(const ForwardConstantIteratorHandle<T>& other) : _ptr(other._ptr->clone()) { }
    ForwardConstantIteratorHandle<T>& operator=(const ForwardConstantIteratorHandle<T>& other) {
        if(_ptr!=other._ptr) { delete _ptr; _ptr=other._ptr->clone(); } return *this; }

    const T& operator*() const { return _ptr->dereference(); }
    const T* operator->() const { return &_ptr->dereference(); }
    ForwardConstantIteratorHandle<T>& operator++() { _ptr->increment(); return *this; }
    ForwardConstantIteratorHandle<T> operator++(Int) { ForwardConstantIteratorHandle<T> tmp(*this); this->_ptr->increment(); return tmp; }
    Bool operator==(const ForwardConstantIteratorHandle<T>& other) const { return this->_ptr->equals(*other._ptr); }
    Bool operator!=(const ForwardConstantIteratorHandle<T>& other) const { return !this->_ptr->equals(*other._ptr); }
    friend OutputStream& operator<<(OutputStream& os, const ForwardConstantIteratorHandle<T>& self) { return os << *self._ptr; }
    //friend OutputStream& operator<<(OutputStream& os, const ForwardConstantIteratorHandle<T>& self) { self._ptr->_write(os); return os; }
};


//! \brief A prototype interface for sets
//! \details Since the Iterator is through constant data, the data may be safely be returned by value rather than by reference,
//! though this is not strictly standards-conforming, and means that operator->() cannot be provided.
template<class BS> class DenotableSetInterface
{
  public:
    inline ForwardConstantIteratorHandle<BS> begin() const;
    inline ForwardConstantIteratorHandle<BS> end() const;
    virtual Void adjoin(const BS&) = 0;
    virtual Void adjoin(const DenotableSetInterface<BS>&) = 0;
  protected:
    virtual ForwardConstantIteratorInterface<BS>* _begin() const = 0;
    virtual ForwardConstantIteratorInterface<BS>* _end() const = 0;
};

class PavingHandle;
class SubPavingHandle;

class PavingInterface;
class SubPavingInterface;

//! \ingroup DiscretisedSetSubModule
//! \brief A prototype interface for subpavings.
//! \details A sub-paving is a paving-type set which is the restriction of another paving to a cell, called the \em root_cell of the subpaving.
//! In general, a sub-paving supports the non-modifying operations of a paving.
//!
//! The only modifying operations on a sub-paving which are logically allowable are those which apply to sub-cells of the root cell.
//! These operations modify the paving itself.
//!
//! \internal
//! NOTE: One difficulty with sub-pavings is that returning a sub-paving requires using either a raw pointer to SubPavingInterface with corresponding memory issues,
//! a shared pointer, which requires using pointer syntax, or a handle, which requires forwarding functions but is cleanest for users.
//! NOTE: Since operations modifying a sub-paving modify the original paving, we need to be careful with constantness, in particular when making copies.
//! This may be an issue in the branch method or when implementing Handle classes.
class SubPavingInterface
    : public virtual LocatedSetInterface<ValidatedTag,RealVector>
    , public virtual Drawable2dInterface
{
    friend class SubPavingHandle;
    friend class PavingHandle;
  public:
    typedef ForwardConstantIteratorHandle<GridCell> ConstIterator;
  public:
    //! \brief A constant Iterator through the cells of the paving.
    inline ForwardConstantIteratorHandle<GridCell> begin() const { return this->_begin(); }
    inline ForwardConstantIteratorHandle<GridCell> end() const { return this->_end(); }
    //! \brief A sub-paving obtained by considering one of the immediate subcells of the root cell.
    inline SubPavingHandle branch(Bool left_or_right) const;
  public:
    //! \brief A dynamically-allocated copy.
    virtual SubPavingInterface* clone() const = 0;

    //! \brief The dimension of the paving.
    virtual DimensionType dimension() const = 0;

    //! \brief The underlying grid of the paving.
    virtual const Grid& grid() const = 0;

    //! \brief The number of activated cells in the set.
    virtual SizeType size() const = 0;

    //! \brief The root cell of the (sub)paving. All enabled cells are subcells of the root cell.
    virtual GridCell root_cell() const = 0;
    //! \brief Tests if the cell \a c is contained in one of the cells of the subpaving.
    virtual Bool superset(const GridCell& c) const = 0; // Could also be called "contains"

    //! \brief Tests if the paving equals another paving on the same Grid.
    virtual Bool equals(const SubPavingInterface&) const = 0;
    //! \brief Tests if the paving is a subset of another paving on the same Grid.
    virtual Bool subset(const SubPavingInterface&) const = 0;
    //! \brief Tests if the paving intersects another paving on the same Grid.
    //! Tests intersection of interiors, so returns false if the pavings touch but do not overlap.
    virtual Bool intersects(const SubPavingInterface&) const = 0;

    //! \brief Modify the underlying paving so that the root cell of the subpaving is either enabled or disabled.
    virtual Void set_root_cell(Bool onoff) = 0;

    virtual UpperBoxType bounding_box() const = 0; // Inherited from CompactSetInterface
    virtual ValidatedLowerKleenean inside(const ExactBoxType& bx) const = 0; // Inherited from CompactSetInterface
    virtual ValidatedLowerKleenean separated(const ExactBoxType& bx) const = 0; // Inherited from ClosedSetInterface
    virtual ValidatedLowerKleenean overlaps(const ExactBoxType& bx) const = 0; // Inherited from OvertSetInterface

    virtual Void mince(Nat fineness) = 0; // Deprecated?
    virtual Void recombine() = 0; // Deprecated?
  protected:
    virtual SubPavingInterface* _branch(Bool left_or_right) const = 0; // Could also be called "child" or "subpaving"
    virtual ForwardConstantIteratorInterface<GridCell>* _begin() const = 0;
    virtual ForwardConstantIteratorInterface<GridCell>* _end() const = 0;
  private:
    virtual OutputStream& _write(OutputStream&) const = 0;
};

inline Bool operator==(const SubPavingInterface& p1, const SubPavingInterface& p2) { return p1.equals(p2); }
inline Bool subset(const SubPavingInterface& p1, const SubPavingInterface& p2) { return p1.subset(p2); }
inline Bool intersect(const SubPavingInterface& p1, const SubPavingInterface& p2) { return p1.intersects(p2); }

//! \ingroup DiscretisedSetSubModule
//! \brief A generic forward Iterator through constant data.
//! \details Since the Iterator is through constant data, the data may be safely be returned by value rather than by reference,
//! though this is not strictly standards-conforming, and means that operator->() cannot be provided.
class SubPavingHandle
{
    SubPavingInterface* _ptr;
  public:
    typedef ForwardConstantIteratorHandle<GridCell> ConstIterator;
  public:
    ~SubPavingHandle() { delete _ptr; }
    SubPavingHandle(SubPavingInterface* p) : _ptr(p) { }
    SubPavingHandle(const SubPavingInterface& p) : _ptr(p.clone()) { }
    SubPavingHandle(const SubPavingHandle& other) : _ptr(other._ptr->clone()) { }
    SubPavingHandle& operator=(const SubPavingHandle& other) {
        if(_ptr!=other._ptr) { delete _ptr; _ptr=other._ptr->clone(); } return *this; }
    operator SubPavingInterface& () { return *this->_ptr; }
    operator const SubPavingInterface& () const { return *this->_ptr; }

    const Grid& grid() const { return this->_ptr->grid(); }
    SizeType size() const { return this->_ptr->size(); }
    GridCell root_cell() const { return this->_ptr->root_cell(); };
    Bool superset(const GridCell& c) const { return this->_ptr->superset(c); }
    Bool subset(const SubPavingInterface& p) const { return this->_ptr->subset(p); }
    Bool intersects(const SubPavingInterface& p) const { return this->_ptr->intersects(p); }
    Void set_root_cell(Bool onoff) { return this->_ptr->set_root_cell(onoff); }

    ForwardConstantIteratorHandle<GridCell> begin() const { return this->_ptr->_begin(); }
    ForwardConstantIteratorHandle<GridCell> end() const { return this->_ptr->_end(); }

    UpperBoxType bounding_box() const { return this->_ptr->bounding_box(); };
    ValidatedLowerKleenean inside(const ExactBoxType& bx) const { return this->_ptr->inside(bx); }
    ValidatedLowerKleenean separated(const ExactBoxType& bx) const { return this->_ptr->separated(bx); }
    ValidatedLowerKleenean overlaps(const ExactBoxType& bx) const { return this->_ptr->overlaps(bx); }

    Void mince(Nat fineness) { this->_ptr->mince(fineness); }
    Void recombine() { this->_ptr->recombine(); }
    friend OutputStream& operator<<(OutputStream& os, const SubPavingHandle& self) { return os << *self._ptr; }
};

inline SubPavingHandle SubPavingInterface::branch(Bool left_or_right) const { return this->_branch(left_or_right); }



//! \ingroup DiscretisedSetSubModule
//! \brief A paving, which is a set constructed as a finite union of non-overlapping cells, where the possible cells are obtained by successive subdivision.
class PavingInterface
    : public virtual SubPavingInterface
{
  public:
    virtual PavingInterface* clone() const = 0;
    virtual GridCell smallest_enclosing_primary_cell(const UpperBoxType& bx) const = 0; // Useful query, but can also be implemented at the Grid level.
    virtual Void adjoin_cells(const PredicateInterface<ExactBoxType>& predicate, const Nat fineness) { ARIADNE_ABSTRACT_METHOD; }
    virtual Void adjoin_outer_approximation(const EffectiveEuclideanCompactSetInterface& set, const Nat fineness) = 0;
    virtual Void adjoin_outer_approximation(const UpperBoxType& set, const Nat fineness) = 0;
    virtual Void adjoin_inner_approximation(const EffectiveEuclideanOpenSetInterface& set, const Nat extent, const Nat fineness) = 0;
    virtual Void adjoin_inner_approximation(const EffectiveEuclideanSetInterface& set, const Nat fineness) = 0;
    virtual Void adjoin(const SubPavingInterface& paving) = 0;
    virtual Void restrict(const SubPavingInterface& paving) = 0;
    virtual Void remove(const SubPavingInterface& paving) = 0;
    virtual Void adjoin(const GridCell& cell) = 0; // Deprecated?
    virtual Void remove(const GridCell& cell) = 0; // Deprecated?
};

//! \ingroup DiscretisedSetSubModule
class PavingHandle
{
    PavingInterface* _ptr;
  public:
    typedef ForwardConstantIteratorHandle<GridCell> ConstIterator;
  public:
    ~PavingHandle() { delete _ptr; }
    PavingHandle(PavingInterface* p) : _ptr(p) { }
    PavingHandle(const SubPavingInterface& p) : _ptr(&dynamic_cast<PavingInterface&>(*p.clone())) { }
    PavingHandle(const PavingHandle& other) : _ptr(other._ptr->clone()) { }
    PavingHandle& operator=(const PavingHandle& other) {
        if(_ptr!=other._ptr) { delete _ptr; _ptr=other._ptr->clone(); } return *this; }
    operator PavingInterface& () { return *this->_ptr; }
    operator const PavingInterface& () const { return *this->_ptr; }

    const Grid& grid() const { return this->_ptr->grid(); }
    SizeType size() const { return this->_ptr->size(); }
    GridCell root_cell() const { return this->_ptr->root_cell(); }
    Bool superset(const GridCell& c) const { return this->_ptr->superset(c); }
    Bool subset(const SubPavingInterface& p) const { return this->_ptr->subset(p); }
    Bool intersects(const SubPavingInterface& p) const { return this->_ptr->intersects(p); }
    Void set_root_cell(Bool onoff) { return this->_ptr->set_root_cell(onoff); }

    ForwardConstantIteratorHandle<GridCell> begin() const { return this->_ptr->_begin(); }
    ForwardConstantIteratorHandle<GridCell> end() const { return this->_ptr->_end(); }
    UpperBoxType bounding_box() const { return this->_ptr->bounding_box(); };
    ValidatedLowerKleenean inside(const ExactBoxType& bx) const { return this->_ptr->inside(bx); }
    ValidatedLowerKleenean separated(const ExactBoxType& bx) const { return this->_ptr->separated(bx); }
    ValidatedLowerKleenean overlaps(const ExactBoxType& bx) const { return this->_ptr->overlaps(bx); }

    //GridCell smallest_enclosing_primary_cell(const ExactBoxType& bx) const { return this->_ptr->smallest_enclosing_primary_cell(); }
    Void adjoin_cells(const PredicateInterface<ExactBoxType>& predicate, const Nat fineness) { return this->_ptr->adjoin_cells(predicate,fineness); }
    Void adjoin_outer_approximation(const EffectiveEuclideanCompactSetInterface& set, const Nat fineness) { return this->_ptr->adjoin_outer_approximation(set,fineness); }
    Void adjoin_inner_approximation(const EffectiveEuclideanOpenSetInterface& set, const Nat extent, const Nat fineness) { return this->_ptr->adjoin_inner_approximation(set,extent,fineness); }
    Void adjoin_inner_approximation(const EffectiveEuclideanSetInterface& set, const Nat fineness) { return this->_ptr->adjoin_inner_approximation(set,fineness); }
    Void adjoin(const SubPavingInterface& paving) { return this->_ptr->adjoin(paving); }
    Void restrict(const SubPavingInterface& paving) { return this->_ptr->restrict(paving); }
    Void remove(const SubPavingInterface& paving) { return this->_ptr->remove(paving); }
    Void adjoin(const GridCell& cell) { return this->_ptr->adjoin(cell); }
    Void remove(const GridCell& cell) { return this->_ptr->remove(cell); }

    friend OutputStream& operator<<(OutputStream& os, const PavingHandle& self) { return os << *self._ptr; }
};

inline PavingHandle join(const SubPavingInterface& p1, const SubPavingInterface& p2) {
    PavingHandle r(p1); r.adjoin(p2); return r; }
inline PavingHandle geometric_union(const SubPavingInterface& p1, const SubPavingInterface& p2) {
    PavingHandle r(p1); r.adjoin(p2); return r; }
inline PavingHandle intersection(const SubPavingInterface& p1, const SubPavingInterface& p2) {
    PavingHandle r(p1); r.restrict(p2); return r; }
inline PavingHandle difference(const SubPavingInterface& p1, const SubPavingInterface& p2) {
    PavingHandle r(p1); r.remove(p2); return r; }



} // namespace Ariadne


#endif // ARIADNE_PAVING_INTERFACE_HPP
