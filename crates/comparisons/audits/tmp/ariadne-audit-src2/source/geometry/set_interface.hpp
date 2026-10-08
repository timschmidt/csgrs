/***************************************************************************
 *            geometry/set_interface.hpp
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

/*! \file geometry/set_interface.hpp
 *  \brief Interface<T>s for open, closed, overt and compact subsets of Euclidean space.
 */

#ifndef ARIADNE_SET_INTERFACE_HPP
#define ARIADNE_SET_INTERFACE_HPP

#include <iosfwd>

#include "utility/declarations.hpp"
#include "utility/tribool.hpp"
#include "utility/writable.hpp"
#include "numeric/numeric.hpp"
#include "function/function_traits.hpp"

#include "set.decl.hpp"
#include "box.decl.hpp"

namespace Ariadne {

template<class X> class Vector;

using DimensionOne = SizeOne;

template<class S> struct ElementTraits;
template<class S> using ElementKind = typename ElementTraits<S>::Kind;
template<class S, class X> using ElementType = typename ValueTraits<ElementKind<S>>::template Type<X>;


template<class T> struct SetTraits;
using EuclideanSetTraits = SetTraits<RealVector>;

template<class T> using DimensionOfType = typename SetTraits<T>::DimensionType;
template<class T> using BasicSetType = typename SetTraits<T>::BasicSetType;
template<class T> using BoundingSetType = typename SetTraits<T>::BoundingSetType;

template<> struct SetTraits<RealScalar> : public ScalarTraits {
    typedef DimensionOne DimensionType;
    typedef FloatDPExactInterval BasicSetType;
    typedef FloatDPUpperInterval BoundingSetType;
};
template<> struct SetTraits<RealVector> : public VectorTraits {
    typedef Ariadne::DimensionType DimensionType;
    typedef FloatDPExactBox BasicSetType;
    typedef FloatDPUpperBox BoundingSetType;
};

template<class UB> class Interval;
typedef FloatDPExactInterval ExactIntervalType;

template<class IVL> class Box;
typedef FloatDPExactBox ExactBoxType;
typedef FloatDPUpperBox UpperBoxType;

struct Overlap {
    constexpr const char* code() const { return "overlap"; }
    template<class S> decltype(auto) operator()(S const& s, ExactBoxType const& bx, Effort eff) const { return s.overlaps(bx,eff); }
    template<class S1, class S2> decltype(auto) operator()(S1 const& s1, S2 const& s2, Effort eff) const { return overlap(s1,s2,eff); }
    template<class S1, class S2> decltype(auto) operator()(S1 const& s1, S2 const& s2) const { return overlap(s1,s2); }
};

struct Separated {
    constexpr const char* code() const { return "separated"; }
    template<class S> decltype(auto) operator()(S const& s, ExactBoxType const& bx, Effort eff) const { return s.separated(bx,eff); }
    template<class S1, class S2> decltype(auto) operator()(S1 const& s1, S2 const& s2, Effort eff) const { return separated(s1,s2,eff); }
};

struct Inside {
    constexpr const char* code() const { return "inside"; }
    template<class S> decltype(auto) operator()(S const& s, ExactBoxType const& bx, Effort eff) const { return s.inside(bx,eff); }
    template<class S1, class S2> decltype(auto) operator()(S1 const& s1, S2 const& s2, Effort eff) const { return inside(s1,s2,eff); }
};

struct Covers {
    constexpr const char* code() const { return "covers"; }
    template<class S> decltype(auto) operator()(S const& s, ExactBoxType const& bx, Effort eff) const { return s.covers(bx,eff); }
    template<class S1, class S2> decltype(auto) operator()(S1 const& s1, S2 const& s2, Effort eff) const { return inside(s2,s1,eff); }
};

//! \brief Base class for sets described by predicates involving boxes.
template<class T> class SetInterfaceBase
    : public virtual WritableInterface
{
  public:
    //! \brief The type of element of the set.
    typedef T ElementType;
    //! \brief The type representing the dimension of the set.
    typedef typename SetTraits<T>::DimensionType DimensionType;
    //! \brief The type of basic set in the space.
    typedef typename SetTraits<T>::BasicSetType BasicSetType;
    //! \brief The type of basic set in the space.
    typedef typename SetTraits<T>::BoundingSetType BoundingSetType;

    //! \brief Virtual destructor.
    virtual ~SetInterfaceBase() = default;
    //! \brief Construct a dynamically-allocated copy.
    virtual SetInterfaceBase* clone() const = 0;
    //! \brief The dimension of the set.
    virtual DimensionType dimension() const = 0;
    //! \brief Write to an output stream.
    virtual OutputStream& _write(OutputStream& os) const = 0;
};


//! \ingroup GeometryModule SetInterfaceSubModule
//! \brief Interface for bounded sets.
template<class P, class T> class BoundedSetInterface
    : public virtual SetInterfaceBase<T>
{
  public:
    using typename SetInterfaceBase<T>::BasicSetType;
    using typename SetInterfaceBase<T>::BoundingSetType;

    virtual BoundedSetInterface<P,T>* clone() const = 0;
    //! \brief Tests if the set is a inside of \a bx.
    //! A set \a A is \em inside \a B if the closure of \a A is a subset of the interior of \a B.
    //! A set \f$A\f$ is \em inside \f$B\f$ if \f$\,\overline{\!A} \subset B^\circ\f$.
    virtual LowerKleeneanType<P> inside(const BasicSetType& bx) const = 0;
    //! \brief Returns a bounding box for the set.
    //! If the set is empty, then the first component of the result should be empty.
    virtual BoundingSetType bounding_box() const = 0;
};


//! \ingroup GeometryModule SetInterfaceSubModule
//! \brief Interface for overt sets, for which intersection with an open box is verifiable.
template<class P, class T> class OvertSetInterface
    : public virtual SetInterfaceBase<T>
{
  public:
    using typename SetInterfaceBase<T>::BasicSetType;

    virtual OvertSetInterface<P,T>* clone() const = 0;
    //! \brief Tests if the set overlaps \a bx.
    //! Sets \a A and \a B \em overlap if the interiors of \a A and \a B intersect.
    //! Sets \f$A\f$ and \f$B\f$ \em overlap if \f$A^\circ \cap B^\circ \neq \emptyset\f$.
    virtual LowerKleeneanType<P> overlaps(const BasicSetType& bx) const = 0;
    //! \brief Tests if \a ovs overlaps \a ops, to a tolerance of \a eps.
    friend LowerKleeneanType<P> overlap(const OvertSetInterface<P,T>& ovs, const OpenSetInterface<P,T>& ops);
    friend LowerKleeneanType<ValidatedTag> overlap(const OvertSetInterface<ValidatedTag,T>& ovs, const OpenSetInterface<ValidatedTag,T>& ops, const RawFloatDP& eps);
};

//! \ingroup GeometryModule SetInterfaceSubModule
//! \brief Interface for open sets.
template<class P, class T> class OpenSetInterface
    : public virtual OvertSetInterface<P,T>
{
  public:
    using typename SetInterfaceBase<T>::BasicSetType;

    virtual OpenSetInterface<P,T>* clone() const = 0;
    //! \brief Tests if the set covers of \a bx.
    //! A set \a A \em covers \a B if the interiors of \a A is a superset of the closure of \a B.
    //! A set \f$A\f$ \em covers \f$B\f$ if \f$A^\circ \supset \overline{B}\f$.
    virtual LowerKleeneanType<P> covers(const BasicSetType& bx) const = 0;
    //! \brief Tests if \a ovs overlaps \a ops, to a tolerance of \a eps.
    friend LowerKleeneanType<P> overlap(const OvertSetInterface<P,T>& ovs, const OpenSetInterface<P,T>& ops);
    friend LowerKleeneanType<ValidatedTag> overlap(const OvertSetInterface<ValidatedTag,T>& ovs, const OpenSetInterface<ValidatedTag,T>& ops, const RawFloatDP& eps);
    //! \brief Tests if \a ls is a inside of \a rs, to a tolerance of \a eps.
    friend LowerKleeneanType<P> inside(const CompactSetInterface<P,T>& ls, const OpenSetInterface<P,T>& rs);
    friend LowerKleeneanType<ValidatedTag> inside(const CompactSetInterface<ValidatedTag,T>& ls, const OpenSetInterface<ValidatedTag,T>& rs, const RawFloatDP& eps);
};

//! \ingroup GeometryModule SetInterfaceSubModule
//! \brief Interface for closed sets.
template<class P, class T> class ClosedSetInterface
    : public virtual SetInterfaceBase<T>
{
  public:
    using typename SetInterfaceBase<T>::BasicSetType;

    virtual ClosedSetInterface<P,T>* clone() const = 0;
    //! \brief Tests if the set is separated from \a bx.
    //! A set \a A is \em separated from \a B if the closures of \a A and \a B are disjoint.
    //! A set \f$A\f$ is \em separated from \f$B\f$ if \f$\,\overline{\!A} \cap \overline{B} = \emptyset\f$.
    virtual LowerKleeneanType<P> separated(const BasicSetType& bx) const = 0;
    //! \brief Tests if \a cps is disjoint from \a cls, to a tolerance of \a eps.
    friend LowerKleeneanType<P> separated(const CompactSetInterface<P,T>& cps, const ClosedSetInterface<P,T>& cls);
    friend LowerKleeneanType<ValidatedTag> separated(const CompactSetInterface<ValidatedTag,T>& cps, const ClosedSetInterface<ValidatedTag,T>& cls, const RawFloatDP& eps);
};

//! \ingroup GeometryModule SetInterfaceSubModule
//! \brief Interface for compact (closed and bounded) sets.
template<class P, class T> class CompactSetInterface
    : public virtual BoundedSetInterface<P,T>,
      public virtual ClosedSetInterface<P,T>
{
  public:
    using typename SetInterfaceBase<T>::BasicSetType;

    virtual CompactSetInterface<P,T>* clone() const = 0;
    //virtual ValidatedSierpinskian empty() const = 0;
    //! \brief Tests if \a ls is a inside of \a rs, to a tolerance of \a eps.
    friend LowerKleeneanType<P> inside(const CompactSetInterface<P,T>& ls, const OpenSetInterface<P,T>& rs);
    friend LowerKleeneanType<ValidatedTag> inside(const CompactSetInterface<ValidatedTag,T>& ls, const OpenSetInterface<ValidatedTag,T>& rs, const RawFloatDP& eps);
    //! \brief Tests if \a cps is disjoint from \a cls, to a tolerance of \a eps.
    friend LowerKleeneanType<P> separated(const CompactSetInterface<P,T>& cps, const ClosedSetInterface<P,T>& cls);
    friend LowerKleeneanType<ValidatedTag> separated(const CompactSetInterface<ValidatedTag,T>& cps, const ClosedSetInterface<ValidatedTag,T>& cls, const RawFloatDP& eps);

};

//! \ingroup GeometryModule SetInterfaceSubModule
//! \brief Interface for regular sets, whose closure is the closure of the interior, and whose interior is the interior of the closure.
template<class P, class T> class RegularSetInterface
    : public virtual OpenSetInterface<P,T>,
      public virtual ClosedSetInterface<P,T>
{
  public:
    using typename SetInterfaceBase<T>::BasicSetType;

    virtual RegularSetInterface<P,T>* clone() const = 0;
    //! \brief Tests if \a ls overlaps \a rs, to a tolerance of \a eps.
    friend KleeneanType<P> overlap(const LocatedSetInterface<P,T>& ls, const RegularSetInterface<P,T>& rs);
    friend KleeneanType<ValidatedTag> overlap(const LocatedSetInterface<ValidatedTag,T>& ls, const RegularSetInterface<ValidatedTag,T>& rs, const RawFloatDP& eps);
    //! \brief Tests if \a ls is a inside of \a rs, to a tolerance of \a eps.
    friend KleeneanType<P> inside(const LocatedSetInterface<P,T>& ls, const RegularSetInterface<P,T>& rs);
    friend KleeneanType<ValidatedTag> inside(const LocatedSetInterface<ValidatedTag,T>& ls, const RegularSetInterface<ValidatedTag,T>& rs, const RawFloatDP& eps);
    //! \brief Tests if \a ls is disjoint from \a rs, to a tolerance of \a eps.
    friend KleeneanType<P> separated(const LocatedSetInterface<P,T>& ls, const RegularSetInterface<P,T>& rs);
    friend KleeneanType<ValidatedTag> separated(const LocatedSetInterface<ValidatedTag,T>& ls, const RegularSetInterface<ValidatedTag,T>& rs, const RawFloatDP& eps);
};


//! \ingroup GeometryModule SetInterfaceSubModule
//! \brief Interface for located (overt and compact) sets.
template<class P, class T> class LocatedSetInterface
    : public virtual OvertSetInterface<P,T>,
      public virtual CompactSetInterface<P,T>
{
  public:
    using typename SetInterfaceBase<T>::BasicSetType;

    virtual LocatedSetInterface<P,T>* clone() const = 0;
    //! \brief Tests if \a ls overlaps \a rs, to a tolerance of \a eps.
    friend KleeneanType<P> overlap(const LocatedSetInterface<P,T>& ls, const RegularSetInterface<P,T>& rs);
    friend KleeneanType<ValidatedTag> overlap(const LocatedSetInterface<ValidatedTag,T>& ls, const RegularSetInterface<ValidatedTag,T>& rs, const RawFloatDP& eps);
    //! \brief Tests if \a ls is a inside of \a rs, to a tolerance of \a eps.
    friend KleeneanType<P> inside(const LocatedSetInterface<P,T>& ls, const RegularSetInterface<P,T>& rs);
    friend KleeneanType<ValidatedTag> inside(const LocatedSetInterface<ValidatedTag,T>& ls, const RegularSetInterface<ValidatedTag,T>& rs, const RawFloatDP& eps);
    //! \brief Tests if \a ls is disjoint from \a rs, to a tolerance of \a eps.
    friend KleeneanType<P> separated(const LocatedSetInterface<P,T>& ls, const RegularSetInterface<P,T>& rs);
    friend KleeneanType<ValidatedTag> separated(const LocatedSetInterface<ValidatedTag,T>& ls, const RegularSetInterface<ValidatedTag,T>& rs, const RawFloatDP& eps);
};

//! \ingroup GeometryModule SetInterfaceSubModule
//! \brief Complete set interface for bounded regular sets.
template<class P, class T> class RegularLocatedSetInterface
    : public virtual RegularSetInterface<P,T>,
      public virtual LocatedSetInterface<P,T>
{
  public:
    virtual RegularLocatedSetInterface<P,T>* clone() const = 0;
};



class GridTreePaving;

//! \brief A Euclidean space \f$\R^d\f$ of dimension \a d.
class EuclideanSpace
{
    using P=EffectiveTag; using T=RealVector;
  public:
    //! \brief The canonical type used for bounding sets in the space.
    typedef ExactBoxType BoundingDomainType;
    //! \brief The interface satisified by bounded sets in the space.
    typedef BoundedSetInterface<P,T> BoundedSetInterfaceType;
    //! \brief The interface satisified by overt sets in the space.
    typedef OvertSetInterface<P,T> OvertSetInterfaceType;
    //! \brief The interface satisified by over sets in the space.
    typedef OpenSetInterface<P,T> OpenSetInterfaceType;
    //! \brief The interface satisified by closed sets in the space.
    typedef ClosedSetInterface<P,T> ClosedSetInterfaceType;
    //! \brief The interface satisified by compact sets in the space.
    typedef CompactSetInterface<P,T> CompactSetInterfaceType;
    //! \brief The interface satisified by regular sets in the space.
    typedef RegularSetInterface<P,T> RegularSetInterfaceType;
    //! \brief The interface satisified by located sets in the space.
    typedef LocatedSetInterface<P,T> LocatedSetInterfaceType;
    //! \brief The interface satisified by bounded regular sets.
    typedef RegularLocatedSetInterface<P,T> RegularLocatedSetInterfaceType;
    //! \brief The type of approximations to sets in the space.
    typedef GridTreePaving SetApproximationType;
  public:
    EuclideanSpace(const SizeType& d) : _dimension(d) { }
    const SizeType& dimension() const { return this->_dimension; }
  private:
    SizeType _dimension;
};


} // namespace Ariadne


#endif // ARIADNE_SET_INTERFACE_HPP
