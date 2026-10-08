/***************************************************************************
 *            hybrid/hybrid_set_interface.hpp
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

/*! \file hybrid/hybrid_set_interface.hpp
 *  \brief Interfaces for open, closed, overt and compact subsets of hybrid spaces.
 */

#ifndef ARIADNE_HYBRID_SET_INTERFACE_HPP
#define ARIADNE_HYBRID_SET_INTERFACE_HPP

#include <iosfwd>
#include <utility>
#include <map>


#include "utility/tribool.hpp"
#include "geometry/set_interface.hpp"
#include "hybrid/discrete_location.hpp"
#include "symbolic/space.hpp"
#include "geometry/set.hpp"
#include "symbolic/expression_set.hpp"
#include "hybrid/hybrid_set.decl.hpp"

namespace Ariadne {

class HybridSetInterfaceBase;

template<class P> class HybridBoundedSetInterface;
template<class P> class HybridOpenSetInterface;
template<class P> class HybridClosedSetInterface;
template<class P> class HybridOvertSetInterface;
template<class P> class HybridCompactSetInterface;
template<class P> class HybridRegularSetInterface;
template<class P> class HybridLocatedSetInterface;
template<class P> class HybridRegularLocatedSetInterface;

using EffectiveHybridBoundedSetInterface = HybridBoundedSetInterface<EffectiveTag>;
using EffectiveHybridOpenSetInterface = HybridOpenSetInterface<EffectiveTag>;
using EffectiveHybridClosedSetInterface = HybridClosedSetInterface<EffectiveTag>;
using EffectiveHybridOvertSetInterface = HybridOvertSetInterface<EffectiveTag>;
using EffectiveHybridCompactSetInterface = HybridCompactSetInterface<EffectiveTag>;
using EffectiveHybridRegularSetInterface = HybridRegularSetInterface<EffectiveTag>;
using EffectiveHybridLocatedSetInterface = HybridLocatedSetInterface<EffectiveTag>;
using EffectiveHybridRegularLocatedSetInterface = HybridRegularLocatedSetInterface<EffectiveTag>;

using ValidatedHybridBoundedSetInterface = HybridBoundedSetInterface<ValidatedTag>;
using ValidatedHybridOpenSetInterface = HybridOpenSetInterface<ValidatedTag>;
using ValidatedHybridClosedSetInterface = HybridClosedSetInterface<ValidatedTag>;
using ValidatedHybridOvertSetInterface = HybridOvertSetInterface<ValidatedTag>;
using ValidatedHybridCompactSetInterface = HybridCompactSetInterface<ValidatedTag>;
using ValidatedHybridRegularSetInterface = HybridRegularSetInterface<ValidatedTag>;
using ValidatedHybridLocatedSetInterface = HybridLocatedSetInterface<ValidatedTag>;
using ValidatedHybridRegularLocatedSetInterface = HybridRegularLocatedSetInterface<ValidatedTag>;

template<class P> using HybridSetInterface = HybridRegularLocatedSetInterface<P>;
using EffectiveHybridSetInterface = HybridSetInterface<EffectiveTag>;
using ValidatedHybridSetInterface = HybridSetInterface<ValidatedTag>;

//! \ingroup HybridSetSubmodule
//! \ingroup EuclideanExpressionSet
//! \brief Base class for sets in a hybrid space.
class HybridSetInterfaceBase
{
  public:
    virtual ~HybridSetInterfaceBase() = default;
    virtual HybridSetInterfaceBase* clone() const = 0;
    virtual Set<RealVariable> variables(DiscreteLocation) const = 0;
    inline EuclideanSetBase euclidean_set(DiscreteLocation loc, RealSpace spc) const {
        return EuclideanSetBase(this->_euclidean_set(loc,spc)); }
    virtual OutputStream& _write(OutputStream& os) const = 0;
    friend OutputStream& operator<<(OutputStream& os, const HybridSetInterfaceBase& hs);
  protected:
    virtual EuclideanSetInterfaceBase* _euclidean_set(DiscreteLocation,RealSpace) const = 0;
};

//! \brief Interface for bounded sets in a hybrid space.
template<> class HybridBoundedSetInterface<EffectiveTag>
    : public virtual HybridSetInterfaceBase
{
  public:
    virtual HybridBoundedSetInterface<EffectiveTag>* clone() const = 0;
    virtual Set<DiscreteLocation> locations() const = 0;
    inline EffectiveEuclideanBoundedSet euclidean_set(DiscreteLocation loc, RealSpace spc) const {
        return EffectiveEuclideanBoundedSet(this->_euclidean_set(loc,spc)); }
    virtual LowerKleenean inside(const HybridExactBoxes& bx) const = 0;
    virtual HybridUpperBoxes bounding_box() const = 0;
  protected:
    virtual EuclideanBoundedSetInterface<EffectiveTag>* _euclidean_set(DiscreteLocation,RealSpace) const = 0;
};

//! \brief Interface for overt sets in a hybrid space.
template<> class HybridOvertSetInterface<EffectiveTag>
    : public virtual HybridSetInterfaceBase
{
  public:
    virtual HybridOvertSetInterface* clone() const = 0;
    inline EffectiveEuclideanOvertSet euclidean_set(DiscreteLocation loc, RealSpace spc) const {
        return EffectiveEuclideanOvertSet(this->_euclidean_set(loc,spc)); }
    virtual LowerKleenean overlaps(const HybridExactBox& bx) const = 0;
  protected:
    virtual EuclideanOvertSetInterface<EffectiveTag>* _euclidean_set(DiscreteLocation,RealSpace) const = 0;
};

//! \brief Interface for open sets in a hybrid space.
template<> class HybridOpenSetInterface<EffectiveTag>
    : public virtual HybridOvertSetInterface<EffectiveTag>
{
  public:
    virtual HybridOpenSetInterface* clone() const = 0;
    inline EffectiveEuclideanOpenSet euclidean_set(DiscreteLocation loc, RealSpace spc) const {
        return EffectiveEuclideanOpenSet(this->_euclidean_set(loc,spc)); }
    virtual LowerKleenean covers(const HybridExactBox& bx) const = 0;
  protected:
    virtual EuclideanOpenSetInterface<EffectiveTag>* _euclidean_set(DiscreteLocation,RealSpace) const = 0;
};

//! \brief Interface for closed sets in a hybrid space.
template<> class HybridClosedSetInterface<EffectiveTag>
    : public virtual HybridSetInterfaceBase
{
  public:
    virtual HybridClosedSetInterface* clone() const = 0;
    inline EffectiveEuclideanClosedSet euclidean_set(DiscreteLocation loc, RealSpace spc) const {
        return EffectiveEuclideanClosedSet(this->_euclidean_set(loc,spc)); }
    virtual LowerKleenean separated(const HybridExactBox& bx) const = 0;
  protected:
    virtual EuclideanClosedSetInterface<EffectiveTag>* _euclidean_set(DiscreteLocation,RealSpace) const = 0;
};

//! \brief Interface for compact (closed and bounded) sets in a hybrid space.
template<> class HybridCompactSetInterface<EffectiveTag>
    : public virtual HybridBoundedSetInterface<EffectiveTag>
    , public virtual HybridClosedSetInterface<EffectiveTag>
{
  public:
    virtual HybridCompactSetInterface* clone() const = 0;
    inline EffectiveEuclideanCompactSet euclidean_set(DiscreteLocation loc, RealSpace spc) const {
        return EffectiveEuclideanCompactSet(this->_euclidean_set(loc,spc)); }
  protected:
    virtual EuclideanCompactSetInterface<EffectiveTag>* _euclidean_set(DiscreteLocation,RealSpace) const = 0;
};

//! \brief Interface for regular (open and closed) sets in a hybrid space.
template<> class HybridRegularSetInterface<EffectiveTag>
    : public virtual HybridOpenSetInterface<EffectiveTag>
    , public virtual HybridClosedSetInterface<EffectiveTag>
{
  public:
    virtual HybridRegularSetInterface* clone() const = 0;
    inline EffectiveEuclideanRegularSet euclidean_set(DiscreteLocation loc, RealSpace spc) const {
        return EffectiveEuclideanRegularSet(this->_euclidean_set(loc,spc)); }
  protected:
    virtual EuclideanRegularSetInterface<EffectiveTag>* _euclidean_set(DiscreteLocation,RealSpace) const = 0;
};

//! \brief Interface for located (overt and compact) sets in a hybrid space.
template<> class HybridLocatedSetInterface<EffectiveTag>
    : public virtual HybridOvertSetInterface<EffectiveTag>
    , public virtual HybridCompactSetInterface<EffectiveTag>
{
  public:
    virtual HybridLocatedSetInterface* clone() const = 0;
    inline EffectiveEuclideanLocatedSet euclidean_set(DiscreteLocation loc, RealSpace spc) const {
        return EffectiveEuclideanLocatedSet(this->_euclidean_set(loc,spc)); }
  protected:
    virtual EuclideanLocatedSetInterface<EffectiveTag>* _euclidean_set(DiscreteLocation,RealSpace) const = 0;
};

//! \brief Complete set interface for bounded regular sets in a hybrid space.
template<> class HybridRegularLocatedSetInterface<EffectiveTag>
    : public virtual HybridRegularSetInterface<EffectiveTag>
    , public virtual HybridLocatedSetInterface<EffectiveTag>
{
  public:
    virtual HybridRegularLocatedSetInterface* clone() const = 0;
    inline EffectiveEuclideanRegularLocatedSet euclidean_set(DiscreteLocation loc, RealSpace spc) const {
        return EffectiveEuclideanRegularLocatedSet(this->_euclidean_set(loc,spc)); }
  protected:
    virtual EuclideanRegularLocatedSetInterface<EffectiveTag>* _euclidean_set(DiscreteLocation,RealSpace) const = 0;
};


inline OutputStream& operator<<(OutputStream& os, const HybridSetInterfaceBase& s) {
    return s._write(os);
}


//! \ingroup GeometryModule EuclideanSetInterfaceSubModule
//! \brief Interface for bounded sets.
template<> class HybridBoundedSetInterface<ValidatedTag>
    : public virtual HybridSetInterfaceBase {
  public:
    virtual HybridBoundedSetInterface<ValidatedTag>* clone() const = 0;
    virtual Set<DiscreteLocation> locations() const = 0;
//    inline ValidatedEuclideanBoundedSet euclidean_set(DiscreteLocation loc, RealSpace spc) const { return this->_euclidean_set(loc,spc); }
    virtual ValidatedLowerKleenean inside(const HybridExactBoxes& bx) const = 0;
    virtual HybridUpperBoxes bounding_box() const = 0;
  protected:
    virtual EuclideanBoundedSetInterface<ValidatedTag>* _euclidean_set(DiscreteLocation,RealSpace) const = 0;
};

//! \brief Interface for overt sets in a hybrid space.
template<> class HybridOvertSetInterface<ValidatedTag>
    : public virtual HybridSetInterfaceBase
{
  public:
    virtual HybridOvertSetInterface<ValidatedTag>* clone() const = 0;
//    inline ValidatedEuclideanOvertSet euclidean_set(DiscreteLocation loc, RealSpace spc) const { return this->_euclidean_set(loc,spc); }
    virtual ValidatedLowerKleenean overlaps(const HybridExactBox& bx) const = 0;
  protected:
    virtual EuclideanOvertSetInterface<ValidatedTag>* _euclidean_set(DiscreteLocation,RealSpace) const = 0;
};

//! \ingroup GeometryModule EuclideanSetInterfaceSubModule
//! \brief Interface for open sets.
template<> class HybridOpenSetInterface<ValidatedTag>
    : public virtual HybridOvertSetInterface<ValidatedTag>
{
  public:
    virtual HybridOpenSetInterface<ValidatedTag>* clone() const = 0;
//    inline EuclideanOpenSet euclidean_set(DiscreteLocation loc, RealSpace spc) const { return this->_euclidean_set(loc,spc); }
    virtual ValidatedLowerKleenean covers(const ExactBoxType& bx) const = 0;
  protected:
    virtual EuclideanOpenSetInterface<ValidatedTag>* _euclidean_set(DiscreteLocation,RealSpace) const = 0;
};

//! \ingroup GeometryModule EuclideanSetInterfaceSubModule
//! \brief Interface for closed sets.
template<> class HybridClosedSetInterface<ValidatedTag>
    : public virtual HybridSetInterfaceBase
{
  public:
    virtual HybridClosedSetInterface<ValidatedTag>* clone() const = 0;
//    inline ValidatedEuclideanClosedSet euclidean_set(DiscreteLocation loc, RealSpace spc) const { return this->_euclidean_set(loc,spc); }
    virtual ValidatedLowerKleenean separated(const HybridExactBox& bx) const = 0;
  protected:
    virtual EuclideanClosedSetInterface<ValidatedTag>* _euclidean_set(DiscreteLocation,RealSpace) const = 0;
};

//! \ingroup GeometryModule EuclideanSetInterfaceSubModule
//! \brief Interface for compact (closed and bounded) sets.
template<> class HybridCompactSetInterface<ValidatedTag>
    : public virtual HybridBoundedSetInterface<ValidatedTag>,
      public virtual HybridClosedSetInterface<ValidatedTag>
{
  public:
    virtual HybridCompactSetInterface<ValidatedTag>* clone() const = 0;
//    inline ValidatedEuclideanCompactSet euclidean_set(DiscreteLocation loc, RealSpace spc) const { return this->_euclidean_set(loc,spc); }
  protected:
    virtual EuclideanCompactSetInterface<ValidatedTag>* _euclidean_set(DiscreteLocation,RealSpace) const = 0;
};

//! \ingroup GeometryModule EuclideanSetInterfaceSubModule
//! \brief Interface for regular sets, whose closure is the closure of the interior, and whose interior is the interior of the closure.
template<> class HybridRegularSetInterface<ValidatedTag>
    : public virtual HybridOpenSetInterface<ValidatedTag>,
      public virtual HybridClosedSetInterface<ValidatedTag>
{
    virtual HybridRegularSetInterface<ValidatedTag>* clone() const = 0;
//    inline ValidatedEuclideanRegularSet euclidean_set(DiscreteLocation loc, RealSpace spc) const { return this->_euclidean_set(loc,spc); }
  protected:
    virtual EuclideanRegularSetInterface<ValidatedTag>* _euclidean_set(DiscreteLocation,RealSpace) const = 0;
};

//! \ingroup GeometryModule EuclideanSetInterfaceSubModule
//! \brief Interface for located (overt and compact) sets.
template<> class HybridLocatedSetInterface<ValidatedTag>
    : public virtual HybridOvertSetInterface<ValidatedTag>,
      public virtual HybridCompactSetInterface<ValidatedTag>
{
    virtual HybridLocatedSetInterface<ValidatedTag>* clone() const = 0;
//    inline ValidatedEuclideanLocatedSet euclidean_set(DiscreteLocation loc, RealSpace spc) const { return this->_euclidean_set(loc,spc); }
  protected:
    virtual EuclideanLocatedSetInterface<ValidatedTag>* _euclidean_set(DiscreteLocation,RealSpace) const = 0;
};

//! \ingroup GeometryModule EuclideanSetInterfaceSubModule
//! \brief Complete set interface for bounded regular sets.
template<> class HybridRegularLocatedSetInterface<ValidatedTag>
    : public virtual HybridRegularSetInterface<ValidatedTag>,
      public virtual HybridLocatedSetInterface<ValidatedTag>
{
  public:
    virtual HybridRegularLocatedSetInterface<ValidatedTag>* clone() const = 0;
//    inline ValidatedEuclideanRegularLocatedSet euclidean_set(DiscreteLocation loc, RealSpace spc) const { return this->_euclidean_set(loc,spc); }
  protected:
    virtual EuclideanRegularLocatedSetInterface<ValidatedTag>* _euclidean_set(DiscreteLocation,RealSpace) const = 0;
};



} // namespace Ariadne


#endif // ARIADNE_SET_INTERFACE
