/***************************************************************************
 *            set_wrapper.hpp
 *
 *  Copyright  2020  Pieter Collins
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

/*! \file set_wrapper.hpp
 *  \brief Wrappers for concrete sets
 */

#ifndef ARIADNE_SET_WRAPPER_HPP
#define ARIADNE_SET_WRAPPER_HPP

#include "utility/handle.hpp"
#include "geometry_concepts.hpp"
#include "set.hpp"

namespace Ariadne {

template<class SET, class P, class T> requires AnOpenSet<SET,P,T> class OpenSetWrapper;
template<class SET, class P, class T> requires AClosedSet<SET,P,T> class ClosedSetWrapper;
template<class SET, class P, class T> requires ARegularSet<SET,P,T> class RegularSetWrapper;
template<class SET, class P, class T> requires AnOvertSet<SET,P,T> class OvertSetWrapper;
template<class SET, class P, class T> requires ACompactSet<SET,P,T> class CompactSetWrapper;
template<class SET, class P, class T> requires ALocatedSet<SET,P,T> class LocatedSetWrapper;
template<class SET, class P, class T> requires ARegularLocatedSet<SET,P,T> class RegularLocatedSetWrapper;


template<class SET, class P, class T> requires AnOpenSet<SET,P,T> class OpenSetWrapper
    : public virtual OpenSet<P,T>::Interface
    , public SET
{
    SET const& base() const { return *this; }
  public:
    using typename OpenSetInterface<P,T>::DimensionType;
    using typename OpenSetInterface<P,T>::BasicSetType;

    using SET::SET;
    explicit OpenSetWrapper(SET const& set) : SET(set) { }
    virtual OutputStream& _write(OutputStream& os) const final override { return os << this->base(); }
    virtual OpenSetInterface<P,T>* clone() const final { return new OpenSetWrapper<SET,P,T>(this->base()); }
    virtual DimensionType dimension() const final override { return this->base().dimension(); }
    virtual LowerKleeneanType<P> overlaps(const BasicSetType& bs) const final { return this->base().overlaps(bs); }
    virtual LowerKleeneanType<P> covers(const BasicSetType& bs) const final { return this->base().covers(bs); }
    // Needed to prevent ambiguity
    friend OutputStream& operator<<(OutputStream& os, OpenSetWrapper<SET,P,T> ops) { return os << ops.base(); }
};

template<class SET, class P, class T> requires AClosedSet<SET,P,T> class ClosedSetWrapper
    : public virtual ClosedSet<P,T>::Interface
    , public SET
{
    SET const& base() const { return *this; }
  public:
    using typename ClosedSetInterface<P,T>::DimensionType;
    using typename ClosedSetInterface<P,T>::BasicSetType;

    using SET::SET;
    explicit ClosedSetWrapper(SET const& set) : SET(set) { }
    virtual OutputStream& _write(OutputStream& os) const final override { return os << this->base(); }
    virtual ClosedSetInterface<P,T>* clone() const final { return new ClosedSetWrapper<SET,P,T>(this->base()); }
    virtual DimensionType dimension() const final override { return this->base().dimension(); }
    virtual LowerKleeneanType<P> separated(const BasicSetType& bs) const final { return this->base().separated(bs); }
    // Needed to prevent ambiguity
    friend OutputStream& operator<<(OutputStream& os, ClosedSetWrapper<SET,P,T> ops) { return os << ops.base(); }
};

template<class SET, class P, class T> requires AnOvertSet<SET,P,T> class OvertSetWrapper
    : public virtual OvertSet<P,T>::Interface
    , public SET
{
    SET const& base() const { return *this; }
  public:
    using typename OvertSetInterface<P,T>::DimensionType;
    using typename OvertSetInterface<P,T>::BasicSetType;

    using SET::SET;
    explicit OvertSetWrapper(SET const& set) : SET(set) { }
    virtual OutputStream& _write(OutputStream& os) const final override { return os << this->base(); }
    virtual OvertSetInterface<P,T>* clone() const final { return new OvertSetWrapper<SET,P,T>(this->base()); }
    virtual DimensionType dimension() const final override { return this->base().dimension(); }
    virtual LowerKleeneanType<P> overlaps(const BasicSetType& bs) const final { return this->base().overlaps(bs); }
    // Needed to prevent ambiguity
    friend OutputStream& operator<<(OutputStream& os, OvertSetWrapper<SET,P,T> ops) { return os << ops.base(); }
};

template<class SET, class P, class T> requires ACompactSet<SET,P,T> class CompactSetWrapper
    : public virtual CompactSet<P,T>::Interface
    , public SET
{
    SET const& base() const { return *this; }
  public:
    using typename CompactSetInterface<P,T>::DimensionType;
    using typename CompactSetInterface<P,T>::BasicSetType;
    using typename CompactSetInterface<P,T>::BoundingSetType;

    using SET::SET;
    explicit CompactSetWrapper(SET const& set) : SET(set) { }
    virtual OutputStream& _write(OutputStream& os) const final override { return os << this->base(); }
    virtual CompactSetInterface<P,T>* clone() const final { return new CompactSetWrapper<SET,P,T>(this->base()); }
    virtual DimensionType dimension() const final override { return this->base().dimension(); }
    virtual BoundingSetType bounding_box() const final { return static_cast<BoundingSetType>(this->base().bounding_box()); }
    virtual LowerKleeneanType<P> inside(const BasicSetType& bs) const final { return this->base().inside(bs); }
    virtual LowerKleeneanType<P> separated(const BasicSetType& bs) const final { return this->base().separated(bs); }
    // Needed to prevent ambiguity
    friend OutputStream& operator<<(OutputStream& os, CompactSetWrapper<SET,P,T> ops) { return os << ops.base(); }
};

template<class SET, class P, class T> requires ARegularSet<SET,P,T> class RegularSetWrapper
    : public virtual RegularSet<P,T>::Interface
    , public SET
{
    SET const& base() const { return *this; }
  public:
    using typename RegularSetInterface<P,T>::DimensionType;
    using typename RegularSetInterface<P,T>::BasicSetType;

    using SET::SET;
    explicit RegularSetWrapper(SET const& set) : SET(set) { }
    virtual OutputStream& _write(OutputStream& os) const final override { return os << this->base(); }
    virtual RegularSetInterface<P,T>* clone() const final { return new RegularSetWrapper<SET,P,T>(this->base()); }
    virtual DimensionType dimension() const final override { return this->base().dimension(); }
    virtual LowerKleeneanType<P> overlaps(const BasicSetType& bs) const final { return this->base().overlaps(bs); }
    virtual LowerKleeneanType<P> covers(const BasicSetType& bs) const final { return this->base().covers(bs); }
    virtual LowerKleeneanType<P> separated(const BasicSetType& bs) const final { return this->base().separated(bs); }
    // Needed to prevent ambiguity
    friend OutputStream& operator<<(OutputStream& os, RegularSetWrapper<SET,P,T> ops) { return os << ops.base(); }
};

template<class SET, class P, class T> requires ALocatedSet<SET,P,T> class LocatedSetWrapper
    : public virtual LocatedSet<P,T>::Interface
    , public SET
{
    SET const& base() const { return *this; }
  public:
    using typename LocatedSetInterface<P,T>::DimensionType;
    using typename LocatedSetInterface<P,T>::BasicSetType;
    using typename LocatedSetInterface<P,T>::BoundingSetType;

    using SET::SET;
    LocatedSetWrapper(SET const& set) : SET(set) { }
    virtual OutputStream& _write(OutputStream& os) const final override { return os << this->base(); }
    virtual LocatedSetInterface<P,T>* clone() const final { return new LocatedSetWrapper<SET,P,T>(this->base()); }
    virtual DimensionType dimension() const final override { return this->base().dimension(); }
    virtual LowerKleeneanType<P> overlaps(const BasicSetType& bs) const final { return this->base().overlaps(bs); }
    virtual LowerKleeneanType<P> separated(const BasicSetType& bs) const final { return this->base().separated(bs); }
    virtual LowerKleeneanType<P> inside(const BasicSetType& bs) const final { return this->base().inside(bs); }
    virtual BoundingSetType bounding_box() const final { return static_cast<BoundingSetType>(this->base().bounding_box()); }
    // Needed to prevent ambiguity
    friend OutputStream& operator<<(OutputStream& os, LocatedSetWrapper<SET,P,T> ops) { return os << ops.base(); }
};

template<class SET, class P, class T> requires ARegularLocatedSet<SET,P,T> class RegularLocatedSetWrapper
    : public virtual RegularLocatedSet<P,T>::Interface
    , public SET
{
    SET const& base() const { return *this; }
  public:
    using typename RegularLocatedSetInterface<P,T>::DimensionType;
    using typename RegularLocatedSetInterface<P,T>::BasicSetType;
    using typename RegularLocatedSetInterface<P,T>::BoundingSetType;

    using SET::SET;
    RegularLocatedSetWrapper(SET const& set) : SET(set) { }
    virtual OutputStream& _write(OutputStream& os) const final override { return os << this->base(); }
    virtual RegularLocatedSetInterface<P,T>* clone() const final {
        return new RegularLocatedSetWrapper<SET,P,T>(this->base()); }
    virtual DimensionType dimension() const final override { return this->base().dimension(); }
    virtual LowerKleeneanType<P> overlaps(const BasicSetType& bs) const final { return this->base().overlaps(bs); }
    virtual LowerKleeneanType<P> covers(const BasicSetType& bs) const final { return this->base().covers(bs); }
    virtual LowerKleeneanType<P> separated(const BasicSetType& bs) const final { return this->base().separated(bs); }
    virtual LowerKleeneanType<P> inside(const BasicSetType& bs) const final { return this->base().inside(bs); }
    virtual BoundingSetType bounding_box() const final { return static_cast<BoundingSetType>(this->base().bounding_box()); }
    // Needed to prevent ambiguity
    friend OutputStream& operator<<(OutputStream& os, RegularLocatedSetWrapper<SET,P,T> ops) { return os << ops.base(); }
};


template<class S, class C> struct WrapperTrait;
template<class S, class P, class T> struct WrapperTrait<S,OpenSet<P,T>> { typedef OpenSetWrapper<S,P,T> Type; };
template<class S, class P, class T> struct WrapperTrait<S,ClosedSet<P,T>> { typedef ClosedSetWrapper<S,P,T> Type; };
template<class S, class P, class T> struct WrapperTrait<S,OvertSet<P,T>> { typedef OvertSetWrapper<S,P,T> Type; };
template<class S, class P, class T> struct WrapperTrait<S,CompactSet<P,T>> { typedef CompactSetWrapper<S,P,T> Type; };
template<class S, class P, class T> struct WrapperTrait<S,RegularSet<P,T>> { typedef RegularSetWrapper<S,P,T> Type; };
template<class S, class P, class T> struct WrapperTrait<S,LocatedSet<P,T>> { typedef LocatedSetWrapper<S,P,T> Type; };
template<class S, class P, class T> struct WrapperTrait<S,RegularLocatedSet<P,T>> { typedef RegularLocatedSetWrapper<S,P,T> Type; };

template<class S, class C> using WrapperType = typename WrapperTrait<S,C>::Type;

template<class C, class S> C wrap(S s) {
    if constexpr (DerivedFrom<S,typename C::Interface>) { return C(s.clone()); }
    else { return C(std::make_shared<WrapperType<S,C>>(s)); } }


template<class P, class T> template<class S> requires AnOpenSet<S,P,T> OpenSet<P,T>::OpenSet(S const& s)
    : OpenSet(wrap<OpenSet<P,T>>(s)) { }
template<class P, class T> template<class S> requires AClosedSet<S,P,T> ClosedSet<P,T>::ClosedSet(S const& s)
    : ClosedSet(wrap<ClosedSet<P,T>>(s)) { }
template<class P, class T> template<class S> requires AnOvertSet<S,P,T> OvertSet<P,T>::OvertSet(S const& s)
    : OvertSet(wrap<OvertSet<P,T>>(s)) { }
template<class P, class T> template<class S> requires ACompactSet<S,P,T> CompactSet<P,T>::CompactSet(S const& s)
    : CompactSet(wrap<CompactSet<P,T>>(s)) { }
template<class P, class T> template<class S> requires ARegularSet<S,P,T> RegularSet<P,T>::RegularSet(S const& s)
    : RegularSet(wrap<RegularSet<P,T>>(s)) { }
template<class P, class T> template<class S> requires ALocatedSet<S,P,T> LocatedSet<P,T>::LocatedSet(S const& s)
    : LocatedSet(wrap<LocatedSet<P,T>>(s)) { }
template<class P, class T> template<class S> requires ARegularLocatedSet<S,P,T> RegularLocatedSet<P,T>::RegularLocatedSet(S const& s)
    : RegularLocatedSet(wrap<RegularLocatedSet<P,T>>(s)) { }


template<class SET> OpenSet<ValidatedTag,Real> wrap_open(SET const& s) {
    using P=ValidatedTag; using T=Real;
    return OpenSet<P,T>(std::make_shared<OpenSetWrapper<SET,P,T>>(s));
}

} // namespace Ariadne

#endif
