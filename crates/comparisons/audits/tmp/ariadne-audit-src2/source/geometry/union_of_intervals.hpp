/***************************************************************************
 *            union_of_intervals.hpp
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

/*! \file union_of_intervals.hpp
 *  \brief Unions of intervals.
 */

#ifndef ARIADNE_UNION_OF_INTERVALS_HPP
#define ARIADNE_UNION_OF_INTERVALS_HPP

#include "interval.hpp"

namespace Ariadne {

struct LowerUpperBoundLess {
    template<class U> decltype(auto) operator() (Interval<U> const& ivl1, Interval<U> const& ivl2) const {
        return ivl1.lower_bound()<ivl2.lower_bound() or (ivl1.lower_bound()==ivl2.lower_bound() and ivl1.upper_bound()<ivl2.upper_bound()); }
};

template<class U> class UnionOfIntervals {
  private: public:
    List<Interval<U>> _data;
  public:
    typedef SetTraits<Real>::BasicSetType BasicSetType;

    typedef decltype(declval<Interval<U>>().width()) MeasureType;
    typedef typename List<Interval<U>>::Iterator Iterator;
    typedef typename List<Interval<U>>::ConstIterator ConstIterator;

    explicit UnionOfIntervals(List<Interval<U>> const& ivls) : _data(ivls) { this->_sort(); this->_combine(); }

    template<class UU> requires Constructible<U,UU>
        UnionOfIntervals(UnionOfIntervals<UU> ivls)
            : _data(ivls._data) { }
    template<class UU, class... PRS> requires Constructible<U,UU,PRS...>
        UnionOfIntervals(List<Interval<UU>> ivls, PRS... prs)
            : _data(Ariadne::apply([&](Interval<UU>const& ivl){return Interval<U>(ivl,prs...);},ivls)) { }

    UnionOfIntervals(InitializerList<Interval<U>> ivls) : UnionOfIntervals(List<Interval<U>>(ivls)) { }
    template<class... PRS> requires Constructible<U,Dyadic,PRS...>
        UnionOfIntervals(InitializerList<Interval<Dyadic>> ivls, PRS... prs) :
            UnionOfIntervals(List<Interval<Dyadic>>(ivls),prs...) { }

    UnionOfIntervals<U> create(IntervalDomainType bs) const {
        static_assert(Constructible<Interval<U>,IntervalDomainType> or Assignable<Interval<U>,DyadicInterval>);
        if constexpr (Constructible<Interval<U>,IntervalDomainType>) {
            return UnionOfIntervals<U>({Interval<U>(bs)});
        } else {
            assert(this->_data.size()!=0);
            Interval<U> bivl=this->_data.front(); bivl=DyadicInterval(bs);
            return UnionOfIntervals<U>({bivl});
        }
    }

    SizeType size() const { return this->_data.size(); }
    Iterator begin() { return this->_data.begin(); }
    ConstIterator begin() const { return this->_data.begin(); }
    ConstIterator end() const { return this->_data.end(); }

    SizeOne dimension() const { return SizeOne(); }
    Bool separated(IntervalDomainType bs) const { return not intersect(*this,this->create(bs)); }
    Bool overlaps(IntervalDomainType bs) const { return intersect(*this,this->create(bs)); }
    Bool covers(IntervalDomainType bs) const { return subset(this->create(bs),*this); }
    Bool inside(IntervalDomainType bs) const { return subset(*this,this->create(bs)); }
    Interval<U> bounding_box() const;

    MeasureType measure() const;
    friend MeasureType measure(UnionOfIntervals<U> const& ivls) { return ivls.measure(); }

    friend UnionOfIntervals<U> intersection(UnionOfIntervals<U> const& ivls1, IntervalDomainType const& ivl2) {
        return intersection(ivls1,ivls1.create(ivl2)); }
    friend UnionOfIntervals<U> intersection(UnionOfIntervals<U> const& ivls1, UnionOfIntervals<U> const& ivls2) {
        return UnionOfIntervals<U>::_intersection(ivls1,ivls2); }
    friend UnionOfIntervals<U> join(UnionOfIntervals<U> const& ivls1, UnionOfIntervals<U> const& ivls2) {
        return UnionOfIntervals<U>::_join(ivls1,ivls2); }
    friend Bool intersect(UnionOfIntervals<U> const& ivls1, UnionOfIntervals<U> const& ivls2) {
        return UnionOfIntervals<U>::_intersect(ivls1,ivls2); }
    friend Bool subset(UnionOfIntervals<U> const& ivls1, UnionOfIntervals<U> const& ivls2) {
        return UnionOfIntervals<U>::_subset(ivls1,ivls2); }
    friend OutputStream& operator<<(OutputStream& os, UnionOfIntervals<U> const& ivls) { return os << ivls._data; }
  private:
    static UnionOfIntervals<U> _intersection(UnionOfIntervals<U> const& ivls1, UnionOfIntervals<U> const& ivls2);
    static UnionOfIntervals<U> _join(UnionOfIntervals<U> const& ivls1, UnionOfIntervals<U> const& ivls2);
    static Bool _intersect(UnionOfIntervals<U> const& ivls1, UnionOfIntervals<U> const& ivls2);
    static Bool _subset(UnionOfIntervals<U> const& ivls1, UnionOfIntervals<U> const& ivls2);
    Void _sort();
    Void _combine();
};

template<class U> UnionOfIntervals<U> interior(UnionOfIntervals<U> ivls, NegationType<U> eps) {
    List<Interval<U>> rivls;
    rivls.reserve(ivls.size());
    for (auto ivl : ivls) {
        Interval<U> rivl(ivl.lower()+eps, ivl.upper()-eps);
        if (not rivl.empty()) {
            rivls.append(rivl);
        }
    }
    return UnionOfIntervals<U>(rivls);
}


template<class U> UnionOfIntervals<U> UnionOfIntervals<U>::_intersection(UnionOfIntervals<U> const& ivls1, UnionOfIntervals<U> const& ivls2) {
    // TODO: Make more efficient
    List<Interval<U>> rivls;
    auto iter1=ivls1.begin();
    auto iter2=ivls2.begin();
    while (iter1!=ivls1.end() && iter2!=ivls2.end()) {
        if (iter1->lower_bound() >= iter2->upper_bound()) {
            ++iter1;
        } else if (iter2->lower_bound() >= iter1->upper_bound()) {
            ++iter2;
        } else {
            rivls.append(Interval<U>(max(iter1->lower_bound(),iter2->lower_bound()),min(iter1->upper_bound(),iter2->upper_bound())));
            if (iter1->upper_bound()<iter2->upper_bound()) { ++iter1; } else { ++iter2; }
        }
    }
    return UnionOfIntervals<U>(rivls);
}

template<class U> UnionOfIntervals<U> UnionOfIntervals<U>::_join(UnionOfIntervals<U> const& ivls1, UnionOfIntervals<U> const& ivls2) {
    // TODO: Make more efficient
    return UnionOfIntervals<U>(catenate(ivls1._data,ivls2._data));
}

template<class U> Bool UnionOfIntervals<U>::_intersect(UnionOfIntervals<U> const& ivls1, UnionOfIntervals<U> const& ivls2) {
    auto iter1=ivls1.begin();
    auto iter2=ivls2.begin();
    while (iter1!=ivls1.end() && iter2!=ivls2.end()) {
        if (iter1->lower_bound() >= iter2->upper_bound()) {
            ++iter2;
        } else if (iter1->upper_bound() <= iter2->lower_bound()) {
            ++iter1;
        } else {
            return true;
        }
    }
    return false;
}

template<class U> Bool UnionOfIntervals<U>::_subset(UnionOfIntervals<U> const& ivls1, UnionOfIntervals<U> const& ivls2) {
    auto iter1=ivls1.begin();
    auto iter2=ivls2.begin();
    while (iter1!=ivls1.end() && iter2!=ivls2.end()) {
        if (iter1->lower_bound() >= iter2->upper_bound()) {
            ++iter2;
        } else if (iter1->lower_bound() >= iter2->lower_bound() && iter1->upper_bound() <= iter2->upper_bound()) {
            ++iter1;
        } else {
            return false;
        }
    }
    if (iter1==ivls1.end()) {
        return true;
    } else {
        return false;
    }
}


template<class U> auto UnionOfIntervals<U>::measure() const -> MeasureType {
    MeasureType r(nul(_data[0].width()));
    for (auto ivl : this->_data) {
        r+=ivl.width();
    }
    return r;
}

template<class U> Interval<U> UnionOfIntervals<U>::bounding_box() const {
    ARIADNE_ASSERT(!_data.empty());
    return Interval<U>(this->_data.front().lower_bound(),this->_data.back().upper_bound());
}

template<class U> Void UnionOfIntervals<U>::_sort() {
    std::sort(this->_data.begin(),this->_data.end(),LowerUpperBoundLess());
}

template<class U> Void UnionOfIntervals<U>::_combine() {
    ConstIterator start=begin();
    Iterator curr=_data.begin();
    ConstIterator next=curr;
    while (next!=end()) {
        *curr=*next;
        ++next;
        while (next!=end() && next->lower_bound()<=curr->upper_bound()) {
            curr->set_upper_bound(max(curr->upper_bound(),next->upper_bound()));
            ++next;
        }
        ++curr;
    }

    _data.resize(static_cast<SizeType>(curr-start));
}

} // namespace Ariadne

#endif
