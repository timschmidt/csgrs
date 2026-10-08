/***************************************************************************
 *            utility/container.hpp
 *
 *  Copyright  2013-20  Pieter Collins
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

/*! \file utility/container.hpp
 *  \brief
 */



#ifndef ARIADNE_CONTAINER_HPP
#define ARIADNE_CONTAINER_HPP

#include "stdlib.hpp"

#include "metaprogramming.hpp"
#include "helper/stlio.hpp"
#include "array.hpp"
#include "macros.hpp"
#include "typedefs.hpp"

namespace Ariadne {

using std::make_tuple;
using std::make_pair;

using SizeType=std::size_t;

template<class T> OutputStream& operator<<(OutputStream& os, const Array<T>& a) {
    bool first=true;
    for(auto x : a) {
        os << (first ? "[" : ",") << x;
        first = false;
    }
    if(first) { os << "["; }
    return os << "]";
}

template<class T> OutputStream& operator<<(OutputStream& os, const SharedArray<T>& a) {
    bool first=true;
    for(auto x : a) {
        os << (first ? "[" : ",") << x;
        first = false;
    }
    if(first) { os << "["; }
    return os << "]";
}

template<class T> class List : public std::vector<T> {
  public:
    typedef typename std::vector<T>::value_type ValueType;
    typedef typename std::vector<T>::reference Reference;
    typedef typename std::vector<T>::const_reference ConstReference;
    typedef typename std::vector<T>::pointer Pointer;
    typedef typename std::vector<T>::const_pointer ConstPointer;
    typedef typename std::vector<T>::iterator Iterator;
    typedef typename std::vector<T>::const_iterator ConstIterator;
    using std::vector<T>::vector;
    List() : std::vector<T>() { }
    List(std::vector<T> const& lst) : std::vector<T>(lst) { }
    List(T const& t) : std::vector<T>(1u,t) { }
    template<ConvertibleTo<T> TT>
        List(const std::vector<TT>& l) : std::vector<T>(l.begin(),l.end()) { }
    template<ExplicitlyConvertibleTo<T> TT>
        explicit List(const std::vector<TT>& l) : std::vector<T>(l.begin(),l.end()) { }
    void append(const T& t) { this->push_back(t); }
    void append(const std::vector<T>& t) {
        for(unsigned int i=0; i!=t.size(); ++i) { this->push_back(t[i]); } }
    void concatenate(const std::vector<T>& t) {
        for(unsigned int i=0; i!=t.size(); ++i) { this->push_back(t[i]); } }
};
template<class T> inline List<T> catenate(const List<T>& l1, const List<T>& l2) {
    List<T> r(l1);
    for(auto iter=l2.begin(); iter!=l2.end(); ++iter) {
        r.append(*iter);
    }
    return r;
}
template<class T> inline List<T> catenate(const List<T>& l1, const T& t2) {
    List<T> r(l1);
    r.append(t2);
    return r;
}
template<class T, Invocable<T> F> inline
List<InvokeResult<F,T>> apply(F&& f, List<T> const& lst) {
    typedef InvokeResult<F,T> R; List<R> res; res.reserve(lst.size());
    for (auto itm : lst) { res.append(f(itm)); } return res;
}

template<class T> OutputStream& operator<<(OutputStream& os, const std::vector<T>& v) {
    bool first=true;
    for(auto x : v) {
        os << (first ? "[" : ",") << x;
        first = false;
    }
    if(first) { os << "["; }
    return os << "]";
}

template<class T> class LinkedList
    : public std::list<T>
{
  public:
    typedef typename std::list<T>::iterator Iterator;
    typedef typename std::list<T>::const_iterator ConstIterator;
    LinkedList() : std::list<T>() { }
    LinkedList(unsigned int n) : std::list<T>(n) { }
    LinkedList(unsigned int n, const T& t) : std::vector<T>(n,t) { }
    LinkedList(const std::list<T>& l) : std::list<T>(l) { }
    template<class X> LinkedList(const List<X>& l) : std::list<T>(l.begin(),l.end()) { }
    template<class X> LinkedList(const LinkedList<X>& l) : std::list<T>(l.begin(),l.end()) { }
    template<class I> LinkedList(const I& b, const I& e) : std::list<T>(b,e) { }
    // Conversion constructor from an value to a single-element list.
    void append(const T& t) { this->push_back(t); }
    void append(const LinkedList<T>& t) { for(unsigned int i=0; i!=t.size(); ++i) { this->push_back(t[i]); } }
    void concatenate(const LinkedList<T>& t) { for(unsigned int i=0; i!=t.size(); ++i) { this->push_back(t[i]); } }
};
template<class T> inline OutputStream&
operator<< (OutputStream &os, const std::list<T>& l) {
    return Helper::write_sequence(os,l.begin(),l.end());
}


template<class T> class ContainerComparison {
  public:
    inline bool operator() (T const& t1, T const& t2) const { return t1 < t2; }
};


template<class T, class CMP> class Set : public std::set<T,CMP> {
  public:
    typedef typename std::set<T,CMP>::iterator Iterator;
    typedef typename std::set<T,CMP>::const_iterator ConstIterator;
    using std::set<T,CMP>::set;
    Set() : std::set<T,CMP>() { }
    template<ConvertibleTo<T> TT>
        Set(const InitializerList<TT>& s) : std::set<T,CMP>(s.begin(),s.end()) { }
    template<ConvertibleTo<T> TT>
        Set(const std::vector<TT>& s) : std::set<T,CMP>(s.begin(),s.end()) { }
    template<ConvertibleTo<T> TT, class CMPTT>
        Set(const std::set<TT,CMPTT>& s) : std::set<T,CMP>(s.begin(),s.end()) { }
    template<ExplicitlyConvertibleTo<T> TT, class CMPTT>
        explicit Set(const std::set<TT,CMPTT>& s) { for(auto x : s) { this->insert(T(x)); } }
    explicit operator List<T> () const { return List<T>(this->begin(),this->end()); }
    bool contains(const T& t) const {
        return this->find(t)!=this->end(); }
    bool subset(const std::set<T,CMP>& s) const {
        for(auto iter=s.begin(); iter!=s.end(); ++iter) {
            if(!this->contains(*iter)) { return false; } } return true; }
    bool disjoint(const std::set<T,CMP>& s) const {
        for(auto iter=s.begin(); iter!=s.end(); ++iter) {
            if(this->contains(*iter)) { return false; } } return true; }
    bool disjoint(const std::vector<T>& lst) const {
        for(auto iter=lst.begin(); iter!=lst.end(); ++iter) {
            if(this->contains(*iter)) { return false; } } return true; }
    Set<T>& adjoin(const std::set<T,CMP>& s) {
        for(auto iter=s.begin(); iter!=s.end(); ++iter) { this->insert(*iter); } return *this; }
    Set<T>& remove(const T& t) {
        auto iter=this->find(t);
        if(iter!=this->end()) { this->erase(iter); }
        return *this; }
    Set<T>& remove(const std::set<T,CMP>& s) {
        auto iter=this->begin();
        while(iter!=this->end()) { if(s.find(*iter)!=s.end()) { this->erase(iter++); } else { ++iter; } }
        return *this; }
    Set<T>& restrict(const std::set<T,CMP>& s) {
        auto iter=this->begin();
        while(iter!=this->end()) { if(s.find(*iter)!=s.end()) { this->erase(iter++); } else { ++iter; } }
        return *this; }
    template<class TT> Set<T>& remove(const std::vector<TT>& l) {
        for(auto iter=l.begin(); iter!=l.end(); ++iter) {
            this->std::set<T,CMP>::erase(static_cast<const T&>(*iter)); }
        return *this; }
};
template<class T, template<class>class C, Invocable<T> F> inline
Set<InvokeResult<F,T>,C<InvokeResult<F,T>>> apply(F&& f, Set<T,C<T>> const& set) {
    typedef InvokeResult<F,T> R; Set<R,C<R>> res; for (auto itm : set) { res.adjoin(f(itm)); } return res; }
template<class T, class CMP> inline Set<T,CMP> join(Set<T,CMP> s1, Set<T,CMP> const& s2) {
    s1.adjoin(s2); return s1; }
template<class T, class CMP> inline bool contains(const std::set<T,CMP>& s, const T& t) {
    return s.find(t)!=s.end(); }
template<class T, class CMP> inline bool subset(const std::set<T,CMP>& s1, const std::set<T,CMP>& s2) {
    for(auto iter=s1.begin(); iter!=s1.end(); ++iter) {
        if(!contains(s2,*iter)) { return false; } } return true; }
template<class T, class CMP> inline bool disjoint(const std::set<T,CMP>& s1, const std::set<T,CMP>& s2) {
    for(auto iter=s1.begin(); iter!=s1.end(); ++iter) {
        if(contains(s2,*iter)) { return false; } } return true; }
template<class T, class CMP> inline bool disjoint(const std::set<T,CMP>& s1, const std::vector<T>& l2) {
    for(auto iter=l2.begin(); iter!=l2.end(); ++iter) {
        if(contains(s1,*iter)) { return false; } } return true; }
template<class T, class CMP> inline Set<T,CMP> intersection(const std::set<T,CMP>& s1, const std::set<T,CMP>& s2) {
    Set<T,CMP> r(s1); restrict(r,s2); return r; }
template<class T, class CMP> inline Set<T,CMP> intersection(const std::set<T,CMP>& s1, const std::vector<T>& l2) {
    Set<T,CMP> r; for(auto iter=l2.begin(); iter!=l2.end(); ++iter) {
        if(contains(s1,*iter)) { r.insert(*iter); } } return r; }
template<class T, class CMP> inline Set<T,CMP> difference(const std::set<T,CMP>& s1, const std::set<T,CMP>& s2) {
    Set<T,CMP> r(s1); remove(r,s2); return r; }
template<class T, class CMP> inline std::set<T,CMP>& remove(std::set<T,CMP>& r, const std::set<T,CMP>& s) {
    auto iter=r.begin();
    while(iter!=r.end()) { if(contains(s,*iter)) { r.erase(iter++); } else { ++iter; } }
    return r; }
template<class T, class CMP> inline std::set<T,CMP>& restrict(std::set<T,CMP>& r, const std::set<T,CMP>& s) {
    auto iter=r.begin();
    while(iter!=r.end()) { if(!contains(s,*iter)) { r.erase(iter++); } else { ++iter; } }
    return r; }

template<class T, class CMP> OutputStream& operator<<(OutputStream& os, const std::set<T,CMP>& v) {
    bool first=true;
    for(auto x : v) {
        os << (first ? "{" : ",") << x;
        first = false;
    }
    if(first) { os << "{"; }
    return os << "}";
}


template<class K, class T, class CMP> class Map : public std::map<K,T,CMP> {
  public:
    typedef typename std::map<K,T,CMP>::iterator Iterator;
    typedef typename std::map<K,T,CMP>::const_iterator ConstIterator;
    template<ConvertibleTo<T> TT, class CCMP>
        Map(const std::map<K,TT,CCMP>& m) : std::map<K,T,CMP>(m.begin(),m.end()) { }
    using std::map<K,T,CMP>::map;
    using std::map<K,T,CMP>::insert;
    T& operator[](K k) { return this->std::map<K,T,CMP>::operator[](k); }
    const T& operator[](K k) const { auto iter=this->find(k); ARIADNE_ASSERT(iter!=this->end()); return iter->second; }
    const T& get(const K& k) const { auto i=this->find(k);
        ARIADNE_ASSERT(i!=this->end()); return i->second; }
    bool has_key(const K& k) const {
        return this->find(k)!=this->end(); }
    T& value(const K& k) {
        auto iter=this->find(k);
        ARIADNE_ASSERT(iter!=this->end()); return iter->second; }
    const T& value(const K& k) const {
        auto iter=this->find(k);
        ARIADNE_ASSERT(iter!=this->end()); return iter->second; }
    void insert(const std::pair<K,T>& kv) {
        this->std::map<K,T,CMP>::insert(kv); }
    void insert(const K& k, const T& v) {
        this->std::map<K,T,CMP>::insert(std::make_pair(k,v)); }
    void adjoin(const std::map<K,T,CMP>& m) {
        for(auto i=m.begin(); i!=m.end(); ++i) { this->insert(*i); } }
    void remove_keys(const Set<K,CMP>& s) {
        for(auto iter=s.begin(); iter!=s.end(); ++iter) { this->erase(*iter); } }
    Set<K,CMP> keys() const {
        Set<K,CMP> res; for(auto iter=this->begin(); iter!=this->end(); ++iter) {
            res.insert(iter->first); } return res; }
    List<T> values() const {
        List<T> res; for(auto iter=this->begin(); iter!=this->end(); ++iter) {
            res.append(iter->second); } return res; }
};
template<class K, class T, class CMP> inline Map<K,T,CMP> join(Map<K,T,CMP> m1, Map<K,T,CMP> const& m2) {
    m1.adjoin(m2); return m1; }
template<class I, class X, class J, class CMP> inline X& insert(Map<I,X,CMP>& m, const J& k, const X& v) {
    return m.std::template map<I,X,CMP>::insert(std::make_pair(k,v)).first->second; }
template<class K, class T, class CMP> Map<K,T,CMP> restrict_keys(const std::map<K,T,CMP>& m, const std::set<K,CMP>& k) {
    Map<K,T,CMP> result; const Set<K,CMP>& keys=static_cast<const Set<K,CMP>&>(k);
    for(auto item_iter=m.begin(); item_iter!=m.end(); ++item_iter) {
        if(keys.contains(item_iter->first)) { result.insert(*item_iter); } } return result; }
template<class K, class T, class CMP> OutputStream& operator<<(OutputStream& os, const std::map<K,T,CMP>& m) {
    bool first=true;
    for(auto x : m) {
        os << (first ? "{ " : ", ") << x.first << ":" << x.second;
        first = false;
    }
    if(first) { os << "{"; }
    return os << " }";
}


template<class T> bool unique_elements(const std::vector<T>& lst) {
    Set<T> found;
    for(auto iter=lst.begin(); iter!=lst.end(); ++iter) {
        if(found.contains(*iter)) { return false; } else { found.insert(*iter); } }
    return true;
}

template<class T> Set<T> duplicate_elements(const std::vector<T>& lst) {
    Set<T> result; Set<T> found;
    for(auto iter=lst.begin(); iter!=lst.end(); ++iter) {
        if(found.contains(*iter)) { result.insert(*iter); } else { found.insert(*iter); } }
    return result;
}


template<class T> inline Array<T> make_array(const T& t) { return Array<T>(1u,t); }
template<class T> inline Array<T> make_array(const std::vector<T>& vec) { return Array<T>(vec.begin(),vec.end()); }
template<class T> inline Array<T> make_array(const Array<T>& ary) { return Array<T>(ary); }

template<class T> inline List<T> make_list(const T& t) { return List<T>(1u,t); }
template<class T> inline List<T> make_list(const std::vector<T>& vec) { return List<T>(vec); }
template<class T> inline List<T> make_list(const Array<T>& ary) { return List<T>(ary); }
template<class T> inline List<T> make_list(const Set<T>& set) { return List<T>(set.begin(),set.end()); }

template<class T> inline Set<T> make_set(const std::vector<T>& lst) { return Set<T>(lst); }

} // namespace Ariadne

#endif
