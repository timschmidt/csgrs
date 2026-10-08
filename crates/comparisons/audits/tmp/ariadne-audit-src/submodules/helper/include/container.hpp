/***************************************************************************
 *            container.hpp
 *
 *  Copyright  2023  Pieter Collins
 *
 ****************************************************************************/

/*
 * This file is part of Helper, under the MIT license.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

/*! \file container.hpp
 *  \brief
 */



#ifndef HELPER_CONTAINER_HPP
#define HELPER_CONTAINER_HPP

#include "stdlib.hpp"

#include "metaprogramming.hpp"
#include "stlio.hpp"
#include "array.hpp"
#include "macros.hpp"

namespace Helper {

using std::make_tuple;
using std::make_pair;
using std::size_t;

template<class T> ostream& operator<<(ostream& os, const Array<T>& a) {
    bool first=true;
    for(auto x : a) {
        os << (first ? "[" : ",") << x;
        first = false;
    }
    if(first) { os << "["; }
    return os << "]";
}

template<class T> ostream& operator<<(ostream& os, const SharedArray<T>& a) {
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

template<class F, class T> List<ResultOf<F(T)>> elementwise(F const& f, List<T> const& l) {
    typedef ResultOf<F(T)> R;
    List<R> r; r.reserve(l.size());
    for(size_t i=0; i!=l.size(); ++i) { r.append(f(l[i])); }
    return r;
}

template<class F, class T1, class T2> List<ResultOf<F(T1,T2)>> elementwise(F const& f, List<T1> const& l1, List<T2> const& l2) {
    typedef ResultOf<F(T1,T2)> R;
    assert(l1.size()==l2.size());
    List<R> r; r.reserve(l1.size());
    for(size_t i=0; i!=l1.size(); ++i) { r.append(f(l1[i],l2[i])); }
    return r;
}

template<class T> ostream& operator<<(ostream& os, const std::vector<T>& v) {
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
template<class T> inline ostream&
operator<< (ostream &os, const std::list<T>& l) {
    return Helper::write_sequence(os,l.begin(),l.end());
}


template<class T> class Set : public std::set<T> {
  public:
    typedef typename std::set<T>::iterator Iterator;
    typedef typename std::set<T>::const_iterator ConstIterator;
    using std::set<T>::set;
    Set() : std::set<T>() { }
    template<ConvertibleTo<T> TT>
        Set(const InitializerList<TT>& s) : std::set<T>(s.begin(),s.end()) { }
    template<ConvertibleTo<T> TT>
        Set(const std::vector<TT>& s) : std::set<T>(s.begin(),s.end()) { }
    template<ConvertibleTo<T> TT>
        Set(const std::set<TT>& s) : std::set<T>(s.begin(),s.end()) { }
    template<ExplicitlyConvertibleTo<T> TT>
        explicit Set(const std::set<TT>& s) { for(auto x : s) { this->insert(T(x)); } }
    explicit operator List<T> () const { return List<T>(this->begin(),this->end()); }
    bool contains(const T& t) const {
        return this->find(t)!=this->end(); }
    bool subset(const std::set<T>& s) const {
        for(auto iter=s.begin(); iter!=s.end(); ++iter) {
            if(!this->contains(*iter)) { return false; } } return true; }
    bool disjoint(const std::set<T>& s) const {
        for(auto iter=s.begin(); iter!=s.end(); ++iter) {
            if(this->contains(*iter)) { return false; } } return true; }
    bool disjoint(const std::vector<T>& lst) const {
        for(auto iter=lst.begin(); iter!=lst.end(); ++iter) {
            if(this->contains(*iter)) { return false; } } return true; }
    Set<T>& adjoin(const std::set<T>& s) {
        for(auto iter=s.begin(); iter!=s.end(); ++iter) { this->insert(*iter); } return *this; }
    Set<T>& remove(const T& t) {
        auto iter=this->find(t);
        if(iter!=this->end()) { this->erase(iter); }
        return *this; }
    Set<T>& remove(const std::set<T>& s) {
        auto iter=this->begin();
        while(iter!=this->end()) { if(s.find(*iter)!=s.end()) { this->erase(iter++); } else { ++iter; } }
        return *this; }
    Set<T>& restrict(const std::set<T>& s) {
        auto iter=this->begin();
        while(iter!=this->end()) { if(s.find(*iter)!=s.end()) { this->erase(iter++); } else { ++iter; } }
        return *this; }
    template<class TT> Set<T>& remove(const std::vector<TT>& l) {
        for(auto iter=l.begin(); iter!=l.end(); ++iter) {
            this->std::set<T>::erase(static_cast<const T&>(*iter)); }
        return *this; }
};
template<class T, Invocable<T> F> inline
Set<InvokeResult<F,T>> apply(F&& f, Set<T> const& set) {
    typedef InvokeResult<F,T> R; Set<R> res; for (auto itm : set) { res.adjoin(f(itm)); } return res; }
template<class T> inline Set<T> join(Set<T> s1, Set<T> const& s2) {
    s1.adjoin(s2); return s1; }
template<class T> inline bool contains(const std::set<T>& s, const T& t) {
    return s.find(t)!=s.end(); }
template<class T> inline bool subset(const std::set<T>& s1, const std::set<T>& s2) {
    for(auto iter=s1.begin(); iter!=s1.end(); ++iter) {
        if(!contains(s2,*iter)) { return false; } } return true; }
template<class T> inline bool disjoint(const std::set<T>& s1, const std::set<T>& s2) {
    for(auto iter=s1.begin(); iter!=s1.end(); ++iter) {
        if(contains(s2,*iter)) { return false; } } return true; }
template<class T> inline bool disjoint(const std::set<T>& s1, const std::vector<T>& l2) {
    for(auto iter=l2.begin(); iter!=l2.end(); ++iter) {
        if(contains(s1,*iter)) { return false; } } return true; }
template<class T> inline Set<T> intersection(const std::set<T>& s1, const std::set<T>& s2) {
    Set<T> r(s1); restrict(r,s2); return r; }
template<class T> inline Set<T> intersection(const std::set<T>& s1, const std::vector<T>& l2) {
    Set<T> r; for(auto iter=l2.begin(); iter!=l2.end(); ++iter) {
        if(contains(s1,*iter)) { r.insert(*iter); } } return r; }
template<class T> inline Set<T> difference(const std::set<T>& s1, const std::set<T>& s2) {
    Set<T> r(s1); remove(r,s2); return r; }
template<class T> inline std::set<T>& remove(std::set<T>& r, const std::set<T>& s) {
    auto iter=r.begin();
    while(iter!=r.end()) { if(contains(s,*iter)) { r.erase(iter++); } else { ++iter; } }
    return r; }
template<class T> inline std::set<T>& restrict(std::set<T>& r, const std::set<T>& s) {
    auto iter=r.begin();
    while(iter!=r.end()) { if(!contains(s,*iter)) { r.erase(iter++); } else { ++iter; } }
    return r; }

template<class T> ostream& operator<<(ostream& os, const std::set<T>& v) {
    bool first=true;
    for(auto x : v) {
        os << (first ? "{" : ",") << x;
        first = false;
    }
    if(first) { os << "{"; }
    return os << "}";
}


template<class K, class T> class Map : public std::map<K,T> {
  public:
    typedef typename std::map<K,T>::iterator Iterator;
    typedef typename std::map<K,T>::const_iterator ConstIterator;
    template<ConvertibleTo<T> TT>
        Map(const std::map<K,TT>& m) : std::map<K,T>(m.begin(),m.end()) { }
    using std::map<K,T>::map;
    using std::map<K,T>::insert;
    T& operator[](K k) { return this->std::map<K,T>::operator[](k); }
    const T& operator[](K k) const { auto iter=this->find(k); HELPER_ASSERT(iter!=this->end()); return iter->second; }
    const T& get(const K& k) const { auto i=this->find(k);
        HELPER_ASSERT(i!=this->end()); return i->second; }
    bool has_key(const K& k) const {
        return this->find(k)!=this->end(); }
    T& value(const K& k) {
        auto iter=this->find(k);
        HELPER_ASSERT(iter!=this->end()); return iter->second; }
    const T& value(const K& k) const {
        auto iter=this->find(k);
        HELPER_ASSERT(iter!=this->end()); return iter->second; }
    void insert(const std::pair<K,T>& kv) {
        this->std::map<K,T>::insert(kv); }
    void insert(const K& k, const T& v) {
        this->std::map<K,T>::insert(std::make_pair(k,v)); }
    void adjoin(const std::map<K,T>& m) {
        for(auto i=m.begin(); i!=m.end(); ++i) { this->insert(*i); } }
    void remove_keys(const Set<K>& s) {
        for(auto iter=s.begin(); iter!=s.end(); ++iter) { this->erase(*iter); } }
    Set<K> keys() const {
        Set<K> res; for(auto iter=this->begin(); iter!=this->end(); ++iter) {
            res.insert(iter->first); } return res; }
    List<T> values() const {
        List<T> res; for(auto iter=this->begin(); iter!=this->end(); ++iter) {
            res.append(iter->second); } return res; }
};
template<class K, class T> inline Map<K,T> join(Map<K,T> m1, Map<K,T> const& m2) {
    m1.adjoin(m2); return m1; }
template<class I, class X, class J> inline X& insert(Map<I,X>& m, const J& k, const X& v) {
    return m.std::template map<I,X>::insert(std::make_pair(k,v)).first->second; }
template<class K, class T> Map<K,T> restrict_keys(const std::map<K,T>& m, const std::set<K>& k) {
    Map<K,T> result; const Set<K>& keys=static_cast<const Set<K>&>(k);
    for(auto item_iter=m.begin(); item_iter!=m.end(); ++item_iter) {
        if(keys.contains(item_iter->first)) { result.insert(*item_iter); } } return result; }
template<class K, class T> ostream& operator<<(ostream& os, const std::map<K,T>& m) {
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

} // namespace Helper

#endif
