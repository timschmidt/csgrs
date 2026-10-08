/***************************************************************************
 *            iterator.hpp
 *
 *  Copyright  2013-20  Pieter Collins
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

/*! \file iterator.hpp
 *  \brief Iterator support, similar to Boost's iterator package.
 */

#ifndef HELPER_ITERATOR_HPP
#define HELPER_ITERATOR_HPP

#include <iterator>

namespace Helper {

using std::ostream;

struct RandomAccessTraversalTag { };
struct ForwardTraversalTag { };

template<class I, class Val, class Cat, class Ref=Val&> class IteratorFacade;

class IteratorCoreAccess {
    template<class I, class Val, class Cat, class Ref> friend class IteratorFacade;

    template<class I, class II> static bool equal(I const& self, II const& other) { return self.equal(other); }
    template<class I, class II> static std::ptrdiff_t distance_to(I const& self, II const& other) { return self.distance_to(other); }
    template<class I> static void increment(I& self) { self.increment(); }
    template<class I> static void advance(I& self, std::ptrdiff_t n) { self.advance(n); }
    template<class I> static typename I::reference dereference(I const& self) { return self.dereference(); }
};


template<class I, class Val, class Ref> class IteratorFacade<I,Val,RandomAccessTraversalTag,Ref> {
    typedef Val* Ptr; typedef RandomAccessTraversalTag Cat;
  private:
    static bool _equal(const I& self, const I& other) { return IteratorCoreAccess::equal(self,other); }
    static std::ptrdiff_t _distance_to(const I& self, const I& other) { return IteratorCoreAccess::distance_to(self,other); }
  public:
    // Standard typedefs
    typedef typename std::remove_const<Val>::type value_type;
    typedef Ref reference;
    typedef std::ptrdiff_t difference_type;
    typedef std::random_access_iterator_tag iterator_category;
    typedef Val* pointer;

    // Ariadne typedefs
    typedef typename std::remove_const<Val>::type ValueType;
    typedef Ref Reference;

    friend bool operator==(const I& self, const I& other) { return _equal(self,other); }
    friend bool operator!=(const I& self, const I& other) { return !_equal(self,other); }
    friend bool operator< (const I& self, const I& other) { return _distance_to(self,other)> 0; }
    friend bool operator<=(const I& self, const I& other) { return _distance_to(self,other)>=0; }
    friend bool operator> (const I& self, const I& other) { return _distance_to(self,other)< 0; }
    friend bool operator>=(const I& self, const I& other) { return _distance_to(self,other)<=0; }
    std::ptrdiff_t operator-(const I& other) const { return -IteratorCoreAccess::distance_to(static_cast<const I&>(*this),other); }
    I operator+(std::ptrdiff_t n) const { I result(static_cast<const I&>(*this)); IteratorCoreAccess::advance(result,n); return result; }
    I operator-(std::ptrdiff_t n) const { I result(static_cast<const I&>(*this)); IteratorCoreAccess::advance(result,-n); return result; }
    I& operator+=(std::ptrdiff_t n) { IteratorCoreAccess::advance(static_cast<I&>(*this),n); return static_cast<I&>(*this); }
    I& operator-=(std::ptrdiff_t n) { IteratorCoreAccess::advance(static_cast<I&>(*this),-n); return static_cast<I&>(*this); }
    I& operator++() { IteratorCoreAccess::advance(static_cast<I&>(*this),1); return static_cast<I&>(*this); }
    I& operator--() { IteratorCoreAccess::advance(static_cast<I&>(*this),-1); return static_cast<I&>(*this); }
    Ref operator*() const { return IteratorCoreAccess::dereference(static_cast<const I&>(*this)); }
    Ptr operator->() const { return &IteratorCoreAccess::dereference(static_cast<const I&>(*this)); }
};

template<class I, class Val, class Ref> class IteratorFacade<I,Val,ForwardTraversalTag, Ref> {
    typedef Val* Ptr;
  private:
    static bool _equal(const I& self, const I& other) { return IteratorCoreAccess::equal(self,other); }
  public:
    // Standard typedefs
    typedef typename std::remove_const<Val>::type value_type;
    typedef Ref reference;
    typedef std::ptrdiff_t difference_type;
    typedef std::forward_iterator_tag iterator_category;
    typedef Val* pointer;

    // Ariadne typedefs
    typedef typename std::remove_const<Val>::type ValueType;
    typedef Ref Reference;

    friend bool operator==(const I& self, const I& other) { return _equal(self,other); }
    friend bool operator!=(const I& self, const I& other) { return !_equal(self,other); }
    I& operator++() { IteratorCoreAccess::increment(static_cast<I&>(*this)); return static_cast<I&>(*this); }
    I operator++(int) { I r=static_cast<const I&>(*this); IteratorCoreAccess::increment(static_cast<I&>(*this)); return r; }
    Ref operator*() const { return IteratorCoreAccess::dereference(static_cast<const I&>(*this)); }
    Ptr operator->() const { return &IteratorCoreAccess::dereference(static_cast<const I&>(*this)); }
};

template<class I1, class I2> class PairIterator
//    : public IteratorFacade<PairIterator<I1,I2>, MultivariateMonomial<X>, RandomAccessTag, MultivariateMonomialReference<X>>
{
    typedef std::pair<I1*,I2*> Reference;
  public:
    I1 _iter1;
    I2 _iter2;
  public:
    typedef std::ptrdiff_t DifferenceType;

    PairIterator(I1 iter1, I2 iter2);
    template<class II1, class II2> PairIterator(const PairIterator<II1,II2>&);
  public:
    template<class II1, class II2> bool equal(const PairIterator<II1,II2>&) const;
    template<class II1, class II2> DifferenceType distance_to(const PairIterator<II1,II2>&) const;
    void advance(DifferenceType k);
    Reference dereference();
};
template<class I1, class I2> inline PairIterator<I1,I2>::PairIterator(I1 iter1, I2 iter2)
    : _iter1(iter1), _iter2(iter2) { }
template<class I1, class I2> template<class II1, class II2> inline PairIterator<I1,I2>::PairIterator(const PairIterator<II1,II2>& iter)
    : _iter1(iter._iter1), _iter2(iter._iter2) { }
template<class I1, class I2> template<class II1, class II2> inline bool PairIterator<I1,I2>::equal(const PairIterator<II1,II2>& other) const {
    return this->_iter1 == other._iter1 && this->_iter2 == other._iter2; }
template<class I1, class I2> inline void PairIterator<I1,I2>::advance(DifferenceType k) {
    _iter1+=(k), _iter2+=(k); }
template<class I1, class I2> inline auto PairIterator<I1,I2>::dereference() -> Reference {
    return std::make_pair(*this->_iter1,*this->_iter2); }
template<class I1, class I2> ostream& operator<<(ostream& os, const PairIterator<I1,I2>& e) {
    return os << "{" << e._iter1 << "," << e._iter2 << "}"; }

} // namespace Helper

#endif /* HELPER_ITERATOR_HPP */
