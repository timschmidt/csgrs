/***************************************************************************
 *            utility/uniform_list.hpp
 *
 *  Copyright  2024-25  Pieter Collins
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

/*! \brief \file utility/uniform_list.hpp
 *  \brief
 */

#ifndef ARIADNE_UNIFORM_LIST_HPP
#define ARIADNE_UNIFORM_LIST_HPP

#include <iostream>
#include <initializer_list>
#include <vector>

#include "uniform_array.hpp"

namespace Ariadne {

template<class T> class UniformList;

template<class T> requires (not HasCharacteristics<T>) class UniformList<T>
    : public List<T>
{
    T _zero_element;
  public:
    using List<T>::List;
    UniformList(Tuple<>) : List<T>() { }
    Tuple<> element_characteristics() const { return Tuple<>(); }
    T const& zero_element() const { return this->_zero_element; }
};

template<class T> requires HasCharacteristics<T> class UniformList<T>
    : public List<T>
{
    T _zero_element;
    static T* uninitialized_new(SizeType n) { return static_cast<T*>(::operator new(n*sizeof(T))); }
  public:
    using Reference = typename List<T>::Reference;
    using ConstReference = typename List<T>::ConstReference;
    using Pointer = typename List<T>::Pointer;
    using ConstPointer = typename List<T>::ConstPointer;
    using Iterator = typename List<T>::Iterator;
    using ConstIterator = typename List<T>::ConstIterator;

    explicit UniformList(CharacteristicsType<T> prps)
        : _zero_element(make_zero_from_characteristics<T>(prps)) { }
    explicit UniformList(SizeType n, CharacteristicsType<T> prps)
        : List<T>(n,make_zero_from_characteristics<T>(prps)), _zero_element(make_zero_from_characteristics<T>(prps)) { }
    explicit UniformList(SizeType n, T const& t)
        : List<T>(n,t), _zero_element(nul(t)) { }
    explicit UniformList(List<T> lst)
            : _zero_element((assert(!lst.empty()),nul(lst[0]))) {
        for(auto val : lst) { this->append(val); } }
    UniformList<T> const& append(T const& t) {
        assert(get_characteristics(t)==this->element_characteristics()); this->List<T>::append(t); return *this; }
    CharacteristicsType<T> element_characteristics() const {
        return get_characteristics(this->_zero_element); }
    T const& zero_element() const {
        return this->_zero_element; }
    Void resize(SizeType n) {
        if(this->size()!=n) {
            List<T> new_lst; new_lst.reserve(n);
            for(SizeType i=0; i!=n; ++i) {
                if(i<this->size()) { new_lst.append((*this)[i]); }
                else { new_lst.append(this->zero_element()); } }
            std::swap(static_cast<List<T>&>(*this),new_lst);
        }
    }
};


} // namespace Ariadne

#endif /* ARIADNE_UNIFORM_LIST_HPP */
