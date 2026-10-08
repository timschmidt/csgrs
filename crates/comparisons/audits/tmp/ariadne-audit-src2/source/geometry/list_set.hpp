/***************************************************************************
 *            geometry/list_set.hpp
 *
 *  Copyright  2008-20  Alberto Casagrande, Pieter Collins
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

/*! \file geometry/list_set.hpp
 *  \brief Sets which are lists of simple shapes.
 */

#ifndef ARIADNE_LIST_SET_HPP
#define ARIADNE_LIST_SET_HPP


#include <vector>
#include "helper/stlio.hpp"
#include "utility/macros.hpp"

#include "io/graphics_interface.hpp"
#include "hybrid/discrete_location.hpp"

#include "geometry/box.hpp"

namespace Ariadne {

template<class ES> class ListSet;
class DiscreteLocation;

struct ListSetSummary { SizeType size; DimensionType dimension; };

/*! \ingroup ListSetSubModule
 *  \brief A set described as the union of sets in a list of basic sets.
 */
template<class BS>
class ListSet
//    : public Drawable2dInterface
{
  private:
    std::vector<BS> _data;

  public:
    typedef typename std::vector<BS>::const_iterator ConstIterator;
    typedef typename std::vector<BS>::iterator Iterator;
    typedef BS value_type;

    virtual ~ListSet() = default;

    ListSet() { };
    explicit ListSet(DimensionType d) { };
    explicit ListSet(const BS& bs) { this->adjoin(bs); }
    template<class BST> ListSet(const ListSet<BST>& ls) {
        this->_data.insert(this->end(),ls.begin(),ls.end()); }
    ListSet(const std::vector<BS>& lst) {
        this->_data.insert(this->end(),lst.begin(),lst.end()); }
    template<class Iter> ListSet(Iter first, Iter last) {
        this->_data.insert(this->end(),first,last); }

    ListSet<BS>* clone() const { return new ListSet<BS>(*this); }

    /*! \brief Tests if the sets are equal. Tests equality of sequences,
     *  including ordering. */
    Bool operator==(const ListSet<BS>& other) const { return this->_data==other._data; }


    /*! \brief Returns true if the list is empty. */
    Bool empty() const { return this->_data.empty(); }

    /*! \brief Returns the number of basic sets forming this object. */
    SizeType size() const { return this->_data.size(); }

    /*! \brief Returns the number of basic sets which can be stored without reallocating memory. */
    SizeType capacity() const { return this->_data.capacity(); }

    /*! \brief Accesses the i-th BasicSet. */
    const BS& operator[](SizeType i) const { return this->_data[i]; };

    /*! \brief Make the set empty. */
    Void clear() { this->_data.clear(); }

    /*! \brief A constant Iterator to the beginning of the list of basic sets. */
    ConstIterator begin() const { return this->_data.begin(); }

    /*! \brief A constant Iterator to the end of the list of basic sets. */
    ConstIterator end() const { return this->_data.end(); };

    /*! \brief A Iterator to the beginning of the list of basic sets. */
    Iterator begin() { return this->_data.begin(); }

    /*! \brief A Iterator to the end of the list of basic sets. */
    Iterator end() { return this->_data.end(); };

    /*! \brief Returns the denotable set's space dimension. */
    DimensionType dimension() const { if(this->empty()) { return 0; } else { return this->_data.back().dimension(); } }

    /*! \brief Removes a set from the list and return it. */
    BS pop() { BS result=this->_data.back(); this->_data.pop_back(); return result; }

    /*! \brief Removes a set given identified by an Iterator from the list. */
    Iterator erase(Iterator iter) { return this->_data.erase(iter); }

    /*! \brief Pushes a basic set to the end of the list. */
    Void push_back(const BS& bs) { this->_data.push_back(bs); }

    /*! \brief Adjoins (makes union with) a basic set. */
    Void adjoin(const BS& bs) {
        ARIADNE_ASSERT(this->empty() || this->_data.back().dimension()==bs.dimension());
        this->_data.push_back(bs); }

    /*! \brief Adjoins (makes union with) another list set. */
    Void adjoin(const ListSet<BS>& ls) {
        ARIADNE_ASSERT(this->empty() || ls.empty() || this->_data.back().dimension()==ls._data.back().dimension());
        this->_data.insert(this->_data.end(),ls.begin(),ls.end()); }

    /*! \brief compute a list of the bounding boxes of the set elements. */
    ListSet<UpperBoxType> bounding_boxes() const {
        ListSet<UpperBoxType> result(this->dimension());
        for(SizeType i=0; i!=this->size(); ++i) {
            result.adjoin((*this)[i].bounding_box());
        }
        return result;
    }

    /*! \brief A bounding box for the whole set. */
    UpperBoxType bounding_box() const {
        if(this->size()==0) { return UpperBoxType(this->dimension()); }
        UpperBoxType result((*this)[0].bounding_box());
        for(SizeType i=1; i!=this->size(); ++i) {
            result=hull(result,(*this)[i].bounding_box()); }
        return result;
    }

    /*! \brief Draw on a canvas.
     *
     *  The ListSet template does not implement the Drawable2dInterface to avoid a dependency on the geometry/box.hpp header file.
     */
    template<class PRJ> Void draw(CanvasInterface& c, const PRJ& p) const {
        for(SizeType i=0; i!=this->size(); ++i) {
            (*this)[i].draw(c,p);
        }
    }

    /*! \brief Write to an output stream. */
    OutputStream& _write(OutputStream& os) const {
        return os << *this;
    }

    /*! \brief Returns a summary of the size and dimension. */
    ListSetSummary summary() const { ListSetSummary summary = { this->size(),this->dimension() }; return summary; }

};

inline OutputStream& operator<<(OutputStream& os, const ListSetSummary& lss) {
    return os << "ListSet( s="<<lss.size<<", d="<<lss.dimension<<")"; }

template<class BS>
OutputStream&
operator<<(OutputStream& os, const ListSet<BS>& ls)
{
    os << "ListSet(";
    if(!ls.empty()) { for(SizeType i=0; i!=ls.size(); ++i) { os << (i==0?" ":", ") << ls[i]; } }
    return os << " )";
}



} // namespace Ariadne

#endif /* ARIADNE_LIST_SET_HPP */
