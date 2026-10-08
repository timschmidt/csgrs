/***************************************************************************
 *            function/projection.hpp
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

/*! \file function/projection.hpp
 *  \brief Projections onto coordinates
 */

#ifndef ARIADNE_PROJECTION_HPP
#define ARIADNE_PROJECTION_HPP

#include "utility/typedefs.hpp"

namespace Ariadne {

//! A projection onto given coordinates.
class Projection
{
    SizeType _as;
    Array<SizeType> _ind;
  public:
    Projection(SizeType as, Array<SizeType> ind) : _as(as), _ind(ind) {
        for(SizeType i=0; i!=this->_ind.size(); ++i) { ARIADNE_PRECONDITION(_ind[i]<_as); } }
    SizeType result_size() const { return this->_ind.size(); }
    SizeType argument_size() const { return this->_as; }
    Array<SizeType> const& indices() const { return this->_ind; }
    SizeType const& operator[](SizeType i) const { return this->_ind[i]; }
    SizeType index(SizeType i) const { return this->_ind[i]; }

    template<class X> Vector<X> operator() (Vector<X> const& v) const {
        Vector<X> r(this->result_size(),v.zero_element());
        for(SizeType i=0; i!=this->result_size(); ++i) { r[i]=v[this->_ind[i]]; }
        return r; }
    friend OutputStream& operator<<(OutputStream& os, Projection const& prj) {
        return os << "Projection("<<prj._ind<<")"; }
};


} // namespace Ariadne


#endif // ARIADNE_FORMULA_HPP
