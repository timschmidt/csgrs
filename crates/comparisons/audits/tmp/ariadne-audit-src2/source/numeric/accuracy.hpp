/***************************************************************************
 *            numeric/accuracy.hpp
 *
 *  Copyright  2019-20  Pieter Collins
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

/*! \file numeric/accuracy.hpp
 *  \brief A class representing a bound on the accuracy of a computation in a metric space.
 */

#ifndef ARIADNE_ACCURACY_HPP
#define ARIADNE_ACCURACY_HPP

#include <iosfwd>
#include "dyadic.hpp"
#include "bits.hpp"

namespace Ariadne {

using OutputStream = std::ostream;

class Bits;

//! \ingroup NumericModule
//! \brief The accuracy of a computation of a value in a metric space.
class Accuracy {
    Dyadic _error;
  public:
    //! <p/>
    Accuracy(Bits precision);
    //! <p/>
    Accuracy(Dyadic error) : _error(error) { }
    //! <p/>
    Dyadic const& error() const { return this->_error; }
    //! <p/>
    friend OutputStream& operator<<(OutputStream& os, Accuracy const& acc);
};
//! \relates Accuracy \brief <p/>
inline Accuracy accuracy(Bits acc) { return Accuracy(acc); }

} // namespace Ariadne

#endif // ARIADNE_ACCURACY_HPP
