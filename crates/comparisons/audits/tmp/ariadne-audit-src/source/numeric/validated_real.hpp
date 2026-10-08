/***************************************************************************
 *            numeric/validated_real.hpp
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

/*! \file numeric/validated_real.hpp
 *  \brief Validated real numbers
 */



#ifndef ARIADNE_VALIDATED_REAL_HPP
#define ARIADNE_VALIDATED_REAL_HPP


#include "utility/typedefs.hpp"
#include "utility/pointer.hpp"
#include "utility/handle.hpp"

#include "foundations/logical.decl.hpp"
#include "numeric/number.decl.hpp"
#include "numeric/float.decl.hpp"

#include "numeric/dyadic.hpp"

namespace Ariadne {

class ValidatedRealInterface;

//! \ingroup NumericModule
//! \brief A generic class representing rigorous bounds on a real number.
//! \see Real
class ValidatedReal
    : public Handle<const ValidatedRealInterface>
{
  public:
    typedef ValidatedRealInterface Interface;

    ValidatedReal(DyadicBounds const&);
    operator DyadicBounds() const;

    Dyadic value();
    Dyadic error();
    Dyadic lower();
    Dyadic upper();

    //! \brief Get dyadic bounds for the number.
    //! \details It is not always clear how this function can be implemented,
    //!   and it may be removed or modified in future versions.
    DyadicBounds get() const;
    //! \brief Get the bounds for the number, representing in double precision.
    FloatDPBounds get(DoublePrecision) const;
    //! \brief Get the bounds for the number, representing to precision \a pr.
    //! Note that increasing the accuracy typically does not yield arbitrarily
    //! tight bounds, as the object is already an approximation to finite accuracy.
    FloatMPBounds get(MultiplePrecision pr) const;
    //! \brief Write to an output stream.
    friend OutputStream& operator<<(OutputStream&, ValidatedReal const&);

    friend ValidatedKleenean sgn(DyadicBounds const& r);
    friend ValidatedKleenean operator==(DyadicBounds const& vr1, DyadicBounds const& vr2);
    friend ValidatedKleenean operator!=(DyadicBounds const& vr1, DyadicBounds const& vr2);
    friend ValidatedKleenean operator<=(DyadicBounds const& vr1, DyadicBounds const& vr2);
    friend ValidatedKleenean operator>=(DyadicBounds const& vr1, DyadicBounds const& vr2);
    friend ValidatedKleenean operator< (DyadicBounds const& vr1, DyadicBounds const& vr2);
    friend ValidatedKleenean operator> (DyadicBounds const& vr1, DyadicBounds const& vr2);
};

} // namespace Ariadne

#endif
