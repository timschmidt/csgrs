/***************************************************************************
 *            numeric/reals.hpp
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

/*! \file numeric/reals.hpp
 *  \brief Real numbers, including lower, upper and naive types, and validated versions.
 */



#ifndef ARIADNE_REALS_HPP
#define ARIADNE_REALS_HPP


#include "utility/typedefs.hpp"
#include "utility/pointer.hpp"
#include "utility/handle.hpp"

#include "foundations/logical.decl.hpp"
#include "numeric/number.decl.hpp"
#include "numeric/float.decl.hpp"

namespace Ariadne {

class ApproximateReal;
class ApproximateRealInterface;

//! \ingroup NumericModule
//! \brief A generic class representing an approximation to a real number
//! of unknown accuracy.
//! \see Real, ValidatedReal
class ApproximateReal
    : public Handle<ApproximateRealInterface>
{
  public:
    typedef ApproximateRealInterface Interface;

    ApproximateReal(DyadicApproximation const&);
    //! \brief Get a dyadic approximation to the number.
    DyadicApproximation get() const;
    //! \brief Get the approximation to the number, converting to double-precision.
    FloatDPApproximation get(DoublePrecision) const;
    //! \brief Get the approximation to the number, converting to a representation
    //! with precision \a pr.
    //! Note that increasing the precision typically does not yield arbitrarily
    //! accurate approximation, as the object already has some error.
    FloatMPApproximation get(MultiplePrecision pr) const;
    //! \brief Write to an output stream.
    friend OutputStream& operator<<(OutputStream&, ApproximateReal const&);
};


} // namespace Ariadne

#endif
