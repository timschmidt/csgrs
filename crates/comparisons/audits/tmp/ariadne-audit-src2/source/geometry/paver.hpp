/***************************************************************************
 *            geometry/paver.hpp
 *
 *  Copyright  2011-20  Pieter Collins
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

/*! \file geometry/paver.hpp
 *  \brief Class for computing outer approximations to nonlinear sets.
 */

#ifndef ARIADNE_PAVER_HPP
#define ARIADNE_PAVER_HPP

#include "geometry/paver_interface.hpp"

namespace Ariadne {

class Paver
    : public Handle<const PaverInterface>
{
  public:
    typedef PaverInterface::SetType SetType;
    using Handle<const PaverInterface>::Handle;
    Void adjoin_outer_approximation(PavingInterface& paving, const SetType& set, Nat fineness) const {
        this->_ptr->adjoin_outer_approximation(paving,set,fineness); }
    friend OutputStream& operator<<(OutputStream& os, Paver const& pv) { return pv._ptr->_write(os); }
};

//! \brief A class for computing outer approximations to sets defined by functions.
class AffinePaver : public PaverInterface
{
  public:
    Void adjoin_outer_approximation(PavingInterface& paving, const SetType& set, Nat fineness) const;
    OutputStream& _write(OutputStream& os) const;
};

//! \brief A class for computing outer approximations to sets defined by functions.
class SubdivisionPaver : public PaverInterface
{
  public:
    Void adjoin_outer_approximation(PavingInterface& paving, const SetType& set, Nat fineness) const;
    Void adjoin_outer_approximation_recursion(PavingInterface& paving, ValidatedConstrainedImageSet const& set, Nat fineness, const Vector<FloatDP>& max_errors) const;
    OutputStream& _write(OutputStream& os) const;
};

//! \brief A class for computing outer approximations to sets defined by functions.
class ReducePaver : public PaverInterface
{
  public:
    Void adjoin_outer_approximation(PavingInterface& paving, const SetType& set, Nat fineness) const;
    OutputStream& _write(OutputStream& os) const;
};

//! \brief A class for computing outer approximations to sets defined by functions.
class ConstraintPaver : public PaverInterface
{
  public:
    Void adjoin_outer_approximation(PavingInterface& paving, const SetType& set, Nat fineness) const;
    OutputStream& _write(OutputStream& os) const;
};

//! \brief A class for computing outer approximations to sets defined by functions.
class OptimalConstraintPaver : public PaverInterface
{
  public:
    Void adjoin_outer_approximation(PavingInterface& paving, const SetType& set, Nat fineness) const;
    OutputStream& _write(OutputStream& os) const;
};

} //namespace Ariadne

#endif /* ARIADNE_PAVER_HPP */
