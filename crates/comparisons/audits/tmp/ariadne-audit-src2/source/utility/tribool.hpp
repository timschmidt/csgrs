/***************************************************************************
 *            utility/tribool.hpp
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

/*! \file utility/tribool.hpp
 *  \brief Three-valued logic variable.
 */

#include "foundations/logical.hpp"

/*
#ifdef DOXYGEN
namespace Ariadne {
    //! \brief A three-valued logic type, with values \f$\top\f$ (true), \f$\bot\f$ (false) and \f$\uparrow\f$ (indeterminate).
    class ValidatedKleenean {
    //! \brief Returns \c true if \a tb is \c true, and \c false if \a tb is \c indeterminate or \c false.
    friend bool definitely(ValidatedKleenean tb);
    //! \brief Returns \c true if \a tb is \c true or \c indeteriminate, and \c false if \a tb is \c false.
    friend bool possibly(ValidatedKleenean tb);
    //! \brief Returns \c true if \a tb is \c indeterminate, and \c false if \a tb is \c true or \c false.
    friend bool indeterminate(ValidatedKleenean tb);
    };
}
#endif // DOXYGEN
*/
