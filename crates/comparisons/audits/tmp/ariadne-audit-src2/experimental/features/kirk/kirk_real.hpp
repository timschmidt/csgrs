/***************************************************************************
 *            external/kirk_real.hpp
 *
 *  Copyright  2017-20  Pieter Collins, Franz Brausse
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received _a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

/*! \file external/kirk_real.hpp
 *  \brief Import and export of real numbers in Kirk format into Ariadne Real.
 */

#ifndef ARIADNE_KIRK_REAL_HPP
#define ARIADNE_KIRK_REAL_HPP

extern "C" {
struct kirk_real_t;
}

namespace Ariadne {

class Real;

kirk_real_t* to_kirk_real(Real const& r);
Real from_kirk_real(kirk_real_t*);
void delete_kirk_real(kirk_real_t*);

} //namespace Ariadne

#endif /* ARIADNE_KIRK_REAL_HPP */
