/***************************************************************************
 *            examples/external/kirk_real_demo.cpp
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

#include "kirk_real.hpp"
#include "numeric/real.hpp"
#include "numeric/floatmp.hpp"
#include "numeric/float_bounds.hpp"

using namespace Ariadne;

int main() {

    // Construct a real number; here r = pi.
    Real r=6*atan(rec(sqrt(Real(3))));

    // Construct a Kirk real from and Ariadne real.
    kirk_real_t* kr = to_kirk_real(r);

    // Construct an Ariadne real from the Kirk real.
    Real akr=from_kirk_real(kr);

    // Output to 128 bits of precision.
    std::cout << akr.get(precision(128)) << std::endl;

    // Delete the memory used by the Kirk real.
    delete_kirk_real(kr);
}
