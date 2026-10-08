/***************************************************************************
 *            arch_suite.cpp
 *
 *  Copyright  2020  Luca Geretti
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

#include "ROBE25.hpp"
#include "CVDP23.hpp"
#include "LALO20.hpp"
#include "LOVO25.hpp"
#include "SPRE22.hpp"
#include "ariadne_main.hpp"

void ariadne_main()
{
    ROBE25();
    CVDP23();
    LALO20();
    LOVO25();
    SPRE22();
}
