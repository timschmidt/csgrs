/***************************************************************************
 *            main.hpp
 *
 *  Copyright  2008-21  Luca Geretti
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

/*! \file ariadne_main.hpp
 *  \brief Main function to be used when creating executables that exploit the CLI.
 */

#ifndef ARIADNE_ARIADNE_MAIN_HPP
#define ARIADNE_ARIADNE_MAIN_HPP

#include "ariadne.hpp"

using namespace Ariadne;

//! \brief Wrapping function for actual main, to be defined in executable file
void ariadne_main();

//! \brief Call to main with proper harness, handling CLI arguments
int main(int argc, const char* argv[])
{
    if (CommandLineInterface::instance().acquire(argc,argv)) {
        ariadne_main();
    }
}

#endif // ARIADNE_ARIADNE_MAIN_HPP
