/***************************************************************************
 *            utility/randomiser.hpp
 *
 *  Copyright  2011-20  Luca Geretti
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

/*! \file utility/randomiser.hpp
 *  \brief Generators of random numbers for a type.
 *  \details The values are generated uniformly in the provided interval.
 */

#ifndef ARIADNE_RANDOMISER_HPP
#define ARIADNE_RANDOMISER_HPP

#include <cstdlib>
#include <time.h>
#include "typedefs.hpp"
#include "numeric/builtin.hpp"

namespace Ariadne {

template<class T> struct Randomiser;

template<> struct Randomiser<ExactDouble> {
    //! \get Return a value between 0 and \a value
    static ExactDouble get(ExactDouble const& min, ExactDouble const& max) {
        double min_d = min.get_d();
        double max_d = max.get_d();
        return ExactDouble((max_d-min_d)*rand()/RAND_MAX + min_d);
    }
};

template<> struct Randomiser<DegreeType> {
    //! \get Return a value between 0 and \a value
    static DegreeType get(DegreeType const& min, DegreeType const& max) {
        return DegreeType(min + rand() % (max-min+1));
    }
};

} // namespace Ariadne

inline bool _init_randomiser() {
    srand(time(nullptr));
    return true;
}

static const bool init_randomiser = _init_randomiser();

#endif /* ARIADNE_RANDOMISER_HPP */
