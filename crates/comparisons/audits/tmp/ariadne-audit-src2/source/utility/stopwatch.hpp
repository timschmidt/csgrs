/***************************************************************************
 *            utility/stopwatch.hpp
 *
 *  Copyright  2018-20  Luca Geretti
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

/*! \file utility/stopwatch.hpp
 *  \brief Stopwatch class to be used for profiling execution times.
 */

#ifndef ARIADNE_STOPWATCH_HPP
#define ARIADNE_STOPWATCH_HPP

#include <chrono>
#include "typedefs.hpp"

namespace Ariadne {

using Seconds = std::chrono::seconds;
using Milliseconds = std::chrono::milliseconds;
using Microseconds = std::chrono::microseconds;

template<class D> class Stopwatch {
public:
    using ResolutionType = std::chrono::high_resolution_clock;
    using TimePointType = std::chrono::time_point<ResolutionType>;

    Stopwatch() { restart(); }

    //! \brief Get the duration in the given type
    D duration() const { return std::chrono::duration_cast<D>(_clicked-_initial); }
    //! \brief Get the duration in seconds, in double precision
    Double elapsed_seconds() const { return std::chrono::duration_cast<std::chrono::duration<Double>>(duration()).count(); }

    //! \brief Restart the watch time to zero
    Stopwatch& restart() { _initial = ResolutionType::now(); _clicked = _initial; return *this; }
    //! \brief Save the current time
    Stopwatch& click() { _clicked = ResolutionType::now(); return *this; }

private:
    TimePointType _initial;
    TimePointType _clicked;
};

} // namespace Ariadne

#endif /* ARIADNE_STOPWATCH_HPP */
