/***************************************************************************
 *            stopwatch.hpp
 *
 *  Copyright  2018-20  Luca Geretti
 *
 ****************************************************************************/

/*
 * This file is part of Helper, under the MIT license.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

/*! \file stopwatch.hpp
 *  \brief Stopwatch class to be used for profiling execution times.
 */

#ifndef HELPER_STOPWATCH_HPP
#define HELPER_STOPWATCH_HPP

#include <chrono>

namespace Helper {

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
    double elapsed_seconds() const { return std::chrono::duration_cast<std::chrono::duration<double>>(duration()).count(); }

    //! \brief Restart the watch time to zero
    Stopwatch& restart() { _initial = ResolutionType::now(); _clicked = _initial; return *this; }
    //! \brief Save the current time
    Stopwatch& click() { _clicked = ResolutionType::now(); return *this; }

private:
    TimePointType _initial;
    TimePointType _clicked;
};

} // namespace Helper

#endif /* HELPER_STOPWATCH_HPP */
