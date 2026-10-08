/***************************************************************************
 *            test_stopwatch.cpp
 *
 *  Copyright  2009-21  Luca Geretti
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

#include <iostream>
#include <thread>
#include "config.hpp"
#include "utility/stopwatch.hpp"

#include "../test.hpp"

using namespace Ariadne;
using namespace std::chrono_literals;

class TestStopwatch {
  public:

    Void test_create() {
        Stopwatch<Milliseconds> sw;
    }

    Void test_duration() {
        Stopwatch<Microseconds> sw;
        std::this_thread::sleep_for(10ms);
        auto duration = sw.click().duration();
        ARIADNE_TEST_ASSERT(duration.count()>10000);
        ARIADNE_TEST_ASSERT(sw.elapsed_seconds() > 0.01);
    }

    Void test() {
        ARIADNE_TEST_CALL(test_create());
        ARIADNE_TEST_CALL(test_duration());
    }

};

Int main() {
    TestStopwatch().test();
    return ARIADNE_TEST_FAILURES;
}
