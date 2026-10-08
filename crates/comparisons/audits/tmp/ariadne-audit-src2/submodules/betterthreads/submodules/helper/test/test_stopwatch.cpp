/***************************************************************************
 *            test_stopwatch.cpp
 *
 *  Copyright  2009-21  Luca Geretti
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

#include <iostream>
#include <thread>
#include "stopwatch.hpp"

#include "test.hpp"

using namespace Helper;
using namespace std::chrono_literals;

class TestStopwatch {
  public:

    void test_create() {
        Stopwatch<Milliseconds> sw;
    }

    void test_duration() {
        Stopwatch<Microseconds> sw;
        std::this_thread::sleep_for(10ms);
        auto duration = sw.click().duration();
        HELPER_TEST_ASSERT(duration.count()>10000);
        HELPER_TEST_ASSERT(sw.elapsed_seconds() > 0.01);
    }

    void test() {
        HELPER_TEST_CALL(test_create());
        HELPER_TEST_CALL(test_duration());
    }

};

int main() {
    TestStopwatch().test();
    return HELPER_TEST_FAILURES;
}
