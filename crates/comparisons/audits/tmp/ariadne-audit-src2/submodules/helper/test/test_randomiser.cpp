/***************************************************************************
 *            test_randomiser.cpp
 *
 *  Copyright  2023  Luca Geretti
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

#include "randomiser.hpp"
#include "container.hpp"

#include "test.hpp"

using namespace Helper;
using namespace std;

class TestRandomiser {
  private:
    size_t _num_tries;
  public:

    TestRandomiser(size_t num_tries) : _num_tries(num_tries) { }

    void test_int() {
        RandomGenerator generator;

        auto rnd = UniformIntRandomiser<unsigned int>(0,255);

        List<unsigned int> values;
        for (size_t i=0; i<_num_tries; ++i)
            values.push_back(rnd.get());

        HELPER_TEST_PRINT(values)
    }

    void test_real() {
        RandomGenerator generator;

        auto rnd = UniformRealRandomiser<double>(0.0,1.0);

        List<double> values;
        for (size_t i=0; i<_num_tries; ++i)
            values.push_back(rnd.get());

        HELPER_TEST_PRINT(values)
    }

    void test() {
        HELPER_TEST_CALL(test_int());
        HELPER_TEST_CALL(test_real());
    }

};

int main() {
    TestRandomiser(10).test();
    return HELPER_TEST_FAILURES;
}
