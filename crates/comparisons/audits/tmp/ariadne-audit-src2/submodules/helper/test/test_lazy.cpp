/***************************************************************************
 *            test_lazy.cpp
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
#include "lazy.hpp"

#include "test.hpp"

using namespace Helper;

class TestClass {
  public:
    TestClass(double a) {
        _value = a*a;
        HELPER_TEST_PRINT("TestClass object created")
    }

    double value() { return _value; }

  private:
    double _value;
};

class TestLazy {
  public:

    void test_creation() {
        double arg = 2.0;
        Lazy<TestClass> lazy([arg]{ return new TestClass(arg); });

        HELPER_TEST_PRINT("Lazy created")
        TestClass obj = lazy();
        HELPER_TEST_EQUAL(obj.value(),4.0)
    }

    void test() {
        HELPER_TEST_CALL(test_creation());
    }

};

int main() {
    TestLazy().test();
    return HELPER_TEST_FAILURES;
}
