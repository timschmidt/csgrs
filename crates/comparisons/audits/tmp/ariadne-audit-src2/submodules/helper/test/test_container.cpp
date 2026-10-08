/***************************************************************************
 *            test_container.cpp
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

#include "container.hpp"

#include "test.hpp"

using namespace Helper;

class TestContainer {
  public:

    void test_map_get() {
        Map<int,int> im = {{1,10},{2,20}};
        HELPER_TEST_FAIL(im.get(3));
        HELPER_TEST_EQUALS(im.get(2),20);
    }

    void test_map_convert() {
        Map<int,int> im = {{1,10},{2,20}};

        Map<int,double> dm(im);
        HELPER_TEST_ASSERT(dm.at(1) == im.at(1) and dm.at(2) == im.at(2));
    }

    void test_map_restrict_keys() {
        Set<int> s = {1,2};
        Map<int,double> m = {{1,1.2},{2,1.5},{3,1.0},{5,0.1}};
        auto restricted = restrict_keys(m,s);
        HELPER_TEST_EQUALS(restricted.size(),2);
        HELPER_TEST_ASSERT(restricted.has_key(1) and restricted.has_key(2));
    }

    void test_make_list_of_set() {
        Set<int> s = {1, 5, 3};
        auto l = make_list(s);
        HELPER_TEST_EQUALS(l.size(),3);
        HELPER_TEST_ASSERT(l.at(0) == 1 and l.at(1) == 3 and l.at(2) == 5);
    }

    void test() {
        HELPER_TEST_CALL(test_map_get());
        HELPER_TEST_CALL(test_map_convert());
        HELPER_TEST_CALL(test_map_restrict_keys());
        HELPER_TEST_CALL(test_make_list_of_set());
    }

};

int main() {
    TestContainer().test();
    return HELPER_TEST_FAILURES;
}
