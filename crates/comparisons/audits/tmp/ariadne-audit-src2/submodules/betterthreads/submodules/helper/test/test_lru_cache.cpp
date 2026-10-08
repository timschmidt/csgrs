/***************************************************************************
 *            test_lru_cache.cpp
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

#include "string.hpp"
#include "lru_cache.hpp"

#include "test.hpp"

using namespace Helper;

using CacheType = LRUCache<String,int>;

class TestLRUCache {
  public:

    void test_construct() {
        HELPER_TEST_FAIL(CacheType(0));
        CacheType cache(1);
        HELPER_TEST_EQUALS(cache.current_size(),0);
        HELPER_TEST_EQUALS(cache.maximum_size(),1);
    }

    void test_find() {
        CacheType cache(2);
        HELPER_TEST_ASSERT(not cache.has_label("something"));
    }

    void test_get_failure() {
        CacheType cache(2);
        HELPER_TEST_FAIL(cache.get("something"));
    }

    void test_put_single() {
        CacheType cache(2);
        cache.put("first",42);
        HELPER_TEST_EQUALS(cache.current_size(),1);
        HELPER_TEST_EQUALS(cache.age("first"),0);
        auto val = cache.get("first");
        HELPER_TEST_EQUALS(val,42);
    }

    void test_put_multiple() {
        CacheType cache(2);
        cache.put("first",42);
        cache.put("second",10);
        HELPER_TEST_EQUALS(cache.current_size(),2);
        HELPER_TEST_EQUALS(cache.age("first"),1);
        HELPER_TEST_EQUALS(cache.age("second"),0);
    }

    void test_put_multiple_over() {
        CacheType cache(3);
        cache.put("first",42);
        cache.put("second",10);
        cache.put("third",5);
        cache.put("fourth",12);
        HELPER_TEST_EQUALS(cache.current_size(),3);
        HELPER_TEST_ASSERT(not cache.has_label("first"));
        HELPER_TEST_EQUALS(cache.age("second"),2);
        HELPER_TEST_EQUALS(cache.age("third"),1);
        HELPER_TEST_EQUALS(cache.age("fourth"),0);
    }

    void test_get() {
        CacheType cache(4);
        cache.put("first",42);
        cache.put("second",10);
        cache.put("third",5);
        cache.put("fourth",12);
        cache.get("second");
        HELPER_TEST_EQUALS(cache.age("second"),0);
        HELPER_TEST_EQUALS(cache.age("first"),3);
        HELPER_TEST_EQUALS(cache.age("third"),2);
        HELPER_TEST_EQUALS(cache.age("fourth"),1);
    }

    void test() {
        HELPER_TEST_CALL(test_construct());
        HELPER_TEST_CALL(test_find());
        HELPER_TEST_CALL(test_get_failure());
        HELPER_TEST_CALL(test_put_single());
        HELPER_TEST_CALL(test_put_multiple());
        HELPER_TEST_CALL(test_put_multiple_over());
        HELPER_TEST_CALL(test_get());
    }

};

int main() {
    TestLRUCache().test();
    return HELPER_TEST_FAILURES;
}
