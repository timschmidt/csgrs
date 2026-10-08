/***************************************************************************
 *            test_lru_cache.cpp
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

#include "config.hpp"
#include "utility/string.hpp"
#include "utility/lru_cache.hpp"

#include "../test.hpp"

using namespace Ariadne;

using CacheType = LRUCache<String,int>;

class TestLRUCache {
  public:

    Void test_construct() {
        ARIADNE_TEST_FAIL(CacheType(0));
        CacheType cache(1);
        ARIADNE_TEST_EQUALS(cache.current_size(),0);
        ARIADNE_TEST_EQUALS(cache.maximum_size(),1);
    }

    Void test_find() {
        CacheType cache(2);
        ARIADNE_TEST_ASSERT(not cache.has_label("something"));
    }

    Void test_get_failure() {
        CacheType cache(2);
        ARIADNE_TEST_FAIL(cache.get("something"));
    }

    Void test_put_single() {
        CacheType cache(2);
        cache.put("first",42);
        ARIADNE_TEST_EQUALS(cache.current_size(),1);
        ARIADNE_TEST_EQUALS(cache.age("first"),0);
        auto val = cache.get("first");
        ARIADNE_TEST_EQUALS(val,42);
    }

    Void test_put_multiple() {
        CacheType cache(2);
        cache.put("first",42);
        cache.put("second",10);
        ARIADNE_TEST_EQUALS(cache.current_size(),2);
        ARIADNE_TEST_EQUALS(cache.age("first"),1);
        ARIADNE_TEST_EQUALS(cache.age("second"),0);
    }

    Void test_put_multiple_over() {
        CacheType cache(3);
        cache.put("first",42);
        cache.put("second",10);
        cache.put("third",5);
        cache.put("fourth",12);
        ARIADNE_TEST_EQUALS(cache.current_size(),3);
        ARIADNE_TEST_ASSERT(not cache.has_label("first"));
        ARIADNE_TEST_EQUALS(cache.age("second"),2);
        ARIADNE_TEST_EQUALS(cache.age("third"),1);
        ARIADNE_TEST_EQUALS(cache.age("fourth"),0);
    }

    Void test_get() {
        CacheType cache(4);
        cache.put("first",42);
        cache.put("second",10);
        cache.put("third",5);
        cache.put("fourth",12);
        cache.get("second");
        ARIADNE_TEST_EQUALS(cache.age("second"),0);
        ARIADNE_TEST_EQUALS(cache.age("first"),3);
        ARIADNE_TEST_EQUALS(cache.age("third"),2);
        ARIADNE_TEST_EQUALS(cache.age("fourth"),1);
    }

    Void test() {
        ARIADNE_TEST_CALL(test_construct());
        ARIADNE_TEST_CALL(test_find());
        ARIADNE_TEST_CALL(test_get_failure());
        ARIADNE_TEST_CALL(test_put_single());
        ARIADNE_TEST_CALL(test_put_multiple());
        ARIADNE_TEST_CALL(test_put_multiple_over());
        ARIADNE_TEST_CALL(test_get());
    }

};

Int main() {
    TestLRUCache().test();
    return ARIADNE_TEST_FAILURES;
}
