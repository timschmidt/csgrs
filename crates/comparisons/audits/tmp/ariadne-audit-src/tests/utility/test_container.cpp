/***************************************************************************
 *            test_container.cpp
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

#include "config.hpp"
#include "utility/container.hpp"

#include "../test.hpp"

using namespace Ariadne;

class TestContainer {
  public:

    Void test_map_get() {
        Map<int,int> im = {{1,10},{2,20}};
        ARIADNE_TEST_FAIL(im.get(3));
        ARIADNE_TEST_EQUALS(im.get(2),20);
    }

    Void test_map_convert() {
        Map<int,int> im = {{1,10},{2,20}};

        Map<int,double> dm(im);
        ARIADNE_TEST_ASSERT(dm.at(1) == im.at(1) and dm.at(2) == im.at(2));
    }

    Void test_map_restrict_keys() {
        Set<int> s = {1,2};
        Map<int,double> m = {{1,1.2},{2,1.5},{3,1.0},{5,0.1}};
        auto restricted = restrict_keys(m,s);
        ARIADNE_TEST_EQUALS(restricted.size(),2);
        ARIADNE_TEST_ASSERT(restricted.has_key(1) and restricted.has_key(2));
    }

    Void test_make_list_of_set() {
        Set<int> s = {1, 5, 3};
        auto l = make_list(s);
        ARIADNE_TEST_EQUALS(l.size(),3);
        ARIADNE_TEST_ASSERT(l.at(0) == 1 and l.at(1) == 3 and l.at(2) == 5);
    }

    Void test() {
        ARIADNE_TEST_CALL(test_map_get());
        ARIADNE_TEST_CALL(test_map_convert());
        ARIADNE_TEST_CALL(test_map_restrict_keys());
        ARIADNE_TEST_CALL(test_make_list_of_set());
    }

};

Int main() {
    TestContainer().test();
    return ARIADNE_TEST_FAILURES;
}
