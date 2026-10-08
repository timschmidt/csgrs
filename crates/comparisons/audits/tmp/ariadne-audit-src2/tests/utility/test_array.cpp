/***************************************************************************
 *            test_array.cpp
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
#include "utility/array.hpp"
#include "utility/uniform_array.hpp"
#include "utility/container.hpp"

#include "../test.hpp"

using namespace Ariadne;

struct TestConvertibleTo {
    TestConvertibleTo(int a_) : a(a_) { }
    int a;
};

struct TestClass {
    TestClass(int a_) : a(a_) { }
    explicit TestClass(TestConvertibleTo const& c) : TestClass(c.a) { }
    int a;
};

class TestArray {
  public:

    Void test_convert() {
        Array<TestClass> tca = {TestClass(1), TestClass(2)};
        Array<TestConvertibleTo> tcta = {TestConvertibleTo(1), TestConvertibleTo(2)};
        ARIADNE_TEST_EXECUTE(Array<TestClass> tcac(tcta));
    }

    Void test_print() {
        Array<int> a1;
        ARIADNE_TEST_PRINT(a1);
        Array<int> a2 = {1, 2};
        ARIADNE_TEST_PRINT(a2);
    }

    Void test() {
        ARIADNE_TEST_CALL(test_convert());
        ARIADNE_TEST_CALL(test_print());
    }

};

class Size {
    std::size_t _n;
  public:
    Size(std::size_t n) : _n(n) { }
    operator std::size_t () const { return this->_n; }
};

class ConstantFunction {
    Size _argument_size;
    double _value;
  public:
    ConstantFunction(Size as) : _argument_size(as), _value(0.0) { }
    ConstantFunction(Size as, double v) : _argument_size(as), _value(v) { }
    Size characteristics() const { return this->_argument_size; }
    friend OutputStream& operator<<(OutputStream& os, ConstantFunction const& cf) {
        return os << "ConstantFunction(as="<<cf._argument_size<<", c="<<cf._value<<")"; }
};

class TestUniformArray {
  public:
    Void test() {
        SizeType as=2u;
        ConstantFunction cf(3);
        cf.characteristics();
        characteristics(cf);
        static_assert(HasMemberCharacteristics<ConstantFunction>);
        static_assert(HasCharacteristics<ConstantFunction>);
        static_assert(Same<CharacteristicsType<ConstantFunction>,Size>);
        UniformArray<ConstantFunction> a(3, as);
        ConstantFunction cf0=ConstantFunction(as,3);
        a.set(0,cf0);
        ConstantFunction cf1=ConstantFunction(as+1,5);
        ARIADNE_TEST_FAIL(a.set(1,cf1));
    }
};

Int main() {
    TestArray().test();
    TestUniformArray().test();
    return ARIADNE_TEST_FAILURES;
}
