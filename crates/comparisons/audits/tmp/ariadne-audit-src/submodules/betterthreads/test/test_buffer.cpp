/***************************************************************************
 *            test_buffer.cpp
 *
 *  Copyright  2022  Luca Geretti
 *
 ****************************************************************************/

/*
 * This file is part of BetterThreads, under the MIT license.
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

#include <thread>
#include "helper/test.hpp"
#include "buffer.hpp"

using namespace BetterThreads;

class TestBuffer {
  public:

    void test_construct() {
        Buffer<size_t> buffer(2);
        HELPER_TEST_EQUALS(buffer.size(),0);
        HELPER_TEST_EQUALS(buffer.capacity(),2);
    }

    void test_construct_invalid() {
        HELPER_TEST_FAIL(Buffer<size_t>(0));
    }

    void test_set_capacity_when_empty() {
        Buffer<size_t> buffer(2);
        buffer.set_capacity(5);
        HELPER_TEST_EQUALS(buffer.capacity(),5);
        buffer.set_capacity(3);
        HELPER_TEST_EQUALS(buffer.capacity(),3);
        HELPER_TEST_FAIL(buffer.set_capacity(0));
    }

    void test_set_capacity_when_filled() {
        Buffer<size_t> buffer(2);
        buffer.push(4);
        buffer.push(2);
        HELPER_TEST_EXECUTE(buffer.set_capacity(5));
        HELPER_TEST_FAIL(buffer.set_capacity(1));
        buffer.pull();
        HELPER_TEST_EXECUTE(buffer.set_capacity(1));
    }

    void test_single_buffer() {
        Buffer<size_t> buffer(2);
        buffer.push(4);
        buffer.push(2);
        HELPER_TEST_EQUALS(buffer.size(),2);
        auto o1 = buffer.pull();
        auto o2 = buffer.pull();
        HELPER_TEST_EQUALS(buffer.size(),0);
        HELPER_TEST_EQUALS(o1,4);
        HELPER_TEST_EQUALS(o2,2);
    }

    void test_io_buffer() {
        Buffer<size_t> ib(2);
        Buffer<size_t> ob(2);

        std::thread thread([&ib,&ob]() {
            while (true) {
                try {
                    auto i = ib.pull();
                    ob.push(i);
                } catch (BufferInterruptPullingException&) {
                    break;
                }
            }
        });
        ib.push(4);
        ib.push(2);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        HELPER_TEST_EQUALS(ib.size(),0);
        HELPER_TEST_EQUALS(ob.size(),2);
        auto o1 = ob.pull();
        HELPER_TEST_EQUALS(ob.size(),1);
        HELPER_TEST_EQUALS(o1,4);
        auto o2 = ob.pull();
        HELPER_TEST_EQUALS(ob.size(),0);
        HELPER_TEST_EQUALS(o2,2);
        ib.interrupt_consuming();
        thread.join();
    }

    void test() {
        HELPER_TEST_CALL(test_construct());
        HELPER_TEST_CALL(test_construct_invalid());
        HELPER_TEST_CALL(test_set_capacity_when_empty());
        HELPER_TEST_CALL(test_set_capacity_when_filled());
        HELPER_TEST_CALL(test_single_buffer());
        HELPER_TEST_CALL(test_io_buffer());
    }
};

int main() {
    TestBuffer().test();
    return HELPER_TEST_FAILURES;
}
