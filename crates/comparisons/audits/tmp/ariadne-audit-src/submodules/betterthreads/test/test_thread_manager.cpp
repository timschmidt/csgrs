/***************************************************************************
 *            test_task_manager.cpp
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
#include "thread_manager.hpp"

using namespace BetterThreads;
using namespace std::chrono_literals;

class TestThreadManager {
  public:

    void test_set_concurrency() {
        auto max_concurrency = ThreadManager::instance().maximum_concurrency();
        ThreadManager::instance().set_concurrency(max_concurrency);
        HELPER_TEST_EQUALS(ThreadManager::instance().concurrency(), max_concurrency)
        ThreadManager::instance().set_maximum_concurrency();
        HELPER_TEST_EQUALS(ThreadManager::instance().concurrency(), max_concurrency)
        HELPER_TEST_FAIL(ThreadManager::instance().set_concurrency(1 + max_concurrency))
    }

    void test_run_task_with_one_thread() {
        ThreadManager::instance().set_concurrency(1);
        int a = 10;
        auto result = ThreadManager::instance().enqueue([&a]{ return a * a; }).get();
        HELPER_TEST_EQUALS(result,100)
    }

    void test_run_task_with_multiple_threads() {
        ThreadManager::instance().set_concurrency(ThreadManager::instance().maximum_concurrency());
        int a = 10;
        auto result = ThreadManager::instance().enqueue([&a]{ return a * a; }).get();
        HELPER_TEST_EQUALS(result,100)
    }

    void test_run_task_with_no_threads() {
        ThreadManager::instance().set_concurrency(0);
        int a = 10;
        auto result = ThreadManager::instance().enqueue([&a]{ return a * a; }).get();
        HELPER_TEST_EQUALS(result,100)
    }

    void test_change_concurrency_and_log_scheduler() {
        HELPER_TEST_EXECUTE(ThreadManager::instance().set_concurrency(1))
        HELPER_TEST_FAIL(ThreadManager::instance().set_logging_immediate_scheduler())
        HELPER_TEST_FAIL(ThreadManager::instance().set_logging_blocking_scheduler())
        HELPER_TEST_FAIL(ThreadManager::instance().set_logging_nonblocking_scheduler())
        HELPER_TEST_EXECUTE(ThreadManager::instance().set_concurrency(0))
        HELPER_TEST_EXECUTE(ThreadManager::instance().set_logging_immediate_scheduler())
        HELPER_TEST_EXECUTE(ThreadManager::instance().set_logging_blocking_scheduler())
        HELPER_TEST_EXECUTE(ThreadManager::instance().set_logging_nonblocking_scheduler())
        HELPER_TEST_EXECUTE(ThreadManager::instance().set_concurrency(1))
        HELPER_TEST_EXECUTE(ThreadManager::instance().set_concurrency(0))
    }

    void test() {
        HELPER_TEST_CALL(test_set_concurrency())
        HELPER_TEST_CALL(test_run_task_with_one_thread())
        HELPER_TEST_CALL(test_run_task_with_multiple_threads())
        HELPER_TEST_CALL(test_run_task_with_no_threads())
        HELPER_TEST_CALL(test_change_concurrency_and_log_scheduler())
    }
};

int main() {
    TestThreadManager().test();
    return HELPER_TEST_FAILURES;
}
