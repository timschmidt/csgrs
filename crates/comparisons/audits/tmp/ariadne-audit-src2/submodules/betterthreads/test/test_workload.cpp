/***************************************************************************
 *            test_workload.cpp
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

#include <functional>
#include "helper/test.hpp"
#include "helper/container.hpp"
#include "workload.hpp"

using namespace BetterThreads;
using namespace Helper;

template<class T> class SynchronisedList : public List<T> {
  public:
    void append(T const& v) { lock_guard<mutex> guard(_mux); return List<T>::push_back(v); }
  private:
    mutex _mux;
};

using StaticWorkloadType = StaticWorkload<int,std::shared_ptr<std::atomic<int>>>;
using DynamicWorkloadType = DynamicWorkload<int,std::shared_ptr<SynchronisedList<int>>>;

void sum_all(int const& val, std::shared_ptr<std::atomic<int>> result) {
    result->operator+=(val);
}

void print(int const& val) {
    CONCLOG_PRINTLN_VAR(val)
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void square_and_store(DynamicWorkloadType::Access& wla, int const& val, std::shared_ptr<SynchronisedList<int>> results) {
    int next_val = val*val;
    if (next_val < 46340) {
        wla.append(next_val);
    }
    results->append(next_val);
}

void progress_acknowledge(int const& val, std::shared_ptr<ProgressIndicator> indicator) {
    indicator->update_current(val);
    indicator->update_final(std::numeric_limits<int>::max());
}

void throw_exception_immediately(DynamicWorkloadType::Access&, int const&, std::shared_ptr<SynchronisedList<int>>) {
    throw new std::exception();
}

void throw_exception_later(DynamicWorkloadType::Access& wla, int const& val, std::shared_ptr<SynchronisedList<int>>) {
    int next_val = val+1;
    if (next_val > 4) throw new std::exception();
    else wla.append(next_val);
}

class TestWorkload {
  public:

    void test_construct_static() {
        ThreadManager::instance().set_concurrency(0);
        auto result = std::make_shared<std::atomic<int>>();
        StaticWorkloadType wl(&sum_all, result);
    }

    void test_construct_dynamic() {
        ThreadManager::instance().set_concurrency(0);
        std::shared_ptr<SynchronisedList<int>> result = std::make_shared<SynchronisedList<int>>();
        DynamicWorkloadType wl(&progress_acknowledge, &square_and_store, result);
        HELPER_TEST_EQUALS(wl.size(),0)
    }

    void test_append() {
        ThreadManager::instance().set_concurrency(0);
        std::shared_ptr<SynchronisedList<int>> result = std::make_shared<SynchronisedList<int>>();
        DynamicWorkloadType wl(&progress_acknowledge, &square_and_store, result);
        wl.append(2);
        HELPER_TEST_EQUALS(wl.size(),1)
        wl.append({10,20});
        HELPER_TEST_EQUALS(wl.size(),3)
    }

    void test_process_nothing() {
        ThreadManager::instance().set_maximum_concurrency();
        auto result = std::make_shared<std::atomic<int>>();
        StaticWorkloadType wl(&sum_all, result);
        HELPER_TEST_EXECUTE(wl.process())
    }

    void test_serial_processing_static() {
        ThreadManager::instance().set_concurrency(0);
        std::shared_ptr<SynchronisedList<int>> result = std::make_shared<SynchronisedList<int>>();
        result->append(2);
        DynamicWorkloadType wl(&progress_acknowledge, &square_and_store, result);
        wl.append(2);
        wl.process();
        HELPER_TEST_PRINT(*result)
        HELPER_TEST_EQUALS(result->size(),5)
    }

    void test_serial_processing_dynamic() {
        ThreadManager::instance().set_concurrency(0);
        std::shared_ptr<SynchronisedList<int>> result = std::make_shared<SynchronisedList<int>>();
        result->append(2);
        DynamicWorkloadType wl(&progress_acknowledge, &square_and_store, result);
        wl.append(2);
        wl.process();
        HELPER_TEST_PRINT(*result)
        HELPER_TEST_EQUALS(result->size(),5)
    }

    void test_concurrent_processing_static() {
        ThreadManager::instance().set_maximum_concurrency();
        auto result = std::make_shared<std::atomic<int>>();
        *result = 0;
        StaticWorkloadType wl(&sum_all, result);
        wl.append({2,7,-3,5,8,10,5,8});
        wl.process();
        HELPER_TEST_EQUALS(*result,42)
    }

    void test_concurrent_processing_dynamic() {
        ThreadManager::instance().set_maximum_concurrency();
        std::shared_ptr<SynchronisedList<int>> result = std::make_shared<SynchronisedList<int>>();
        result->append(2);
        DynamicWorkloadType wl(&progress_acknowledge, &square_and_store, result);
        wl.append(2);
        wl.process();
        HELPER_TEST_PRINT(*result)
        HELPER_TEST_EQUALS(result->size(),5)
    }

    void test_print_hold() {
        ThreadManager::instance().set_concurrency(0);
        Logger::instance().configuration().set_verbosity(2);
        StaticWorkload<int> wl(&print);
        wl.append({1,2,3,4,5});
        wl.process();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        Logger::instance().configuration().set_verbosity(0);
    }

    void test_throw_serial_exception_immediately() {
        ThreadManager::instance().set_concurrency(0);
        std::shared_ptr<SynchronisedList<int>> result = std::make_shared<SynchronisedList<int>>();
        DynamicWorkloadType wl(&progress_acknowledge, &throw_exception_immediately, result);
        wl.append(2);
        HELPER_TEST_FAIL(wl.process())
    }

    void test_throw_serial_exception_later() {
        ThreadManager::instance().set_concurrency(0);
        std::shared_ptr<SynchronisedList<int>> result = std::make_shared<SynchronisedList<int>>();
        DynamicWorkloadType wl(&progress_acknowledge, &throw_exception_later, result);
        wl.append(2);
        HELPER_TEST_FAIL(wl.process())
    }

    void test_throw_concurrent_exception_immediately() {
        ThreadManager::instance().set_maximum_concurrency();
        std::shared_ptr<SynchronisedList<int>> result = std::make_shared<SynchronisedList<int>>();
        DynamicWorkloadType wl(&progress_acknowledge, &throw_exception_immediately, result);
        wl.append(2);
        HELPER_TEST_FAIL(wl.process())
    }

    void test_throw_concurrent_exception_later() {
        ThreadManager::instance().set_maximum_concurrency();
        std::shared_ptr<SynchronisedList<int>> result = std::make_shared<SynchronisedList<int>>();
        DynamicWorkloadType wl(&progress_acknowledge, &throw_exception_later, result);
        wl.append(2);
        HELPER_TEST_FAIL(wl.process())
    }

    void test_multiple_append() {
        ThreadManager::instance().set_maximum_concurrency();
        std::shared_ptr<SynchronisedList<int>> result = std::make_shared<SynchronisedList<int>>();
        DynamicWorkloadType wl(&progress_acknowledge, &square_and_store, result);
        result->append(2);
        result->append(3);
        wl.append({2,3});
        wl.process();
        HELPER_TEST_PRINT(*result)
        HELPER_TEST_EQUALS(result->size(),10)
    }

    void test_multiple_process() {
        ThreadManager::instance().set_maximum_concurrency();
        std::shared_ptr<SynchronisedList<int>> result = std::make_shared<SynchronisedList<int>>();
        result->append(2);
        DynamicWorkloadType wl(&progress_acknowledge, &square_and_store, result);
        wl.append(2);
        wl.process();
        result->clear();
        result->append(3);
        wl.append(3);
        wl.process();
        HELPER_TEST_PRINT(*result)
        HELPER_TEST_EQUALS(result->size(),5)
    }

    void test() {
        HELPER_TEST_CALL(test_construct_static())
        HELPER_TEST_CALL(test_construct_dynamic())
        HELPER_TEST_CALL(test_append())
        HELPER_TEST_CALL(test_process_nothing())
        HELPER_TEST_CALL(test_serial_processing_static())
        HELPER_TEST_CALL(test_serial_processing_dynamic())
        HELPER_TEST_CALL(test_concurrent_processing_static())
        HELPER_TEST_CALL(test_concurrent_processing_dynamic())
        HELPER_TEST_CALL(test_print_hold())
        HELPER_TEST_CALL(test_throw_serial_exception_immediately())
        HELPER_TEST_CALL(test_throw_serial_exception_later())
        HELPER_TEST_CALL(test_throw_concurrent_exception_immediately())
        HELPER_TEST_CALL(test_throw_concurrent_exception_later())
        HELPER_TEST_CALL(test_multiple_append())
        HELPER_TEST_CALL(test_multiple_process())
    }

};

int main() {
    TestWorkload().test();
    return HELPER_TEST_FAILURES;
}
