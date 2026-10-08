/***************************************************************************
 *            test_thread.cpp
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

#include "helper/test.hpp"
#include "helper/container.hpp"
#include "conclog/logging.hpp"
#include "conclog/thread_registry_interface.hpp"
#include "thread.hpp"
#include "using.hpp"

using namespace BetterThreads;
using namespace Helper;

using namespace std::chrono_literals;

class ThreadRegistry : public ConcLog::ThreadRegistryInterface {
public:
    ThreadRegistry() : _threads_registered(0) { }
    bool has_threads_registered() const override { return _threads_registered > 0; }
    void set_threads_registered(unsigned int threads_registered) { _threads_registered = threads_registered; }
private:
    unsigned int _threads_registered;
};

class TestThread {
  public:

    void test_create() const {
        Thread thread1([]{}, "thr");
        HELPER_TEST_EXECUTE(thread1.id())
        HELPER_TEST_EQUALS(thread1.name(),"thr")
        Thread thread2([]{});
        HELPER_TEST_EQUALS(to_string(thread2.id()),thread2.name())
    }

    void test_destroy_before_completion() const {
        Thread thread([] { std::this_thread::sleep_for(100ms); },"");
    }

    void test_task() const {
        int a = 0;
        Thread thread([&a] { a++; });
        std::this_thread::sleep_for(10ms);
        HELPER_TEST_EQUALS(a,1)
        HELPER_TEST_ASSERT(thread.exception() == nullptr)
    }

    void test_exception() const {
        Thread thread([] { throw new std::exception(); });
        std::this_thread::sleep_for(10ms);
        HELPER_TEST_ASSERT(thread.exception() != nullptr)
    }

    void test_atomic_multiple_threads() const {
        size_t n_threads = 10*std::thread::hardware_concurrency();
        HELPER_TEST_PRINT(n_threads)
        List<shared_ptr<Thread>> threads;

        std::atomic<size_t> a = 0;
        for (size_t i=0; i<n_threads; ++i) {
            threads.push_back(std::make_shared<Thread>([&a] { a++; }));
        }

        std::this_thread::sleep_for(100ms);
        HELPER_TEST_EQUALS(a,n_threads)
        threads.clear();
    }

    void test() {
        HELPER_TEST_CALL(test_create())
        HELPER_TEST_CALL(test_destroy_before_completion())
        HELPER_TEST_CALL(test_task())
        HELPER_TEST_CALL(test_exception())
        HELPER_TEST_CALL(test_atomic_multiple_threads())
    }

};

int main() {
    ThreadRegistry registry;
    ConcLog::Logger::instance().attach_thread_registry(&registry);
    TestThread().test();
    return HELPER_TEST_FAILURES;
}
