/***************************************************************************
 *            task_manager.cpp
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

#include "helper/macros.hpp"
#include "conclog/logging.hpp"
#include "thread_manager.hpp"

namespace BetterThreads {

using ConcLog::Logger;

ThreadManager::ThreadManager() : _maximum_concurrency(std::thread::hardware_concurrency()), _concurrency(0), _pool(0) {}

bool ThreadManager::has_threads_registered() const {
    return _concurrency;
}

size_t ThreadManager::maximum_concurrency() const {
    return _maximum_concurrency;
}

size_t ThreadManager::concurrency() const {
    lock_guard<mutex> lock(_concurrency_mutex);
    return _concurrency;
}

void ThreadManager::set_concurrency(size_t value) {
    HELPER_PRECONDITION(value <= _maximum_concurrency);
    lock_guard<mutex> lock(_concurrency_mutex);
    _concurrency = value;
    _pool.set_num_threads(value);
}

void ThreadManager::set_maximum_concurrency() {
    set_concurrency(_maximum_concurrency);
}

void ThreadManager::set_logging_immediate_scheduler() const {
    HELPER_PRECONDITION(_concurrency == 0)
    Logger::instance().use_immediate_scheduler();
}

void ThreadManager::set_logging_blocking_scheduler() const {
    HELPER_PRECONDITION(_concurrency == 0)
    Logger::instance().use_blocking_scheduler();
}

void ThreadManager::set_logging_nonblocking_scheduler() const {
    HELPER_PRECONDITION(_concurrency == 0)
    Logger::instance().use_nonblocking_scheduler();
}

}