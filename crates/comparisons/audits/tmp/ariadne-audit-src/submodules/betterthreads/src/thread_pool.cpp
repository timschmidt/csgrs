/***************************************************************************
 *            thread_pool.cpp
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

#include "thread_pool.hpp"

namespace BetterThreads {

String construct_thread_name(String prefix, size_t number, size_t max_number) {
    std::ostringstream ss;
    ss << prefix;
    if (max_number > 9 and number <= 9) ss << "0";
    ss << number;
    return ss.str();
}

VoidFunction ThreadPool::_task_wrapper_function(size_t i) {
    return [i, this] {
        while (true) {
            VoidFunction task;
            bool got_task = false;
            {
                unique_lock<mutex> lock(_task_availability_mutex);
                _task_availability_condition.wait(lock, [=, this] {
                    return _finish_all_and_stop or (_num_active_threads > _num_threads_to_use) or not _tasks.empty();
                });
                if (_finish_all_and_stop and _tasks.empty()) return;
                if (not _tasks.empty()) {
                    task = std::move(_tasks.front());
                    _tasks.pop();
                    got_task = true;
                }
            }
            if (got_task) task();
            if (i>=_num_threads_to_use) {
                lock_guard<mutex> active_threads_lock(_num_active_threads_mutex);
                _num_active_threads--;
                if (_num_active_threads == _num_threads_to_use) _all_unused_threads_stopped_promise.set_value();
                return;
            }
        }
    };
}

void ThreadPool::_append_thread_range(size_t lower, size_t upper) {
    for (size_t i=lower; i<upper; ++i) {
        _threads.push_back(make_shared<Thread>(ThreadPool::_task_wrapper_function(i), construct_thread_name(_name,i,upper)));
    }
}

ThreadPool::ThreadPool(size_t size, String name)
        : _name(name), _finish_all_and_stop(false), _num_active_threads(size), _num_threads_to_use(size),
          _all_unused_threads_stopped_future(_all_unused_threads_stopped_promise.get_future())
{
    _append_thread_range(0,size);
}

String ThreadPool::name() const {
    return _name;
}

size_t ThreadPool::num_threads() const {
    lock_guard<mutex> lock(_num_threads_mutex);
    return _threads.size();
}

void ThreadPool::set_num_threads(size_t number) {
    lock_guard<mutex> lock(_num_threads_mutex);
    auto old_size = _threads.size();
    _num_threads_to_use = number;
    if (number > old_size) {
        _num_active_threads = number;
        _append_thread_range(old_size,number);
    } else if (number < old_size) {
        _task_availability_condition.notify_all();
        _all_unused_threads_stopped_future.get();
        _threads.resize(number);
        _all_unused_threads_stopped_promise = promise<void>();
        _all_unused_threads_stopped_future = _all_unused_threads_stopped_promise.get_future();
    }
}

size_t ThreadPool::queue_size() const {
    lock_guard<mutex> lock(_task_availability_mutex);
    return _tasks.size();
}

ThreadPool::~ThreadPool() {
    {
        lock_guard<mutex> task_availability_lock(_task_availability_mutex);
        _finish_all_and_stop = true;
    }
    _task_availability_condition.notify_all();
    _threads.clear();
}

}
