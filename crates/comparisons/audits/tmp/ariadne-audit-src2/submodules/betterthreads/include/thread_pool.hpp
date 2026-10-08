/***************************************************************************
 *            thread_pool.hpp
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

/*! \file thread_pool.hpp
 *  \brief A pool of threads
 */

#ifndef BETTERTHREADS_THREAD_POOL_HPP
#define BETTERTHREADS_THREAD_POOL_HPP

#include <queue>
#include "conclog/logging.hpp"
#include "helper/container.hpp"
#include "thread.hpp"
#include "templates.hpp"
#include "using.hpp"

namespace BetterThreads {

using std::make_shared;
using Helper::List;

const String THREAD_POOL_DEFAULT_NAME = "thr";

//! \brief Exception for stopping a thread pool
class StoppedThreadPoolException : public std::exception { };

//! \brief A pool of Thread objects managed internally given a (variable) number of threads
//! \details Differently from managing a single BufferedThread, the task queue for a pool is not upper-bounded, i.e., BufferedThread
//! objects use a buffer of one element, which receives once the wrapped task that consumes elements from the task queue.
class ThreadPool {
  public:
    //! \brief Construct from a given number of threads and possibly a name
    ThreadPool(size_t num_threads, String name = THREAD_POOL_DEFAULT_NAME);

    //! \brief Enqueue a task for execution, returning the future handler
    //! \details The is no limits on the number of tasks to enqueue
    template<class F, class... AS> auto enqueue(F &&f, AS &&... args) -> future<ResultOf<F(AS...)>>;

    //! \brief The name of the pool
    String name() const;

    //! \brief The size of the tasks queue
    size_t queue_size() const;

    //! \brief The number of threads
    size_t num_threads() const;

    //! \brief Set the number of threads
    //! \details If reducing the current number, this method will block until
    //! all the previous tasks are completed, previous threads are destroyed
    //! and new threads are spawned
    void set_num_threads(size_t number);

    ~ThreadPool();

  private:

    //! \brief The function wrapper handling the extraction from the queue
    //! \details Takes \a i as the index of the thread in the list, for identification when stopping selectively
    VoidFunction _task_wrapper_function(size_t i);
    //! \brief Append threads in the given range
    void _append_thread_range(size_t lower, size_t upper);

  private:
    const String _name;
    List<shared_ptr<Thread>> _threads;
    std::queue<VoidFunction> _tasks;

    mutable mutex _task_availability_mutex;
    condition_variable _task_availability_condition;
    bool _finish_all_and_stop; // Wait till the queue is empty before stopping the thread, used for destruction
    size_t _num_active_threads; // Down-counter for checking whether all the threads to stop have been stopped
    size_t _num_threads_to_use; // Reference on the number of threads to use: if lower than the threads size, the last threads will stop
    mutable mutex _num_active_threads_mutex;
    mutable mutex _num_threads_mutex;
    promise<void> _all_unused_threads_stopped_promise;
    future<void> _all_unused_threads_stopped_future;
};

template<class F, class... AS>
auto ThreadPool::enqueue(F &&f, AS &&... args) -> future<ResultOf<F(AS...)>> {
    using ReturnType = ResultOf<F(AS...)>;

    auto task = make_shared<packaged_task<ReturnType()> >(std::bind(std::forward<F>(f), std::forward<AS>(args)...));
    future<ReturnType> result = task->get_future();
    {
        unique_lock<mutex> lock(_task_availability_mutex);
        if (_finish_all_and_stop) throw StoppedThreadPoolException();
        _tasks.emplace([task]{ (*task)(); });
    }
    _task_availability_condition.notify_one();
    return result;
}

//! \brief Utility function to construct a thread name from a \a prefix and a \a number,
//! accounting for a maximum number of threads given by \a max_number
String construct_thread_name(String prefix, size_t number, size_t max_number);

}

#endif // BETTERTHREADS_THREAD_POOL_HPP
