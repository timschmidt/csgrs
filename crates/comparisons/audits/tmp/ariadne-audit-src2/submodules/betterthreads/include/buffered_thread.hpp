/***************************************************************************
 *            buffered_thread.hpp
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

/*! \file buffered_thread.hpp
 *  \brief A wrapper for smart handling of a thread with a buffer for incoming tasks
 */

#ifndef BETTERTHREADS_BUFFERED_THREAD_HPP
#define BETTERTHREADS_BUFFERED_THREAD_HPP

#include <utility>
#include <thread>
#include <future>
#include <mutex>
#include <atomic>
#include <functional>
#include "helper/string.hpp"
#include "templates.hpp"
#include "buffer.hpp"
#include "using.hpp"

namespace BetterThreads {

using Helper::String;

//! \brief A class for handling a thread that accepts multiple tasks to be enqueued.
//! \details It allows to wait for the start of the \a task before extracting the thread id, which is held along with
//! a readable \a name. The thread can execute only one task at a time. Compared with Thread, this is meant to be used in
//! isolation, not in pool. It is functionally equivalent to a ThreadPool of one Thread only.
class BufferedThread {
  public:

    //! \brief Construct with a name.
    //! \details The thread will start and store the id. The name will be the id if left empty
    BufferedThread(String name = String());

    //! \brief Enqueue a task for execution, returning the future handler
    //! \details If the buffer is full, successive calls will block until an execution is started.
    template<class F, class... AS>
    auto enqueue(F&& f, AS&&... args) -> future<ResultOf<F(AS...)>>;

    //! \brief Get the thread id
    thread::id id() const;
    //! \brief Get the readable name
    String name() const;

    //! \brief The current size of the queue
    size_t queue_size() const;
    //! \brief The capacity of the tasks to execute
    size_t queue_capacity() const;
    //! \brief Change the queue capacity
    //! \details Capacity cannot be changed to a value lower than the current size
    void set_queue_capacity(size_t capacity);

    //! \brief Destroy the instance
    ~BufferedThread();

  private:
    String _name;
    thread::id _id;
    std::thread _thread;
    Buffer<std::function<void(void)>> _task_buffer;
    promise<void> _got_id_promise;
    future<void> _got_id_future;
};

template<class F, class... AS> auto BufferedThread::enqueue(F&& f, AS&&... args) -> future<ResultOf<F(AS...)>>
{
    using ReturnType = ResultOf<F(AS...)>;

    auto task = std::make_shared<packaged_task<ReturnType()>>(std::bind(std::forward<F>(f), std::forward<AS>(args)...));
    future<ReturnType> result = task->get_future();
    _task_buffer.push([task](){ (*task)(); });
    return result;
}

} // namespace BetterThreads

#endif // BETTERTHREADS_BUFFERED_THREAD_HPP
