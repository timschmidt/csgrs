/***************************************************************************
 *            thread.hpp
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

/*! \file thread.hpp
 *  \brief A wrapper for smart handling of a thread
 */

#ifndef BETTERTHREADS_THREAD_HPP
#define BETTERTHREADS_THREAD_HPP

#include <utility>
#include <thread>
#include <future>
#include <mutex>
#include <atomic>
#include <functional>
#include "helper/macros.hpp"
#include "helper/string.hpp"

namespace BetterThreads {

using std::exception_ptr;
using std::thread;
using std::promise;
using std::future;

using VoidFunction = std::function<void(void)>;
using Helper::String;

//! \brief A class for handling a thread for a pool in a smarter way.
//! \details It allows to wait for the start of the \a task before extracting the thread id, which is held along with
//! a readable \a name.
class Thread {
  public:

    //! \brief Construct with a \a name and \a active specification
    //! \details The thread will start and store the id if active, otherwise activate() will be needed.
    Thread(VoidFunction task, String name, bool active);

    //! \brief Construct with default active=true and possibly default String name equal to the thread id
    Thread(VoidFunction task, String name = std::string());

    //! \brief Get the thread id
    thread::id id() const;
    //! \brief Get the readable name
    String name() const;

    //! \brief Activate the thread
    //! \details If already active (whether at construction of by a previous call to activate()), will do nothing.
    void activate();

    //! \brief The exception, if it exists
    exception_ptr const& exception() const;

    //! \brief Destroy the instance
    ~Thread();

  private:
    String _name;
    thread::id _id;
    std::thread _thread;
    promise<void> _got_id_promise;
    future<void> _got_id_future;
    std::atomic<bool> _active;
    promise<void> _ready_for_task_promise;
    future<void> _ready_for_task_future;
    exception_ptr _exception;
};

} // namespace BetterThreads

#endif // BETTERTHREADS_THREAD_HPP
