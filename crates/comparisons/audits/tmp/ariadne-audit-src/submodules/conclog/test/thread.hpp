/***************************************************************************
 *            thread.hpp
 *
 *  Copyright  2021  Luca Geretti
 *
 ****************************************************************************/

/*
 * This file is part of ConcLog, under the MIT license.
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

#ifndef CONCLOG_THREAD_HPP
#define CONCLOG_THREAD_HPP

#include <utility>
#include <thread>
#include <future>
#include <mutex>
#include <atomic>
#include <functional>
#include "logging.hpp"

namespace ConcLog {

template<class T> inline std::string to_string(const T& t) { std::stringstream ss; ss << t; return ss.str(); }

//! \brief A class for handling a thread for a pool in a smarter way.
//! \details It allows to wait for the start of the \a task before extracting the thread id, which is held along with
//! a readable \a name.
class Thread {
  public:

    //! \brief Construct with an optional name
    //! \details The thread will start and store the id
    Thread(std::function<void()> const& task, std::string const& name = std::string())
        : _name(name), _got_id_future(_got_id_promise.get_future()), _registered_thread_future(_registered_thread_promise.get_future())
    {
            _thread = std::thread([this, task]() {
                _id = std::this_thread::get_id();
                _got_id_promise.set_value();
                _registered_thread_future.get();
                task();
            });
            _got_id_future.get();
            if (_name.empty()) _name = to_string(_id);
            Logger::instance().register_thread(this->id(), this->name());
            _registered_thread_promise.set_value();
    }

    //! \brief Get the thread id
    std::thread::id const& id() const { return _id; }
    //! \brief Get the readable name
    std::string const& name() const { return _name; }

    //! \brief Destroy the instance
    ~Thread() {
        _thread.join();
        Logger::instance().unregister_thread(this->id());
    }

  private:
    std::string _name;
    std::thread::id _id;
    std::promise<void> _got_id_promise;
    std::future<void> _got_id_future;
    std::promise<void> _registered_thread_promise;
    std::future<void> _registered_thread_future;
    std::thread _thread;
};

} // namespace ConcLog

#endif // CONCLOG_THREAD_HPP
