/***************************************************************************
 *            buffer.hpp
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

/*! \file buffer.hpp
 *  \brief A multiple-thread-safe queue usable as a buffer.
 */

#ifndef BETTERTHREADS_BUFFER_HPP
#define BETTERTHREADS_BUFFER_HPP

#include <utility>
#include <mutex>
#include <condition_variable>
#include <queue>
#include "helper/macros.hpp"
#include "using.hpp"

namespace BetterThreads {

//! \brief Exception useful when the buffer is allowed to stay in the receiving condition
class BufferInterruptPullingException : public std::exception { };

//! \brief A class for handling a buffer
template<class E> class Buffer
{
  public:
    Buffer(size_t capacity) : _capacity(capacity), _interrupt(false) { HELPER_PRECONDITION(capacity > 0); }

    //! \brief Push an object into the buffer
    //! \details Will block if the capacity has been reached
    void push(E const& e) {
        unique_lock<mutex> locker(mux);
        cond.wait(locker, [this](){return _queue.size() < _capacity;});
        _queue.push(e);
        cond.notify_all();
    }

    //! \brief Pulls an object from the buffer
    //! \details Will block if the capacity is zero
    E pull() {
        unique_lock<mutex> locker(mux);
        cond.wait(locker, [this](){return not _queue.empty() || _interrupt;});
        if (_interrupt and _queue.empty()) { _interrupt = false; throw BufferInterruptPullingException(); }
        E back = _queue.front();
        _queue.pop();
        cond.notify_all();
        return back;
    }

    //! \brief The current size of the queue
    size_t size() const {
        lock_guard<mutex> locker(mux);
        return _queue.size();
    }

    //! \brief The maximum size for the queue
    size_t capacity() const {
        return _capacity;
    }

    //! \brief Change the capacity
    void set_capacity(size_t capacity) {
        HELPER_PRECONDITION(capacity>0);
        HELPER_ASSERT_MSG(capacity>=size(),"Reducing capacity below currenty buffer size is not allowed.");
        _capacity = capacity;
    }

    //! \brief Interrupt consuming in the case that the queue is empty and the buffer in the waiting state for input
    //! \details Needs to
    void interrupt_consuming() {
        _interrupt = true;
        cond.notify_all();
    }

private:
    mutable mutex mux;
    condition_variable cond;
    std::queue<E> _queue;
    std::atomic<size_t> _capacity;
    bool _interrupt;
};

} // namespace BetterThreads

#endif // BETTERTHREADS_BUFFER_HPP
