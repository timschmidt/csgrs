/***************************************************************************
 *            buffered_thread.cpp
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

#include "conclog/logging.hpp"
#include "buffered_thread.hpp"
#include "using.hpp"

namespace BetterThreads {

using ConcLog::Logger;
using Helper::to_string;

BufferedThread::BufferedThread(String name)
        : _name(name), _task_buffer(1), _got_id_future(_got_id_promise.get_future())
{
    _thread = std::thread([=,this]() {
        _id = std::this_thread::get_id();
        _got_id_promise.set_value();
        while(true) {
            try {
                std::function<void(void)> task = _task_buffer.pull();
                task();
            } catch(BufferInterruptPullingException&) { return; }
        }
    });
    _got_id_future.get();
    if (name == String()) _name = to_string(_id);
    Logger::instance().register_thread(this->id(), this->name());
}

thread::id BufferedThread::id() const {
    return _id;
}

String BufferedThread::name() const {
    return _name;
}

size_t BufferedThread::queue_size() const {
    return _task_buffer.size();
}

size_t BufferedThread::queue_capacity() const {
    return _task_buffer.capacity();
}

void BufferedThread::set_queue_capacity(size_t capacity) {
    return _task_buffer.set_capacity(capacity);
}

BufferedThread::~BufferedThread() {
    _task_buffer.interrupt_consuming();
    _thread.join();
    Logger::instance().unregister_thread(this->id());
}

} // namespace BetterThreads
