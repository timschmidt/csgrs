/***************************************************************************
 *            thread.cpp
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
#include "thread.hpp"

namespace BetterThreads {

using ConcLog::Logger;
using Helper::to_string;

Thread::Thread(VoidFunction task, String name, bool active)
        : _name(std::move(name)), _got_id_future(_got_id_promise.get_future()), _active(active), _ready_for_task_future(_ready_for_task_promise.get_future()),
          _exception(nullptr)
{
    _thread = std::thread([=,this]() {
        _id = std::this_thread::get_id();
        _got_id_promise.set_value();
        _ready_for_task_future.get();
        if (_active) {
            try { task(); }
            catch(...) { _exception = std::current_exception(); }
        }
    });
    _got_id_future.get();
    if (_name.empty()) _name = to_string(_id);
    if (active) {
        Logger::instance().register_thread(_id,_name);
        _ready_for_task_promise.set_value();
    }
}

Thread::Thread(VoidFunction task, String name) : Thread(task, name, true)
{ }

thread::id Thread::id() const {
    return _id;
}

String Thread::name() const {
    return _name;
}

void Thread::activate()  {
    if (not _active) {
        _active = true;
        Logger::instance().register_thread(_id,_name);
        _ready_for_task_promise.set_value();
    }
}

exception_ptr const& Thread::exception() const {
    return _exception;
}

Thread::~Thread() {
    if (not _active) _ready_for_task_promise.set_value();
    else Logger::instance().unregister_thread(_id);
    _thread.join();

}

} // namespace BetterThreads
