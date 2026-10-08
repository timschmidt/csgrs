/***************************************************************************
 *            workload_advancement.cpp
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
#include "workload_advancement.hpp"

namespace BetterThreads {

using std::lock_guard;

WorkloadAdvancement::WorkloadAdvancement(size_t initial) : _num_waiting(initial), _num_processing(0), _num_completed(0) { }

size_t WorkloadAdvancement::waiting() const {
    lock_guard<mutex> lock(_mux);
    return _num_waiting;
}

size_t WorkloadAdvancement::processing() const {
    lock_guard<mutex> lock(_mux);
    return _num_processing;
}

size_t WorkloadAdvancement::completed() const {
    lock_guard<mutex> lock(_mux);
    return _num_completed;
}

size_t WorkloadAdvancement::total() const {
    lock_guard<mutex> lock(_mux);
    return _num_waiting + _num_processing + _num_completed;
}

void WorkloadAdvancement::add_to_waiting(size_t n) {
    HELPER_PRECONDITION(n > 0);
    lock_guard<mutex> lock(_mux);
    _num_waiting+=n;
}

void WorkloadAdvancement::add_to_processing(size_t n) {
    HELPER_PRECONDITION(n <= _num_waiting);
    lock_guard<mutex> lock(_mux);
    _num_waiting-=n;
    _num_processing+=n;
}

void WorkloadAdvancement::add_to_completed(size_t n) {
    HELPER_PRECONDITION(n <=_num_processing);
    lock_guard<mutex> lock(_mux);
    _num_processing-=n;
    _num_completed+=n;
}

double WorkloadAdvancement::completion_rate() const {
    lock_guard<mutex> lock(_mux);
    auto total = static_cast<double>(_num_waiting + _num_processing + _num_completed);
    return static_cast<double>(_num_completed) / total;
}

bool WorkloadAdvancement::has_finished() const {
    lock_guard<mutex> lock(_mux);
    return _num_processing == 0 and _num_waiting == 0;
}

} // namespace BetterThreads
