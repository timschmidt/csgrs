/***************************************************************************
 *            workload_advancement.hpp
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

/*! \file workload_advancement.hpp
 *  \brief Synchronised class to manage the status of multiple elements to process
 */

#ifndef BETTERTHREADS_WORKLOAD_ADVANCEMENT_HPP
#define BETTERTHREADS_WORKLOAD_ADVANCEMENT_HPP

#include <algorithm>
#include <mutex>

namespace BetterThreads {

using std::mutex;

//! \brief Synchronised class to manage the status of multiple elements to process
class WorkloadAdvancement {
  public:
    WorkloadAdvancement(size_t initial = 0);

    //! \brief The elements waiting to be processed
    size_t waiting() const;
    //! \brief The elements under processing
    size_t processing() const;
    //! \brief The completed elements
    size_t completed() const;

    //! \brief All the elements (sum of waiting, processing and completed)
    size_t total() const;

    //! \brief Add n elements to waiting
    void add_to_waiting(size_t n = 1);
    //! \brief Move n waiting to processing
    void add_to_processing(size_t n = 1);
    //! \brief Move n processing to completed
    void add_to_completed(size_t n = 1);

    //! \brief The rate of completion r (0<=r<=1) related to the progress
    double completion_rate() const;

    //! \brief If no other processing remains
    bool has_finished() const;

  private:
    size_t _num_waiting;
    size_t _num_processing;
    size_t _num_completed;

    mutex mutable _mux;
};

} // namespace BetterThreads

#endif // BETTERTHREADS_WORKLOAD_ADVANCEMENT_HPP
