/***************************************************************************
 *            workload_interface.hpp
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

/*! \file workload_interface.hpp
 *  \brief Interface for a workload, i.e., multiple elements to process
 */

#ifndef BETTERTHREADS_WORKLOAD_INTERFACE_HPP
#define BETTERTHREADS_WORKLOAD_INTERFACE_HPP

#include <functional>
#include "helper/container.hpp"
#include "using.hpp"

namespace BetterThreads {

using Helper::List;

//! \brief Interface for a workload expressed as a stack of elements to work on, supplied with a function to process them
//! \details E: stack element type
//!          AS: optional input arguments for processing the elements; if used as output, their synchronisation
//!              in the concurrent case is up to the designer
//!          The workload handles the non-concurrent case separately, in order to unroll the tasks breadth-first: if
//!          tasks were instead enqueued to the ThreadManager, a depth-first execution would be performed.
template<class E, class... AS>
class WorkloadInterface {
public:

    //! \brief Process the given elements until completion
    virtual void process() = 0;

    //! \brief The size of the workload, i.e., the number of tasks to process
    virtual size_t size() const = 0;

    //! \brief Append one element to process
    virtual WorkloadInterface& append(E const &e) = 0;

    //! \brief Append a list of elements to process
    virtual WorkloadInterface& append(List<E> const &es) = 0;
};

}

#endif // BETTERTHREADS_WORKLOAD_INTERFACE_HPP