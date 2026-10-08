/***************************************************************************
 *            task_manager.hpp
 *
 *  Copyright  2022 Luca Geretti
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

/*! \file task_manager.hpp
 *  \brief Singleton class for managing tasks across the library
 */

#ifndef BETTERTHREADS_THREAD_MANAGER_HPP
#define BETTERTHREADS_THREAD_MANAGER_HPP

#include <algorithm>
#include "conclog/logging.hpp"
#include "conclog/thread_registry_interface.hpp"
#include "thread_pool.hpp"
#include "templates.hpp"

namespace BetterThreads {

using ConcLog::ThreadRegistryInterface;
using ConcLog::Logger;

//! \brief Manages threads based on concurrency availability.
class ThreadManager : public ThreadRegistryInterface {
  private:
    ThreadManager();
  public:
    ThreadManager(ThreadManager const&) = delete;
    void operator=(ThreadManager const&) = delete;

    virtual ~ThreadManager() = default;

    //! \brief The singleton instance of this class
    static ThreadManager& instance() {
        auto& logger = Logger::instance();
        static ThreadManager instance;
        if (not logger.has_thread_registry_attached())
            logger.attach_thread_registry(&instance);
        return instance;
    }

    //! \brief Whether threads have already been registered
    bool has_threads_registered() const override;

    //! \brief Get the maximum concurrency allowed by this machine
    size_t maximum_concurrency() const;
    //! \brief Get the preferred concurrency to be used
    //! \details A concurrency of zero is allowed, meaning that a task
    //! will be run sequentially
    size_t concurrency() const;

    //! \brief Synchronised method for updating the preferred concurrency to be used
    void set_concurrency(size_t value);

    //! \brief Set the concurrency to the maximum allowed by this machine
    void set_maximum_concurrency();

    //! \brief Set the Logger scheduler to the immediate one
    //! \details Fails if the concurrency is not zero
    void set_logging_immediate_scheduler() const;
    //! \brief Set the Logger scheduler to the blocking one
    //! \details Fails if the concurrency is not zero
    void set_logging_blocking_scheduler() const;
    //! \brief Set the Logger scheduler to the nonblocking one
    //! \details Fails if the concurrency is not zero
    void set_logging_nonblocking_scheduler() const;

    //! \brief Enqueue a task for execution, returning the future handler
    //! \details The is no limits on the number of tasks to enqueue. If concurrency is zero,
    //! then the task is executed sequentially with no threads involved
    template<class F, class... AS> auto enqueue(F &&f, AS &&... args) -> future<ResultOf<F(AS...)>>;

  private:
    const size_t _maximum_concurrency;
    size_t _concurrency;
    mutable mutex _concurrency_mutex;

    ThreadPool _pool;
};

template<class F, class... AS> auto ThreadManager::enqueue(F &&f, AS &&... args) -> future<ResultOf<F(AS...)>> {
    if (_concurrency == 0) {
        using ReturnType = ResultOf<F(AS...)>;
        auto task = packaged_task<ReturnType()>(std::bind(std::forward<F>(f), std::forward<AS>(args)...));
        future<ReturnType> result = task.get_future();
        task();
        return result;
    } else return _pool.enqueue(f,args...);
}

} // namespace BetterThreads

#endif // BETTERTHREADS_THREAD_MANAGER_HPP
