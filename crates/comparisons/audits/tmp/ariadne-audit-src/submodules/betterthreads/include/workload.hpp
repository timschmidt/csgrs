/***************************************************************************
 *            workload.hpp
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

/*! \file workload.hpp
 *  \brief A stack of objects to work on, supplied with a function to process them
 */

#ifndef BETTERTHREADS_WORKLOAD_HPP
#define BETTERTHREADS_WORKLOAD_HPP

#include <functional>
#include <iomanip>
#include "helper/container.hpp"
#include "helper/tuple.hpp"
#include "conclog/progress_indicator.hpp"
#include "workload_interface.hpp"
#include "thread_manager.hpp"
#include "workload_advancement.hpp"

namespace BetterThreads {

using ConcLog::ProgressIndicator;
using ConcLog::LogScopeManager;
using ConcLog::Logger;

using Helper::List;
using Helper::make_lpair;

using std::mutex;
using std::unique_lock;
using std::lock_guard;
using std::condition_variable;

//! \brief Base class implementation
template<class E, class... AS>
class WorkloadBase : public WorkloadInterface<E,AS...> {
  protected:
    WorkloadBase() : _progress_acknowledge_func(std::bind_front(&WorkloadBase::_default_progress_acknowledge, this)), _advancement(0), _logger_level(0), _progress_indicator(new ProgressIndicator(0)) { }
  public:
    using TaskFunctionType = std::function<void(E const &)>;
    using ProgressAcknowledgeFunctionType = std::function<void(E const &, shared_ptr<ProgressIndicator>)>;
    using CompletelyBoundFunctionType = std::function<void(void)>;

    void process() override {
        _log_scope_manager.reset(new LogScopeManager(HELPER_PRETTY_FUNCTION,0));
        _logger_level = Logger::instance().current_level();
        while (true) {
            unique_lock<mutex> lock(_element_availability_mutex);
            _element_availability_condition.wait(lock, [=,this] { return _advancement.has_finished() or not _sequential_queue.empty() or _exception != nullptr; });
            if (_exception != nullptr) rethrow_exception(_exception);
            if (_advancement.has_finished()) { _log_scope_manager.reset(); return; }

            CompletelyBoundFunctionType task, progress_acknowledge;
            make_lpair(task,progress_acknowledge) = _sequential_queue.front();
            _sequential_queue.pop();
            if (_using_concurrency()) {
                ThreadManager::instance().enqueue([this, task, progress_acknowledge] { _concurrent_task_wrapper(task, progress_acknowledge); });
            } else {
                _advancement.add_to_processing();
                if (not Logger::instance().is_muted_at(0)) { progress_acknowledge(); _print_hold(); }
                task();
                _advancement.add_to_completed();
            }
        }
    }

    size_t size() const override { return _sequential_queue.size(); }

    WorkloadInterface<E,AS...>& append(E const& e) override {
        _advancement.add_to_waiting();
        _sequential_queue.push(std::make_pair(std::bind(std::forward<TaskFunctionType const>(_task_func), std::forward<E const&>(e)),
                                                   std::bind(std::forward<ProgressAcknowledgeFunctionType const>(_progress_acknowledge_func), std::forward<E const&>(e),
                                                  _progress_indicator)
                         ));
        return *this;
    }

    WorkloadInterface<E,AS...>& append(List<E> const& es) override { for (auto e : es) append(e); return *this; }

  private:

    bool _using_concurrency() const { return ThreadManager::instance().concurrency() > 0; }

    void _concurrent_task_wrapper(CompletelyBoundFunctionType const& task, CompletelyBoundFunctionType const& progress_acknowledge) {
        _advancement.add_to_processing();
        if (_logger_level > Logger::instance().current_level()) Logger::instance().increase_level(_logger_level-Logger::instance().current_level());
        else Logger::instance().decrease_level(Logger::instance().current_level()-_logger_level);

        if (not Logger::instance().is_muted_at(0)) { progress_acknowledge(); _print_hold(); }
        try {
            task();
        } catch (...) {
            {
                lock_guard<mutex> lock(_element_availability_mutex);
                _exception = std::current_exception();
            }
            _element_availability_condition.notify_one();
        }

        {
            lock_guard<mutex> lock(_element_availability_mutex);
            _advancement.add_to_completed();
        }
        if (_advancement.has_finished()) { _element_availability_condition.notify_one(); }
    }

    void _default_progress_acknowledge(E const& e, shared_ptr<ProgressIndicator> indicator) {
        indicator->update_current(static_cast<double>(_advancement.completed()));
        indicator->update_final(static_cast<double>(_advancement.total()));
    }

    void _print_hold() {
        std::ostringstream logger_stream;
        logger_stream << "[" << _progress_indicator->symbol() << "] " << _progress_indicator->percentage() << "% ";
        logger_stream << " (w="<<std::setw(2)<<std::left<<_advancement.waiting()
                      << " p="<<std::setw(2)<<std::left<<_advancement.processing()
                      << " c="<<std::setw(3)<<std::left<<_advancement.completed()
                      << ")";
        Logger::instance().hold(_log_scope_manager->scope(),logger_stream.str());
    }

  protected:

    void _enqueue(E const& e) {
        if (_using_concurrency()) {
            _advancement.add_to_waiting();
            auto task = std::bind(std::forward<TaskFunctionType const>(_task_func), std::forward<E const&>(e));
            auto progress_acknowledge = std::bind(std::forward<ProgressAcknowledgeFunctionType const>(_progress_acknowledge_func),
                                                  std::forward<E const&>(e), _progress_indicator);
            ThreadManager::instance().enqueue([this,task,progress_acknowledge]{ _concurrent_task_wrapper(task, progress_acknowledge); });
        } else {
            // Locking and notification are used when concurrency is set to zero during processing, in order to avoid a race and to resume processing _sequential_queue respectively
            {
                lock_guard<mutex> lock(_element_appending_mutex);
                append(e);
            }
            _element_availability_condition.notify_one();
        }
    }

  protected:

    TaskFunctionType _task_func;
    ProgressAcknowledgeFunctionType _progress_acknowledge_func;
    WorkloadAdvancement _advancement;

  private:

    // Queue of task-progress_acknowledge pairs for initial consumption and for consumption when using no concurrency
    std::queue<std::pair<CompletelyBoundFunctionType,CompletelyBoundFunctionType>> _sequential_queue;

    unsigned int _logger_level; // The logger level to impose to the running threads
    shared_ptr<LogScopeManager> _log_scope_manager; // The scope manager required to properly hold print
    shared_ptr<ProgressIndicator> _progress_indicator; // The progress indicator to hold print

    mutex _element_appending_mutex;
    mutex _element_availability_mutex;
    condition_variable _element_availability_condition;

    exception_ptr _exception;
};

//! \brief A basic static workload where all elements are appended and then processed
template<class E, class... AS>
class StaticWorkload : public WorkloadBase<E,AS...> {
public:
    using TaskFunctionType = std::function<void(E const&, AS...)>;

    StaticWorkload(TaskFunctionType f, AS... as) : WorkloadBase<E, AS...>() {
        this->_task_func = std::bind(std::forward<TaskFunctionType const>(f), std::placeholders::_1, std::forward<AS>(as)...);
    }
};

//! \brief A dynamic workload in which it is possible to append new elements from the called function
template<class E, class... AS>
class DynamicWorkload : public WorkloadBase<E,AS...> {
  public:
    //! \brief Reduced interface to be used by the processing function (and any function called by it)
    class Access {
        friend DynamicWorkload;
    protected:
        Access(DynamicWorkload& parent) : _load(parent) { }
    public:
        void append(E const &e) { _load._enqueue(e); }
    private:
        DynamicWorkload& _load;
    };
  public:
    using TaskFunctionType = std::function<void(Access&, E const&, AS...)>;
    using ProgressAcknowledgeFunctionType = std::function<void(E const&, shared_ptr<ProgressIndicator>)>;

    DynamicWorkload(ProgressAcknowledgeFunctionType p, TaskFunctionType t, AS... as) : WorkloadBase<E, AS...>(), _access(Access(*this)) {
        this->_task_func = std::bind(std::forward<TaskFunctionType const>(t),
                                     std::forward<Access const&>(_access),
                                     std::placeholders::_1,
                                     std::forward<AS>(as)...);
        this->_progress_acknowledge_func = p;
    }

  private:
    Access const _access;
};

}

#endif // BETTERTHREADS_WORKLOAD_HPP