/***************************************************************************
 *            concurrency_submodule.cpp
 *
 *  Copyright  2008-21  Luca Geretti
 *
 ****************************************************************************/

/*
 *  This file is part of Ariadne.
 *
 *  Ariadne is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Ariadne is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with Ariadne.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "pybind11.hpp"
#include "utilities.hpp"
#include "betterthreads/thread_manager.hpp"

using namespace Ariadne;
using BetterThreads::ThreadManager;

Void export_task_manager(pybind11::module& module)
{
    auto const& reference = pybind11::return_value_policy::reference;

    pybind11::class_< ThreadManager > task_manager_class(module,"TaskManager");
    task_manager_class.def_static("instance", &ThreadManager::instance, reference);
    task_manager_class.def("concurrency", &ThreadManager::concurrency);
    task_manager_class.def("set_concurrency", &ThreadManager::set_concurrency);
    task_manager_class.def("maximum_concurrency", &ThreadManager::maximum_concurrency);
    task_manager_class.def("set_maximum_concurrency", &ThreadManager::set_maximum_concurrency);
}

Void concurrency_submodule(pybind11::module& module)
{
    export_task_manager(module);
}

