/***************************************************************************
 *            io_submodule.cpp
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

#include "conclog/logging.hpp"
#include "io/command_line_interface.hpp"

using namespace ConcLog;

using namespace Ariadne;

Void export_theme(pybind11::module& module)
{
    pybind11::class_<TerminalTextTheme> tt_text_theme_class(module,"TerminalTextTheme");
    module.attr("TT_THEME_LIGHT") = TT_THEME_LIGHT;
    module.attr("TT_THEME_DARK") = TT_THEME_DARK;
    module.attr("TT_THEME_NONE") = TT_THEME_NONE;
}

Void export_logger(pybind11::module& module)
{
    auto const& reference = pybind11::return_value_policy::reference;

    pybind11::class_<LoggerConfiguration> logger_configuration_class(module,"LoggerConfiguration");
    logger_configuration_class.def("verbosity", &LoggerConfiguration::verbosity);
    logger_configuration_class.def("set_verbosity", &LoggerConfiguration::set_verbosity);
    logger_configuration_class.def("set_theme", &LoggerConfiguration::set_theme);

    pybind11::class_<Logger> logger_class(module,"Logger");
    logger_class.def_static("instance", &Logger::instance, reference);
    logger_class.def("configuration", &Logger::configuration, reference);
    logger_class.def("use_immediate_scheduler", &Logger::use_immediate_scheduler);
    logger_class.def("use_blocking_scheduler", &Logger::use_blocking_scheduler);
    logger_class.def("use_nonblocking_scheduler", &Logger::use_nonblocking_scheduler);
}

Void export_cli(pybind11::module& module)
{
    auto const& reference = pybind11::return_value_policy::reference;

    pybind11::class_<CommandLineInterface> cli_class(module,"CommandLineInterface");
    cli_class.def_static("instance", &CommandLineInterface::instance, reference);
    cli_class.def("acquire", pybind11::overload_cast<List<String> const&>(&CommandLineInterface::acquire,pybind11::const_));
}

Void io_submodule(pybind11::module& module)
{
    export_theme(module);
    export_logger(module);
    export_cli(module);
}

