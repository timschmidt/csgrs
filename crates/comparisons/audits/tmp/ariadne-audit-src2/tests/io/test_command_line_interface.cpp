/***************************************************************************
 *            test_command_line_interface.cpp
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

#include "io/command_line_interface.hpp"
#include "conclog/logging.hpp"
#include "betterthreads/thread_manager.hpp"
#include "../test.hpp"

using namespace ConcLog;

using namespace Ariadne;

using BetterThreads::ThreadManager;

class TestCommandLineInterface {
  public:

    Void test() {
        ARIADNE_TEST_CALL(test_empty_argument_stream())
        ARIADNE_TEST_CALL(test_nonempty_argument_stream())
        ARIADNE_TEST_CALL(test_cli_instantiation())
        ARIADNE_TEST_CALL(test_from_c_arguments())
        ARIADNE_TEST_CALL(test_concurrency_parsing())
        ARIADNE_TEST_CALL(test_drawer_parsing())
        ARIADNE_TEST_CALL(test_graphics_parsing())
        ARIADNE_TEST_CALL(test_scheduler_parsing())
        ARIADNE_TEST_CALL(test_theme_parsing())
        ARIADNE_TEST_CALL(test_verbosity_parsing())
        ARIADNE_TEST_CALL(test_multiple_argument_parsing())
        ARIADNE_TEST_CALL(test_unrecognised_argument())
        ARIADNE_TEST_CALL(test_duplicate_argument())
        ARIADNE_TEST_CALL(test_print_help())
    }

    void test_empty_argument_stream() {
        ARIADNE_TEST_FAIL(ArgumentStream({}))
    }

    void test_nonempty_argument_stream() {
        auto stream = ArgumentStream({"a","b"});
        ARIADNE_TEST_ASSERT(not stream.empty())
        ARIADNE_TEST_EQUALS(stream.size(),2)
        ARIADNE_TEST_EQUALS(stream.peek(),"a")
        auto val1 = stream.pop();
        ARIADNE_TEST_EQUALS(val1,"a")
        auto val2 = stream.pop();
        ARIADNE_TEST_EQUALS(val2,"b")
        ARIADNE_TEST_ASSERT(stream.empty())
        ARIADNE_TEST_FAIL(stream.peek())
        ARIADNE_TEST_FAIL(stream.pop())
    }

    void test_cli_instantiation() {
        ARIADNE_TEST_EXECUTE(CommandLineInterface::instance())
    }

    void test_from_c_arguments() {
        const char* argv1[] = {""};
        Bool success1 = CommandLineInterface::instance().acquire(1,argv1);
        ARIADNE_TEST_ASSERT(success1)
        const char* argv2[] = {"",""};
        Bool success2 = CommandLineInterface::instance().acquire(2,argv2);
        ARIADNE_TEST_ASSERT(not success2)
    }

    void test_concurrency_parsing() {
        Bool success1 = CommandLineInterface::instance().acquire({"", "-c", "2"});
        ARIADNE_TEST_ASSERT(success1)
        ARIADNE_TEST_EQUALS(ThreadManager::instance().concurrency(),2)
        Bool success2 = CommandLineInterface::instance().acquire({"", "--concurrency", "0"});
        ARIADNE_TEST_ASSERT(success2)
        ARIADNE_TEST_EQUALS(ThreadManager::instance().concurrency(),0)
        Bool success3 = CommandLineInterface::instance().acquire({"", "-c", "-2"});
        ARIADNE_TEST_ASSERT(not success3)
        Bool success4 = CommandLineInterface::instance().acquire({"", "-c", "q"});
        ARIADNE_TEST_ASSERT(not success4)
        Bool success5 = CommandLineInterface::instance().acquire({"", "-c"});
        ARIADNE_TEST_ASSERT(not success5)
        Bool success6 = CommandLineInterface::instance().acquire({"", "-c", "max"});
        ARIADNE_TEST_ASSERT(success6)
        ARIADNE_TEST_EQUALS(ThreadManager::instance().concurrency(),ThreadManager::instance().maximum_concurrency())
    }

    void test_drawer_parsing() {
        Bool success1 = CommandLineInterface::instance().acquire({"", "-d", "box"});
        ARIADNE_TEST_ASSERT(success1)
        Bool success2 = CommandLineInterface::instance().acquire({"", "--drawer", "box"});
        ARIADNE_TEST_ASSERT(success2)
        Bool success3 = CommandLineInterface::instance().acquire({"", "-d", "bx"});
        ARIADNE_TEST_ASSERT(not success3)
        Bool success4 = CommandLineInterface::instance().acquire({"", "-d", "affine"});
        ARIADNE_TEST_ASSERT(not success4)
        Bool success5 = CommandLineInterface::instance().acquire({"", "-d", "affine@e"});
        ARIADNE_TEST_ASSERT(not success5)
        Bool success6 = CommandLineInterface::instance().acquire({"", "-d", "affine@2"});
        ARIADNE_TEST_ASSERT(success6)
        Bool success7 = CommandLineInterface::instance().acquire({"", "-d", "grid"});
        ARIADNE_TEST_ASSERT(not success7)
        Bool success8 = CommandLineInterface::instance().acquire({"", "-d", "grid@e"});
        ARIADNE_TEST_ASSERT(not success8)
        Bool success9 = CommandLineInterface::instance().acquire({"", "-d", "grid@2"});
        ARIADNE_TEST_ASSERT(success9)
        Bool success10 = CommandLineInterface::instance().acquire({"", "-d", "gri@2"});
        ARIADNE_TEST_ASSERT(not success10)
        Bool success11 = CommandLineInterface::instance().acquire({"", "-d"});
        ARIADNE_TEST_ASSERT(not success11)
        Bool success12 = CommandLineInterface::instance().acquire({"", "-d", "none"});
        ARIADNE_TEST_ASSERT(success12)
    }

    void test_graphics_parsing() {
        Bool success1 = CommandLineInterface::instance().acquire({"", "-g", "cairo"});
        ARIADNE_TEST_ASSERT(success1)
        Bool success2 = CommandLineInterface::instance().acquire({"", "--graphics", "cairo"});
        ARIADNE_TEST_ASSERT(success2)
        Bool success3 = CommandLineInterface::instance().acquire({"", "-g", "wrong"});
        ARIADNE_TEST_ASSERT(not success3)
        Bool success4 = CommandLineInterface::instance().acquire({"", "-g", "gnuplot"});
        ARIADNE_TEST_ASSERT(success4)
        Bool success5 = CommandLineInterface::instance().acquire({"", "-g", "none"});
        ARIADNE_TEST_ASSERT(success5)
        Bool success6 = CommandLineInterface::instance().acquire({"", "-g"});
        ARIADNE_TEST_ASSERT(not success6)
    }

    void test_scheduler_parsing() {
        ThreadManager::instance().set_concurrency(0);
        Bool success1 = CommandLineInterface::instance().acquire({"", "-s", "immediate"});
        ARIADNE_TEST_ASSERT(success1)
        Bool success2 = CommandLineInterface::instance().acquire({"", "--scheduler", "immediate"});
        ARIADNE_TEST_ASSERT(success2)
        Bool success3 = CommandLineInterface::instance().acquire({"", "-s", "wrong"});
        ARIADNE_TEST_ASSERT(not success3)
        Bool success4 = CommandLineInterface::instance().acquire({"", "-s", "blocking"});
        ARIADNE_TEST_ASSERT(success4)
        Bool success5 = CommandLineInterface::instance().acquire({"", "-s", "nonblocking"});
        ARIADNE_TEST_ASSERT(success5)
        Bool success6 = CommandLineInterface::instance().acquire({"", "-s"});
        ARIADNE_TEST_ASSERT(not success6)
        Bool success7 = CommandLineInterface::instance().acquire({"", "-c", "max", "-s", "immediate"});
        ARIADNE_TEST_ASSERT(success7)
    }

    void test_theme_parsing() {
        Bool success1 = CommandLineInterface::instance().acquire({"", "-t", "none"});
        ARIADNE_TEST_ASSERT(success1)
        Bool success2 = CommandLineInterface::instance().acquire({"", "--theme", "none"});
        ARIADNE_TEST_ASSERT(success2)
        Bool success3 = CommandLineInterface::instance().acquire({"", "-t", "nn"});
        ARIADNE_TEST_ASSERT(not success3)
        Bool success4 = CommandLineInterface::instance().acquire({"", "-t", "light"});
        ARIADNE_TEST_ASSERT(success4)
        Bool success5 = CommandLineInterface::instance().acquire({"", "-t", "dark"});
        ARIADNE_TEST_ASSERT(success5)
        Bool success6 = CommandLineInterface::instance().acquire({"", "-t"});
        ARIADNE_TEST_ASSERT(not success6)
    }

    void test_verbosity_parsing() {
        Bool success1 = CommandLineInterface::instance().acquire({"", "-v", "5"});
        ARIADNE_TEST_ASSERT(success1)
        ARIADNE_TEST_EQUALS(Logger::instance().configuration().verbosity(),5)
        Bool success2 = CommandLineInterface::instance().acquire({"", "--verbosity", "0"});
        ARIADNE_TEST_ASSERT(success2)
        ARIADNE_TEST_EQUALS(Logger::instance().configuration().verbosity(),0)
        Bool success3 = CommandLineInterface::instance().acquire({"", "-v", "-2"});
        ARIADNE_TEST_ASSERT(not success3)
        Bool success4 = CommandLineInterface::instance().acquire({"", "-v", "q"});
        ARIADNE_TEST_ASSERT(not success4)
        Bool success5 = CommandLineInterface::instance().acquire({"", "-v"});
        ARIADNE_TEST_ASSERT(not success5)
    }

    void test_multiple_argument_parsing() {
        Bool success = CommandLineInterface::instance().acquire({"", "-c", "2", "--verbosity", "4"});
        ARIADNE_TEST_ASSERT(success)
        ARIADNE_TEST_EQUALS(ThreadManager::instance().concurrency(),2)
        ARIADNE_TEST_EQUALS(Logger::instance().configuration().verbosity(),4)
    }

    void test_unrecognised_argument() {
        Bool success = CommandLineInterface::instance().acquire({"", "--invalid"});
        ARIADNE_TEST_ASSERT(not success)
    }

    void test_duplicate_argument() {
        Bool success = CommandLineInterface::instance().acquire({"", "--verbosity", "2", "-v", "5"});
        ARIADNE_TEST_ASSERT(not success)
    }

    void test_print_help() {
        Bool success = CommandLineInterface::instance().acquire({"", "-h"});
        ARIADNE_TEST_ASSERT(not success)
    }

};

Int main(Int argc, const char* argv[]) {
    TestCommandLineInterface().test();
    return ARIADNE_TEST_FAILURES;
}

