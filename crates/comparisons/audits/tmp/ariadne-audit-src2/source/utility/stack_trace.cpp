/***************************************************************************
 *            utility/utility.cpp
 *
 *  Copyright  2017-20  Pieter Collins
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

#include "config.hpp"
#include "stack_trace.hpp"

#ifdef ARIADNE_ENABLE_STACK_TRACE

#include <cassert>
#include <iostream>

#include <execinfo.h>
#include <dlfcn.h>
#include <cxxabi.h>

namespace Ariadne {

void stack_trace() {
    static const unsigned int CALLSTACK_SIZE = 128;
    int skip=0;
    void *callstack[CALLSTACK_SIZE];
    const int nMaxFrames = sizeof(callstack) / sizeof(callstack[0]);
    int nFrames = backtrace(callstack, nMaxFrames);
    char **symbols = backtrace_symbols(callstack, nFrames);
    assert (symbols != nullptr);
    for (int i = skip; i < nFrames; i++) {
        Dl_info info;
        if (dladdr(callstack[i], &info)) {
            char* no_output_buffer = nullptr;
            size_t length;
            int status;
            char* demangled = abi::__cxa_demangle(symbols[i], no_output_buffer, &length, &status);
            std::cerr << (status == 0 ? demangled : info.dli_sname) << "\n";
            free(demangled);
        } else {
            std::cerr << i << " " << callstack[i] << "\n";
        }
    }
    free(symbols);
}

} // namespace Ariadne

#else /* ARIADNE_ENABLE_STACK_TRACE */

namespace Ariadne {

void stack_trace() { }

}

#endif /* ARIADNE_ENABLE_STACK_TRACE */

