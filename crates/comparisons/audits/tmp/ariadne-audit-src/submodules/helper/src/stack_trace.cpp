/***************************************************************************
 *            stack_trace.cpp
 *
 *  Copyright  2017-20  Pieter Collins
 *
 ****************************************************************************/

/*
 * This file is part of Helper, under the MIT license.
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

#include "stack_trace.hpp"

#ifdef ENABLE_STACK_TRACE

#include <cassert>
#include <iostream>

#include <execinfo.h>
#include <dlfcn.h>
#include <cxxabi.h>

namespace Helper {

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

} // namespace Helper

#else /* ENABLE_STACK_TRACE */

namespace Helper {

void stack_trace() { }

}

#endif /* ENABLE_STACK_TRACE */

