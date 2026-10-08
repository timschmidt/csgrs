/***************************************************************************
 *            string.hpp
 *
 *  Copyright  2023  Pieter Collins
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

/*! \file string.hpp
 *  \brief Wrapper for string class
 */

#ifndef HELPER_STRING_HPP
#define HELPER_STRING_HPP

#include <string>
#include <sstream>

namespace Helper {

//! \brief A wrapper for the standard string class.
class String : public std::string {
  public:
    using std::string::string;
    String(std::string const& str) : std::string(str) { };
    String() = default;
};

inline const char* c_str(const String& str) {
    return str.c_str(); }
template<class T> inline String to_string(const T& t) {
    std::stringstream ss; ss << t; return ss.str(); }
template<class T> inline String to_str(T const& t) {
    return to_string(t); }

template<class T> String class_name();
template<> inline String class_name<String>() { return "String"; }

} // namespace Helper

#endif /* HELPER_STRING_HPP */
