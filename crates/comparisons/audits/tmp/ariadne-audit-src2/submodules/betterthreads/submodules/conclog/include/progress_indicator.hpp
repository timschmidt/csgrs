/***************************************************************************
 *            progress_indicator.hpp
 *
 *  Copyright  2021  Luca Geretti
 *
 ****************************************************************************/

/*
 * This file is part of ConcLog, under the MIT license.
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

#ifndef CONCLOG_PROGRESS_INDICATOR_HPP
#define CONCLOG_PROGRESS_INDICATOR_HPP

#include <cmath>

namespace ConcLog {

//! \brief Helper class to display progress percentage when holding text in Logger
class ProgressIndicator {
  public:
    ProgressIndicator(double final_value = 0.0) : _final_value(final_value), _current_value(0u), _step(0u) { }
    double final_value() const { return _final_value; }
    double current_value() const { return _current_value; }
    void update_current(double new_value) { if (_current_value != new_value) { _current_value = new_value; ++_step; } }
    void update_final(double new_final) { _final_value = new_final; }
    char symbol() const { switch (_step % 4) { case 0: return '\\'; case 1: return '|'; case 2: return '/'; default: return '-'; }}
    unsigned int percentage() const { return static_cast<unsigned int>(std::round(_current_value/_final_value*100u)); }
  private:
    double _final_value;
    double _current_value;
    unsigned int _step;
};

} // namespace ConcLog

#endif // CONCLOG_PROGRESS_INDICATOR_HPP
