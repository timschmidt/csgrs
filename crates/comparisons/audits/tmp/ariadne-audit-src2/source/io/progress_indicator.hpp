/***************************************************************************
 *            io/progress_indicator.hpp
 *
 *  Copyright  2007-20  Luca Geretti
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

/*! \file io/progress_indicator.hpp
 *  \brief Support for showing progress with respect to a given metric.
 */

#ifndef ARIADNE_PROGRESS_INDICATOR_HPP
#define ARIADNE_PROGRESS_INDICATOR_HPP

#include <cmath>

namespace Ariadne {

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

} // namespace Ariadne

#endif // ARIADNE_PROGRESS_INDICATOR_HPP
