/***************************************************************************
 *            arch.hpp
 *
 *  Copyright  2020  Luca Geretti
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

#include <cstdarg>
#include "ariadne.hpp"
#include "utility/stopwatch.hpp"

#ifndef ARIADNE_ARCH_HPP
#define ARIADNE_ARCH_HPP

namespace Ariadne {

class ArchBenchmark;

class ArchBenchmarkInstance {
    friend class ArchBenchmark;
  protected:
    ArchBenchmarkInstance(String const& benchmark_name, String const& instance_name) : _benchmark_name(benchmark_name), _instance_name(instance_name), _verified(0), _execution_time(0), _losses(List<double>()) { }
    ArchBenchmarkInstance(String const& benchmark_name) : ArchBenchmarkInstance(benchmark_name,"") { }
  public:
    ArchBenchmarkInstance& set_verified(int value) { _verified = value; return *this; }
    ArchBenchmarkInstance& set_execution_time(double value) { _execution_time = value; return *this; }
    ArchBenchmarkInstance& add_loss(double value) { _losses.push_back(value); return *this; }
    void write() const {
        std::ofstream outfile;
        outfile.open(_filename, std::ios_base::app);
        outfile << "Ariadne, " << _benchmark_name << ", " <<
                   _instance_name << ", " <<
                   _verified << ", " <<
                   (_execution_time != 0 ? to_string(_execution_time) : "");
        for (SizeType i=0; i < _losses.size(); ++i)
            outfile << ", " << _losses[i];
        outfile << std::endl;
        outfile.close();
    }
  private:
    String _benchmark_name;
    String _instance_name;
    int _verified;
    double _execution_time;
    List<double> _losses;
  private:
    inline static const String _filename = "results.csv";
};

class ArchBenchmark {
  public:
    ArchBenchmark(String const& name) : _name(name) { }
    ArchBenchmarkInstance create_instance() const { return ArchBenchmarkInstance(_name); }
    ArchBenchmarkInstance create_instance(String const& instance_name) const { return ArchBenchmarkInstance(_name,instance_name); }
    String name() const { return _name; }
  private:
    String _name;
};

} // namespace Ariadne

#endif // ARIADNE_ARCH_HPP
