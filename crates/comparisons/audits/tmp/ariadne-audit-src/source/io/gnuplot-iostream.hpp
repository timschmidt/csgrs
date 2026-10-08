/*
Copyright (c) 2020 Daniel Stahlke (dan@stahlke.org)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

// Edit by Mirko Albanese

/* A C++ interface to gnuplot.
 * Web page: http://www.stahlke.org/dan/gnuplot-iostream
 * Documentation: https://github.com/dstahlke/gnuplot-iostream/wiki
 *
 * The whole library consists of this monolithic header file, for ease of installation (the
 * Makefile and *.cc files are only for examples and tests).
 *
*/
#ifndef GNUPLOT_IOSTREAM_HPP
#define GNUPLOT_IOSTREAM_HPP

// {{{1 Includes and defines

#define GNUPLOT_IOSTREAM_VERSION 3

// C system includes
#include <cstdio>
#ifdef GNUPLOT_ENABLE_PTY
#    include <termios.h>
#    include <unistd.h>
#ifdef __APPLE__
#    include <util.h>
#else
#    include <pty.h>
#endif
#endif // GNUPLOT_ENABLE_PTY

// C++ system includes
#include <fstream>
#include <iostream>
#include <ostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <iomanip>
#include <vector>
#include <complex>
#include <cstdint>
#include <cstdlib>
#include <cmath>
#include <tuple>
#include <type_traits>

#include <streambuf>

#ifdef _MSC_VER
# include <io.h>
#else
# include <unistd.h>
#endif

#    define GNUPLOT_DEPRECATE(msg)

#    define GNUPLOT_PCLOSE pclose
#    define GNUPLOT_POPEN  popen
#    define GNUPLOT_FILENO fileno

#ifdef _WIN32
#    define GNUPLOT_ISNAN _isnan
#else
// cppreference.com says std::isnan is only for C++11.  However, this seems to work on Linux
// and I am assuming that if isnan exists in math.h then std::isnan exists in cmath.
#    define GNUPLOT_ISNAN std::isnan
#endif

// MSVC gives a warning saying that fopen and getenv are not secure.  But they are secure.
// Unfortunately their replacement functions are not simple drop-in replacements.  The best
// solution is to just temporarily disable this warning whenever fopen or getenv is used.
// http://stackoverflow.com/a/4805353/1048959
#if defined(_MSC_VER) && _MSC_VER >= 1400
#    define GNUPLOT_MSVC_WARNING_4996_PUSH \
        __pragma(warning(push)) \
        __pragma(warning(disable:4996))
#    define GNUPLOT_MSVC_WARNING_4996_POP \
        __pragma(warning(pop))
#else
#    define GNUPLOT_MSVC_WARNING_4996_PUSH
#    define GNUPLOT_MSVC_WARNING_4996_POP
#endif

#ifndef GNUPLOT_DEFAULT_COMMAND
#ifdef _WIN32

#    define GNUPLOT_DEFAULT_COMMAND "gnuplot -persist 2> NUL"
#else
#    define GNUPLOT_DEFAULT_COMMAND "gnuplot -persist"
#endif
#endif

namespace gnuplotio {



//FileHandleWrapper

// This holds the file handle that gnuplot commands will be sent to.  The purpose of this
// wrapper is twofold:
// 1. It allows storing the FILE* before it gets passed to the fdostream
// constructor (which is a base class of the main Gnuplot class).  This is accomplished
//    via multiple inheritance as described at http://stackoverflow.com/a/3821756/1048959
// 2. It remembers whether the handle needs to be closed via fclose or pclose.

struct FileHandleWrapper{
    FileHandleWrapper(std::FILE *_fh, bool _should_use_pclose)  :wrapped_fh(_fh), should_use_pclose(_should_use_pclose) {}

    void fh_close() {
        if(should_use_pclose) {
            if(GNUPLOT_PCLOSE(wrapped_fh)) {
                std::cerr << "pclose returned error: " << strerror(errno) << std::endl;
            }
        } else {
            if(fclose(wrapped_fh)) {
                std::cerr << "fclose returned error" << std::endl;
            }
        }
    }

    int fh_fileno() {
        return GNUPLOT_FILENO(wrapped_fh);
    }

    std::FILE *wrapped_fh;
    bool should_use_pclose;

};

//streambuf class from file descriptor
class fdoutbuf : public std::streambuf {
  protected:
    int fd;    // file descriptor
  public:
    // constructor
    fdoutbuf(){}
    fdoutbuf (int _fd) : fd(_fd) {}
  protected:
    // write one character
    virtual int_type overflow (int_type c) {
        if (c != EOF) {
            char z = c;
            if (write(fd, &z, 1) != 1) {
                return EOF;
            }
        }
        return c;
    }

};

class fdostream : public std::ostream {
  protected:
    fdoutbuf buf;
  public:
    fdostream() {}
    fdostream (int fd) : std::ostream(0), buf(fd) {
        rdbuf(&buf);
    }
};


//Main class

class Gnuplot :
    // Using a multiple inheritance trick,
    // as described at http://stackoverflow.com/a/3821756/1048959
    private FileHandleWrapper,
    public fdostream
{
public:
    bool debug_messages;
    bool transport_tmpfile;
private:
    static FileHandleWrapper open_cmdline(const std::string &in) {
        std::string cmd = in;
        FILE *fh = GNUPLOT_POPEN(cmd.c_str(), "w");
        if(!fh) throw std::ios_base::failure("cannot open pipe "+cmd);
        return FileHandleWrapper(fh, true);
    }

public:
    explicit Gnuplot(const std::string &_cmd="") :
        FileHandleWrapper(open_cmdline(_cmd)),
        fdostream(fh_fileno()),
        debug_messages(false),
        transport_tmpfile(false)

    {
        set_stream_options(*this);
    }


    explicit Gnuplot(FILE *_fh) :
        FileHandleWrapper(_fh, 0),
        fdostream(fh_fileno()),
        debug_messages(false),
        transport_tmpfile(false)
       
    {
        set_stream_options(*this);
    }

private:
    // noncopyable
    Gnuplot(const Gnuplot &) = delete;
    const Gnuplot& operator=(const Gnuplot &) = delete;

public:
    ~Gnuplot() {
        if(debug_messages) {
            std::cerr << "ending gnuplot session" << std::endl;
        }
        do_flush();

        fh_close();

    }

    void useTmpFile(bool state) {
        transport_tmpfile = state;
    }

    void clearTmpfiles() {

    }

public:
    void do_flush() {
        *this << std::flush;
        fflush(wrapped_fh);
    }

private:

    void set_stream_options(std::ostream &ostream) const
    {
        ostream << std::defaultfloat << std::setprecision(17);  // refer <iomanip>
    }

};

} // namespace gnuplotio

// The first version of this library didn't use namespaces, and now this must be here forever
// for reverse compatibility.
using gnuplotio::Gnuplot;

#endif // GNUPLOT_IOSTREAM_HPP
