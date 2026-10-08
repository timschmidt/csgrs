

# ConcLog

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT) [![Unix Status](https://github.com/ariadne-cps/conclog/workflows/Unix/badge.svg)](https://github.com/ariadne-cps/conclog/actions/workflows/unix.yml)
[![Windows Status](https://github.com/ariadne-cps/conclog/workflows/Windows/badge.svg)](https://github.com/ariadne-cps/conclog/actions/workflows/windows.yml) [![codecov](https://codecov.io/gh/ariadne-cps/conclog/branch/main/graph/badge.svg)](https://codecov.io/gh/ariadne-cps/conclog)

ConcLog is a library for concurrent logging.
It features the following:
1) Print from different threads with no overlapping of the output
2) Support for automatic registration/deregistration of threads (with example Thread implementation provided in the tests)
3) Set relative indentation on all logger calls, even on free functions
4) Set runtime verbosity to efficiently filter out unnecessarily detailed calls
5) Themes for highlighting keywords (and ability to add custom keywords)
6) Different output schedulers to offer different levels of guarantee on output order
7) Support for holding text on the bottom line, useful for progress indicators (provided in the library) and similar displays
8) A lot of configuration options for optionally printing entry/exit functions, thread identifiers, etc.

### Building

To build the library from sources in a clean way, it is preferable that you set up a build subdirectory, say:

```
$ mkdir build && cd build
```

Then you can prepare the build environment, choosing a Release build for maximum performance:

```
$ cmake .. -DCMAKE_BUILD_TYPE=Release
```

At this point, if no error arises, you can build with:

```
$ cmake --build .
```

The library is meant to be used as a dependency, in particular by disabling testing as long as the *tests* target is already defined in an enclosing project.

## Contribution guidelines ##

If you would like to contribute to ConcLog, please contact the developer: 

* Luca Geretti <luca.geretti@univr.it>
