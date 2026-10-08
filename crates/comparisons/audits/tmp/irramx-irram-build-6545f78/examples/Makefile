# Makefile for user_programs

prefix      = /tmp/irramx-irram-install-6545f78
exec_prefix = ${prefix}

CC          = gcc
CFLAGS      = -g -O2
CPP         = @CPP@
CPPFLAGS    =     -I${prefix}/include
CXX         = g++
CXXCPP      = g++ -E
CXXFLAGS    = -g -O2
LDFLAGS     = -Xlinker -rpath -Xlinker ${exec_prefix}/lib
LDLIBS      = -L${exec_prefix}/lib -liRRAM -lmpfr -lgmp  -lm

EXAMPLES_BIN = \
	algebraic-BFMS \
	analytic \
	e_example \
	euler \
	factorial \
	fileio \
	float_extension \
	gamma_bernoulli \
	harmonic \
	hilbert \
	interval_test \
	itsyst \
	itsyst2 \
	itsyst3 \
	jmmuller \
	jmmuller2 \
	lambov \
	limit_example \
	matrix \
	mp_extension \
	pi_example \
	probe \
	rational_sep \
	sinus \
	swanseatest \
	swanseatest-2 \
	test_commandline \
	test_DYADIC \
	test_exceptions \
	test_INTEGER \
	test_module \
	test-MPFR-iRRAM \
	test_round \
	test_strings \
	test_values \
	thread_test \
	timings-iRRAM \
	timings-MPFR-iRRAM \

all: $(EXAMPLES_BIN)

thread_test: CXXFLAGS += -pthread

maintainer-clean: distclean

distclean: clean
	rm -f Makefile

clean:
	rm -f $(EXAMPLES_BIN)

install:
