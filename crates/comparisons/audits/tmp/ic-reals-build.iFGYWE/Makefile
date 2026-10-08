REALDIR=/home/lindsay/Desktop/ic-reals-6.2-beta
GMPDIR=/home/lindsay/Desktop/gmp-4.3.1

# -DDAVINCI enables the davinci interface
# -DTRACE=traceOn enables tracing accoring to library function debugTrace()
# -DTRACE=0 disable tracing completely (or simply omit -DTRACE)
# -DSTACK_SIZE is the number of K words in the stack

CFLAGS = \
	$(INCLUDE) \
	-g \
	-DPACK_DIGITS \
	-DDEFAULT_FORCE_COUNT=1 \
	-DSTACK_SIZE=20 \
	-DFORCE_DEC_UPPER_BOUND=10000 \
	-DREALDIR=\\\"$(REALDIR)\\\" \
	-O3

# The following flags are not used. They are here for convenience; they can
# be cut and pasted easily into the list above
OTHER_FLAGS = \
	-pg \
	-DTRACE=traceOn \
	-DDAVINCI \
	-DTRACE=1 \
	-DDEFAULT_FORCE_COUNT=1 \
	END

LIB=$(GMPDIR)/libgmp.a

INCLUDE = \
    -I.. \
    -I$(GMPDIR) \
    -I$(GMPDIR)/mpn/

CC = gcc
FLAGS_TO_PASS = \
	"CC=$(CC)" \
	"CFLAGS=$(CFLAGS)" \
	"GMPDIR=$(GMPDIR)" \
	"REALDIR=$(REALDIR)"

real.a : force
	cd base; $(MAKE) $(FLAGS_TO_PASS)
	cd math-lib; $(MAKE) $(FLAGS_TO_PASS)
	ar rc real.a math-lib/*.o base/*.o

clean:
	cd base; $(MAKE) clean
	cd math-lib; $(MAKE) clean
	cd tests; $(MAKE) clean
	rm -f real.a

force:
.PHONY: force
