# KMB 2003 Mar 31, Apr 18; 2006 Jul 27
# Makefile for xrc

VERSION=1.2 # FIXME make this set version info everywhere
CC=/usr/bin/gcc
CPP=/usr/bin/g++
CC=/usr/bin/gcc
CPP=/usr/bin/g++
#GCC_FLAGS=-g -ansi
GCC_FLAGS=-Wall -ansi -O3
GMPINC=#-I/usr/local/include
GMPLIB=#-L/usr/local/lib
PROF=#-pg
SOURCES=README Makefile xr.1 xr.h *.c *.cc

xr.o: xr.c xr.h
	${CC} $(GMPINC) $(GCC_FLAGS) $(PROF) -DTIMING -c xr.c
	@echo Now do \"make test\" to run the tests

test: pi_test graham_test exp_test harmonic_test logistic_test logistic_test++ mueller_test 
	@echo ==== pi test =================================================
	time ./pi_test 4
	@echo ==== graham test =================================================
	time ./graham_test
	# delete this if you don't have graphviz ...
	dot -Tps graham_test.dot >| graham_test.ps 
	@echo ==== exp test ===============================================
	time ./exp_test
	@echo ==== harmonic test ===============================================
	time ./harmonic_test
	@echo ===== logistic test ==============================================
	time ./logistic_test
	@echo ===== logistic C++ test =========================================
	time ./logistic_test++
	@echo ====== mueller test ==============================================
	time nice ./mueller_test 2

%_test: %_test.c xr.o
	${CC} $(GMPINC) $(GCC_FLAGS) $(PROF) $< xr.o -lm $(GMPLIB) -lgmp -o $@
	
%_test++: %_test++.cc xr++.h xr.o
	${CPP} $(GCC_FLAGS) $(PROF) $< xr.o -lm -lgmp -o $@
	
man: xr.1
	groff -man -e xr.1 > xr.1.ps
	ps2pdf xr.1.ps
	man2html xr.1 > xr.1.html

exp_test.dot: exp_test
	./exp_test

graham_test.dot: graham_test
	./graham_test

fig: exp_test.dot graham_test.dot
	dot -Tps2 exp_test.dot > exp_test.ps
	dot -Tps2 graham_test.dot > graham_test.ps

flop:
	make tar
	mcopy -o ../xrc-1.2.tar.gz a:
	#mcopy -v -o $(SOURCES) exclude-from-tar a:

bu:
	scp $(SOURCES) europa:xrc-1.2
	#scp $(SOURCES) wulf1:xrc-1.2

clean:
	rm -f a.out core *o *test *test++ example graham_test.dot graham_test.ps graham_test.pdf exp_test.dot exp_test.ps exp_test.pdf


tar:
	make clean
	make man
	(cd ..; tar --exclude-from xrc-1.2/exclude-from-tar -zcvf xrc-1.2.tar.gz xrc-1.2)
