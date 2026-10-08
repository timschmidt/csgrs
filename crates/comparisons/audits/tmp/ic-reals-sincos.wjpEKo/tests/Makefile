GMPDIR=$(HOME)/Desktop/gmp-4.3.1

LIB = \
	../real.a \
	$(GMPDIR)/.libs/libgmp.a \
	-lm

INCLUDE = \
	-I$(GMPDIR) \
	-I$(GMPDIR)/mpn \
	-I..

# CFLAGS = $(INCLUDE) -g -pg
CFLAGS = $(INCLUDE) -g
CC = gcc

tan_R : tan_R.o $(LIB)
	$(CC) tan_R.o $(LIB) -o tan_R

tan_R_digit : tan_R_digit.o $(LIB)
	$(CC) tan_R_digit.o $(LIB) -o tan_R_digit

asin_R : asin_R.o $(LIB)
	$(CC) asin_R.o $(LIB) -o asin_R

acos_R : acos_R.o $(LIB)
	$(CC) acos_R.o $(LIB) -o acos_R

atan_R : atan_R.o $(LIB)
	$(CC) atan_R.o $(LIB) -o atan_R

atanh_R : atanh_R.o $(LIB)
	$(CC) atanh_R.o $(LIB) -o atanh_R

acosh_R : acosh_R.o $(LIB)
	$(CC) acosh_R.o $(LIB) -o acosh_R

asinh_R : asinh_R.o $(LIB)
	$(CC) asinh_R.o $(LIB) -o asinh_R

pow_R_R : pow_R_R.o $(LIB)
	$(CC) pow_R_R.o $(LIB) -o pow_R_R

tan_QZ : tan_QZ.o $(LIB)
	$(CC) tan_QZ.o $(LIB) -o tan_QZ

cos_Q : cos_Q.o $(LIB)
	$(CC) cos_Q.o $(LIB) -o cos_Q

tanh_R : tanh_R.o $(LIB)
	$(CC) tanh_R.o $(LIB) -o tanh_R

sinh_R : sinh_R.o $(LIB)
	$(CC) sinh_R.o $(LIB) -o sinh_R

cosh_R : cosh_R.o $(LIB)
	$(CC) cosh_R.o $(LIB) -o cosh_R

sqrt_R : sqrt_R.o $(LIB)
	$(CC) sqrt_R.o $(LIB) -o sqrt_R

sqrt_QZ : sqrt_QZ.o $(LIB)
	$(CC) sqrt_QZ.o $(LIB) -o sqrt_QZ

abs_R : abs_R.o $(LIB)
	$(CC) abs_R.o $(LIB) -o abs_R

log_R : log_R.o $(LIB)
	$(CC) log_R.o $(LIB) -o log_R

exp_R : exp_R.o $(LIB)
	$(CC) exp_R.o $(LIB) -o exp_R

exp_QInt : exp_QInt.o $(LIB)
	$(CC) exp_QInt.o $(LIB) -o exp_QInt

sin_R : sin_R.o $(LIB)
	$(CC) sin_R.o $(LIB) -o sin_R

cos_R : cos_R.o $(LIB)
	$(CC) cos_R.o $(LIB) -o cos_R

pi : pi.o $(LIB)
	$(CC) pi.o $(LIB) -o pi

iter : iter.o $(LIB)
	$(CC) iter.o $(LIB) -o iter

t01 : t01.o $(LIB)
	$(CC) t01.o $(LIB) -o t01

t0 : t0.o $(LIB)
	$(CC) t0.o $(LIB) -o t0

t1 : t1.o $(LIB)
	$(CC) t1.o $(LIB) -o t1

t2 : t2.o $(LIB)
	$(CC) t2.o $(LIB) -o t2

t3 : t3.o $(LIB)
	$(CC) t3.o $(LIB) -o t3

iterate : iterate.o $(LIB)
	$(CC) iterate.o $(LIB) -o iterate

$(OBJS): ../real.h ../real-impl.h

TARGETS=\
	abs_R \
	acosh_R \
	asinh_R \
	acos_R \
	asin_R \
	atan_R \
	atanh_R \
	cos_Q \
	cos_R \
	cosh_R \
	exp_R \
	log_R \
	pi \
	pow_R_R \
	sin_R \
	sinh_R \
	sqrt_QZ \
	sqrt_R \
	tan_QZ \
	tan_R \
	tan_R_digit \
	tanh_R \
	t01 \
	t0 \
	t1 \
	t2 \
	iter \
	iterate

all : $(TARGETS)

clean:
	rm -f abs_R abs_R.o
	rm -f acosh_R acosh_R.o
	rm -f asinh_R asinh_R.o
	rm -f atan_R atan_R.o
	rm -f acos_R acos_R.o
	rm -f asin_R asin_R.o
	rm -f atanh_R atanh_R.o
	rm -f cos_Q cos_Q.o
	rm -f cos_R cos_R.o
	rm -f cosh_R cosh_R.o
	rm -f exp_R exp_R.o
	rm -f log_R log_R.o
	rm -f pi pi.o
	rm -f pow_R_R pow_R_R.o
	rm -f sin_R sin_R.o
	rm -f sinh_R sinh_R.o
	rm -f sqrt_QZ sqrt_QZ.o
	rm -f sqrt_R sqrt_R.o
	rm -f tan_QZ tan_QZ.o
	rm -f tan_R tan_R.o
	rm -f tan_R_digit tan_R_digit.o
	rm -f tanh_R tanh_R.o
	rm -f t01 t01.o
	rm -f t0 t0.o
	rm -f t1 t1.o
	rm -f t2 t2.o
	rm -f iterate iterate.o
	rm -f iter iter.o
	rm -f gmon.out
