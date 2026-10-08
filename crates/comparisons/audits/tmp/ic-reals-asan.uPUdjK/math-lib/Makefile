OBJS = init.o \
	tan_R.o \
	tan_Q.o \
	atan_R.o \
	atan_Q.o \
	tanh_R.o \
	tanh_Q.o \
	atanh_R.o \
	atanh_Q.o \
	sin_R.o \
	sin_Q.o \
	asin_R.o \
	asin_Q.o \
	sinh_R.o \
	sinh_Q.o \
	asinh_R.o \
	asinh_Q.o \
	cos_R.o \
	cos_Q.o \
	acos_R.o \
	acos_Q.o \
	cosh_R.o \
	cosh_Q.o \
	acosh_R.o \
	acosh_Q.o \
	secant.o \
	cosecant.o \
	cotangent.o \
	log_R.o \
	log_Q.o \
	exp_R.o \
	exp_Q.o \
	sqrt_R.o \
	sqrt_Q.o \
	pow_R_R.o \
	pi.o \
	abs_R.o \
	neg_R.o \
	stdTensorCont.o

force: $(OBJS)
$(OBJS): ../real.h ../real-impl.h

stdTensor.o : math-lib.h
tan_R.o : math-lib.h
atan_R.o : math-lib.h
exp_R.o : math-lib.h
log_R.o : math-lib.h

clean:
	rm -f $(OBJS)
