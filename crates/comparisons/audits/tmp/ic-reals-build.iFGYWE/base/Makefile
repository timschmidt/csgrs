# -DDAVINCI enables the davinci interface
# -DEBUG some debugging (for now just turns on nodeIds)
# -TRACE=traceOn enables tracing accoring to library function debugTrace()

OBJS = \
	Vector.o \
	DigsX.o \
	MatX.o \
	TenXY.o \
	SignX.o \
	Alt.o \
	realLib.o \
	boolLib.o \
	force_R.o \
	force_B.o \
	forceFuncLookupTable.o \
	delay.o \
	emitDigit.o \
	emitSign.o \
	digitHandling.o \
	util.o \
	dump.o \
	epsDel.o \
	debug.o \
	nodeId.o \
	strategy.o \
	print.o \
	boolUtil.o \
	gteq0.o \
	gt0.o \
	boolOp.o \
	stack.o \
	davinciInterface.o \
	error.o

# garbage.o
# strsep.o
# strictAlt.o
# reduce.o

force : $(OBJS)

$(OBJS): ../real.h ../real-impl.h

clean:
	rm -f $(OBJS)
