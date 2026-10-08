/*
 * Copyright (C) 2000, Imperial College
 *
 * This file is part of the Imperial College Exact Real Arithmetic Library.
 * See the copyright notice included in the distribution for conditions
 * of use.
 */

#include <stdio.h>
#include "real.h"
#include "real-impl.h"

/*
 * The ``abstract machine''  uses a single stack to record what work needs
 * to be done. Each frame is consists of a pointer to a force function
 * and the arguments to the force function.
 */

StackWord *stack;
StackWord *stackBound;
StackWord *sp;

void
initStack()
{
	if ((stack = (StackWord *) malloc(stackSize * sizeof(StackWord))) == NULL)
		Error(FATAL, E_INT, "initStack", "malloc failed");
	sp = stack - 1;
	stackBound = stack + stackSize;
}

void
runStack()
{
	void (*f)();
	void dumpTopOfStack();

	if (DAVINCI)
		runStackViaDaVinci();
	else {
		while (sp >= stack) {
			if (TRACE)
				dumpTopOfStack();
			f = (void (*)()) POP;
			(*f)();
		}
	}
}

/*
 * Retrieves the descriptor for the top of the stack.
 */
ForceFuncDesc *
getDescForTOS()
{
	ForceFuncDesc *p;

	p = getDescForForceFunc((void (*)()) *sp);

	if (p == NULL) {
		debugp("", "tos = %x\n", (unsigned) *sp);
		Error(FATAL, E_INT, "getDescForTOS", "failed to find %x", (void *) *sp);
	}
	return p;
}

void
dumpStack()
{
	StackWord *p;
	ForceFuncDesc *d;

	for (p = sp; p >= stack; p--) {
		fprintf(stderr, "%x %d", *p, *p);
		d = getDescForForceFunc((void (*)()) *p);
		if (d != NULL)
			fprintf(stderr, " %s", d->funcName);
		fprintf(stderr, "\n");
	}
		
}

void
dumpTopOfStack()
{
	ForceFuncDesc *q;
	StackWord *sp1;

	if (sp >= stack) {
		q = getDescForTOS();

		/*
		 * If there are not the expected number of arguments on the stack
		 * then we dump what we can and then bail out.
		 */
		if (sp - stack < q->nArgs - 1) {
			fprintf(stderr, "%s ", q->funcName);
			for (sp1 = sp - 1; sp1 >= stack; sp1--)
				fprintf(stderr, "%x ", *sp1);
			fprintf(stderr, "\n");
			Error(FATAL, E_INT, "dumpTopOfStack", "stack underflow");
		}

		switch (q->nArgs) {
			case 2 :
				debugp(q->funcName, "%x", (unsigned) *(sp - 1));
				break;
			case 3 :
				debugp(q->funcName, "%x %d", (unsigned) *(sp - 1),
												(unsigned) *(sp - 2));
				break;
			case 4 :
				debugp(q->funcName, "%x %x %d", (unsigned) *(sp - 1),
													(unsigned) *(sp - 2),
													(unsigned) *(sp - 3));
				break;
			default :
				break;
		}
		fprintf(stderr, "\n");
	}
}

/*
 * The following function will highlight the edge in the daVinci
 * window associated with the frame at the top of the stack. This is
 * called after a new frame is pushed.
 */
void
highlightTOS()
{
	ForceFuncDesc *p;
	Real x;

	p = getDescForTOS();
	x = (Real) (*(sp - 1));

	switch (x->gen.tag.type) {
	case ALT :
		break;
	case VECTOR :
		break;
	case MATX :
		highlightEdge(x, ((MatX *) x)->x, 0);
		break;
	case TENXY :
		if (isLeftFunc(p))
			highlightEdge(x, ((TenXY *) x)->x, 0);
		else {
			if (isRightFunc(p))
				highlightEdge(x, ((TenXY *) x)->y, 1);
		}
		break;
	case DIGSX :
		highlightEdge(x, ((DigsX *) x)->x, 0);
		break;
	case SIGNX :
		highlightEdge(x, ((SignX *) x)->x, 0);
		break;
	case CLOSURE :
		break;
	case BOOLX :
		highlightEdge(x, ((BoolX *) x)->x, 0);
		break;
	case BOOLXY :
		if (isLeftFunc(p))
			highlightEdge(x, ((BoolXY *) x)->x, 0);
		else {
			if (isRightFunc(p))
				highlightEdge(x, ((BoolXY *) x)->y, 1);
		}
		break;
	case PREDX :
		highlightEdge(x, ((PredX *) x)->x, 0);
		break;
	default :
		Error(FATAL, E_INT, "highlightTOS", "object has unknown type");
	}
}

void
unhighlightTOS()
{
	ForceFuncDesc *p;
	Real x;

	p = getDescForTOS();
	x = (Real) (*(sp - 1));

	switch (x->gen.tag.type) {
	case ALT :
		break;
	case VECTOR :
		break;
	case MATX :
		unhighlightEdge(x, ((MatX *) x)->x, 0);
		break;
	case TENXY :
		if (isLeftFunc(p))
			unhighlightEdge(x, ((TenXY *) x)->x, 0);
		else {
			if (isRightFunc(p))
				unhighlightEdge(x, ((TenXY *) x)->y, 1);
		}
		break;
	case DIGSX :
		unhighlightEdge(x, ((DigsX *) x)->x, 0);
		break;
	case SIGNX :
		unhighlightEdge(x, ((SignX *) x)->x, 0);
		break;
	case CLOSURE :
		break;
	case BOOLX :
		unhighlightEdge(x, ((BoolX *) x)->x, 0);
		break;
	case BOOLXY :
		if (isLeftFunc(p))
			unhighlightEdge(x, ((BoolXY *) x)->x, 0);
		else
			if (isRightFunc(p))
				unhighlightEdge(x, ((BoolXY *) x)->y, 1);
		break;
	case PREDX :
		unhighlightEdge(x, ((PredX *) x)->x, 0);
		break;
	default :
		Error(FATAL, E_INT, "unhighlightTOS", "object has unknown type");
	}
}

int
isLeftFunc(ForceFuncDesc *p)
{
	return (p->argXOrY == ARG_X);
}

int
isRightFunc(ForceFuncDesc *p)
{
	return (p->argXOrY == ARG_Y);
}
