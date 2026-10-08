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
 * This file contains support for realIf (aka alternation).
 */

#include <stdarg.h>

Real
realIf(int numGE, ...)
{
	va_list ap;
	Alt *alt;
	GuardedExpr *geList;
	void force_To_Alt_From_The_Abyss();
	int i;
	bool isSigned = FALSE;

	if ((alt = (Alt *) malloc(sizeof(Alt))) == NULL)
		Error(FATAL, E_INT, "realIf", "malloc failed (Alt)");

	if (TRACE || DAVINCI)
		newNodeId(alt);

	va_start(ap, numGE);
	
	if ((geList = (GuardedExpr *) malloc(sizeof(GuardedExpr) * numGE)) == NULL)
		Error(FATAL, E_INT, "realIf", "malloc failed (GE)");

	alt->tag.type = ALT;
	alt->tag.dumped = FALSE;

	alt->GE = geList;
	alt->numGE = numGE;
	alt->nextGE = 0;
	alt->force = force_To_Alt_From_The_Abyss;
	alt->redirect = NULL;

	if (DAVINCI) {
		beginGraphUpdate();
		newNode(alt, ALT);
		endGraphUpdate();
	}

	/*
	 * Now we consume the arguments. Each is a guard/value pair. 
	 */
	for (i = 0; i < numGE; i++) {
		geList[i].guard = va_arg(ap, Bool);
		geList[i].x = va_arg(ap, Real);
		isSigned = isSigned || geList[i].x->gen.tag.isSigned;

		if (DAVINCI) {
			beginGraphUpdate();
			newEdgeToChildN(alt, geList[i].guard, i);
			newEdgeToChildN(alt, geList[i].x, i);
			endGraphUpdate();
		}
	}
	va_end(ap);

	alt->tag.isSigned = isSigned;
	return (Real) alt;
}

void
force_To_Alt_From_The_Abyss()
{
	Error(FATAL, E_INT, "force_To_Alt_From_The_Abyss",
					"trying to force a conditional");
}

/*
 * This force method is used to force (evaluate) an alt. To accommodate
 * the case where the alt itself evaluates to an alt, the force methods are
 * applied recursively and the alt chain reduced. Thus, once the force is
 * complete, the alt on the stack has a value alt->redirect which is a real
 * and which is not itself an alt. If we were writing a recursive function,
 * rather than using an explicit stack, the following three functions
 * including the reduction would be coded as a single function as follows:
 * 
 * force_Alt(Alt *alt)
 * {
 *   if (alt->redirect == NULL)
 *     force_Alt_Eval(alt);     evaluate the single alt and set alt->redirect 
 *   if (alt->redirect->gen.tag.type == ALT) {
 *     force_Alt(alt->redirect);   make the recusive call and do the reduction
 *     alt->redirect = alt->redirect->alt.redirect;
 *   }  
 * }
 */
void
force_To_Alt_Entry()
{
	Alt *alt;
	void force_To_Alt_Cont();
	void force_Alt_Eval();
	Bool guard;

	alt = (Alt *) POP;
	
	PUSH_2(force_To_Alt_Cont, alt);

	/*
	 * If alt->redirect is not valid (equals NULL) then the value of
	 * the conditional has not been determined so we need to force it.
	 * This means forcing the first guard.
	 */
	if (alt->redirect == NULL) {
		PUSH_2(force_Alt_Eval, alt);
		guard = alt->GE[alt->nextGE].guard;
		PUSH_2(guard->gen.force, guard);
	}
}

void
force_To_Alt_Cont()
{
	Alt *alt;
	void force_To_Alt_Reduce();

	alt = (Alt *) POP;

	/*
	 * So we have evaluated the alternation and alt->redirect is the real
	 * associated with the guard that evaluated to true. However, that
	 * real may itself be an alt. If so, then we arrange to evaluate the
	 * second alt and also to reduce the chain.
	 */
	if (alt->redirect->gen.tag.type == ALT) {
		PUSH_2(force_To_Alt_Reduce, alt);
		PUSH_2(force_To_Alt_Entry, alt->redirect);
	}
}

/*
 * The following is used only when the alt evaluates to a real which is
 * itself an alt. This function performs the reduction.
 */
void
force_To_Alt_Reduce()
{
	Alt *alt;

	alt = (Alt *) POP;

	if (DAVINCI) {
		beginGraphUpdate();
		deleteOnlyEdge(alt, alt->redirect);
		newEdgeToOnlyChild(alt, alt->redirect->alt.redirect);
		endGraphUpdate();
	}

	alt->redirect = alt->redirect->alt.redirect;

	if (alt->redirect->gen.tag.type == ALT)
		Error(FATAL, E_INT, "force_To_Alt_Cont_Cont",
			"alt chain not fully reduced");
}

/*
 * Our objective here is to evaluate a single conditional. This means
 * forcing each guard in turn and checking to see if it becomes true.
 * This force function is only activated by force_To_Alt_Entry and by itself.
 * When activated, we know that we have forced one of the guards so
 * we start by checking the value of the guard.
 */
void
force_Alt_Eval()
{
	Alt *alt;
	Bool guard;
	int advanceToNextGE(Alt *alt);

	alt = (Alt *) POP;
	guard = alt->GE[alt->nextGE].guard;

	switch (guard->gen.tag.value) {
	/*
	 * If the current guard is true, then record the value of the guard
	 * in the Alt structure. Thereafter we will used the stored value. At this
	 * point it might be wise to make all the pointers in the guard/value
	 * pairs to NULL so that the garbage collector can have them.
	 */
	case LAZY_TRUE :
		alt->redirect = alt->GE[alt->nextGE].x;
		if (DAVINCI) {
			deleteEdgeToChildN(alt, alt->GE[alt->nextGE].guard, alt->nextGE);
			deleteEdgeToChildN(alt, alt->GE[alt->nextGE].x, alt->nextGE);
			drawEqEdge(alt, alt->GE[alt->nextGE].x);
			alt->GE[alt->nextGE].guard = NULL;
			alt->GE[alt->nextGE].x = NULL;
			while (advanceToNextGE(alt)) {
				deleteEdgeToChildN(alt, alt->GE[alt->nextGE].guard, alt->nextGE);
				deleteEdgeToChildN(alt, alt->GE[alt->nextGE].x, alt->nextGE);
				alt->GE[alt->nextGE].guard = NULL;
				alt->GE[alt->nextGE].x = NULL;
			}
		}
		break;

	/*
	 * If the current guard evaluates to false, then we want to eliminate the
	 * the guard (and value) from future consideration. So we set the fields
	 * to NULL and advance to the next pair and force the guard.
	 */
	case LAZY_FALSE :
		if (DAVINCI) {
			beginGraphUpdate();
			deleteEdgeToChildN(alt, alt->GE[alt->nextGE].guard, alt->nextGE);
			deleteEdgeToChildN(alt, alt->GE[alt->nextGE].x, alt->nextGE);
			endGraphUpdate();
		}
		alt->GE[alt->nextGE].guard = NULL;
		alt->GE[alt->nextGE].x = NULL;
		if (!advanceToNextGE(alt))
			Error(FATAL, E_INT, "force_Alt_Eval", "no guards left");
		PUSH_2(force_Alt_Eval, alt);
		PUSH_2(alt->GE[alt->nextGE].guard->gen.force,
								alt->GE[alt->nextGE].guard);
		break;

	/*
	 * Finally, if the guard is still unknown, we simply advance to the
	 * next pair and force the guard.
	 */
	case LAZY_UNKNOWN :
		if (!advanceToNextGE(alt))
			Error(FATAL, E_INT, "force_Alt_Eval", "no guards left");
		PUSH_2(force_Alt_Eval, alt);
		PUSH_2(alt->GE[alt->nextGE].guard->gen.force,
								alt->GE[alt->nextGE].guard);
		break;

	default :
		Error(FATAL, E_INT, "force_Alt_Eval",
			"invalid boolean value encountered");
		break;
	}
}

/*
 * This local function scans through the list of guarded expressions looking
 * for the index of the next valid pair following the given index. Pairs in
 * which the guard has been evaluated to false have been set to NULL.
 * So we are looking for the first non-NULL pair. It may be the same
 * index we are given though to be honest, if we are down to one GE pair
 * without a true guard, then there is a pretty good chance the programmer
 * has chosen guards which don't overlap.
 * This returns 0 (false) if no pair is found.
 */
int
advanceToNextGE(Alt *alt)
{
	int i;

	for (i = (alt->nextGE + 1) % alt->numGE;
				i != alt->nextGE; i = (i + 1) % alt->numGE) {
		if (alt->GE[i].guard != NULL) {
			alt->nextGE = i;
			return 1;
		}
	}

	if (alt->GE[i].guard != NULL) {
			alt->nextGE = i;
			return 1;
	}
	return 0;
}
