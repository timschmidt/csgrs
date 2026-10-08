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
 * This computes the absolute value of a real. For now the argument
 * must be either a vector or a stream.
 */
Real
abs_R(Real x)
{
	Real r;
	Cls *cls;
	SignX *signX;
	int sign_a, sign_b;
	void force_To_SignX_From_Abs_Entry();
	void force_To_SignX_From_Abs_Cont();
	void force_To_DigsX_From_Abs_Entry();
	void force_To_DigsX_From_Abs_Cont();
	void force_To_DigsX_From_Abs_Copy_Entry();
	void force_To_DigsX_From_Abs_Copy_Cont();
	void force_To_DigsX_From_Abs_Copy_and_Negate();
	void force_To_DigsX_From_Abs_Copy_and_Negate_Entry();
	void force_To_DigsX_From_Abs_Copy_and_Negate_Cont();
	static int doneInit = 0;

	if (!doneInit) {
		registerForceFunc(force_To_SignX_From_Abs_Entry,
							"force_To_SignX_From_Abs_Entry", 2);
		registerForceFunc(force_To_SignX_From_Abs_Cont,
							"force_To_SignX_From_Abs_Cont", 2);
		registerForceFunc(force_To_DigsX_From_Abs_Entry,
							"force_To_DigsX_From_Abs_Entry", 3);
		registerForceFunc(force_To_DigsX_From_Abs_Cont,
							"force_To_DigsX_From_Abs_Cont", 3);
		registerForceFunc(force_To_DigsX_From_Abs_Copy_Entry,
							"force_To_DigsX_From_Abs_Copy_Entry", 3);
		registerForceFunc(force_To_DigsX_From_Abs_Copy_Cont,
							"force_To_DigsX_From_Abs_Copy_Cont", 3);
		registerForceFunc(force_To_DigsX_From_Abs_Copy_and_Negate_Entry,
							"force_To_DigsX_From_Abs_Copy_and_Negate_Entry", 3);
		registerForceFunc(force_To_DigsX_From_Abs_Copy_and_Negate_Cont,
							"force_To_DigsX_From_Abs_Copy_and_Negate_Cont", 3);
		doneInit++;
	}

	/*
	 * A slight optimization. In the case of a vector we just inspect the 
	 * entries and then make them both the same sign (in a copy of the vector).
	 */
	if (x->gen.tag.type == VECTOR) {
		sign_a = mpz_sgn(x->vec.vec[0]);
		sign_b = mpz_sgn(x->vec.vec[1]);
		if (sign_a * sign_b < 0) {
			r = vector_Z(x->vec.vec[0], x->vec.vec[1]);
			mpz_neg(r->vec.vec[0], r->vec.vec[0]);
			return r;
		}
		else
			return x;
	}

	/*
	 * For all other argument types, we simply work with the stream
	 * representation of the argument.
	 */
	if (x->gen.tag.isSigned) {
		r = makeStream(x);	/* this will make a signed stream */

		/*
		 * All the work is done in the closure.
		 */
		cls = allocCls(force_To_SignX_From_Abs_Entry, (void *) r);
		if (DAVINCI) {
			beginGraphUpdate();
			newEdgeToOnlyChild(cls, r);
			endGraphUpdate();
		}
		signX = allocSignX((Real) cls, SIGN_UNKN);
		signX->force = force_To_SignX_From_Abs_Entry;
	}
	else
		return x;
}

/*
 * In spite of the fact that the output is positive, we don't
 * really know the sign of abs_R(x) until we get the sign of x.
 */
void
force_To_SignX_From_Abs_Entry()
{
	SignX *signX, *source;
	Cls *cls;
	void force_To_SignX_From_Abs_Cont();

	signX = (SignX *) POP;
	cls = (Cls *) signX->x;
	source = (SignX *) cls->userData;

	PUSH_2(force_To_SignX_From_Abs_Cont, signX);

	if (source->tag.value == SIGN_UNKN)
		PUSH_2(source->force, source);
}

/*
 * When this gets activated, then we know the sign of the argument
 * has been determined. The sign of the output is the same as the argument
 * unless the argument is SNEG in which case we switch to SPOS.
 */
void
force_To_SignX_From_Abs_Cont()
{
	SignX *signX;
	Cls *cls;
	SignX *source;
	void force_To_DigsX_From_Abs_Entry();

	signX = (SignX *) POP;
	cls = (Cls *) signX->x;
	source = (SignX *) cls->userData;

	/*
	 * This allocates an empty DigsX structure between the sign and the
	 * closure. That's where we will copy digits to eventually.
	 */
	introDigsX(signX);
	signX->x->digsX.force = force_To_DigsX_From_Abs_Entry;

	switch (source->tag.value) {
	case SPOS :
	case SZERO :
	case SINF :
		signX->tag.value = source->tag.value;
		break;
	case SNEG :
		signX->tag.value = SPOS; /* the only case where sign is changed */
		break;
	case SIGN_UNKN :
		Error(FATAL, E_INT, "force_To_SignX_From_Abs_Cont", "sign not set");
	default :
		Error(FATAL, E_INT, "force_To_SignX_From_Abs_Cont", "bad sign");
	}

	/*
	 * Now we skip over the sign of the argument. It has been ``absorbed''.
	 * But we keep a copy of the value for future use.
	 */
	cls->tag.value = source->tag.value;
	cls->userData = (void *) source->x;

	if (DAVINCI) {
		beginGraphUpdate();
		deleteOnlyEdge(cls, source);
		newEdgeToOnlyChild(cls, source->x);
		endGraphUpdate();
	}
}

void
force_To_DigsX_From_Abs_Entry()
{
	DigsX *target, *source;
	Cls *cls;
	int digitsNeeded;
	void force_To_DigsX_From_Abs_Cont();

	target = (DigsX *) POP;
	digitsNeeded = (int) POP;
	cls = (Cls *) target->x;
	source = (DigsX *) cls->userData;

	PUSH_3(force_To_DigsX_From_Abs_Cont, target, digitsNeeded);

	/*
	 * Now see if the source has the number of digits we need. If not,
	 * then force the remaining.
	 */
	if (source->count < digitsNeeded)
		PUSH_3(source->force, source, digitsNeeded - source->count);
}

/*
 * The is the function which translates one stream of digits into 
 * another. The rewrite rules are as follows depending on the sign of
 * the argument x.
 *   SZERO : if the word in the characteristic pair is negative, then
 *         the argument is negative and we must negate the word and every
 *         word in subsequent characteristic pairs.
 *   SINF : if the word in the characteristic pair is positive, then
 *         the argument is negative and we must negate the word and every
 *         word in subsequent characteristic pairs.
 *   SNEG : the number is negative, so we must negate the word of every
 *         characteristic pair.
 *   SPOS : just copy characteristic pairs from the source to the target.
 *         There *might* be an opportunity to optimize things by just
 *         short circuiting around the closure. Needs thought.
 * The code is more or less copied from the force methods for DigsX structures
 * with the negation stuff thrown in along the way.
 */
void
force_To_DigsX_From_Abs_Cont()
{
	DigsX *target, *source;
	Cls *cls;
	int digitsNeeded;
	void force_To_DigsX_From_Abs_Copy_Entry();
	void force_To_DigsX_From_Abs_Copy_and_Negate_Entry();

	target = (DigsX *) POP;
	digitsNeeded = (int) POP;
	cls = (Cls *) target->x;
	source = (DigsX *) cls->userData;

	if (source->count > 0) {
		switch (cls->tag.value) {
		case SZERO :
#ifdef OPTIM
			if (source->count <= DIGITS_PER_WORD) {
				if (source->word.small < 0)
					cls->force = force_To_DigsX_From_Abs_Copy_and_Negate_Entry;
				else
					cls->force = force_To_DigsX_From_Abs_Copy_Entry;
			}
			else {
#endif
				if (mpz_sgn(source->word.big) < 0)
					cls->force = force_To_DigsX_From_Abs_Copy_and_Negate_Entry;
				else
					cls->force = force_To_DigsX_From_Abs_Copy_Entry;
#ifdef OPTIM
			}
#endif
			break;

		case SINF:
#ifdef OPTIM
			if (source->count <= DIGITS_PER_WORD) {
				if (source->word.small > 0)
					cls->force = force_To_DigsX_From_Abs_Copy_and_Negate_Entry;
				else
					cls->force = force_To_DigsX_From_Abs_Copy_Entry;
			}
			else {
#endif
				if (mpz_sgn(source->word.big) > 0)
					cls->force = force_To_DigsX_From_Abs_Copy_and_Negate_Entry;
				else
					cls->force = force_To_DigsX_From_Abs_Copy_Entry;
#ifdef OPTIM
			}
#endif
			break;

		case SNEG:
			cls->force = force_To_DigsX_From_Abs_Copy_and_Negate_Entry;
			break;

		case SPOS:
			cls->force = force_To_DigsX_From_Abs_Copy_Entry;
			break;

		default :
			Error(FATAL, E_INT, "force_To_DigsX_From_Abs_Cont",
					"bad sign");
		}
		target->force = cls->force;
		PUSH_3(target->force, target, digitsNeeded);
	}
}

void
force_To_DigsX_From_Abs_Copy_and_Negate_Entry()
{
	DigsX *target, *source;
	Cls *cls;
	int digitsNeeded;
	void force_To_DigsX_From_Abs_Copy_and_Negate_Cont();

	target = (DigsX *) POP;
	digitsNeeded = (int) POP;
	cls = (Cls *) target->x;
	source = (DigsX *) cls->userData;

	PUSH_3(force_To_DigsX_From_Abs_Copy_and_Negate_Cont, target, digitsNeeded);

	/*
	 * See if the source has the number of digits we need. If not,
	 * then force the remaining.
	 */
	if (source->count < digitsNeeded)
		PUSH_3(source->force, source, digitsNeeded - source->count);
}

void
force_To_DigsX_From_Abs_Copy_and_Negate_Cont()
{
	DigsX *target, *source;
	Cls *cls;
	int digitsNeeded;

	target = (DigsX *) POP;
	digitsNeeded = (int) POP;
	cls = (Cls *) target->x;
	source = (DigsX *) cls->userData;

#ifdef OPTIM
	if (target->count + source->count <= DIGITS_PER_WORD)
			target->word.small = target->word.small * (1 << source->count)
										- source->word.small;
	else {
		if (target->count <= DIGITS_PER_WORD)
			mpz_init_set_si(target->word.big, target->word.small);
#endif
		mpz_mul_2exp(target->word.big, target->word.big, source->count);
#ifdef OPTIM
		if (source->count <= DIGITS_PER_WORD) {
			if (source->word.small >= 0)
				mpz_sub_ui(target->word.big, target->word.big,
												source->word.small);
			else
				mpz_add_ui(target->word.big, target->word.big,
												-(source->word.small));
		}
		else
#endif
			mpz_sub(target->word.big, target->word.big, source->word.big);
#ifdef OPTIM
	}
#endif

	target->count += source->count;

	if (TRACE) {
		debugp("force_To_DigsX_From_Abs_Copy_and_Negate",
				"%x %x emitted=%d\n",
				(unsigned) target,
				(unsigned) source,
				source->count);
	}

	/*
	 * We've consumed the source so advance to the next possible source
	 * of information
	 */
	cls->userData = (void *) source->x;

	if (DAVINCI) {
		beginGraphUpdate();
		deleteOnlyEdge(cls, source);
		newEdgeToOnlyChild(cls, source->x);
		endGraphUpdate();
	}
	newDigsX(target);
}

void
force_To_DigsX_From_Abs_Copy_Entry()
{
	DigsX *target, *source;
	Cls *cls;
	int digitsNeeded;
	void force_To_DigsX_From_Abs_Copy_Cont();

	target = (DigsX *) POP;
	digitsNeeded = (int) POP;
	cls = (Cls *) target->x;
	source = (DigsX *) cls->userData;

	PUSH_3(force_To_DigsX_From_Abs_Copy_Cont, target, digitsNeeded);

	/*
	 * See if the source has the number of digits we need. If not,
	 * then force the remaining.
	 */
	if (source->count < digitsNeeded)
		PUSH_3(source->force, source, digitsNeeded - source->count);
}

void
force_To_DigsX_From_Abs_Copy_Cont()
{
	DigsX *target, *source;
	Cls *cls;
	int digitsNeeded;

	target = (DigsX *) POP;
	digitsNeeded = (int) POP;
	cls = (Cls *) target->x;
	source = (DigsX *) cls->userData;

#ifdef OPTIM
	if (target->count + source->count <= DIGITS_PER_WORD)
		target->word.small = target->word.small * (1 << source->count)
										+ source->word.small;
	else {
		if (target->count <= DIGITS_PER_WORD)
			mpz_init_set_si(target->word.big, target->word.small);
#endif
		mpz_mul_2exp(target->word.big, target->word.big, source->count);
#ifdef OPTIM
		if (source->count <= DIGITS_PER_WORD) {
			if (source->word.small >= 0) {
				mpz_add_ui(target->word.big, target->word.big,
												source->word.small);
			}
			else {
				mpz_sub_ui(target->word.big, target->word.big,
												-(source->word.small));
			}
		}
		else
#endif
			mpz_add(target->word.big, target->word.big, source->word.big);
#ifdef OPTIM
	}
#endif

	target->count += source->count;

	if (TRACE) {
		debugp("force_To_DigsX_From_Abs_Copy",
				"%x %x emitted=%d\n",
				(unsigned) target,
				(unsigned) source,
				source->count);
	}

	/*
	 * We've consumed the source so advance to the next possible source
	 * of information
	 */
	cls->userData = (void *) source->x;

	if (DAVINCI) {
		beginGraphUpdate();
		deleteOnlyEdge(cls, source);
		newEdgeToOnlyChild(cls, source->x);
		endGraphUpdate();
	}
	newDigsX(target);
}

#ifdef JUNK
In some cases it would be better to use the matices to
negate things. But to do this for absolute value
we must know the sign of the argument.

As a rule we want to use a matrix for reciprocal and negation
except at the top level when we might prefer to use algorithms
in Peters thesis. This is also the case when we real is
the argument to a predicate.
#endif
