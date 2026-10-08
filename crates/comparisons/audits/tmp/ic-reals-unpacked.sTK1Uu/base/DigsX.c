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
 * Digits are removed from LFTs and deposited into DigsX structure.
 * Each DigsX can hold an arbitrary number of digits. Nevertheless,
 * these structures are chained since it makes it easier whenr
 * there are many consumers.
 */

extern int epsDelMatrix(Matrix, int);
extern void epsDelTensor(Tensor, int, int *, int *);
extern int epsDelTensorX(Tensor, int);

extern bool emitDigitFromVector(Vector, Digit *);
extern bool emitDigitFromMatrix(Matrix, Digit *);
extern bool emitDigitFromTensor(Tensor, Digit *);

void setDigsXMethod(DigsX *);
void absorbDigsXIntoDigsX(DigsX *);

void redirectDigsX(DigsX *, Real);

DigsX *
allocDigsX()
{
	DigsX *digsX;

	if ((digsX = (DigsX *) malloc (sizeof(DigsX))) == NULL) 
		Error(FATAL, E_INT, "allocDigsX", "malloc failed");

	if (TRACE || DAVINCI)
		newNodeId(digsX);

	digsX->tag.type = DIGSX;
	digsX->tag.dumped = FALSE;
	digsX->tag.isSigned = FALSE;
	digsX->count = 0;
#ifdef PACK_DIGITS
	digsX->word.small = 0;
#else
	mpz_init(digsX->word.big);
#endif

	if (DAVINCI) {
		beginGraphUpdate();
		newNode(digsX, DIGSX);
		endGraphUpdate();
	}

	return digsX;
}

/*
 * The use of the following function is a little subtle. Digits are emitted
 * from LFTs into DigsX structures. Once this is done, we create a new
 * (empty) DigsX structure and link it between where the digits were
 * deposited and the residual of the LFT. This way, the consumer can take
 * the digits, advance its pointer to the argument of the DigsX, and still
 * be sure to point to another DigsX structure. It keeps the methods and
 * list handling much simpler.
 */
void
newDigsX(DigsX *digsX)
{
	DigsX *new;
	void force_To_DigsX_From_DigsX_Entry();

	new = allocDigsX();

	new->x = digsX->x;
	new->force = digsX->force;

	if (DAVINCI) {
		beginGraphUpdate();
		newEdgeToOnlyChild(new, digsX->x);
		deleteOnlyEdge(digsX, digsX->x);
		newEdgeToOnlyChild(digsX, new);
		endGraphUpdate();
	}

	digsX->x = (Real) new;
	digsX->force = force_To_DigsX_From_DigsX_Entry;
}

/*
 * This forces digits into and another list of digits. If the idea of
 * having lists of DigsX structures (rather than just one) then remember
 * that reals can be shared with digits consumed at different rates.
 */
void
force_To_DigsX_From_DigsX_Entry()
{
	DigsX *target, *source;
	int digitsNeeded;
	void force_To_DigsX_From_DigsX_Cont();

	target = (DigsX *) POP;
	digitsNeeded = (int) POP;
	source = (DigsX *) target->x;

	PUSH_3(force_To_DigsX_From_DigsX_Cont, target, digitsNeeded);

	/*
	 * Now see if the source has the number of digits we need. If not,
	 * then force the remaining.
	 */
	if (source->count < digitsNeeded)
		PUSH_3(source->force, source, digitsNeeded - source->count);
}

void
force_To_DigsX_From_DigsX_Cont()
{
	DigsX *target, *source;
	int digitsNeeded;

	target = (DigsX *) POP;
	digitsNeeded = (int) POP;
	source = (DigsX *) target->x;

	absorbDigsXIntoDigsX(target);
}

/*
 * This forces a vector to emit the requested number of digits.
 * vecCont is the real (Vec *) (ie a vector) from which the 
 * digits are required and digsX is where to place the digits. I'm not
 * sure this function will ever be used. Vectors represent
 * rationals so I suppose when the numerator and denominator are
 * very big numbers, and relatively little information is needed, then
 * we might pull digits out rather than consume the whole rational.
 *
 * Nevertheless, having vectors is extremely useful for debugging.
 */
void
force_To_DigsX_From_Vec()
{
	DigsX *digsX;
	Vec *vecCont;
	int digitsNeeded;
	int digitsEmitted, bitsShifted = 0;

	digsX = (DigsX *) POP;
	digitsNeeded = (int) POP;
	vecCont = (Vec *) digsX->x;

	digitsEmitted = emitDigits(digsX,
							(edf) emitDigitFromVector,
							(void *) (vecCont->vec),
							digitsNeeded);

	if (digitsEmitted > 0)
		bitsShifted = normalizeVector(vecCont->vec);

	if (TRACE) {
		debugp("force_To_DigsX_From_Vec",
				"%x %x emitted=%d shifted=%d\n",
				(unsigned) digsX,
				(unsigned) vecCont,
				digitsEmitted,
				bitsShifted);
	}

	if (digitsEmitted < digitsNeeded)
		Error(FATAL, E_INT, "force_To_DigsX_From_Vec",
						"failed to get enough digits");

	newDigsX(digsX);
}

/*
 * This forces a matrix to emit the requested number of digits.
 * matX is the real (MatX *) (ie a matrix with argument) from which the 
 * digits are required and digsX is where to place the digits.
 *
 * It is assumed that the number of digits needed is > 0.
 */
void
force_To_DigsX_From_MatX_Entry()
{
	DigsX *digsX;
	MatX *matX;
	int digitsNeeded;
	void force_To_DigsX_From_Vec();
	void force_To_DigsX_From_MatX_Cont();

	digsX = (DigsX *) POP;
	digitsNeeded = (int) POP;
	matX = (MatX *) digsX->x;

	if (matX->tag.type == VECTOR) {
		digsX->force = force_To_DigsX_From_Vec;
		PUSH_3(digsX->force, digsX, digitsNeeded);
		return;
	}

	matX->totalEmitted = 0;
	PUSH_3(force_To_DigsX_From_MatX_Cont, digsX, digitsNeeded);
}

void
force_To_DigsX_From_MatX_Cont()
{
	DigsX *digsX;
	MatX *matX;
	int digitsNeeded;
	int nArg, digitsEmitted, bitsShifted = 0;
	void force_To_DigsX_From_Vec();

	digsX = (DigsX *) POP;
	digitsNeeded = (int) POP;
	matX = (MatX *) digsX->x;

	/*
	 * First of all, we need to check that the MatX argument hasn't been
	 * reduced to a vector.
	 */
	if (matX->tag.type == VECTOR) {
		digsX->force = force_To_DigsX_From_Vec;
		PUSH_3(digsX->force, digsX, digitsNeeded);
		return;
	}

	/*
	 * First emit all the digits we can (up to the number of
	 * digits requested)
	 */
	digitsEmitted = emitDigits(digsX,
							(edf) emitDigitFromMatrix,
							(void *) (matX->mat),
							digitsNeeded);

	/*
	 * If something has been emitted, then try to remove powers of 2
	 * from the residual matrix.
	 */
	if (digitsEmitted > 0)
		bitsShifted = normalizeMatrix(matX->mat);

	if (TRACE) {
		debugp("force_To_DigsX_From_MatX",
				"%x %x emitted=%d shifted=%d\n",
				(unsigned) digsX,
				(unsigned) matX,
				digitsEmitted,
				bitsShifted);
	}

	matX->totalEmitted += digitsEmitted;
	digitsNeeded -= digitsEmitted;
	
	if (digitsNeeded <= 0) {
		/*
		 * If we have managed to emit anything at all, then we
		 * introduce a new empty DigsX structure between where
		 * the digits have been deposited and the matrix.
		 */
		if (matX->totalEmitted > 0)
			newDigsX(digsX);
		return;
	}

	/*
	 * So now we emitted what we can but still need more. First arrange
	 * to come back and try to emit again after forcing that necessary
	 * number of digits from the the argument.
	 */
	PUSH_3(force_To_DigsX_From_MatX_Cont, digsX, digitsNeeded);

	nArg = epsDelMatrix(matX->mat, digitsNeeded);
	if (nArg > 0)
		PUSH_3(matX->force, matX, nArg);
	else
		PUSH_3(matX->force, matX, defaultForceCount);

	/*
	 * ### If we have not successfully absorbed anything then we should give up.
	 */
}

/*
 * This forces a tensor to emit the requested number of digits.  tenXY is
 * the real (TenXY *) (ie a tensor with two arguments) from which the
 * digits are required and digsX is where to place the digits.
 */
void
force_To_DigsX_From_TenXY_Entry()
{
	DigsX *digsX;
	TenXY *tenXY;
	int digitsNeeded;
	void force_To_DigsX_From_TenXY_Cont();
	void force_To_DigsX_From_MatX_Entry();

	digsX = (DigsX *) POP;
	digitsNeeded = (int) POP;
	tenXY = (TenXY *) digsX->x;

	if (tenXY->tag.type != TENXY) {
		digsX->force = force_To_DigsX_From_MatX_Entry;
		PUSH_3(digsX->force, digsX, digitsNeeded);
		return;
	}

	tenXY->totalEmitted = 0;
	PUSH_3(force_To_DigsX_From_TenXY_Cont, digsX, digitsNeeded);
}

void
force_To_DigsX_From_TenXY_Cont()
{
	DigsX *digsX;
	TenXY *tenXY;
	int digitsNeeded;
	int nX, nY;
	int digitsEmitted, bitsShifted = 0;
	void force_To_DigsX_From_MatX_Entry();
	void force_To_DigsX_From_MatX_Cont();
	void force_To_DigsX_From_TenXY_Cont_X();

	digsX = (DigsX *) POP;
	digitsNeeded = (int) POP;
	tenXY = (TenXY *) digsX->x;

	if (tenXY->tag.type != TENXY) {
		digsX->force = force_To_DigsX_From_MatX_Entry;
		PUSH_3(force_To_DigsX_From_MatX_Cont, digsX, digitsNeeded);
		return;
	}

	digitsEmitted = emitDigits(digsX,
								(edf) emitDigitFromTensor,
								(void *) tenXY->ten,
								digitsNeeded);

	if (digitsEmitted > 0)
		bitsShifted = normalizeTensor(tenXY->ten);

	if (TRACE) {
		debugp("force_To_DigsX_From_TenXY_Cont",
				"%x %x emitted=%d shifted=%d\n",
				(unsigned) digsX,
				(unsigned) tenXY,
				digitsEmitted,
				bitsShifted);
	}

	tenXY->totalEmitted += digitsEmitted;
	digitsNeeded -= digitsEmitted;

	if (digitsNeeded <= 0) {
		if (tenXY->totalEmitted > 0)
			newDigsX(digsX);
		return;
	}

	/*
	 * So now we emitted what we can but still need more. So we figure out
	 * how many we need from each of the arguments.
	 */

	epsDelTensor(tenXY->ten, digitsNeeded, &nX, &nY);

	if (TRACE) {
		debugp("force_To_DigsX_From_TenXY_Cont",
				"%x %x nX=%d nY=%d\n",
				(unsigned) digsX,
				(unsigned) tenXY,
				nX,
				nY);
	}

	/*
	 * When the calculations for the number of digits needed from x and y
	 * yields values which are both less than 0, then we need some other
	 * scheme to decide from which branch to consume digits. I have tried
	 * two schemes. Right now the code implements Peter's strategy.
	 */
	if (nX <= 0 && nY <= 0) {
		nX = 0;
		nY = 0;
		if (tensorIsRefining(tenXY->ten)) {
			if (tensorStrategy(tenXY->ten) == 1) {
				nY = defaultForceCount;
				if (TRACE) {
					debugp("force_To_DigsX_From_TenXY_Cont",
							"tensor refining, choosing y\n");
				}
			}
			else {
				nX = defaultForceCount;
				if (TRACE) {
					debugp("force_To_DigsX_From_TenXY_Cont",
							"tensor refining, choosing x\n");
				}
			}
		}
		else {
			if (tenXY->tensorFairness > 0) {
				nY = defaultForceCount;
				tenXY->tensorFairness = 0;
				if (TRACE) {
					debugp("force_To_DigsX_From_TenXY_Cont",
							"tensor fairness, choosing y\n");
				}
			}
			else {
				nX = defaultForceCount;
				tenXY->tensorFairness = 1;
				if (TRACE) {
					debugp("force_To_DigsX_From_TenXY_Cont",
							"tensor fairness, choosing x\n");
				}
			}
		}
	}

	if (nX > 0) {
		if (nY > 0) {
			/*
			 * If both x and y are to be forced, we do them in two stages
			 * (y first) using an intermediate continuation. This is because
			 * the first force may end up reducing the tensor to a matrix.
			 */
			tenXY->xDigitsNeeded = nX;
			PUSH_3(force_To_DigsX_From_TenXY_Cont_X, digsX, digitsNeeded);
			PUSH_3(tenXY->forceY, tenXY, nY);
		}
		else {
			PUSH_3(force_To_DigsX_From_TenXY_Cont, digsX, digitsNeeded);
			PUSH_3(tenXY->forceX, tenXY, nX);
		}
	}
	else {
		if (nY > 0) {
			PUSH_3(force_To_DigsX_From_TenXY_Cont, digsX, digitsNeeded);
			PUSH_3(tenXY->forceY, tenXY, nY);
		}
		else 
			Error(FATAL, E_INT, "force_To_DigsX_From_TenXY_Cont",
					"assumed impossible case");
	}
}

void
force_To_DigsX_From_TenXY_Cont_X()
{
	DigsX *digsX;
	TenXY *tenXY;
	int digitsNeeded;
	void force_To_DigsX_From_MatX_Entry();
	void force_To_DigsX_From_MatX_Cont();

	digsX = (DigsX *) POP;
	digitsNeeded = (int) POP;
	tenXY = (TenXY *) digsX->x;

	if (tenXY->tag.type != TENXY) {
		digsX->force = force_To_DigsX_From_MatX_Entry;
		PUSH_3(force_To_DigsX_From_MatX_Cont, digsX, digitsNeeded);
		return;
	}

	PUSH_3(force_To_DigsX_From_TenXY_Cont, digsX, digitsNeeded);

	if (tenXY->xDigitsNeeded > 0)
		PUSH_3(tenXY->forceX, tenXY, tenXY->xDigitsNeeded);
}

void
force_To_DigsX_From_Alt_Entry()
{
	DigsX *digsX;
	Alt *alt;
	void force_To_Alt_Entry();
	void force_To_DigsX_From_Alt_Cont();
	int digitsNeeded;

	digsX = (DigsX *) POP;
	digitsNeeded = (int) POP;
	alt = (Alt *) digsX->x;
	
	PUSH_3(force_To_DigsX_From_Alt_Cont, digsX, digitsNeeded);

	/*
	 * If alt->redirect is not valid (equals NULL) then the value of
	 * the conditional has not been determined so we need to force it.
	 */
	if (alt->redirect == NULL)
		PUSH_2(force_To_Alt_Entry, alt);
}

void
force_To_DigsX_From_Alt_Cont()
{
	DigsX *digsX;
	Alt *alt;
	int digitsNeeded;

	digsX = (DigsX *) POP;
	digitsNeeded = (int) POP;
	alt = (Alt *) digsX->x;

	redirectDigsX(digsX, alt->redirect);
	PUSH_3(digsX->force, digsX, digitsNeeded);
}

void
force_To_DigsX_From_Cls_Entry()
{
	DigsX *digsX;
	Cls *cls;
	void force_To_DigsX_From_Cls_Cont();
	int digitsNeeded;

	digsX = (DigsX *) POP;
	digitsNeeded = (int) POP;
	cls = (Cls *) digsX->x;
	
	PUSH_3(force_To_DigsX_From_Cls_Cont, digsX, digitsNeeded);

	/*
	 * If cls->redirect is not valid (equals NULL) then the value of
	 * the closure has not been determined so we need to force it.
	 */
	if (cls->redirect == NULL)
		PUSH_2(cls->force, cls);
}

void
force_To_DigsX_From_Cls_Cont()
{
	DigsX *digsX;
	Cls *cls;
	int digitsNeeded;

	digsX = (DigsX *) POP;
	digitsNeeded = (int) POP;
	cls = (Cls *) digsX->x;

	redirectDigsX(digsX, cls->redirect);
	PUSH_3(digsX->force, digsX, digitsNeeded);
}

void
setDigsXMethod(DigsX *digsX)
{
	void force_To_DigsX_From_Alt_Entry();
	void force_To_DigsX_From_Cls_Entry();
	void force_To_DigsX_From_DigsX_Entry();
	void force_To_DigsX_From_Vec();
	void force_To_DigsX_From_MatX_Entry();
	void force_To_DigsX_From_TenXY_Entry();

	switch (digsX->x->gen.tag.type) {
	case ALT :
		digsX->force = force_To_DigsX_From_Alt_Entry;
		break;
	case SIGNX :
		Error(FATAL, E_INT, "setDigsXMethod", "DigsX guarding a SignX");
		break;
	case DIGSX :
		digsX->force = force_To_DigsX_From_DigsX_Entry;
		break;
	case VECTOR :
		digsX->force = force_To_DigsX_From_Vec;
		break;
	case MATX :
		digsX->force = force_To_DigsX_From_MatX_Entry;
		break;
	case TENXY :
		digsX->force = force_To_DigsX_From_TenXY_Entry;
		break;
	case CLOSURE :
		digsX->force = force_To_DigsX_From_Cls_Entry;
		break;
	default :
		Error(FATAL, E_INT, "setDigsXMethod", "argument has bad type");
		break;
	}
}

/*
 * This function reduces a chain of DigsX structures to single DigsX holding
 * digits followed by an empty structure.
 */
void
reduceDigsXList(DigsX *target)
{
	Real source;

	source = (Real) target->x;
	while (source->gen.tag.type == DIGSX && source->digsX.count > 0) {
		absorbDigsXIntoDigsX(target);
		source = (Real) target->x;
	}
}

/*
 * It can happen that we have lists of DigsX structures. This function
 * reduces a pair of DigsX structures in a list.
 */
void
absorbDigsXIntoDigsX(DigsX *target)
{
	DigsX *source;

	source = (DigsX *) target->x;

	if (source->count > 0) {
#ifdef PACK_DIGITS
		/*
		 * Now consume the digits from the source and add them to the target.
		 * There are three cases here. We might be accumulating into a machine
		 * word, we might be accumulating into a large integer, or we might
		 * have been accumulating into a small word but not have enough room
		 * for the new digits and need to switch to a large integer.
		 */
		if (target->count + source->count <= DIGITS_PER_WORD)
			target->word.small =
				target->word.small * (1 << source->count) + source->word.small;
		else {
			if (target->count <= DIGITS_PER_WORD)
				mpz_init_set_si(target->word.big, target->word.small);
#endif
			mpz_mul_2exp(target->word.big, target->word.big, source->count);
#ifdef PACK_DIGITS
			if (source->count <= DIGITS_PER_WORD)
				if (source->word.small >= 0) {
					mpz_add_ui(target->word.big, target->word.big,
														source->word.small);
				}
				else {
					mpz_sub_ui(target->word.big, target->word.big,
														-(source->word.small));
				}
			else
#endif
				mpz_add(target->word.big, target->word.big, source->word.big);
#ifdef PACK_DIGITS
		}
#endif
		target->count += source->count;
	
		if (TRACE) {
			debugp("absorbDigsXIntoDigsX",
					"%x %x emitted=%d\n",
					(unsigned) target,
					(unsigned) source,
					source->count);
		}
	
		/*
		 * We've consumed the source so advance to the next possible source
		 * of information
		 */
		target->x = source->x;

		if (DAVINCI) {
			beginGraphUpdate();
			deleteOnlyEdge(target, source);
			newEdgeToOnlyChild(target, source->x);
			endGraphUpdate();
		}
	}
}

void
redirectDigsX(DigsX *digsX, Real x)
{
	Real r;

	void force_To_DigsX_From_Cls_Entry();
	void force_To_DigsX_From_Alt_Entry();
	void force_To_DigsX_From_DigsX_Entry();
	void force_To_DigsX_From_Vec();
	void force_To_DigsX_From_MatX_Entry();
	void force_To_DigsX_From_TenXY_Entry();

	if (DAVINCI) {
		beginGraphUpdate();
		deleteOnlyEdge(digsX, digsX->x);
		newEdgeToOnlyChild(digsX, x);
		endGraphUpdate();
	}
	digsX->x = x;

	switch (x->gen.tag.type) {
	case SIGNX :
		Error(FATAL, E_INT, "redirectDigsX", "DigsX guarding a SignX");
		break;
	case DIGSX :
		digsX->force = force_To_DigsX_From_DigsX_Entry;
		break;
	case CLOSURE :
		digsX->force = force_To_DigsX_From_Cls_Entry;
		break;
	case ALT :
		digsX->force = force_To_DigsX_From_Alt_Entry;
		break;
	case VECTOR :
		/*
		 * First we check that the Vector does not have an equivalent stream.
		 * If it doesn't then the DigsX consumer we already have, will
		 * become the root of the new equivalent stream. Note that we
		 * are oblidged to make a copy of the vector since there may be
		 * other consumers, and emitting the sign will change the vector.
		 */
		if (x->vec.strm == NULL) {
			r = vector_Z(x->vec.vec[0], x->vec.vec[1]);
			digsX->x = r;
			x->vec.strm = (Real) digsX;
			if (DAVINCI) {
				beginGraphUpdate();
				deleteOnlyEdge(digsX, x);
				newEdgeToOnlyChild(digsX, r);
				drawEqEdge(digsX, x);
				endGraphUpdate();
			}
			digsX->force = force_To_DigsX_From_Vec;
		}
		/*
		 * If there already is an equivalent stream, then we arrange to
		 * equate the two streams.
		 */ 
		else {
			if (DAVINCI) {
				beginGraphUpdate();
				deleteOnlyEdge(digsX, x);
				newEdgeToOnlyChild(digsX, x->vec.strm);
				endGraphUpdate();
			}
			digsX->x = x->vec.strm;

			/* we already have a stream, so we must equate them as above */
			switch (x->vec.strm->gen.tag.type) {
			case SIGNX :
				Error(FATAL, E_INT, "redirectDigsX",
						"DigsX to guard a SignX vector stream");
			case DIGSX :
				digsX->force = force_To_DigsX_From_DigsX_Entry;
				break;
			default :
				Error(FATAL, E_INT, "redirectDigsX",
						"vector stream is not a stream");
			}
		}
		break;

	case MATX :
		/*
		 * This code is the same as that for the vector case
		 */
		if (x->matX.strm == NULL) {
			r = matrix_Z(x->matX.x,
					x->matX.mat[0][0], x->matX.mat[0][1],
					x->matX.mat[1][0], x->matX.mat[1][1]);
			digsX->x = r;
			x->matX.strm = (Real) digsX;
			if (DAVINCI) {
				beginGraphUpdate();
				deleteOnlyEdge(digsX, x);
				newEdgeToOnlyChild(digsX, r);
				drawEqEdge(digsX, x);
				endGraphUpdate();
			}
			digsX->force = force_To_DigsX_From_MatX_Entry;
		}

		else {
			if (DAVINCI) {
				beginGraphUpdate();
				deleteOnlyEdge(digsX, x);
				newEdgeToOnlyChild(digsX, x->matX.strm);
				endGraphUpdate();
			}
			digsX->x = x->matX.strm;

			/* we already have a stream, so we must equate them as above */
			switch (x->matX.strm->gen.tag.type) {
			case SIGNX :
				Error(FATAL, E_INT, "redirectDigsX",
						"DigsX to guard a SignX matrix stream");
			case DIGSX :
				digsX->force = force_To_DigsX_From_DigsX_Entry;
				break;
			default :
				Error(FATAL, E_INT, "redirectDigsX",
						"matrix stream is not a stream");
			}
		}
		break;

	case TENXY :
		/*
		 * This code is the same as that for the vector and matrix cases
		 * except that we don't need to worry about sharing and hence there
		 * is not need to make a copy of the tensor.
		 */
		if (x->tenXY.strm == NULL) {
			x->tenXY.strm = (Real) digsX;
			digsX->force = force_To_DigsX_From_TenXY_Entry;
		}

		else {
			if (DAVINCI) {
				beginGraphUpdate();
				deleteOnlyEdge(digsX, x);
				newEdgeToOnlyChild(digsX, x->tenXY.strm);
				endGraphUpdate();
			}
			digsX->x = x->tenXY.strm;

			/* we already have a stream, so we must equate them as above */
			switch (x->tenXY.strm->gen.tag.type) {
			case SIGNX :
				Error(FATAL, E_INT, "redirectDigsX",
						"DigsX to guard a SignX tensor stream");
			case DIGSX :
				digsX->force = force_To_DigsX_From_DigsX_Entry;
				break;
			default :
				Error(FATAL, E_INT, "redirectDigsX",
						"tensor stream is not a stream");
			}
		}
		break;
	default :
		Error(FATAL, E_INT, "redirectDigsX", "redirection is not a real");
	}
}
