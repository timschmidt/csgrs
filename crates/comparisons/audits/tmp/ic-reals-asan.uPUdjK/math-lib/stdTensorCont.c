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
#include "math-lib.h"

/*
 * A number of functions on reals behave in a uniform way in the sense
 * that they create a chain of tensors lazily. The only difference are
 * the tensors they create. This is the standard tensor continuation. 
 * It is the force method in a closure and the assumption is that
 * the closure data contains a pointer to a function which when called
 * produces the next tensor in the chain.
 *
 * If the argument is a stream terminated by a vector, then there is
 * no reduction. As a rule, guarding a vector by a stream inhibits
 * reduction of a matrix or tensor against the rational.
 *
 * When the tensor is created, there may be some information
 * about the argument available.  At present, we do not absorb this
 * information. Perhaps we should. It is not clear what the best policy is.
 * 
 * Note that in the code below, the closure (Cls) is never shared so
 * we are free to clobber the data in it.
 */
void
stdTensorCont()
{
	Cls *cls, *newCls;
	TenXY *tenXY, *newTenXY;
	ClsData *data;
	void force_To_TenXY_X_Until_Refining();

	cls = (Cls *) POP;

	data = (ClsData *) cls->userData;

	newCls = allocCls(stdTensorCont, (void *) data);
	newCls->tag.isSigned = FALSE;

	newTenXY = (*(data->nextTensor))(data->x, (Real) newCls, data->n);
	newTenXY->tag.isSigned = FALSE;
	data->n++;

	PUSH_2(force_To_TenXY_X_Until_Refining, newTenXY);

	if (newTenXY->x->gen.tag.isSigned)
		PUSH_2(newTenXY->forceX, newTenXY);

	cls->userData = NULL;
	cls->redirect = (Real) newTenXY;
}
