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
 * Temporary storage for use in the math library
 */
mpz_t tmpw_z, tmpx_z, tmpy_z, tmpz_z;

Real E;

void
initReals()
{
	void initRealBase();
	void stdTensorCont();
	void failCls();

	initRealBase();

	mpz_init(tmpw_z);
	mpz_init(tmpx_z);
	mpz_init(tmpy_z);
	mpz_init(tmpz_z);

	registerForceFunc(stdTensorCont, "stdTensorCont", 2);
	registerForceFunc(failCls, "failCls", 2);

	initPi();
	E = exp_QInt(1, 1);	/* there is better way but this will come later */
}

Real
realError(char *p)
{
	void failCls();
	Cls *cls;

	cls = allocCls(failCls, (void *) p);
	cls->tag.isSigned = FALSE;

	return  (Real) cls;
}

void
failCls()
{
	Cls *cls;
	char *p;

	cls = (Cls *) POP;
	p = (char *) cls->userData;
	
	if (p != NULL)
		Error(FATAL, E_INT, "failCls", p);
	else
		Error(FATAL, E_INT, "failCls", "failure closure activated");
}
