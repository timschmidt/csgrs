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
 * realDelay(f, x) allows for a user to construct a real valued
 * closure which reduces to f(x) when forced.
 */
typedef struct {
	Delay_Arg x;
	Delay_Fun f;
} DelayData;

Real
realDelay(Delay_Fun f, Delay_Arg x)
{
	Cls *cls;
	DelayData *data;
	void delayCont();

	if ((data = (DelayData *) malloc(sizeof(DelayData))) == NULL)
		Error(FATAL, E_INT, "realDelay", "malloc failed");

	data->f = f;
	data->x = x;

	cls = allocCls(delayCont, data);
	cls->tag.isSigned = TRUE;
	return (Real) cls;
}

void
delayCont()
{
	Cls *cls;
	DelayData *data;

	cls = (Cls *) POP;
	data = (DelayData *) cls->userData;
	cls->redirect = (data->f)(data->x);
}
