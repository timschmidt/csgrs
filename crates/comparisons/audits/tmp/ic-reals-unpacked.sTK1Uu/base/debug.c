/*
 * Copyright (C) 2000, Imperial College
 *
 * This file is part of the Imperial College Exact Real Arithmetic Library.
 * See the copyright notice included in the distribution for conditions
 * of use.
 */

#include <stdarg.h>
#include <stdio.h>

/*
 * A small utility for procedures to write debug information.
 */
void
debugp(char *proc, char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	fprintf(stderr, "%-35s", proc);
	vfprintf(stderr, fmt, ap);
	va_end(ap);
}
