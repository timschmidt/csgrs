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
#include <stdarg.h>
#include <string.h>
#include <errno.h>

/*
 * An application may choose to set this to a string giving the name
 * of the program, typically argv[0].
 */
char *MyName = NULL;

/*
 * Prints an error message
 * Error(fatal, error_type, proc, fmt, arg1, arg2, ....)
 *
 * fatal = FATAL then we exit after printing a message
 * error_type = E_SYS then perror or E_INT
 * proc - a pointer to a string containing the name of the calling procedure
 * fmt - format string for printf
 * arg1...
 */
void
Error(int fatal, int error_type, char *proc, char *fmt, ...)
{
	va_list ap;

	if (MyName != NULL)
		fprintf(stderr, "%s: ", MyName);

	if (fatal == FATAL)
		fprintf(stderr, "fatal error: %s: ", proc);
	else
		fprintf(stderr, "%s: ", proc);

	va_start(ap, fmt);
	vfprintf(stderr, fmt, ap);
	va_end(ap);

	switch (error_type) {
	case E_INT :
		fprintf(stderr, "\n");
		break;

	case E_SYS :
		perror(proc);
		break;

	default :
		fprintf(stderr, "\n%s: %s: unknown error type: %d\n", 
					MyName, proc, error_type);
		break;
	}

	if (fatal == FATAL)
		exit(1);
}
