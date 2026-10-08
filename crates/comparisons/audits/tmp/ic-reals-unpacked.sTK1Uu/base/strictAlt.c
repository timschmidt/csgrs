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
 * The conditional is implemented as a function:
 *
 *  void * realAlt(Bool, *void, ...)
 * 
 * where the arguments are a list of guard/value pairs. This is implemented
 * using stdarg(3). The last guard must be the constant DEFAULT_GUARD.
 */

typedef struct ALT_CELL {
	Bool guard;
	void *value;
	struct ALT_CELL *next;
} AltCell;

#include <stdarg.h>

/*
 * ### A better scheme would be to put the alt in the heap as well. Perhaps
 * later.
 */
void *
realAlt(Bool guard, void *value, ...)
{
	va_list ap;
	AltCell *head, *ptr;
	AltCell **prev;
	void *defaultValue;
	void runStack();

	/*
	 * The caller is a wally if the first argument is the default guard.
	 * But we accept it.
	 */
	if ((unsigned) guard == DEFAULT_GUARD)
		return value;

	if (guard->gen.tag.value == LAZY_TRUE)
		return value;

	if ((head = (AltCell *) alloca(sizeof(AltCell))) == NULL)
		Error(FATAL, E_INT, "realAlt", "alloca failed");

	head->guard = guard;
	head->value = value;
	head->next = NULL;
	ptr = head;

	/*
	 * Now we consume the arguments. Each is a guard/value pair. If
	 * the guard is true, then we just return the value. If not then
	 * we allocate a cell with the guard and value and add it to
	 * the list of cells.
	 */
	va_start(ap, value);
	while ((unsigned) (guard = va_arg(ap, Bool)) != DEFAULT_GUARD) {
		value = va_arg(ap, void *);

		if (guard->gen.tag.value == LAZY_TRUE)
			return value;

		if ((ptr->next = (AltCell *) alloca(sizeof(AltCell))) == NULL)
			Error(FATAL, E_INT, "realAlt", "alloca failed");

		ptr = ptr->next;
		ptr->guard = guard;
		ptr->value = value;
		ptr->next = NULL;
	}
	defaultValue = va_arg(ap, void *);
	va_end(ap);

	/*
	 * If we reach here, then we have a list of AltCells which
	 * have guards having value LAZY_UNKNOWN. We now walk down this
	 * list repeatedly. For each alternative, we force the next value
	 * in the boolean stream. If any guard becomes true, then we
	 * return the corresponding value. If something becomes false,
	 * it gets removed from the list. The loop ends when the list
	 * becomes empty whereupon we return the default value provided
	 * at the end of the argument list.
	 */
	ptr = head;
	prev = &head;
	while (ptr != NULL) {
		if (ptr->guard->gen.tag.value == LAZY_UNKNOWN) {
			PUSH_2(ptr->guard->gen.force, ptr->guard);
			runStack();
		}
		switch (ptr->guard->gen.tag.value) {
			case LAZY_TRUE :
				return ptr->value;
				break;
			case LAZY_FALSE :
				*prev = ptr->next; /* unlink the cell */
				ptr = ptr->next;
				break;
			case LAZY_UNKNOWN :
				prev = &(ptr->next);
				ptr = ptr->next;
				break;
			default :
				Error(FATAL, E_INT, "realAlt",
								"invalid boolean value encountered");
				break;
		}

		/*
		 * Reached the end of the list so start from the beginning again.
		 */
		if (ptr == NULL) {
			ptr = head;
			prev = &head;
		}
	}
	return defaultValue;
}
