/*
 * Some UNIX distributions don't come with strsep.
 */
#include <stdio.h>
#include <string.h>

char *
strsep(char **str_p, char *delim)
{
	char *start, *end;

	start = *str_p;
	if (start == NULL)
		return NULL;

	end = strpbrk(start, delim);
	if (end) {
		*end++ = '\0';
		*str_p = end;
    }
	else
		*str_p = NULL;

	return start;
}
