#include <stdlib.h>
#include <ixp.h>
#include "fids.h"

extern char *paths[];
extern unsigned long mtimes[];
extern unsigned long atimes[];

int
fnametopath(char *name) {
	int i;
	char *p = name;
	char *q;

	while (*p == '/')
		p++;

	for (i = 0; paths[i] != NULL; i++) {
		q = paths[i];
		while (*q == '/')
			q++;

		if (strcasecmp(p, q) == 0)
			return (i);
	}

	return -1;
}

FidAux*
newfidaux(char *name) {
	FidAux *ret;

	ret = calloc (1, sizeof(FidAux));
	ret->name = paths[fnametopath(name)];

	return ret;
}

