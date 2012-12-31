typedef struct _FidAux {
	char		*name;
	unsigned char	type;
	unsigned int 	version;
	unsigned long	path;
	int		offset;
} FidAux;

int fnametopath(char *name);
FidAux* newfidaux(char *name, IxpQid *qid);
