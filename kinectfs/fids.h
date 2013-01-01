typedef struct FileId {
	unsigned long fid;
	void *aux;
	void *conn;
	struct FileId *next;
} FileId;

typedef struct _FidAux {
	char		*name;
	unsigned int 	version;
	unsigned long	length;
	unsigned long	offset;
} FidAux;

int fnametopath(char *name);
FidAux* newfidaux(char *name);
