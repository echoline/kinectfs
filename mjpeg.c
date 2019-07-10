#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/sendfile.h>
#include <sys/stat.h>
#include <fcntl.h>
 
int
main(int argc, char **argv) {
	int s, x, y, o, n;
	int rc;
	unsigned char *buf;
	unsigned char *bbuf;
	FILE *file;
	struct stat statbuf;

	if (argc != 2) {
		fprintf(stderr, "usage:\n\t%s /path/to/jpg\n", argv[0]);
		return -1;
	}

	while (1) {
		o = open(argv[1], O_RDONLY);
		if (o < 0)
			return -1;

		if (fstat(o, &statbuf) != 0)
			return -1;
		printf("--ffserver\r\nContent-Type: image/jpeg\r\nContent-Length: %ld\r\n\r\n", statbuf.st_size);
		fflush(stdout);
		while ((rc = read(o, bbuf, 640*480*4)) > 0)
			write(1, bbuf, rc);

		fflush(stdout);
		close(o);

		usleep(250 * 1000);
	}
	free(bbuf);

	return 0;
}
