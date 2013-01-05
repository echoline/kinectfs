#include <stdlib.h>

int
minv (char *array, int len) {
	int i;
	char max = 127;
	int ret = -1;

	for (i = 0; i < len; i++) {
		if (array[i] < max) {
			ret = i;
			max = array[i];
		}
	}

	return ret;
}

int
maxv (char *array, int len) {
	int i;
	char max = -127;
	int ret = -1;

	for (i = 0; i < len; i++) {
		if (array[i] > max) {
			ret = i;
			max = array[i];
		}
	}

	return ret;
}

int
main(int argc, char **argv) {
	int maxdex, mindex, i, j;
	union {
		unsigned char buf[4];
		unsigned int pixel[1];
	} u;
	double rgb[3];
	double hsv[3];
	double delta;
	int r, g, b;
	unsigned int ***p;

	p = malloc (sizeof (unsigned int **) * 256);

	for (r = 0; r < 256; r++) {
		p[r] = malloc (sizeof (unsigned int *) * 256);

		for (g = 0; g < 256; g++)
			p[r][g] = malloc (sizeof (unsigned int) * 256);
	}


	for (r = 0; r < 256; r++) for (g = 0; g < 256; g++)
			for (b = 0; b < 256; b++) {

		u.buf[0] = r;
		u.buf[1] = g;
		u.buf[2] = b;
		u.buf[3] = 0x7F;

		maxdex = maxv(u.buf, 3);
		mindex = minv(u.buf, 3);

		for (i = 0; i < 3; i++)
			rgb[i] = u.buf[i] / 255.0;

		hsv[2] = rgb[maxdex];

		delta = rgb[maxdex] - rgb[mindex];

		if (rgb[maxdex] == rgb[mindex]) {
			hsv[1] = hsv[0] = 0;

		} else {
			hsv[1] = delta / rgb[maxdex];

			switch (maxdex) {
			case 0:
				hsv[0] = (rgb[1] - rgb[2]) / delta;
				break;
			case 1:
				hsv[0] = 2.0 + (rgb[2] - rgb[0]) / delta;
				break;
			case 2:
				hsv[0] = 4.0 + (rgb[0] - rgb[1]) / delta;
				break;
			}

			hsv[0] *= 60;

			while (hsv[0] < 0)
				hsv[0] += 360;
		}

		u.buf[0] = hsv[0] / 360.0 * 255.0;
		for (i = 1; i < 3; i++)
			u.buf[i] = hsv[i] * 255.0;

		p[r][g][b] = u.pixel[0];

	}

	i = 0;
	for (;;) {
		j = read (0, &u.buf[i], 3 - i);

		if (j < 0)
			continue;

		if (j <= 3)
			i += j;

		if (i == 3) {
			u.pixel[0] = p[u.buf[0]][u.buf[1]][u.buf[2]];

			write (1, u.buf, 3);

			i = 0;
		}

		if (i > 3)
			break;
	}

	perror ("read");
}
