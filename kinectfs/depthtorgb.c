#include <stdio.h>
#include <math.h>
#define DIM 640 * 480

unsigned short lut[2048];

int main() {
	int i, j = 0;
	float v;
	int c;
	unsigned short s;

	for (i = 0; i < 2048; i++) {
		v = i/2048.0;
		v = powf(v, 3) * 6;
		lut[i] = v * 6 * 256;
	}

	while (j++ < DIM) {
		c = getc(stdin);
		if (c == EOF)
			break;
		s = c;
		c = getc(stdin);
		if (c == EOF)
			break;
		s |= (c << 8);

		if (s >= 2048)
			s = 0;

		i = lut[s];
		c = i & 0xff;

		switch (i >> 8) {
		case 0:
			printf ("%c%c%c", 255, 255 - c, 255 - c);
			break;
		case 1:
			printf ("%c%c%c", 255, c, 0);
			break;
		case 2:
			printf ("%c%c%c", 255 - c, 255, 0);
			break;
		case 3:
			printf ("%c%c%c", 0, 255, c);
			break;
		case 4:
			printf ("%c%c%c", 0, 255 - c, 255);
			break;
		case 5:
			printf ("%c%c%c", 0, 0, 255 - c);
			break; 
		default:
			printf ("%c%c%c", 0, 0, 0);
			break;
		}
	}

	fflush (stdout);
}

