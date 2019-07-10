#include <X11/Xlib.h>
#include <X11/IntrinsicP.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <jpeglib.h>
#include <sys/sendfile.h>
#include <sys/stat.h>
#include <fcntl.h>
 
int
main(int argc, char **argv) {
	Display *d;
	Window w;
	XEvent e;
	int s, x, y, o, n;
	GC gc;
	XImage *i;
	int rc;
	struct jpeg_decompress_struct cinfo;
	struct jpeg_error_mgr jerr;
	int row_stride, width, height, pixel_size;
	unsigned char *buf;
	unsigned char *bbuf;
	FILE *file;
	struct stat statbuf;

	if (argc != 2) {
		fprintf(stderr, "usage:\n\t%s /path/to/jpg\n", argv[0]);
		return -1;
	}

	file = fopen(argv[1], "rb");
	if (file == NULL)
		return -1;

	cinfo.err = jpeg_std_error(&jerr);
	jpeg_create_decompress(&cinfo);
	jpeg_stdio_src(&cinfo, file);

	rc = jpeg_read_header(&cinfo, FALSE);
	if (rc != 1) {
		jpeg_destroy_decompress(&cinfo);
		fprintf(stderr, "error: jpeg_read_header\n");
		return -1;
	}

	jpeg_start_decompress(&cinfo);
	width = cinfo.output_width;
	height = cinfo.output_height;
	pixel_size = cinfo.output_components;
	row_stride = width * pixel_size;

	buf = malloc(row_stride + 1);
	bbuf = malloc(height * width * 4 + 4);

	while (cinfo.output_scanline < cinfo.output_height)
		jpeg_read_scanlines(&cinfo, &buf, 1);

	jpeg_finish_decompress(&cinfo);
	jpeg_destroy_decompress(&cinfo);
	fclose(file);

	d = XOpenDisplay(NULL);
	if (d == NULL) {
		fprintf(stderr, "Cannot open display\n");
		exit(1);
	}
 
	s = DefaultScreen(d);
	w = XCreateSimpleWindow(d, RootWindow(d, s), 0, 0, width, height, 1,
					BlackPixel(d, s), BlackPixel(d, s));
	XSelectInput(d, w, ExposureMask | KeyPressMask);
	XMapWindow(d, w);
	gc = DefaultGC (d, s);

	while (1) {
		o = open(argv[1], O_RDONLY);
		if (o < 0)
			return -1;

		if (fstat(o, &statbuf) != 0)
			return -1;
		printf("--ffserver\r\nContent-Type: image/jpeg\r\nContent-Length: %ld\r\n\r\n", statbuf.st_size);
		while ((rc = read(o, bbuf, width*height*4)) > 0)
			write(1, bbuf, rc);

		fflush(stdout);
		lseek(o, 0, SEEK_SET);
		file = fdopen(o, "rb");

		cinfo.err = jpeg_std_error(&jerr);
		jpeg_create_decompress(&cinfo);
		jpeg_stdio_src(&cinfo, file);

		rc = jpeg_read_header(&cinfo, FALSE);
		if (rc != 1) {
			jpeg_destroy_decompress(&cinfo);
			fprintf(stderr, "error: jpeg_read_header\n");
			continue;
		}

		jpeg_start_decompress(&cinfo);
		width = cinfo.output_width;
		height = cinfo.output_height;
		pixel_size = cinfo.output_components;
		row_stride = width * pixel_size;

//		fprintf(stderr, "%d %d %d %d\n", width, height, pixel_size, row_stride);

		if (pixel_size == 3) {
			while (cinfo.output_scanline < cinfo.output_height) {
				jpeg_read_scanlines(&cinfo, &buf, 1);
				for (o = 0; o < width; o++) {
					n = o + cinfo.output_scanline*width;
					bbuf[n*4] = buf[o*3+2];
					bbuf[n*4+1] = buf[o*3+1];
					bbuf[n*4+2] = buf[o*3];
				}
			}
		} else if (pixel_size == 1) {
			while (cinfo.output_scanline < cinfo.output_height) {
				jpeg_read_scanlines(&cinfo, &buf, 1);
				for (o = 0; o < width; o++) {
					n = o + cinfo.output_scanline*width;
					bbuf[n*4] =
					bbuf[n*4+1] =
					bbuf[n*4+2] = buf[o];
				}
			}
		}

		jpeg_finish_decompress(&cinfo);
		jpeg_destroy_decompress(&cinfo);

		fclose(file);
		close(o);

		i = XCreateImage(d, DefaultVisual(d, s), DefaultDepth(d, s),
			ZPixmap, 0, 0, width, height, 32, 0);
		i->data = bbuf;

		if (i != NULL)
			XPutImage (d, w, gc, i, 0, 0, 0, 0, width, height);

		if (XCheckWindowEvent(d, w, ExposureMask | KeyPressMask, &e)) {
			//fprintf(stderr, "%p\n", &e);
			if (e.type == Expose && i != NULL)
				XPutImage (d, w, gc, i, 0, 0, 0, 0, width, height);

			if (e.type == KeyPress) {
				if (e.xkey.keycode == 9) {
					break;
				}
			}
		}

		i->data = NULL;

		if (i != NULL)
			XDestroyImage(i);

		usleep(33 * 1000);
	}
	free(bbuf);

	XCloseDisplay(d);
	return 0;
}
