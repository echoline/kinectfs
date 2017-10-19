#include <X11/Xlib.h>
#include <X11/IntrinsicP.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <jpeglib.h>
 
#define KWIDTH 640
#define KHEIGHT 480
#define KSIZE KWIDTH*KHEIGHT

int main(void) {
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
	unsigned char jbuf[640*480*8];
	unsigned char bbuf[640*480*8];
	FILE *file;

	d = XOpenDisplay(NULL);
	if (d == NULL) {
		fprintf(stderr, "Cannot open display\n");
		exit(1);
	}
 
	s = DefaultScreen(d);
	w = XCreateSimpleWindow(d, RootWindow(d, s), 0, 0, 640, 480, 1,
					BlackPixel(d, s), BlackPixel(d, s));
	XSelectInput(d, w, ExposureMask | KeyPressMask);
	XMapWindow(d, w);
	gc = DefaultGC (d, s);

	while (1) {
		file = fopen("/home/eli/kinect/rgb.jpg", "rb");
		if (file == NULL)
			return -1;

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

		//fprintf(stderr, "%d %d %d %d\n", width, height, pixel_size, row_stride);

		if (width != KWIDTH || height != KHEIGHT) {
			jpeg_finish_decompress(&cinfo);
			jpeg_destroy_decompress(&cinfo);
			fprintf(stderr, "error: incorrect jpeg dimensions\n");
			return 0;
		}
		buf = malloc(row_stride + 1);

		while (cinfo.output_scanline < cinfo.output_height) {
			if (cinfo.output_scanline == KHEIGHT)
				break;
			jpeg_read_scanlines(&cinfo, &buf, 1);
			for (o = 0; o < KWIDTH; o++) {
				n = o + cinfo.output_scanline*KWIDTH;
				bbuf[n*4] = buf[o*3+2];
				bbuf[n*4+1] = buf[o*3+1];
				bbuf[n*4+2] = buf[o*3];
			}
		}

		free(buf);
		jpeg_finish_decompress(&cinfo);
		jpeg_destroy_decompress(&cinfo);

		fclose(file);

		i = XCreateImage(d, DefaultVisual(d, s), DefaultDepth(d, s),
				ZPixmap, 0, bbuf, 640, 480, 32, 640 * 4);

		XPutImage (d, w, gc, i, 0, 0, 0, 0, 640, 480);

		if (XCheckWindowEvent(d, w, ExposureMask | KeyPressMask, &e)) {
			fprintf(stderr, "%p\n", &e);
			if (e.type == Expose)
				XPutImage (d, w, gc, i, 0, 0, 0, 0, 640, 480);

			if (e.type == KeyPress) {
				if (e.xkey.keycode == 9)
					break;
			}
		}

		XDestroyImage(i);
	}
 
	XCloseDisplay(d);
	return 0;
}
