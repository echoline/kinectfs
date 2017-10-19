#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <jpeglib.h>

#define KWIDTH 200
#define KHEIGHT 150
#define KSIZE KWIDTH*KHEIGHT

int
decompressjpg(unsigned long jsize, unsigned char *jbuf, unsigned long bsize, unsigned char *bbuf)
{
	int rc;
	struct jpeg_decompress_struct cinfo;
	struct jpeg_error_mgr jerr;
	int row_stride, width, height, pixel_size;
	unsigned char *buf;

	if (bsize != KSIZE) {
		fprintf(stderr, "error: bmp size\n");
		return 0;
	}

	cinfo.err = jpeg_std_error(&jerr);
	jpeg_create_decompress(&cinfo);
	jpeg_mem_src(&cinfo, jbuf, jsize);

	rc = jpeg_read_header(&cinfo, TRUE);
	if (rc != 1) {
		jpeg_destroy_decompress(&cinfo);
		fprintf(stderr, "error: jpeg_read_header\n");
		return 0;
	}

	jpeg_start_decompress(&cinfo);
	width = cinfo.output_width;
	height = cinfo.output_height;
	pixel_size = cinfo.output_components;
	row_stride = width * pixel_size;

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
		memcpy(bbuf + KWIDTH * cinfo.output_scanline, buf, KWIDTH);
	}

	free(buf);
	jpeg_finish_decompress(&cinfo);
	jpeg_destroy_decompress(&cinfo);
}
