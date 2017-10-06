#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <limits.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
#include <ixp.h>
#include <libfreenect_sync.h>
#ifdef USE_JPEG
#include <jpeglib.h>
#endif

unsigned char r2blut[0x1000000];
unsigned char k2rlut[0x800][3];
enum {
	Qroot = 0,
	Qtilt,
	Qled,
	Qrgb,
	Qdepth,
	Qextra,
	Qedge,
	Qbw,
#ifdef USE_JPEG
	Qrgbjpg,
	Qdepthjpg,
	Qextrajpg,
	Qedgejpg,
	Qbwjpg,
#endif
#ifdef USE_AUDIO
	Qmic0,
	Qmic1,
	Qmic2,
	Qmic3,
#endif
	Npaths,
};
char *paths[] = { "/", "tilt", "led", "rgb.pnm", "depth.pnm", "extra.pnm",
	"edge.pnm", "bw.pnm",
#ifdef USE_JPEG
	"rgb.jpg", "depth.jpg", "extra.jpg", "edge.jpg", "bw.jpg",
#endif
#ifdef USE_AUDIO
	"mic0", "mic1", "mic2", "mic3",
#endif
	 };
unsigned long mtimes[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
unsigned long atimes[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

typedef struct {
	unsigned char* image;
	unsigned long length;
	unsigned int width;
	unsigned int height;
	unsigned int components;
	unsigned int colorspace;
	unsigned char hdrlen;
} FrnctImg;
static FrnctImg rgbimg;
static FrnctImg depthimg;
static FrnctImg extraimg;
static FrnctImg edgeimg;
static FrnctImg bwimg;
#ifdef USE_JPEG
static FrnctImg rgbjpg;
static FrnctImg depthjpg;
static FrnctImg extrajpg;
static FrnctImg edgejpg;
static FrnctImg bwjpg;
#endif
static struct timeval rgbdlast;
static int rgbdlock = 0;
static freenect_raw_tilt_state *tiltstate = NULL;
static struct timeval tiltlast;
#define KWIDTH 640 
#define KHEIGHT 480

typedef struct _FidAux {
	unsigned int 	version;
	unsigned long	length;
	unsigned long	offset;
	int		index; // for dirs
	FrnctImg	*fim;
} FidAux;

int
fnametopath(char *name) {
	int i;
	char *q;

	q = strrchr(name, '/');
	if (q != NULL)
		name = q + 1;

	for (i = 0; i < Npaths; i++) {
		q = paths[i];
		while (*q == '/')
			q++;

		if (strcasecmp(name, q) == 0)
			return (i);
	}

	return -1;
}

FidAux*
newfidaux(int path) {
	FidAux *ret;

	ret = calloc (1, sizeof(FidAux));
	if (path == Qrgb || path == Qdepth || path == Qextra || path == Qedge
	  || path == Qbw
#ifdef USE_JPEG
	  || path == Qrgbjpg || path == Qdepthjpg || path == Qextrajpg
	  || path == Qedgejpg || path == Qbwjpg
#endif
	) {
		ret->fim = calloc(1, sizeof(FrnctImg));
		// shouldn't need more than this
		ret->fim->image = calloc(1, KWIDTH*2*KHEIGHT*2*4);
	}

	return ret;
}

char
istime(struct timeval *last, double epsilon) {
	struct timeval now;
	double delta;
 
	gettimeofday(&now, NULL);
	delta = (now.tv_sec + now.tv_usec / 1000000.0) - (last->tv_sec + last->tv_usec / 1000000.0);

	// waaaaait...
	if (delta < epsilon)
		return 0;

	last->tv_sec = now.tv_sec;
	last->tv_usec = now.tv_usec;

	return 1;
}

int tilt;
int led = 1;
int depthmode = 4;
int rgbmode = 0;

#ifdef USE_AUDIO
unsigned char *audio[4] = { NULL, NULL, NULL, NULL };
size_t audiolengths[4] = { 0, 0, 0, 0 };
size_t audiohead[4] = { 0, 0, 0, 0 };
#define AUDIO_BUFFER_SIZE 65536 * 8
#endif

//#define debug(...) fprintf (stderr, __VA_ARGS__);
#define debug(...) {}; 

int
dostat(int path, IxpStat *stat) {
	static char *none = "none";

	stat->type = 0;
	stat->dev = 0;
	stat->qid.type = (path != 0? P9_QTFILE: P9_QTDIR);
	stat->qid.version = 0;
	stat->qid.path = path;
	stat->mode = P9_DMREAD;
	stat->length = 0;
	switch (stat->qid.path) {
	case Qroot:
		stat->mode |= P9_DMDIR|P9_DMEXEC;
		break;
	case Qrgb:
		stat->length = rgbimg.length;
		stat->mode |= P9_DMWRITE;
		break;
	case Qdepth:
		stat->length = depthimg.length;
		stat->mode |= P9_DMWRITE;
		break;
	case Qextra:
		stat->length = extraimg.length;
		break;
	case Qedge:
		stat->length = edgeimg.length;
		break;
	case Qbw:
		stat->length = bwimg.length;
		break;
#ifdef USE_JPEG
	case Qrgbjpg:
		stat->length = rgbjpg.length;
		break;
	case Qdepthjpg:
		stat->length = depthjpg.length;
		break;
	case Qextrajpg:
		stat->length = extrajpg.length;
		break;
	case Qedgejpg:
		stat->length = edgejpg.length;
		break;
	case Qbwjpg:
		stat->length = bwjpg.length;
		break;
#endif
	case Qtilt:
	case Qled:
		stat->mode |= P9_DMWRITE;
	default:
		break;
	}
	stat->atime = atimes[path];
	stat->mtime = mtimes[path];
	stat->name = paths[path];
	stat->uid = stat->gid = stat->muid = none;

	return ixp_sizeof_stat(stat);
}

#ifdef USE_AUDIO
void
in_callback(freenect_device *dev, int num_samples, int32_t *mic0,
		int32_t *mic1, int32_t *mic2, int32_t *mic3,
		int16_t *cancelled, void *unknown) {
	int i;
	int length;
	int32_t *mic;

	for (i = 0; i < 4; i++) {
		switch (i) {
		case 0:
			mic = mic0;
			break;
		case 1:
			mic = mic1;
			break;
		case 2:
			mic = mic2;
			break;
		case 3:
			mic = mic3;
			break;
		}

		length = num_samples * sizeof (int32_t);
		audiolengths[i] += length;
		audiohead[i] += length;	

		if (audiolengths[i] > AUDIO_BUFFER_SIZE)
			audiolengths[i] = AUDIO_BUFFER_SIZE;

		audio[i] = realloc (audio[i], audiolengths[i]);
		memmove(audio[i], &audio[i][length], audiolengths[i]-length);
		memcpy(&audio[i][audiolengths[i] - length], mic, length);
	}
}
#endif

#ifdef USE_JPEG
void
compressjpg(FrnctImg *img)
{
	struct jpeg_compress_struct cinfo;
	struct jpeg_error_mgr jerr;
	JSAMPROW row_pointer[1];
	unsigned char *buf;
	size_t buflen = 0;

	cinfo.err = jpeg_std_error( &jerr );
	jpeg_create_compress(&cinfo);

	cinfo.image_width = img->width;
	cinfo.image_height = img->height;
	cinfo.input_components = img->components;
	cinfo.in_color_space = img->colorspace;

	jpeg_mem_dest( &cinfo, &buf, &buflen );
	jpeg_set_defaults( &cinfo );

	jpeg_start_compress( &cinfo, TRUE );
	while( cinfo.next_scanline < cinfo.image_height ) {
		row_pointer[0] = &img->image[ cinfo.next_scanline * cinfo.image_width * cinfo.input_components + img->hdrlen];

		jpeg_write_scanlines( &cinfo, row_pointer, 1 );
	}

	jpeg_finish_compress( &cinfo );

	if (buflen < img->length)
		img->length = buflen;
	memcpy(img->image, buf, img->length);

	jpeg_destroy_compress( &cinfo );
	free(buf);
}
#endif

static void
copyimg(FrnctImg *in, FrnctImg *out) {
	memcpy(out->image, in->image, in->length);
	out->length = in->length;
	out->width = in->width;
	out->height = in->height;
	out->hdrlen = in->hdrlen;
	out->components = in->components;
	out->colorspace = in->colorspace;
}

static void fs_open(Ixp9Req *r)
{
	FidAux *f;
	unsigned int ts;
	unsigned char *rgbbuf;
	unsigned char *depthbuf;
	int c;
	unsigned short s;
	int j, k, l, x, xm, xp, y, ym, yp;
	int path = r->fid->qid.path;

	if (path < 0 || path >= Npaths) {
		ixp_respond(r, "file not found");
		return;
	}

	debug ("fs_open %s %lu\n", paths[path], r->fid->fid);

	r->fid->aux = newfidaux(path);
	f = r->fid->aux;

	if (path == Qrgb || path == Qdepth || path == Qextra || path == Qedge || path == Qbw
#ifdef USE_JPEG
	|| path == Qrgbjpg || path == Qdepthjpg || path == Qextrajpg || path == Qedgejpg || path == Qbwjpg
#endif
	) {
//		r->ofcall.ropen.iounit = 680*480*4;
		if (istime(&rgbdlast, 1.0/30.0)) {
			freenect_sync_get_tilt_state(&tiltstate, 0);
			gettimeofday(&tiltlast, NULL);
			if (freenect_sync_get_video((void**)(&rgbbuf), &ts, 0, rgbmode) != 0) {
				ixp_respond(r, "freenect_sync_get_video");
				return;
			}
			if (freenect_sync_get_depth((void**)(&depthbuf), &ts, 0, depthmode) != 0) {
				ixp_respond(r, "freenect_sync_get_depth");
				return;
			}
			rgbdlock = 1;

			rgbimg.width = KWIDTH;
			rgbimg.height = KHEIGHT;
			rgbimg.hdrlen = 15;
			rgbimg.components = 3;
			rgbimg.length = rgbimg.width*rgbimg.height*rgbimg.components+rgbimg.hdrlen;
			memcpy(rgbimg.image, "P6\n640 480\n255\n", rgbimg.hdrlen);

			depthimg.width = KWIDTH;
			depthimg.height = KHEIGHT;
			depthimg.hdrlen = 15;
			depthimg.components = 1;
			depthimg.length = depthimg.width*depthimg.height*depthimg.components+depthimg.hdrlen;
			memcpy(depthimg.image, "P5\n640 480\n255\n", depthimg.hdrlen);

			extraimg.width = KWIDTH*2;
			extraimg.height = KHEIGHT*2;
			extraimg.hdrlen = 16;
			extraimg.components = 1;
			extraimg.length = extraimg.width*extraimg.height*extraimg.components+extraimg.hdrlen;
			memcpy(extraimg.image, "P5\n1280 960\n255\n", extraimg.hdrlen);

			edgeimg.width = KWIDTH;
			edgeimg.height = KHEIGHT;
			edgeimg.hdrlen = 15;
			edgeimg.components = 1;
			edgeimg.length = edgeimg.width*edgeimg.height*edgeimg.components+edgeimg.hdrlen;
			memcpy(edgeimg.image, "P5\n640 480\n255\n", edgeimg.hdrlen);

			bwimg.width = KWIDTH;
			bwimg.height = KHEIGHT;
			bwimg.hdrlen = 15;
			bwimg.components = 1;
			bwimg.length = bwimg.width*bwimg.height*bwimg.components+bwimg.hdrlen;
			memcpy(bwimg.image, "P5\n640 480\n255\n", bwimg.hdrlen);

			memcpy(rgbimg.image+rgbimg.hdrlen, rgbbuf, rgbimg.length-rgbimg.hdrlen);
			memset(edgeimg.image+edgeimg.hdrlen, 0, edgeimg.length-edgeimg.hdrlen);
			memset(extraimg.image+extraimg.hdrlen, 0, extraimg.length-extraimg.hdrlen);
			// need black and white first for edges
			for (x = 0; x < KWIDTH; x++) for (y = 0; y < KHEIGHT; y++) {
				j = y * KWIDTH + x;
				c = r2blut[((*(uint32_t*)&rgbbuf[j * 3]) & 0xFFFFFF00) >> 8];
				extraimg.image[y * (KWIDTH*2) + x + extraimg.hdrlen] = bwimg.image[j + bwimg.hdrlen] = (c & 0xFF);
			}
			for (x = 0; x < KWIDTH; x++) for (y = 0; y < KHEIGHT; y++) {
				j = y * KWIDTH + x;
				l = j + bwimg.hdrlen;

				if (x > 0 && x < (KWIDTH-1) && y > 0 && y < (KHEIGHT-1)) {
					k = abs(bwimg.image[l - (KWIDTH+1)] * -1 + bwimg.image[l - (KWIDTH-1)] * 1 + bwimg.image[l - 1] * -2 + bwimg.image[l + 1] * 2 + bwimg.image[l + (KWIDTH-1)] * -1 + bwimg.image[l + (KWIDTH+1)] * 1 + bwimg.image[l - (KWIDTH+1)] * 1 + bwimg.image[l - KWIDTH] * 2 + bwimg.image[l - (KWIDTH-1)] * 1 + bwimg.image[l + (KWIDTH-1)] * -1 + bwimg.image[l + KWIDTH] * -2 + bwimg.image[l + (KWIDTH+1)] * -1);
					extraimg.image[y * (KWIDTH*2) + x + KWIDTH + extraimg.hdrlen] = edgeimg.image[l] = k & 0xFF;
				}

				s = depthbuf[j * 2];
				s |= depthbuf[j * 2 + 1] << 8;
	
				if (s >= 2048)
					s = 0;

				k = s & 0x7FF;
				extraimg.image[(y+KHEIGHT) * (KWIDTH*2) + x + extraimg.hdrlen] = depthimg.image[l] = k >> 3;
			}
#ifdef USE_JPEG
			copyimg(&rgbimg, &rgbjpg);
			rgbjpg.colorspace = JCS_RGB;
			compressjpg(&rgbjpg);

			copyimg(&depthimg, &depthjpg);
			depthjpg.colorspace = JCS_GRAYSCALE;
			compressjpg(&depthjpg);

			copyimg(&extraimg, &extrajpg);
			extrajpg.colorspace = JCS_GRAYSCALE;
			compressjpg(&extrajpg);

			copyimg(&edgeimg, &edgejpg);
			edgejpg.colorspace = JCS_GRAYSCALE;
			compressjpg(&edgejpg);

			copyimg(&bwimg, &bwjpg);
			bwjpg.colorspace = JCS_GRAYSCALE;
			compressjpg(&bwjpg);
#endif
			rgbdlock = 0;
		}

RGBDLOCK:
		while(rgbdlock != 0) sleep(1);
		switch(path){
		case Qrgb:
			copyimg(&rgbimg, f->fim);
			break;
		case Qdepth:
			copyimg(&depthimg, f->fim);
			break;
		case Qextra:
			copyimg(&extraimg, f->fim);
			break;
		case Qedge:
			copyimg(&edgeimg, f->fim);
			break;
		case Qbw:
			copyimg(&bwimg, f->fim);
			break;
#ifdef USE_JPEG
		case Qrgbjpg:
			copyimg(&rgbjpg, f->fim);
			break;
		case Qdepthjpg:
			copyimg(&depthjpg, f->fim);
			break;
		case Qextrajpg:
			copyimg(&extrajpg, f->fim);
			break;
		case Qedgejpg:
			copyimg(&edgejpg, f->fim);
			break;
		case Qbwjpg:
			copyimg(&bwjpg, f->fim);
			break;
#endif
		default:
			ixp_respond(r, "file not found");
			return;
		}
		if(rgbdlock != 0)
			goto RGBDLOCK;
	}

	ixp_respond (r, NULL);
}

static void fs_walk(Ixp9Req *r)
{
	int path;
	FidAux *f = r->fid->aux;

	path = r->fid->qid.path;

	if (path < 0 || path >= Npaths) {
		ixp_respond(r, "file not found");
		return;
	}

	debug ("fs_walk from %s fid:%lu newfid:%lu\n", paths[path], r->fid->fid, r->newfid->fid);

	if (r->ifcall.twalk.nwname == 0 || strcmp(r->ifcall.twalk.wname[0], ".") == 0) {
		ixp_respond (r, NULL);
		return;
	} else if (r->ifcall.twalk.nwname != 1) {
		ixp_respond (r, "file not found");
		return;
	} else if (strcmp(r->ifcall.twalk.wname[0], "..") == 0) {
		path = 0;
	} else {
		path = fnametopath(r->ifcall.twalk.wname[0]);
	}

	if (path < 0 || path > Npaths) {
		ixp_respond (r, "file not found");
		return;
	}

	r->ofcall.rwalk.wqid[0].version = 0;
	r->ofcall.rwalk.wqid[0].path = path;
	if (path == 0)
		r->ofcall.rwalk.wqid[0].type = P9_QTDIR;
	else
		r->ofcall.rwalk.wqid[0].type = P9_QTFILE;

	r->ofcall.rwalk.nwqid = 1;
//	r->newfid->aux = newfidaux(path);

	ixp_respond(r, NULL);
}

static void fs_read(Ixp9Req *r)
{
	FidAux *f = r->fid->aux;
	IxpStat stat;
	IxpMsg m;
	char *buf;
	int n, size;
	int path;
	static double dx, dy, dz;
	int i;

	debug ("fs_read read:%llu offset:%lu\n", r->ifcall.tread.offset, f->offset);

	path = r->fid->qid.path;

	if (path < 0 || path >= Npaths) {
		ixp_respond(r, "file not found");
		return;
	}

	if (r->ifcall.tread.offset == 0)
		f->offset = 0;

	switch (path) {
	case Qroot:
		f->index++;
		// END OF FILES
		if (f->index >= Npaths) {
			ixp_respond(r, NULL);
			return;
		}

		size = r->ifcall.tread.count;
		buf = malloc (size);
		m = ixp_message((unsigned char*)buf, size, MsgPack);
		n = dostat(f->index, &stat);
		if (n < 0) {
			free (buf);
			ixp_respond (r, "fs_read dostat failed");
			return;
		}
		ixp_pstat(&m, &stat);

		r->ofcall.rread.count = n;
		r->ofcall.rread.data = m.data;

		ixp_respond (r, NULL);
		return;
	case Qrgb:
	case Qdepth:
	case Qextra:
	case Qedge:
	case Qbw:
#ifdef USE_JPEG
	case Qrgbjpg:
	case Qdepthjpg:
	case Qextrajpg:
	case Qedgejpg:
	case Qbwjpg:
#endif
		size = r->ifcall.tread.count;
		if (size < 0) {
			ixp_respond(r, "no.");
			return;
		}		

		r->ofcall.rread.count = f->fim->length - r->ifcall.tread.offset;

		if (r->ofcall.rread.count <= 0)
			r->ofcall.rread.count = 0;

		if (size < r->ofcall.rread.count)
			r->ofcall.rread.count = size;

		if (r->ofcall.rread.count != 0) {
			r->ofcall.rread.data = malloc(r->ofcall.rread.count);
			memcpy (r->ofcall.rread.data, &f->fim->image[r->ifcall.tread.offset], r->ofcall.rread.count);
		}

		ixp_respond (r, NULL);
		return;
#ifdef USE_AUDIO
	case Qmic0:
	case Qmic1:
	case Qmic2:
	case Qmic3:
		i = path - Qmic0;

		size = r->ifcall.tread.count;
		buf = malloc (size);

		n = audiohead[i] - f->offset;

		if (size > audiolengths[i])
			size = audiolengths[i];

		if (size < n)
			n = size;
	
		if ((audiolengths[i] >= sizeof(int32_t)) && (n == 0))
			size = n = sizeof(int32_t);

		memcpy (buf, &audio[i][audiolengths[i] - size], n);

		f->offset = audiohead[i] - size + n;

		r->ofcall.rread.data = buf;
		r->ofcall.rread.count = n;

		ixp_respond(r, NULL);
		return;
#endif
	case Qtilt:
		if (r->ifcall.tread.offset == 0) {
			size = r->ifcall.tread.count;

			if ((tiltstate==NULL || istime(&tiltlast,1.0/30.0)) &&
			    freenect_sync_get_tilt_state(&tiltstate, 0) != 0) {
				ixp_respond(r, "freenect_sync_get_tilt_state");
				return;
			} else {
				freenect_get_mks_accel (tiltstate, &dx, &dy, &dz);

				buf = malloc (size);
				snprintf(buf, size, "%d %lf %lf %lf\n", tilt, dx, dy, dz);
				r->ofcall.rread.count = strlen(buf);
				r->ofcall.rread.data = buf;
			}
			atimes[path] = time(NULL);
		} else {
			r->ofcall.rread.count = 0;
		}
		ixp_respond(r, NULL);
		return;
	case Qled:
		if (r->ifcall.tread.offset == 0) {
			size = r->ifcall.tread.count;
			buf = malloc (size);

			snprintf(buf, size, "%d\n", led);
			r->ofcall.rread.count = strlen(buf);
			r->ofcall.rread.data = buf;

			atimes[path] = time(NULL);
		} else {
			r->ofcall.rread.count = 0;
		}
		ixp_respond(r, NULL);
		return;
	}

	ixp_respond (r, "invalid path");
}

static void fs_stat(Ixp9Req *r)
{
	IxpStat stat;
	IxpMsg m;
	int size;
	char *buf;
	int path = r->fid->qid.path;

	debug ("fs_stat fid:%lu\n", r->fid->fid);

	if (path < 0 || path >= Npaths) {
		ixp_respond(r, "file not found");
		return;
	}

	size = dostat(path, &stat);

	buf = malloc (size);
	m = ixp_message(buf, size, MsgPack);
	ixp_pstat(&m, &stat);
	r->ofcall.rstat.nstat = size;
	r->ofcall.rstat.stat = m.data;
	r->fid->qid = stat.qid;
	ixp_respond(r, NULL);
}

static void fs_write(Ixp9Req *r)
{
	FidAux *f = r->fid->aux;
	char *buf;
	int path = r->fid->qid.path;

	debug ("fs_write\n");

	if (path < 0 || path >= Npaths) {
		ixp_respond (r, "file not found");
		return;
	}

	if (r->ifcall.twrite.count == 0) {
		ixp_respond (r, NULL);
		return;
	}

	buf = malloc (r->ifcall.twrite.count + 1);
	memcpy(buf, r->ifcall.twrite.data, r->ifcall.twrite.count);
	buf[r->ifcall.twrite.count] = '\0';

	switch (path) {
	default:
		ixp_respond(r, "permission denied");
		break;
	case Qrgb:
		r->ofcall.rwrite.count = r->ifcall.twrite.count;
		rgbmode = atoi(buf);

		if ((rgbmode % 7) != rgbmode)
			rgbmode = 0;

		mtimes[path] = time(NULL);
		ixp_respond(r, NULL);
		break;
	case Qdepth:
		r->ofcall.rwrite.count = r->ifcall.twrite.count;
		depthmode = atoi(buf);

		if ((depthmode % 6) != depthmode)
			depthmode = 0;

		mtimes[path] = time(NULL);
		ixp_respond(r, NULL);
		break;
	case Qtilt:
		r->ofcall.rwrite.count = r->ifcall.twrite.count;
		tilt = atoi (buf);

		mtimes[path] = time(NULL);

		if (freenect_sync_set_tilt_degs (tilt, 0))
			ixp_respond(r, "kinect disconnected");
		else {
			ixp_respond(r, NULL);
		}
		break;
	case Qled:
		r->ofcall.rwrite.count = r->ifcall.twrite.count;
		led = atoi (buf);

		if (led < 0) led = 0;
		else if (led > 6) led = 6;

		mtimes[path] = time(NULL);

		if (freenect_sync_set_led (led, 0))
			ixp_respond(r, "kinect disconnected");
		else {
			ixp_respond(r, NULL);
		}
		break;
	}

	free (buf);
}

static void fs_clunk(Ixp9Req *r)
{
	debug ("fs_clunk fid:%lu\n", r->fid->fid);

	ixp_respond (r, NULL);
}

static void fs_flush(Ixp9Req *r)
{
	debug ("fs_flush fid:%lu\n", r->fid->fid);

	ixp_respond(r, NULL);
}

static void fs_attach(Ixp9Req *r)
{
	r->fid->qid.type = P9_QTDIR;
	r->fid->qid.version = 0;
	r->fid->qid.path = 0;
	r->fid->aux = newfidaux(0);
	r->ofcall.rattach.qid = r->fid->qid;

	debug ("fs_attach fid:%lu\n", r->fid->fid);

	ixp_respond (r, NULL);
}

static void fs_create(Ixp9Req *r)
{
	debug ("fs_create\n");

	ixp_respond (r, "permission denied");
}

static void fs_remove(Ixp9Req *r)
{
	debug ("fs_remove\n");

	ixp_respond (r, "permission denied");
}

static void fs_freefid(IxpFid *f)
{
	FidAux *faux = f->aux;

	debug ("fs_freefid");

	if (faux != NULL) {
		if (faux->fim != NULL) {
			free(faux->fim->image);
			free(faux->fim);
		}

		free (f->aux);
		f->aux = NULL;

		debug (" (free'd)");
	}

	debug ("\n");
}

static void fs_wstat(Ixp9Req *r)
{
	debug ("fs_wstat\n");

	ixp_respond (r, NULL);
}

static IxpServer server;
Ixp9Srv p9srv = {
	.open		= fs_open,
	.walk		= fs_walk,
	.read		= fs_read,
	.stat		= fs_stat,
	.write		= fs_write,
	.clunk		= fs_clunk,
	.flush		= fs_flush,
	.attach		= fs_attach,
	.create		= fs_create,
	.remove		= fs_remove,
	.freefid	= fs_freefid,
	.wstat		= fs_wstat,
};

/*void
freenect_do_one (long ms, void *aux)
{
	struct timeval tv = { 0, 0 };
	if (freenect_process_events_timeout ((freenect_context*)aux, &tv) < 0) {
		perror ("freenect_process_events");
		exit (-1);
	}
	ixp_settimer(&server, 1, freenect_do_one, aux);
}*/

int
main(int argc, char *argv[]) {
	int i, fd, c;
	IxpConn *acceptor;
	freenect_context *f_ctx;
	float v;

	if (argc != 2) {
		fprintf (stderr, "usage:\n\t%s proto!addr[!port]\n", argv[0]);
		return -1;
	}

	if (freenect_init (&f_ctx, NULL) < 0) {
		perror ("freenect_init");
		return -1;
	}
#ifdef USE_AUDIO
	freenect_select_subdevices (f_ctx, FREENECT_DEVICE_AUDIO);
#endif

	if (freenect_num_devices (f_ctx) < 1) {
		fprintf (stderr, "kinect not found\n");
		freenect_shutdown (f_ctx);
		return -1;
	}

	freenect_sync_set_led (led, 0);

	for (i = 0; i < 0x1000000; i++) {
		r2blut[i] = (double)((i & 0xFF0000) >> 16) * 0.2989 +
			    (double)((i & 0xFF00) >> 8) * 0.5870 +
			    (double)(i & 0xFF) * 0.1140;
	}
	for (i = 0; i < 0x800; i++) {
		c = ((i & 0x1FF)/(double)0x200)*256;
		switch (i & 0x600) {
		case 0x000:
			if (c == 0)
				memset(k2rlut[i], 0x00, 3);
			else {
				k2rlut[i][0] = 255;
				k2rlut[i][1] = c;
				k2rlut[i][2] = 0;
			}
			break;
		case 0x200:
			k2rlut[i][0] = 255 - c;
			k2rlut[i][1] = 255;
			k2rlut[i][2] = 0;
			break;
		case 0x400:
			k2rlut[i][0] = 0;
			k2rlut[i][1] = 255;
			k2rlut[i][2] = c;
			break;
		case 0x600:
			k2rlut[i][0] = 0;
			k2rlut[i][1] = 255 - c;
			k2rlut[i][2] = 255;
			break;
		default:
			debug ("k: %x\n", k);
		}
	}

#ifdef USE_AUDIO
	if (freenect_open_device (f_ctx, &f_dev, 0) < 0) {
		fprintf (stderr, "could not open kinect audio\n");
		freenect_shutdown (f_ctx);
		return -1;
	}

	freenect_set_audio_in_callback (f_dev, in_callback);
	freenect_start_audio (f_dev);
#endif
	memset(&rgbdlast, 0, sizeof(struct timeval));
	memset(&tiltlast, 0, sizeof(struct timeval));
	rgbimg.length = 640*480*3+15;
	depthimg.length = 640*480*1+15;
	extraimg.length = 1280*960*1+16;
	edgeimg.length = 640*480*1+15;
	bwimg.length = 640*480*1+15;
	rgbimg.image = calloc(1, rgbimg.length);
	depthimg.image = calloc(1, depthimg.length);
	extraimg.image = calloc(1, extraimg.length);
	edgeimg.image = calloc(1, edgeimg.length);
	bwimg.image = calloc(1, bwimg.length);
#ifdef USE_JPEG
	rgbjpg.image = calloc(1, rgbimg.length);
	depthjpg.image = calloc(1, depthimg.length);
	extrajpg.image = calloc(1, extraimg.length);
	edgejpg.image = calloc(1, edgeimg.length);
	bwjpg.image = calloc(1, bwimg.length);
#endif

	fd = ixp_announce (argv[1]);
	if (fd < 0) {
		perror ("ixp_announce");
		return -1;
	}

//	ixp_settimer(&server, 1, freenect_do_one, f_ctx);

	for (i = 0; i < Npaths; i++) {
		mtimes[i] = time(NULL);
	}

	acceptor = ixp_listen(&server, fd, &p9srv, ixp_serve9conn, NULL);

	ixp_serverloop(&server);
	fprintf (stderr, "%s\n", ixp_errbuf());

	freenect_shutdown (f_ctx);

	return -1;
}
