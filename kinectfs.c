#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <limits.h>
#include <time.h>
#include <math.h>
#include <unistd.h>
#include <ixp.h>
#include <pthread.h>
#include <libfreenect.h>
#ifdef USE_AUDIO
#include <libfreenect_audio.h>
#ifdef USE_OGG
#include <vorbis/vorbisenc.h>
#endif
#endif
#ifdef USE_JPEG
#include <jpeglib.h>
#endif

freenect_device *f_dev = NULL;
unsigned char *rgbbuf;
unsigned char *rgbback;
unsigned char *depthbuf;
unsigned char *depthback;
pthread_mutex_t rgbmutex;
pthread_cond_t rgbcond;
unsigned char rgbvalid = 0;
pthread_mutex_t depthmutex;
pthread_cond_t depthcond;
unsigned char depthvalid = 0;
pthread_mutex_t audiomutex;
pthread_cond_t audiocond;

unsigned char r2blut[0x1000000];
enum {
	Qroot = 0,
	Qtilt,
	Qled,
	Qrgbpnm,
	Qdepthpnm,
	Qextrapnm,
	Qedgepnm,
	Qbwpnm,
	Qinputs,
#ifdef USE_JPEG
	Qrgbjpg,
	Qdepthjpg,
	Qextrajpg,
	Qedgejpg,
	Qbwjpg,
#endif
#ifdef USE_PLAN9
	Qrgb,
	Qdepth,
	Qextra,
	Qedge,
	Qbw,
#endif
#ifdef USE_AUDIO
	Qmic0,
	Qmic1,
	Qmic2,
	Qmic3,
#ifdef USE_OGG
	Qmics,
#endif
#endif
	Npaths,
};
char *paths[] = { "/", "tilt", "led", "rgb.pnm", "depth.pnm", "extra.pnm",
	"edge.pnm", "bw.pnm", "inputs.bit",
#ifdef USE_JPEG
	"rgb.jpg", "depth.jpg", "extra.jpg", "edge.jpg", "bw.jpg",
#endif
#ifdef USE_PLAN9
	"rgb.bit", "depth.bit", "extra.bit", "edge.bit", "bw.bit",
#endif
#ifdef USE_AUDIO
	"mic0.raw", "mic1.raw", "mic2.raw", "mic3.raw",
#ifdef USE_OGG
	"mics.ogg"
#endif
#endif
	 };
unsigned long mtimes[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
unsigned long atimes[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

typedef struct {
	unsigned char* image;
	unsigned long length;
	unsigned int width;
	unsigned int height;
	unsigned int components;
	unsigned char hdrlen;
} FrnctImg;
static FrnctImg rgbpnm;
static FrnctImg depthpnm;
static FrnctImg extrapnm;
static FrnctImg edgepnm;
static FrnctImg bwpnm;
static FrnctImg inputs;
#ifdef USE_JPEG
static FrnctImg rgbjpg;
static FrnctImg depthjpg;
static FrnctImg extrajpg;
static FrnctImg edgejpg;
static FrnctImg bwjpg;
#endif
#ifdef USE_PLAN9
static FrnctImg rgbimg;
static FrnctImg depthimg;
static FrnctImg extraimg;
static FrnctImg edgeimg;
static FrnctImg bwimg;
#endif
static struct timeval rgbdlast;
static freenect_raw_tilt_state *tiltstate = NULL;
static struct timeval tiltlast;
static int KWIDTH = 640;
static int KHEIGHT = 480;
static int SCALE = 0;
#define FRNCTIMG 1
#define OGGINFO 2
static IxpServer server;

#ifdef USE_AUDIO
#ifdef USE_OGG
typedef struct _OggInfo {
	ogg_stream_state os;
	ogg_page	 og;
	ogg_packet       op;
	vorbis_info      vi;
	vorbis_comment   vc;
	vorbis_dsp_state vd;
	vorbis_block     vb;
} OggInfo;
#endif
#endif

typedef struct _FidAux {
	unsigned int 		version;
	unsigned long		length;
	unsigned long long	offset;
	size_t			index;
	unsigned char		type;
	void			*data;
	unsigned long long 	ogghead;
	unsigned long long	ogglast;
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
	if (path == Qrgbpnm || path == Qdepthpnm || path == Qextrapnm
	  || path == Qedgepnm || path == Qbwpnm || path == Qinputs
#ifdef USE_JPEG
	  || path == Qrgbjpg || path == Qdepthjpg || path == Qextrajpg
	  || path == Qedgejpg || path == Qbwjpg
#endif
#ifdef USE_PLAN9
	  || path == Qrgb || path == Qdepth || path == Qextra || path == Qedge
	  || path == Qbw
#endif
	) {
		ret->type = FRNCTIMG;
		ret->data = (void*)calloc(1, sizeof(FrnctImg));
		// shouldn't need more than this
		((FrnctImg*)ret->data)->image = calloc(1, KWIDTH*2*KHEIGHT*2*4);
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
unsigned long long audiohead[4] = { 0, 0, 0, 0 };
#define AUDIO_BUFFER_SIZE 8192
unsigned char audio[4][AUDIO_BUFFER_SIZE];

#ifdef USE_OGG
unsigned char *ogghdr = NULL;
size_t ogghdrlen = 0;
OggInfo *oi = NULL;
#endif
#endif

static void*
pthread_loop(void *aux)
{
	while(1) {
		struct timeval tv = { 1, 0 };
		if (freenect_process_events_timeout ((freenect_context*)aux, &tv) < 0) {
			perror ("freenect_process_events");
			exit(-1);
		}
	}
}

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
	stat->mode = 0444;
	stat->length = 0;
	switch (stat->qid.path) {
	case Qroot:
		stat->mode |= P9_DMDIR|0111;
		break;
	case Qrgbpnm:
		stat->length = rgbpnm.length;
		stat->mode |= 0222;
		break;
	case Qdepthpnm:
		stat->length = depthpnm.length;
		stat->mode |= 0222;
		break;
	case Qextrapnm:
		stat->length = extrapnm.length;
		break;
	case Qedgepnm:
		stat->length = edgepnm.length;
		break;
	case Qbwpnm:
		stat->length = bwpnm.length;
		break;
	case Qinputs:
		stat->length = inputs.length;
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
#ifdef USE_PLAN9
	case Qrgb:
		stat->length = rgbimg.length;
		break;
	case Qdepth:
		stat->length = depthimg.length;
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
#endif
#ifdef USE_AUDIO
	case Qmic0:
	case Qmic1:
	case Qmic2:
	case Qmic3:
		stat->length = AUDIO_BUFFER_SIZE;
		break;
#endif
	case Qtilt:
	case Qled:
		stat->mode |= 0222;
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
audio_callback(freenect_device *dev, int num_samples, int32_t *mic0,
		int32_t *mic1, int32_t *mic2, int32_t *mic3,
		int16_t *cancelled, void *unknown) {
	int i, j, k, n;
	int length;
	int32_t *mic;

	length = num_samples * sizeof (int32_t);

	pthread_mutex_lock(&audiomutex);
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

		j = audiohead[i] & (AUDIO_BUFFER_SIZE-1);
		k = AUDIO_BUFFER_SIZE - j;

		if (k >= length)
			memcpy(audio[i] + j, mic, length);
		else {
			memcpy(audio[i] + j, mic, k);
			memcpy(audio[i], mic + k, length - k);
		}
		audiohead[i] += length;
	}
	pthread_cond_signal(&audiocond);
	pthread_mutex_unlock(&audiomutex);
}
#endif

void
video_callback (freenect_device *dev, void *buf, uint32_t timestamp) {
	pthread_mutex_lock(&rgbmutex);
	rgbback = rgbbuf;
	freenect_set_video_buffer(dev, rgbback);
	rgbbuf = buf;
	rgbvalid = 1;
//	pthread_cond_signal(&rgbcond);
	pthread_mutex_unlock(&rgbmutex);
}

void
depth_callback (freenect_device *dev, void *buf, uint32_t timestamp) {
	pthread_mutex_lock(&depthmutex);
	depthback = depthbuf;
	freenect_set_depth_buffer(dev, depthback);
	depthbuf = buf;
	depthvalid = 1;
//	pthread_cond_signal(&depthcond);
	pthread_mutex_unlock(&depthmutex);
}

#ifdef USE_JPEG
void
compressjpg(FrnctImg *img)
{
	struct jpeg_compress_struct cinfo;
	struct jpeg_error_mgr jerr;
	JSAMPROW row_pointer[1];
	unsigned char *buf = NULL;
	unsigned long buflen = 0;

	cinfo.err = jpeg_std_error( &jerr );
	jpeg_create_compress(&cinfo);

	cinfo.image_width = img->width;
	cinfo.image_height = img->height;
	cinfo.input_components = img->components;
	cinfo.in_color_space = img->components == 3? JCS_RGB: JCS_GRAYSCALE;

	jpeg_mem_dest( &cinfo, &buf, &buflen );
	jpeg_set_defaults( &cinfo );
	jpeg_set_quality(&cinfo, 100, TRUE);

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

/** Copywritten Plan 9 stuff */

#define CHUNK   8000
#define NMEM	1024
#define NMATCH	3
#define NRUN	(NMATCH+31)
#define NDUMP	128
#define NCBLOCK	6000
#define HSHIFT  5
#define NHASH   (1<<(HSHIFT*NMATCH))
#define HMASK   (NHASH-1)
#define hupdate(h, c)   ((((h)<<HSHIFT)^(c))&HMASK)
typedef struct Hlist Hlist;
struct Hlist{
	unsigned char *s;
	Hlist *next, *prev;
};

void
compressplan9(FrnctImg *img)
{
	unsigned char *outbuf, *outp, *eout;	    /* encoded data, pointer, end */
	unsigned char *loutp;			   /* start of encoded line */
	Hlist *hash;			    /* heads of hash chains of past strings */
	Hlist *chain, *hp;		      /* hash chain members, pointer */
	Hlist *cp;			      /* next Hlist to fall out of window */
	int h;				  /* hash value */
	unsigned char *line, *eline;		    /* input line, end pointer */
	unsigned char *data, *edata;		    /* input buffer, end pointer */
	unsigned long n;				/* length of input buffer */
	unsigned long nb;			       /* # of bytes returned by unloadimage */
	int bpl;				/* input line length */
	int offs, runlen;		       /* offset, length of consumed data */
	unsigned char dumpbuf[NDUMP];		   /* dump accumulator */
	int ndump;			      /* length of dump accumulator */
	int chunk, ncblock;
	unsigned char *p, *q, *s, *es, *t;
	unsigned char *buffer = malloc(KWIDTH*2*KHEIGHT*2*4);
	int y, len;

	bpl = img->width * img->components;
	n = img->height * bpl;
	data = malloc(n);
	memcpy(data, img->image + img->hdrlen, n);

	ncblock = bpl*2;
	if (ncblock < NCBLOCK)
		ncblock = NCBLOCK;
	hash = malloc(NHASH*sizeof(Hlist));
	chain = malloc(NMEM*sizeof(Hlist));
	outbuf = malloc(ncblock);
	if (data == 0 || hash == 0 || chain == 0 || outbuf == 0)
		goto ErrOut;
	edata = data+n;
	eout = outbuf+ncblock;
	line = data;
	y = 0;

	sprintf(buffer, "compressed\n%11s %11d %11d %11d %11d ", img->components == 1? "k8": "b8g8r8", 0, 0, img->width, img->height);
	len = 11+5*12;

	while (line != edata) {
		memset(hash, 0, NHASH*sizeof(Hlist));
		memset(chain, 0, NMEM*sizeof(Hlist));
		cp = chain;
		h = 0;
		outp = outbuf;
		for(n = 0; n != NMATCH; n++)
			h = hupdate(h, line[n]);
		loutp = outbuf;
		while(line != edata){
			ndump = 0;
			eline = line + bpl;
			for(p = line; p != eline;){
				if(eline-p < NRUN)
					es = eline;
				else
					es = p+NRUN;
				q = 0;
				runlen = 0;
				for(hp = hash[h].next; hp; hp = hp->next){
					s = p + runlen;
					if(s >= es)
						continue;
					t = hp->s + runlen;
					for(; s >= p; s--)
						if(*s != *t--)
							goto matchloop;
					t += runlen+2;
					s += runlen+2;
					for(; s < es; s++)
						if(*s != *t++)
							break;
					n = s-p;
					if(n > runlen){
						runlen = n;
						q = hp->s;
						if(n == NRUN)
							break;
					}
matchloop:				;
				}
				if(runlen < NMATCH){
					if(ndump == NDUMP){
						if(eout-outp < ndump+1)
							goto Bfull;
						*outp++ = ndump-1+128;
						memmove(outp, dumpbuf, ndump);
						outp += ndump;
						ndump = 0;
					}
					dumpbuf[ndump++] = *p;
					runlen = 1;
				}
				else{
					if(ndump != 0){
						if(eout-outp < ndump+1)
							goto Bfull;
						*outp++ = ndump-1+128;
						memmove(outp, dumpbuf, ndump);
						outp += ndump;
						ndump = 0;
					}
					offs = p-q-1;
					if(eout-outp < 2)
						goto Bfull;
					*outp++ = ((runlen-NMATCH)<<2) + (offs>>8);
					*outp++ = offs&255;
				}
				for(q = p+runlen; p != q; p++){
					if(cp->prev)
						cp->prev->next = 0;
					cp->next = hash[h].next;
					cp->prev = &hash[h];
					if(cp->next)
						cp->next->prev = cp;
					cp->prev->next = cp;
					cp->s = p;
					if(++cp == &chain[NMEM])
						cp = chain;
					if(edata-p > NMATCH)
						h = hupdate(h, p[NMATCH]);
				}
			}
			if(ndump != 0){
				if(eout-outp < ndump+1)
					goto Bfull;
				*outp++ = ndump-1+128;
				memmove(outp, dumpbuf, ndump);
				outp += ndump;
			}
			line = eline;
			loutp = outp;
			y++;
		}
	Bfull:
		if(loutp == outbuf) {
			fprintf(stderr, "skipping to ErrOut\n");
			goto ErrOut;
		}
		n = loutp-outbuf;
		sprintf(buffer + len, "%11d %11ld ", y, n);
		len += 12*2;
		memcpy(buffer + len, outbuf, n);
		len += n;
	}

	img->length = len;
	memcpy(img->image, buffer, img->length);

ErrOut:
	free(buffer);
	free(data);
	free(chain);
	free(hash);
	free(outbuf);
}

static void
copyimg(FrnctImg *in, FrnctImg *out)
{
	memcpy(out->image, in->image, in->length);
	out->length = in->length;
	out->width = in->width;
	out->height = in->height;
	out->hdrlen = in->hdrlen;
	out->components = in->components;
}

static unsigned char*
halfscalealloc(int width, int height, unsigned char *data) {
	int length = width * height;
	int newlength = length / 4;
	unsigned char *newdata;
	int w = width / 2, h = height / 2;
	int x, y;

	if (width & 1 || height & 1) {
		return NULL;
	}

	newdata = malloc(newlength);
	if (newdata == NULL)
		return NULL;

	for (x = 0; x < w; x++) for(y = 0; y < h; y++) {
		newdata[y*w+x] = (data[((y*2)*width+(x*2))] +
				data[((y*2+1)*width+(x*2+1))] +
				data[((y*2)*width+(x*2+1))] +
				data[((y*2+1)*width+(x*2))]) / 4;
	}

	return newdata;
}

static unsigned char*
middleextract(int width, int height, unsigned char *data) {
	unsigned char *ret;
	int hwidth, hheight, x, y, i, j, m, n;
	if (width < 20 || height < 15)
		return NULL;
	ret = malloc(20 * 15);
	if (height == 15 && width == 20) {
		memcpy(ret, data, width * height);
	} else {
		hwidth = width / 2;
		hheight = height / 2;
		x = hwidth - 10;
		m = x + 20;
		y = hheight - 7;
		n = y + 15;
		if (height & 1 && width & 1) {
			fprintf(stderr, "unimplemented\n");
			free(ret);
			return NULL;
		} else if (height & 1) {
			for (i = y; i < n; i++) for(j = x; j < m; j++)
				ret[(i-y)*20+(j-x)] = data[(i*width+j)];
		} else if (width & 1) {
			fprintf(stderr, "unimplemented\n");
			free(ret);
			return NULL;
		} else {
			y--; n--;
			for (i = y; i < n; i++) for(j = x; j < m; j++)
				ret[(i-y)*20+(j-x)] = (data[(i*width+j)]+data[((i+1)*width+j)])>>1;
		}
	}
	return ret;
}

static void fs_open(Ixp9Req *r)
{
	FidAux *f;
	unsigned int ts;
	int c;
	unsigned short s;
	int j, k, l, x, xm, xp, y, ym, yp;
	int path;
	unsigned char *bufa, *bufb, *bufc, *bufd;

	path = r->fid->qid.path;
	if (path < 0 || path >= Npaths) {
		ixp_respond(r, "file not found");
		return;
	}

	debug ("fs_open %s %u\n", paths[path], r->fid->fid);

	r->fid->aux = newfidaux(path);
	f = r->fid->aux;

	if (path == Qrgbpnm || path == Qdepthpnm || path == Qextrapnm
	  || path == Qedgepnm || path == Qbwpnm || path == Qinputs
#ifdef USE_JPEG
	  || path == Qrgbjpg || path == Qdepthjpg || path == Qextrajpg
	  || path == Qedgejpg || path == Qbwjpg
#endif
#ifdef USE_PLAN9
	  || path == Qrgb || path == Qdepth || path == Qextra || path == Qedge
	  || path == Qbw
#endif
	) {
		if (istime(&rgbdlast, 1.0/30.0)) {
			rgbpnm.width = KWIDTH;
			rgbpnm.height = KHEIGHT;
			rgbpnm.hdrlen = 15;
			rgbpnm.components = 3;
			rgbpnm.length = rgbpnm.width*rgbpnm.height*rgbpnm.components+rgbpnm.hdrlen;
			memcpy(rgbpnm.image, "P6\n640 480\n255\n", rgbpnm.hdrlen);

			depthpnm.width = KWIDTH;
			depthpnm.height = KHEIGHT;
			depthpnm.hdrlen = 15;
			depthpnm.components = 1;
			depthpnm.length = depthpnm.width*depthpnm.height*depthpnm.components+depthpnm.hdrlen;
			memcpy(depthpnm.image, "P5\n640 480\n255\n", depthpnm.hdrlen);

			extrapnm.width = KWIDTH;
			extrapnm.height = KHEIGHT;
			extrapnm.hdrlen = 15;
			extrapnm.components = 3;
			extrapnm.length = extrapnm.width*extrapnm.height*extrapnm.components+extrapnm.hdrlen;
			memcpy(extrapnm.image, "P6\n640 480\n255\n", extrapnm.hdrlen);

			edgepnm.width = KWIDTH;
			edgepnm.height = KHEIGHT;
			edgepnm.hdrlen = 15;
			edgepnm.components = 1;
			edgepnm.length = edgepnm.width*edgepnm.height*edgepnm.components+edgepnm.hdrlen;
			memcpy(edgepnm.image, "P5\n640 480\n255\n", edgepnm.hdrlen);

			bwpnm.width = KWIDTH;
			bwpnm.height = KHEIGHT;
			bwpnm.hdrlen = 15;
			bwpnm.components = 1;
			bwpnm.length = bwpnm.width*bwpnm.height*bwpnm.components+bwpnm.hdrlen;
			memcpy(bwpnm.image, "P5\n640 480\n255\n", bwpnm.hdrlen);

			inputs.width = 20*6;
			inputs.height = 15*2;
			inputs.hdrlen = 60;
			inputs.components = 1;
			inputs.length = inputs.width*inputs.height*inputs.components+inputs.hdrlen;
			sprintf(inputs.image, "%11s %11d %11d %11d %11d ", "k8", 0, 0, 6*20, 2*15);

			pthread_mutex_lock(&rgbmutex);
//			while (rgbvalid == 0)
//				pthread_cond_wait(&rgbcond, &rgbmutex);
			if(SCALE) for (x = 0; x < KWIDTH; x++) for (y = 0; y < KHEIGHT; y++) {
				for (l = 0; l < 3; l++) {
					j = 0;
					for (xp = 0; xp < 4; xp++) for (yp = 0; yp < 4; yp++)
						j += rgbbuf[(((y*4+yp)*KWIDTH+x)*4+xp)*3+l];
					rgbpnm.image[rgbpnm.hdrlen+((y*KWIDTH)+x)*3+l] = j / 16;
				}
			} else for (x = 0; x < KWIDTH; x++) for (y = 0; y < KHEIGHT; y++) {
				for (l = 0; l < 3; l++)
					rgbpnm.image[rgbpnm.hdrlen+((y*KWIDTH)+x)*3+l] = rgbbuf[(y*KWIDTH+x)*3+l];
			}
			rgbvalid = 0;
			pthread_mutex_unlock(&rgbmutex);
			// need black and white first for edges
			for (x = 0; x < KWIDTH; x++) for (y = 0; y < KHEIGHT; y++) {
				j = y * KWIDTH + x;
				c = r2blut[((rgbpnm.image[j*3+rgbpnm.hdrlen]<<16) | (rgbpnm.image[j*3+rgbpnm.hdrlen+1]<<8) | (rgbpnm.image[j*3+rgbpnm.hdrlen+2]))];
				bwpnm.image[j + bwpnm.hdrlen] = c;
				extrapnm.image[j*3 + extrapnm.hdrlen] = c;
			}
			pthread_mutex_lock(&depthmutex);
//			while (depthvalid == 0)
//				pthread_cond_wait(&depthcond, &depthmutex);
			for (x = 0; x < KWIDTH; x++) for (y = 0; y < KHEIGHT; y++) {
				j = y * KWIDTH + x;
				l = j + bwpnm.hdrlen;

				if (x > 0 && x < (KWIDTH-1) && y > 0 && y < (KHEIGHT-1)) {
					k = abs(bwpnm.image[l - (KWIDTH+1)] * -1 + bwpnm.image[l - (KWIDTH-1)] * 1 + bwpnm.image[l - 1] * -2 + bwpnm.image[l + 1] * 2 + bwpnm.image[l + (KWIDTH-1)] * -1 + bwpnm.image[l + (KWIDTH+1)] * 1 + bwpnm.image[l - (KWIDTH+1)] * 1 + bwpnm.image[l - KWIDTH] * 2 + bwpnm.image[l - (KWIDTH-1)] * 1 + bwpnm.image[l + (KWIDTH-1)] * -1 + bwpnm.image[l + KWIDTH] * -2 + bwpnm.image[l + (KWIDTH+1)] * -1);
					extrapnm.image[(y * KWIDTH + x) * 3 + 1 + extrapnm.hdrlen] = edgepnm.image[l] = k & 0xFF;
				}

				k = 0;
				if (SCALE) {
					for (xp = 0; xp < 4; xp++) for (yp = 0; yp < 4; yp++) {
						s = depthbuf[((((y*4)+yp) * KWIDTH + x) * 4 + xp) * 2];
						s |= depthbuf[((((y*4)+yp) * KWIDTH + x) * 4 + xp) * 2 + 1] << 8;

						if (s >= 0x1000)
							s = 0;

						k += s & 0xFFF;
					}
					k /= 16;
				} else {
					s = depthbuf[(y * KWIDTH + x) * 2];
					s |= depthbuf[(y * KWIDTH + x) * 2 + 1] << 8;

					if (s >= 0x1000)
						s = 0;

					k += s & 0xFFF;
				}
				k >>= 4;
				extrapnm.image[(y * KWIDTH + x) * 3 + 2 + extrapnm.hdrlen] = depthpnm.image[l] = k == 0? 0: 255 - k;
			}
			depthvalid = 0;
			pthread_mutex_unlock(&depthmutex);
		}

		switch(path){
		case Qrgbpnm:
			copyimg(&rgbpnm, f->data);
			break;
		case Qdepthpnm:
			copyimg(&depthpnm, f->data);
			break;
		case Qextrapnm:
			copyimg(&extrapnm, f->data);
			break;
		case Qedgepnm:
			copyimg(&edgepnm, f->data);
			break;
		case Qbwpnm:
			copyimg(&bwpnm, f->data);
			break;
		case Qinputs:
			bufc = malloc(640*480);
			bufd = malloc(640*480);
			for (x = 0; x < 640; x++) for (y = 0; y < 480; y++) {
				bufc[y*640+x] = extrapnm.image[extrapnm.hdrlen+(y*640+x)*3+1];
				bufd[y*640+x] = extrapnm.image[extrapnm.hdrlen+(y*640+x)*3+2];
			}
			for (j = 0; j < 6; j++) {
				bufa = middleextract(640>>j, 480>>j, bufc);
				bufb = middleextract(640>>j, 480>>j, bufd);
				for (x = 0; x < 20; x++) for (y = 0; y < 15; y++)
					inputs.image[inputs.hdrlen+j*20+y*120+x] = bufa[y*20+x];
				for (x = 0; x < 20; x++) for (y = 0; y < 15; y++)
					inputs.image[inputs.hdrlen+j*20+y*120+x+1800] = bufb[y*20+x];
				free(bufa);
				free(bufb);
				bufa = halfscalealloc(640>>j, 480>>j, bufc);
				bufb = halfscalealloc(640>>j, 480>>j, bufd);
				free(bufc);
				free(bufd);
				bufc = bufa;
				bufd = bufb;
			}
			free(bufc);
			free(bufd);
			copyimg(&inputs, f->data);
			break;
#ifdef USE_JPEG
		case Qrgbjpg:
			copyimg(&rgbpnm, &rgbjpg);
			compressjpg(&rgbjpg);
			copyimg(&rgbjpg, f->data);
			break;
		case Qdepthjpg:
			copyimg(&depthpnm, &depthjpg);
			compressjpg(&depthjpg);
			copyimg(&depthjpg, f->data);
			break;
		case Qextrajpg:
			copyimg(&extrapnm, &extrajpg);
			compressjpg(&extrajpg);
			copyimg(&extrajpg, f->data);
			break;
		case Qedgejpg:
			copyimg(&edgepnm, &edgejpg);
			compressjpg(&edgejpg);
			copyimg(&edgejpg, f->data);
			break;
		case Qbwjpg:
			copyimg(&bwpnm, &bwjpg);
			compressjpg(&bwjpg);
			copyimg(&bwjpg, f->data);
			break;
#endif
#ifdef USE_PLAN9
		case Qrgb:
			copyimg(&rgbpnm, &rgbimg);
			compressplan9(&rgbimg);
			copyimg(&rgbimg, f->data);
			break;
		case Qdepth:
			copyimg(&depthpnm, &depthimg);
			compressplan9(&depthimg);
			copyimg(&depthimg, f->data);
			break;
		case Qextra:
			copyimg(&extrapnm, &extraimg);
			compressplan9(&extraimg);
			copyimg(&extraimg, f->data);
			break;
		case Qedge:
			copyimg(&edgepnm, &edgeimg);
			compressplan9(&edgeimg);
			copyimg(&edgeimg, f->data);
			break;
		case Qbw:
			copyimg(&bwpnm, &bwimg);
			compressplan9(&bwimg);
			copyimg(&bwimg, f->data);
			break;
#endif
		default:
			ixp_respond(r, "file not found");
			return;
		}
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

	debug ("fs_walk from %s fid:%u newfid:%u\n", paths[path], r->fid->fid, r->newfid->fid);

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
	int i, j, l;
#ifdef USE_OGG
	int done;
	float **obuf;
	unsigned char *oggnew;
	unsigned char ogg[AUDIO_BUFFER_SIZE];
#endif

	debug ("fs_read read:%lu\n", r->ifcall.tread.offset);

	path = r->fid->qid.path;

	if (path < 0 || path >= Npaths) {
		ixp_respond(r, "file not found");
		return;
	}

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
	case Qrgbpnm:
	case Qdepthpnm:
	case Qextrapnm:
	case Qedgepnm:
	case Qbwpnm:
	case Qinputs:
#ifdef USE_JPEG
	case Qrgbjpg:
	case Qdepthjpg:
	case Qextrajpg:
	case Qedgejpg:
	case Qbwjpg:
#endif
#ifdef USE_PLAN9
	case Qrgb:
	case Qdepth:
	case Qextra:
	case Qedge:
	case Qbw:
#endif
		size = r->ifcall.tread.count;
		if (size < 0) {
			ixp_respond(r, "no.");
			return;
		}

		r->ofcall.rread.count = ((FrnctImg*)f->data)->length - r->ifcall.tread.offset;

		if (r->ofcall.rread.count <= 0)
			r->ofcall.rread.count = 0;

		if (size < r->ofcall.rread.count)
			r->ofcall.rread.count = size;

		if (r->ofcall.rread.count != 0) {
			r->ofcall.rread.data = malloc(r->ofcall.rread.count);
			memcpy (r->ofcall.rread.data, &((FrnctImg*)f->data)->image[r->ifcall.tread.offset], r->ofcall.rread.count);
		}

		ixp_respond (r, NULL);
		return;
#ifdef USE_AUDIO
	case Qmic0:
	case Qmic1:
	case Qmic2:
	case Qmic3:
		i = path - Qmic0;

		if (f->offset == 0)
			f->offset = audiohead[i];

		pthread_mutex_lock(&audiomutex);
		while ((n = (audiohead[i] - f->offset)) == 0)
			pthread_cond_wait(&audiocond, &audiomutex);
		if (n > r->ifcall.tread.count)
			n = r->ifcall.tread.count;
		l = AUDIO_BUFFER_SIZE - n % AUDIO_BUFFER_SIZE;
		size = AUDIO_BUFFER_SIZE - l;
		debug ("fs_read offset:%llu l:%d n:%d size:%d\n", f->offset, l, n, size);
		buf = malloc(n);
		if ((l + n) <= AUDIO_BUFFER_SIZE)
			memcpy(buf, audio[i] + l, n);
		else {
			debug ("fs_read wrap\n");
			memcpy(buf, audio[i] + l, size);
			memcpy(buf + size, audio[i], n - size);
		}
		pthread_mutex_unlock(&audiomutex);

		r->ofcall.rread.count = n;
		r->ofcall.rread.data = buf;
		f->offset += n;

		ixp_respond(r, NULL);
		debug ("fs_read offset:%llu l:%d n:%d size:%d\n", f->offset, l, n, size);
		return;
#ifdef USE_OGG
	case Qmics:
		if (f->offset < ogghdrlen) {
			n = ogghdrlen - f->offset;
			if (n > r->ifcall.tread.count)
				n = r->ifcall.tread.count;
			buf = malloc(n);
			memcpy(buf, ogghdr + f->offset, n);
			r->ofcall.rread.data = buf;
			r->ofcall.rread.count = n;
			f->offset += n;

			ixp_respond(r, NULL);
			return;
		} else if (f->offset == ogghdrlen) {
			f->ogglast = audiohead[0] - (AUDIO_BUFFER_SIZE>>1);
			f->index = f->length = 0;
		}

		size = audiohead[0] - f->ogglast;
		j = (f->ogglast & (AUDIO_BUFFER_SIZE-1));
		if (size < 0)
			size = audiohead[0] + (AUDIO_BUFFER_SIZE - j);

		if (f->index == f->length) {
			j = j >> 2;
			l = size >> 2;
			n = (AUDIO_BUFFER_SIZE>>2)-1;
			obuf = vorbis_analysis_buffer(&oi->vd, l);
			for (i = 0; i < l; i++) {
				obuf[0][i] = (float)((int32_t*)audio[0])[(j + i) & n] / INT_MAX;
				obuf[1][i] = (float)((int32_t*)audio[1])[(j + i) & n] / INT_MAX;
				obuf[2][i] = (float)((int32_t*)audio[2])[(j + i) & n] / INT_MAX;
				obuf[3][i] = (float)((int32_t*)audio[3])[(j + i) & n] / INT_MAX;
			}
			vorbis_analysis_wrote(&oi->vd, l);
			f->ogglast += size;
			usleep(((float)size/64)*1000);

			n = 0;
			while (1) {
				i = vorbis_analysis_blockout(&oi->vd, &oi->vb);
				if (i < 0) {
					fprintf(stderr, "error: vorbis_analysis_blockout\n");
					break;
				}
				if (i == 0)
					break;
				if (vorbis_analysis(&oi->vb, NULL) != 0) {
					fprintf(stderr, "error: vorbis_analysis\n");
					break;
				}
				if (vorbis_bitrate_addblock(&oi->vb) < 0) {
					fprintf(stderr, "error: vorbis_bitrate_addblock\n");
					break;
				}
				while (vorbis_bitrate_flushpacket(&oi->vd, &oi->op) == 1) {
					while (1) {
						i = ogg_stream_pageout(&oi->os, &oi->og);
						if (i != 0)
							break;
						j = oi->og.header_len + oi->og.body_len;
						oggnew = malloc(j);
						memcpy(oggnew, oi->og.header, oi->og.header_len);
						memcpy(oggnew + oi->og.header_len, oi->og.body, oi->og.body_len);
						l = AUDIO_BUFFER_SIZE - n;
						if (l >= j) {
							memcpy(ogg + n, oggnew, j);
							n += j;
						} else if (l != 0) {
							memcpy(ogg + n, oggnew, l);
							n += l;
						}
						free(oggnew);
						if (ogg_page_eos(&oi->og))
							break;
					}
				}
			}

			if (f->data != NULL) {
				free(f->data);
				f->data = NULL;
			}
			f->data = malloc(n);
			memcpy(f->data, ogg, n);
			f->length = n;
			f->index = 0;
			f->offset += n;
		}

		n = f->length - f->index;
		if (n > r->ifcall.tread.count)
			n = r->ifcall.tread.count;
		if (n <= 0) {
			r->ofcall.rread.count = 0;
		} else {
			buf = malloc(n);
			memcpy(buf, f->data + f->index, n);
			r->ofcall.rread.data = buf;
			r->ofcall.rread.count = n;
			f->index += n;
		}

		ixp_respond(r, NULL);

		return;
#endif
#endif
	case Qtilt:
		if (r->ifcall.tread.offset == 0) {
			size = r->ifcall.tread.count;

			if (tiltstate==NULL || istime(&tiltlast,1.0/4.0)) {
				freenect_update_tilt_state(f_dev);
			    	tiltstate = freenect_get_tilt_state(f_dev);
				if (tiltstate == NULL) {
					ixp_respond(r, "freenect_get_tilt_state");
					return;
				}
			}

			freenect_get_mks_accel (tiltstate, &dx, &dy, &dz);

			buf = malloc (size);
			snprintf(buf, size, "%d %lf %lf %lf\n", tilt, dx, dy, dz);
			r->ofcall.rread.count = strlen(buf);
			r->ofcall.rread.data = buf;

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

	debug ("fs_stat fid:%u\n", r->fid->fid);

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
	case Qrgbpnm:
		r->ofcall.rwrite.count = r->ifcall.twrite.count;
		rgbmode = atoi(buf);

		if ((rgbmode % 7) != rgbmode)
			rgbmode = 0;

		mtimes[path] = time(NULL);
		ixp_respond(r, NULL);
		break;
	case Qdepthpnm:
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

		if (freenect_set_tilt_degs (f_dev, tilt)) {
			ixp_respond(r, "kinect disconnected");
		} else {
			ixp_respond(r, NULL);
		}
		break;
	case Qled:
		r->ofcall.rwrite.count = r->ifcall.twrite.count;
		led = atoi (buf);

		if (led < 0) led = 0;
		else if (led > 6) led = 6;

		mtimes[path] = time(NULL);

		if (freenect_set_led (f_dev, led)) {
			ixp_respond(r, "kinect disconnected");
		} else { 
			ixp_respond(r, NULL);
		}
		break;
	}

	free (buf);
}

static void fs_clunk(Ixp9Req *r)
{
	debug ("fs_clunk fid:%u\n", r->fid->fid);

	ixp_respond (r, NULL);
}

static void fs_flush(Ixp9Req *r)
{
	debug ("fs_flush fid:%u\n", r->fid->fid);

	ixp_respond(r, NULL);
}

static void fs_attach(Ixp9Req *r)
{
	r->fid->qid.type = P9_QTDIR;
	r->fid->qid.version = 0;
	r->fid->qid.path = 0;
	r->fid->aux = newfidaux(0);
	r->ofcall.rattach.qid = r->fid->qid;

	debug ("fs_attach fid:%u\n", r->fid->fid);

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
		if (faux->data != NULL) {
			if (faux->type == FRNCTIMG)
				free(((FrnctImg*)faux->data)->image);
			free(faux->data);
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

	ixp_respond (r, "permission denied");
}

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

int
main(int argc, char *argv[]) {
	int i, fd, c, addr;
	IxpConn *acceptor;
	float v;
	freenect_context *f_ctx;
	int devices = FREENECT_DEVICE_MOTOR | FREENECT_DEVICE_CAMERA;
#ifdef USE_AUDIO
	devices |= FREENECT_DEVICE_AUDIO;
#ifdef USE_OGG
	ogg_packet header;
	ogg_packet header_comm;
	ogg_packet header_code;
#endif
#endif

	if (argc < 2) {
		fprintf (stderr, "usage:\n\t%s [-s] proto!addr[!port]\n", argv[0]);
		return -1;
	}

	for (addr = 1; addr < (argc-1); addr++) {
		if (strcmp(argv[addr], "-s") == 0) {
			//SCALE = 1;
			//KWIDTH = 160;
			//KHEIGHT = 120;
		}
	}

	pthread_mutex_init(&rgbmutex, NULL);
	pthread_cond_init(&rgbcond, NULL);
	pthread_mutex_init(&depthmutex, NULL);
	pthread_cond_init(&depthcond, NULL);
	pthread_mutex_init(&audiomutex, NULL);
	pthread_cond_init(&audiocond, NULL);

	if (freenect_init (&f_ctx, NULL) < 0) {
		perror ("freenect_init");
		return -1;
	}
//	freenect_set_log_level (f_ctx, FREENECT_LOG_SPEW);
	freenect_select_subdevices (f_ctx, devices);

	if (freenect_num_devices (f_ctx) < 1) {
		fprintf (stderr, "kinect not found\n");
		freenect_shutdown (f_ctx);
		return -1;
	}

	if (freenect_open_device (f_ctx, &f_dev, 0) < 0) {
		fprintf (stderr, "could not open kinect\n");
		freenect_shutdown (f_ctx);
		return -1;
	}
	freenect_set_led (f_dev, led);

	for (i = 0; i < 0x1000000; i++) {
		r2blut[i] = (double)((i & 0xFF0000) >> 16) * 0.2989 +
			    (double)((i & 0xFF00) >> 8) * 0.5870 +
			    (double)(i & 0xFF) * 0.1140;
	}

#ifdef USE_AUDIO
#ifdef USE_OGG
	oi = calloc(1, sizeof(OggInfo));
	vorbis_info_init(&oi->vi);
	i = vorbis_encode_init_vbr(&oi->vi, 4, 16000, 0.4);
	if (i != 0) {
		fprintf(stderr, "error: vorbis_encode_init_vbr");
		return -1;
	}
	vorbis_comment_init(&oi->vc);
	vorbis_comment_add_tag(&oi->vc, "ENCODER", "kinectfs");
	vorbis_analysis_init(&oi->vd, &oi->vi);
	vorbis_block_init(&oi->vd, &oi->vb);
	srand(time(NULL));
	ogg_stream_init(&oi->os, rand());
	vorbis_analysis_headerout(&oi->vd, &oi->vc, &header, &header_comm, &header_code);
	ogg_stream_packetin(&oi->os, &header);
	ogg_stream_packetin(&oi->os, &header_comm);
	ogg_stream_packetin(&oi->os, &header_code);
	i = ogg_stream_flush(&oi->os, &oi->og);
	while (i != 0) {
		ogghdrlen += oi->og.header_len;
		ogghdr = realloc(ogghdr, ogghdrlen);
		memcpy(ogghdr + ogghdrlen - oi->og.header_len, oi->og.header, oi->og.header_len);
		ogghdrlen += oi->og.body_len;
		ogghdr = realloc(ogghdr, ogghdrlen);
		memcpy(ogghdr + ogghdrlen - oi->og.body_len, oi->og.body, oi->og.body_len);
		i = ogg_stream_flush(&oi->os, &oi->og);
	}
#endif
#endif

#ifdef USE_AUDIO
	freenect_set_audio_in_callback (f_dev, audio_callback);
	freenect_start_audio (f_dev);
#endif
	rgbbuf = calloc(640, 480 * 3);
	rgbback = calloc(640, 480 * 3);
	depthbuf = calloc(640, 480 * 2);
	depthback = calloc(640, 480 * 2);

	freenect_set_video_callback (f_dev, video_callback);
	freenect_set_video_buffer (f_dev, rgbback);
	freenect_set_video_mode(f_dev, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, rgbmode));
	freenect_set_depth_callback (f_dev, depth_callback);
	freenect_set_depth_buffer (f_dev, depthback);
	freenect_set_depth_mode(f_dev, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, depthmode));

	freenect_start_video (f_dev); 
	freenect_start_depth (f_dev);

	memset(&rgbdlast, 0, sizeof(struct timeval));
	memset(&tiltlast, 0, sizeof(struct timeval));
	rgbpnm.length = KWIDTH*KHEIGHT*3+15;
	depthpnm.length = KWIDTH*KHEIGHT*1+15;
	extrapnm.length = KWIDTH*KHEIGHT*3+15;
	edgepnm.length = KWIDTH*KHEIGHT*1+15;
	bwpnm.length = KWIDTH*KHEIGHT*1+15;
	inputs.length = 20*15*6*2;
	rgbpnm.image = calloc(1, rgbpnm.length);
	depthpnm.image = calloc(1, depthpnm.length);
	extrapnm.image = calloc(1, extrapnm.length);
	edgepnm.image = calloc(1, edgepnm.length);
	bwpnm.image = calloc(1, bwpnm.length);
	inputs.image = calloc(1, inputs.length + 60);
#ifdef USE_JPEG
	rgbjpg.image = calloc(1, rgbpnm.length);
	depthjpg.image = calloc(1, depthpnm.length);
	extrajpg.image = calloc(1, extrapnm.length);
	edgejpg.image = calloc(1, edgepnm.length);
	bwjpg.image = calloc(1, bwpnm.length);
#endif
#ifdef USE_PLAN9
	// these need to be huge because compression may increase size
	rgbimg.image = calloc(1, rgbpnm.length*2);
	depthimg.image = calloc(1, depthpnm.length*2);
	extraimg.image = calloc(1, extrapnm.length*2);
	edgeimg.image = calloc(1, edgepnm.length*2);
	bwimg.image = calloc(1, bwpnm.length*2);
#endif

	ixp_pthread_init();

	fd = ixp_announce (argv[addr]);
	if (fd < 0) {
		perror ("ixp_announce");
		return -1;
	}

	pthread_t camthread;
	if (pthread_create(&camthread, NULL, pthread_loop, (void*)f_ctx) != 0) {
		perror("pthread_create");
		return -1;
	}

	for (i = 0; i < Npaths; i++) {
		mtimes[i] = time(NULL);
	}

	acceptor = ixp_listen(&server, fd, &p9srv, ixp_serve9conn, NULL);

	ixp_serverloop(&server);
	fprintf (stderr, "%s\n", ixp_errbuf());

	freenect_shutdown (f_ctx);

#ifdef USE_AUDIO
#ifdef USE_OGG
	ogg_stream_clear(&oi->os);
	vorbis_block_clear(&oi->vb);
	vorbis_dsp_clear(&oi->vd);
	vorbis_comment_clear(&oi->vc);
	vorbis_info_clear(&oi->vi);
#endif
#endif

	return -1;
}

