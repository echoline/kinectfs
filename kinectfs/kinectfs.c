#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <math.h>
#include <ixp.h>
#include <libfreenect_sync.h>
#ifdef USE_JPEG
#include <jpeglib.h>
#endif

unsigned short d2rlut[2048];
char *paths[] = { "/", "rgb", "depth", "tilt", "led",
#ifdef USE_AUDIO
	"mic0", "mic1", "mic2", "mic3",
#endif
	 };
int Npaths = (sizeof(paths)/sizeof(paths[0]));
unsigned long mtimes[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
unsigned long atimes[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

typedef struct {
	struct timeval last;
	unsigned char* image;
	unsigned long length;
} FrnctImg;
static FrnctImg rgbimg;
static FrnctImg depthimg;

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
	if (path == 1 || path == 2) {
		ret->fim = calloc(1, sizeof(FrnctImg));
		ret->fim->length = 640*480*3;
		ret->fim->image = calloc(1, ret->fim->length);
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
	switch (stat->qid.path) {
	case 0:
		stat->mode |= P9_DMDIR|P9_DMEXEC;
		break;
	case 2:
	case 3:
	case 4:
		stat->mode |= P9_DMWRITE;
	default:
		break;
	}
	stat->atime = atimes[path];
	stat->mtime = mtimes[path];
	stat->length = 0;
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

	cinfo.image_width = 640;
	cinfo.image_height = 480;
	cinfo.input_components = 3;
	cinfo.in_color_space = JCS_RGB;

	jpeg_mem_dest( &cinfo, &buf, &buflen );
	jpeg_set_defaults( &cinfo );

	jpeg_start_compress( &cinfo, TRUE );
	while( cinfo.next_scanline < cinfo.image_height ) {
		row_pointer[0] = &img->image[ cinfo.next_scanline * cinfo.image_width * cinfo.input_components];

		jpeg_write_scanlines( &cinfo, row_pointer, 1 );
	}

	jpeg_finish_compress( &cinfo );

	img->length = buflen;
	memcpy(img->image, buf, buflen);

	jpeg_destroy_compress( &cinfo );
	free(buf);
}
#endif

static void fs_open(Ixp9Req *r)
{
	FidAux *f;
	unsigned int ts;
	unsigned char *buf;
	int c;
	unsigned short s;
	int j, k, l, m;
	int path = r->fid->qid.path;

	if (path < 0 || path >= Npaths) {
		respond(r, "file not found");
		return;
	}

	debug ("fs_open %s %lu\n", paths[path], r->fid->fid);

	r->fid->aux = newfidaux(path);
	f = r->fid->aux;

	if (path == 1) {
		if (istime(&rgbimg.last, 1.0/30.0)) {
			if (freenect_sync_get_video((void**)(&buf), &ts, 0, FREENECT_VIDEO_RGB) != 0) {
				respond(r, "freenect_sync_get_video");
				return;
			}
			memcpy(rgbimg.image, buf, 640*480*3);
			rgbimg.length = 640*480*3;
#ifdef USE_JPEG
			compressjpg(&rgbimg);
#endif
		}
		memcpy(f->fim->image, rgbimg.image, rgbimg.length);
		f->fim->length = rgbimg.length;
	}
	if (path == 2) {
		if (istime(&depthimg.last, 1.0/30.0)) {
			if (freenect_sync_get_depth((void**)(&buf), &ts, 0, depthmode) != 0) {
				respond(r, "freenect_sync_get_depth");
				return;
			}
			for (j = 0; j < (640 * 480); j++) {
				s = buf[j * 2];
				s |= buf[j * 2 + 1] << 8;
	
				if (s >= 2048)
					s = 0;

				k = d2rlut[s];
				c = k & 0xff;

				switch (k >> 8) {
				case 0:
					depthimg.image[j * 3] = 255;
					depthimg.image[j * 3 + 1] = 255 - c;
					depthimg.image[j * 3 + 2] = 255 - c;
					break;
				case 1:
					depthimg.image[j * 3] = 255;
					depthimg.image[j * 3 + 1] = c;
					depthimg.image[j * 3 + 2] = 0;
					break;
				case 2:
					depthimg.image[j * 3] = 255 - c;
					depthimg.image[j * 3 + 1] = 255;
					depthimg.image[j * 3 + 2] = 0;
					break;
				case 3:
					depthimg.image[j * 3] = 0;
					depthimg.image[j * 3 + 1] = 255;
					depthimg.image[j * 3 + 2] = c;
					break;
				case 4:
					depthimg.image[j * 3] = 0;
					depthimg.image[j * 3 + 1] = 255 - c;
					depthimg.image[j * 3 + 2] = 255;
					break;
				case 5:
					depthimg.image[j * 3] = 0;
					depthimg.image[j * 3 + 1] = 0;
					depthimg.image[j * 3 + 2] = 255 - c;
					break;
				default:
					depthimg.image[j * 3] = 0;
					depthimg.image[j * 3 + 1] = 0;
					depthimg.image[j * 3 + 2] = 0;
					break;
				}
			}
			depthimg.length = 640*480*3;
#ifdef USE_JPEG
			compressjpg(&depthimg);
#endif
		}
		memcpy(f->fim->image, depthimg.image, depthimg.length);
		f->fim->length = depthimg.length;
	}

	respond (r, NULL);
}

static void fs_walk(Ixp9Req *r)
{
	char *name = malloc (PATH_MAX);
	int path;
	FidAux *f = r->fid->aux;

	path = r->fid->qid.path;

	if (path < 0 || path >= Npaths) {
		respond(r, "file not found");
		return;
	}

	debug ("fs_walk from %s fid:%lu newfid:%lu\n", paths[path], r->fid->fid, r->newfid->fid);

	if (r->ifcall.twalk.nwname == 0 || strcmp(r->ifcall.twalk.wname[0], ".") == 0) {
		respond (r, NULL);
		return;
	} else if (r->ifcall.twalk.nwname != 1) {
		respond (r, "file not found");
		return;
	} else if (strcmp(r->ifcall.twalk.wname[0], "..") == 0) {
		path = 0;
	} else {
		path = fnametopath(r->ifcall.twalk.wname[0]);
	}

	if (path < 0 || path > Npaths) {
		respond (r, "file not found");
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

	respond(r, NULL);
}

static void fs_read(Ixp9Req *r)
{
	FidAux *f = r->fid->aux;
	IxpStat stat;
	IxpMsg m;
	char *buf;
	int n, size;
	int path;
	freenect_raw_tilt_state *state;
	static double dx, dy, dz;
	static struct timeval lasttilt = { 0, 0 };
	int i;

	debug ("fs_read read:%llu offset:%lu\n", r->ifcall.tread.offset, f->offset);

	path = r->fid->qid.path;

	if (path < 0 || path >= Npaths) {
		respond(r, "file not found");
		return;
	}

	if (r->ifcall.tread.offset == 0)
		f->offset = 0;

	switch (path) {
	case 0:
		f->index++;
		// END OF FILES
		if (f->index >= Npaths) {
			respond(r, NULL);
			return;
		}

		size = r->ifcall.tread.count;
		buf = malloc (size);
		m = ixp_message((unsigned char*)buf, size, MsgPack);
		n = dostat(f->index, &stat);
		if (n < 0) {
			free (buf);
			respond (r, "fs_read dostat failed");
			return;
		}
		ixp_pstat(&m, &stat);

		r->ofcall.rread.count = n;
		r->ofcall.rread.data = m.data;

		respond (r, NULL);
		return;
	case 1:
	case 2:
		size = r->ifcall.tread.count;
		if (size < 0) {
			respond(r, "no.");
			return;
		}		

		r->ofcall.rread.count = f->fim->length - r->ifcall.tread.offset;

		if (r->ofcall.rread.count <= 0)
			r->ofcall.rread.count = 0;

		if (size < r->ofcall.rread.count)
			r->ofcall.rread.count = size;

		if (r->ofcall.rread.count != 0) {
			r->ofcall.rread.data = calloc(1, f->fim->length);
			memcpy (r->ofcall.rread.data, &f->fim->image[r->ifcall.tread.offset], r->ofcall.rread.count);
		}

		respond (r, NULL);
		return;
#ifdef USE_AUDIO
	case 5:
	case 6:
	case 7:
	case 8:
		i = path - 5;

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

		respond(r, NULL);
		return;
#endif
	case 3:
		if (f->offset == 0) {
			size = r->ifcall.tread.count;

			if (freenect_sync_get_tilt_state(&state, 0) != 0) {
				respond(r, "freenect_sync_get_tilt_state");
				return;
			} else {
				freenect_get_mks_accel (state, &dx, &dy, &dz);

				buf = malloc (size);
				snprintf(buf, size, "%d %lf %lf %lf\n", tilt, dx, dy, dz);
				r->ofcall.rread.count = strlen(buf);
				r->ofcall.rread.data = buf;
			}
			f->offset += r->ofcall.rread.count;
			atimes[path] = time(NULL);
		} else {
			r->ofcall.rread.count = 0;
		}
		respond(r, NULL);
		return;
	case 4:
		if (f->offset == 0) {
			size = r->ifcall.tread.count;
			buf = malloc (size);

			snprintf(buf, size, "%d\n", led);
			r->ofcall.rread.count = strlen(buf);
			r->ofcall.rread.data = buf;

			f->offset++;
			atimes[path] = time(NULL);
		} else {
			r->ofcall.rread.count = 0;
		}
		respond(r, NULL);
		return;
	}

	respond (r, "invalid path");
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
		respond(r, "file not found");
		return;
	}

	size = dostat(path, &stat);

	buf = malloc (size);
	m = ixp_message(buf, size, MsgPack);
	ixp_pstat(&m, &stat);
	r->ofcall.rstat.nstat = size;
	r->ofcall.rstat.stat = m.data;
	r->fid->qid = stat.qid;
	respond(r, NULL);
}

static void fs_write(Ixp9Req *r)
{
	FidAux *f = r->fid->aux;
	char *buf;
	int path = r->fid->qid.path;

	debug ("fs_write\n");

	if (path < 0 || path >= Npaths) {
		respond (r, "file not found");
		return;
	}

	if (r->ifcall.twrite.count == 0) {
		mtimes[path] = time(NULL);
		respond (r, NULL);
		return;
	}

	buf = malloc (r->ifcall.twrite.count + 1);
	memcpy(buf, r->ifcall.twrite.data, r->ifcall.twrite.count);
	buf[r->ifcall.twrite.count] = '\0';

	switch (path) {
	default:
		respond(r, "permission denied");
		break;
	case 2:
		r->ofcall.rwrite.count = r->ifcall.twrite.count;
		depthmode = atoi(buf);

		if ((depthmode % 6) != depthmode)
			depthmode = 0;

		mtimes[path] = time(NULL);
		respond(r, NULL);
		break;
	case 3:
		r->ofcall.rwrite.count = r->ifcall.twrite.count;
		tilt = atoi (buf);

		mtimes[path] = time(NULL);

		if (freenect_sync_set_tilt_degs (tilt, 0))
			respond(r, "kinect disconnected");
		else {
			respond(r, NULL);
		}
		break;
	case 4:
		r->ofcall.rwrite.count = r->ifcall.twrite.count;
		led = atoi (buf);

		if (led < 0) led = 0;
		else if (led > 6) led = 6;

		mtimes[path] = time(NULL);

		if (freenect_sync_set_led (led, 0))
			respond(r, "kinect disconnected");
		else {
			respond(r, NULL);
		}
		break;
	}

	free (buf);
}

static void fs_clunk(Ixp9Req *r)
{
	debug ("fs_clunk fid:%lu\n", r->fid->fid);

	respond (r, NULL);
}

static void fs_flush(Ixp9Req *r)
{
	debug ("fs_flush fid:%lu\n", r->fid->fid);

	respond(r, NULL);
}

static void fs_attach(Ixp9Req *r)
{
	r->fid->qid.type = P9_QTDIR;
	r->fid->qid.version = 0;
	r->fid->qid.path = 0;
	r->fid->aux = newfidaux(0);
	r->ofcall.rattach.qid = r->fid->qid;

	printf ("fs_attach fid:%lu\n", r->fid->fid);

	respond (r, NULL);
}

static void fs_create(Ixp9Req *r)
{
	debug ("fs_create\n");

	respond (r, "permission denied");
}

static void fs_remove(Ixp9Req *r)
{
	debug ("fs_remove\n");

	respond (r, "permission denied");
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

	respond (r, NULL);
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
	int i, fd;
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
	freenect_select_subdevices (f_ctx, FREENECT_DEVICE_AUDIO);

	if (freenect_num_devices (f_ctx) < 1) {
		fprintf (stderr, "kinect not found\n");
		freenect_shutdown (f_ctx);
		return -1;
	}

	freenect_sync_set_led (led, 0);

	for (i = 0; i < 2048; i++) {
		v = i/2048.0;
		v = powf(v, 3) * 6;
		d2rlut[i] = v * 6 * 256;
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
	memset(&rgbimg.last, 0, sizeof(struct timeval));
	memset(&depthimg.last, 0, sizeof(struct timeval));
	rgbimg.image = calloc(1, 640*480*3);
	depthimg.image = calloc(1, 640*480*3);
	rgbimg.length = 640*480*3;
	depthimg.length = 640*480*3;

	fd = ixp_announce (argv[1]);
	if (fd < 0) {
		perror ("ixp_announce");
		return -1;
	}

//	ixp_settimer(&server, 1, freenect_do_one, f_ctx);

	acceptor = ixp_listen(&server, fd, &p9srv, serve_9pcon, NULL);

	ixp_serverloop(&server);
	fprintf (stderr, "%s\n", ixp_errbuf());

	freenect_shutdown (f_ctx);

	return -1;
}

