#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <ixp.h>
#include <libfreenect_sync.h>
#include "fids.h"

char *paths[] = { "/", "rgb", "depth", "tilt", "led",
	"audio0", "audio1", "audio2", "audio3", NULL };
unsigned long mtimes[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
unsigned long atimes[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
int tilt;
int led;
unsigned char *audio[4];
size_t audiolengths[4];
unsigned int audioclients = 0;

#define debug(...) fprintf (stderr, __VA_ARGS__);
//#define debug(...) {}; 

typedef struct {
	struct timeval last;
	void* image;
	unsigned long length;
} FrnctImg;
static freenect_device *f_dev;

// have epsilon seconds passed since last?
// if so, update last and return 1;
// else return 0;
char
istime(struct timeval *last, double epsilon) {
	struct timeval now;

	gettimeofday(&now, NULL);

	if (now.tv_sec < last->tv_sec)
		goto ISTIMEEND;

	if (now.tv_sec > (last->tv_sec + epsilon))
		goto ISTIMEEND;

	// waaaaait...
	if (((last->tv_sec + last->tv_usec / 1000000.0) - 
	    (now.tv_sec + now.tv_usec / 1000000.0)) < epsilon)
		return 0;

ISTIMEEND:
	last->tv_sec = now.tv_sec;
	last->tv_usec = now.tv_usec;

	return 1;
}

int
dostat(char *name, IxpStat *stat) {
	static char *none = "none";
	int path = fnametopath(name);

	if (path < 0) {
		return -1;
	}

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

static void fs_open(Ixp9Req *r)
{
	FidAux *f = r->fid->aux;
	int path;

	debug ("fs_open\n");

	if (f == NULL) {
		respond (r, "fs_open (f == NULL)");
		return;
	}

	path = fnametopath(f->name);

	switch (path) {
	case 5:
	case 6:
	case 7:
	case 8:
		if (audioclients == 0)
			freenect_start_audio (f_dev);

		audioclients++;
	default:
		break;
	}

	respond (r, NULL);
}

static void fs_walk(Ixp9Req *r)
{
	char *name = malloc (PATH_MAX);
	int path;
	FidAux *f = r->fid->aux;

	if (f == NULL) {
		respond (r, "fs_walk (f == NULL)");
		return;
	}

	if (r->ifcall.twalk.nwname == 0) {
		path = fnametopath(f->name);
	} else if (r->ifcall.twalk.nwname != 1) {
		respond (r, "no such file");
		return;
	} else if (strcmp(r->ifcall.twalk.wname[0], "..") == 0) {
		path = 0;
	} else {
		path = fnametopath(r->ifcall.twalk.wname[0]);
	}

	if (path < 0) {
		respond (r, "no such file");
		return;
	}

	r->ofcall.rwalk.wqid[0].version = 0;
	r->ofcall.rwalk.wqid[0].path = path;
	if (path == 0)
		r->ofcall.rwalk.wqid[0].type = P9_QTDIR;
	else
		r->ofcall.rwalk.wqid[0].type = P9_QTFILE;

	r->newfid->aux = newfidaux(paths[path]);
	r->ofcall.rwalk.nwqid = 1;

	debug ("fs_walk fid:%d newfid:%d name:%s\n", r->fid->fid, 
			r->newfid->fid, paths[path]);

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
	static FrnctImg imgs[2] = { { { 0, 0 }, NULL, 640 * 480 * 3 },
				    { { 0, 0 }, NULL, 640 * 480 * 2 } };
	int i;
	uint32_t ts;

	debug ("fs_read offset:%d\n", r->ifcall.tread.offset);

	if (f == NULL) {
		respond(r, "fs_read (fid->aux == NULL)\n");
		return;
	}

	path = fnametopath(f->name);

	if (r->ifcall.tread.offset == 0)
		f->offset = 0;

	switch (path) {
	case 0:
		f->offset++;

		// END OF FILES
		if (paths[f->offset] == NULL){
			respond(r, NULL);
			return;
		}

		size = r->ifcall.tread.count;
		buf = malloc (size);
		m = ixp_message((unsigned char*)buf, size, MsgPack);
		n = dostat(paths[f->offset], &stat);
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
		buf = malloc (size);
		i = path - 1;

		if (istime(&(imgs[i].last), 1)) {
			if (imgs[i].image)
				free (imgs[i].image);
			imgs[i].image = malloc (imgs[i].length);
			if (i == 0?
					freenect_sync_get_video(
						(void**)&imgs[i].image, &ts, 0,
						FREENECT_VIDEO_RGB)
				:
					freenect_sync_get_depth(
						(void**)&imgs[i].image, &ts, 0,
						FREENECT_DEPTH_REGISTERED)) {
				respond (r, "is kinect connected?");
				free (buf);
				return;
			}
			f->version++;

			// end current reads if we have a new one
			if (r->ofcall.rread.offset != 0) {
				respond (r, NULL);
				free (buf);
				return;
			}
		}
		r->ofcall.rread.count = imgs[i].length - r->ifcall.tread.offset;
		if (r->ofcall.rread.count < 0)
			r->ofcall.rread.count = 0;
		if (size < r->ofcall.rread.count)
			r->ofcall.rread.count = size;
		if (r->ofcall.rread.count != 0)
			memcpy (buf,
				&((char*)imgs[i].image)[r->ifcall.tread.offset],
				r->ofcall.rread.count);
		r->ofcall.rread.data = buf;

		respond (r, NULL);
		return;
	case 5:
	case 6:
	case 7:
	case 8:
		size = r->ifcall.tread.count;
		buf = malloc (size);

		respond(r, NULL);
//		respond(r, "unimplemented");
		return;
	case 3:
		if (f->offset == 0) {
			size = r->ifcall.tread.count;
			buf = malloc (size);

			if (istime(&lasttilt, 0.1)) {
				if (freenect_sync_get_tilt_state(&state, 0)) {
					respond (r, "kinect not connected");
					return;
				}

				freenect_get_mks_accel (state, &dx, &dy, &dz);
			}

			snprintf(buf, size, "%d %lf %lf %lf\n", tilt, dx, dy, dz);
			r->ofcall.rread.count = strlen(buf);
			r->ofcall.rread.data = buf;

			f->offset++;
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
	FidAux *f = r->fid->aux;
	IxpStat stat;
	IxpMsg m;
	int size;
	char *buf;

	debug ("fs_stat fid:%d\n", r->fid->fid);

	if (f == NULL) {
		respond (r, "fs_stat (f == NULL)");
		return;
	}

	size = dostat(f->name, &stat);

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
	int path;

	debug ("fs_write\n");

	if (f == NULL) {
		respond (r, "fs_write (fid->aux == nil)");
		return;
	}

	path = fnametopath(f->name);

	if (r->ifcall.twrite.count == 0) {
		mtimes[path] = time(NULL);
		respond (r, NULL);
		return;
	}

	buf = malloc (r->ifcall.twrite.count + 1);
	memcpy(buf, r->ifcall.twrite.data, r->ifcall.twrite.count);
	buf[r->ifcall.twrite.count] = '\0';

	switch (path) {
		// yes this is REALLY what i mean
	case 0:
	case 1:
	case 2:
	case 5:
	case 6:
	case 7:
	case 8:
	default:
		respond(r, "not even implemented");
		break;
	case 3:
		r->ofcall.rwrite.count = r->ifcall.twrite.count;
		tilt = atoi (buf);

		if (freenect_sync_set_tilt_degs (tilt, 0))
			respond(r, "kinect disconnected");
		else {
			mtimes[path] = time(NULL);
			respond(r, NULL);
		}
		break;
	case 4:
		r->ofcall.rwrite.count = r->ifcall.twrite.count;
		led = atoi (buf);

		if (led < 0) led = 0;
		else if (led > 6) led = 6;

		if (freenect_sync_set_led (led, 0))
			respond(r, "kinect disconnected");
		else {
			mtimes[path] = time(NULL);
			respond(r, NULL);
		}
		break;
	}

	free (buf);
}

static void fs_clunk(Ixp9Req *r)
{
	FidAux *f = r->fid->aux;
	int path;

	debug ("fs_clunk fid:%d\n", r->fid->fid);

	if (f == NULL) {
		respond (r, "fs_clunk (f == NULL)");
		return;
	}

	path = fnametopath(f->name);

	switch (path) {
	case 5:
	case 6:
	case 7:
	case 8:
		if (audioclients == 0)
			freenect_stop_audio (f_dev);

		audioclients++;
	default:
		break;
	}

	respond (r, NULL);
}

static void fs_flush(Ixp9Req *r)
{
	debug ("fs_flush\n");

	respond(r, NULL);
}

static void fs_attach(Ixp9Req *r)
{
	r->fid->qid.type = P9_QTDIR;
	r->fid->qid.version = 0;
	r->fid->qid.path = 0;
	r->fid->aux = newfidaux("/");
	r->ofcall.rattach.qid = r->fid->qid;

	debug ("fs_attach fid:%u\n", r->fid->fid);

	respond (r, NULL);
}

static void fs_create(Ixp9Req *r)
{
	debug ("fs_create\n");

	respond (r, "not even implemented");
}

static void fs_remove(Ixp9Req *r)
{
	debug ("fs_remove\n");

	respond (r, "removals not implemented");
}

static void fs_freefid(IxpFid *f)
{
	debug ("fs_freefid");

	if (f->aux != NULL) {
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

void
freenect_do_one (long ms, void *aux)
{
	struct timeval tv = { 0, 0 };
	if (freenect_process_events_timeout ((freenect_context*)aux, &tv) < 0) {
		fprintf (stderr, "freenect_process_events\n");
		exit (-1);
	}
	ixp_settimer(&server, 1, freenect_do_one, aux);
}

int
main(int argc, char *argv[]) {
	int fd;
	IxpConn *acceptor;
	freenect_context *f_ctx;

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

	if (freenect_open_device (f_ctx, &f_dev, 0) < 0) {
		fprintf (stderr, "could not open kinect audio\n");
		freenect_shutdown (f_ctx);
		return -1;
	}

	freenect_sync_set_led (led, 0);

	fd = ixp_announce (argv[1]);
	if (fd < 0) {
		perror ("ixp_announce");
		return -1;
	}

	ixp_settimer(&server, 1, freenect_do_one, f_ctx);

	acceptor = ixp_listen(&server, fd, &p9srv, serve_9pcon, NULL);

	ixp_serverloop(&server);
	fprintf (stderr, "%s\n", ixp_errbuf());

	freenect_shutdown (f_ctx);

	return -1;
}

