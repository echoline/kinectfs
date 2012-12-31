#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <ixp.h>
#include <libfreenect_sync.h>
#include <png.h>

int tilt;
int led;

#define debug(...) fprintf (stderr, __VA_ARGS__);
//#define debug(...) {}; 

typedef struct {
	struct timeval last;
	void* image;
} FrnctImg;

static int fnametopath(char *name);
typedef struct _FidAux {
	char		*name;
	uint8_t		type;
	uint32_t 	version;
	uint64_t	path;
	int		offset;
} FidAux;
char *paths[] = { "/", "rgb", "depth", "tilt", "led", "audio", NULL };
long mtimes[] = { 0, 0, 0, 0, 0, 0 };
long atimes[] = { 0, 0, 0, 0, 0, 0 };
static FidAux* newfidaux(char *name, IxpQid *qid) {
	FidAux *ret;

	ret = calloc (1, sizeof(FidAux));
	ret->type = qid->type;
	ret->version = qid->version;
	ret->path = fnametopath(name);
	ret->name = paths[ret->path];

	return ret;
}
static int fnametopath(char *name) {
	int i;
	char *p = name;
	char *q;

	while (*p == '/')
		p++;

	for (i = 0; paths[i] != NULL; i++) {
		q = paths[i];
		while (*q == '/')
			q++;

		if (strcasecmp(p, q) == 0)
			return (i);
	}

	return -1;
}

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
	stat->qid.type = path? P9_QTFILE: P9_QTDIR;
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
	debug ("fs_open\n");

	respond (r, NULL);
}

static void fs_walk(Ixp9Req *r)
{
	char *name = malloc (PATH_MAX);
	int path;

	if (r->ifcall.twalk.nwname == 0) {
		debug ("fs_walk (zero-length)\n");

		path = 0;
	} else if (r->ifcall.twalk.nwname != 1) {
		respond (r, "no such file");
		return;
	} else {
		path = fnametopath(r->ifcall.twalk.wname[0]);
		if (path < 0) {
			if (!strcmp(r->ifcall.twalk.wname[0], ".")) {
				path = r->fid->qid.path;
			} else {
				respond (r, "no such file");
				return;
			}
		}
	}

	r->ofcall.rwalk.wqid[0].version = r->fid->qid.version;
	r->ofcall.rwalk.wqid[0].path = path;
	if (path == 0)
		r->ofcall.rwalk.wqid[0].type = P9_QTDIR;
	else
		r->ofcall.rwalk.wqid[0].type = P9_QTFILE;

	debug ("fs_walk path:%d name:%s\n", path, paths[path]);

	r->newfid->aux = newfidaux(paths[path], &r->ofcall.rwalk.wqid[0]);
	r->ofcall.rwalk.nwqid = 1;

	respond(r, NULL);
}

static void fs_read(Ixp9Req *r)
{
	FidAux *f = r->fid->aux;
	IxpStat stat;
	IxpMsg m;
	char *buf;
	int n, size;

	freenect_raw_tilt_state *state;
	static double dx, dy, dz;
	static struct timeval lasttilt = { 0, 0 };
	static FrnctImg imgs[2];
	int i;
	uint32_t ts;

	debug ("fs_read offset:%d\n", r->ifcall.tread.offset);
	if (f == NULL) {
		respond(r, "fs_read (fid->aux == NULL)\n");
		return;
	}

	if (r->ifcall.tread.offset == 0)
		f->offset = 0;

	switch (f->path) {
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
#define RGBSIZE 640 * 480 * 3
		size = r->ifcall.tread.count;
		buf = malloc (size);
		i = f->path - 1;

		if (istime(&(imgs[i].last), 1)) {
			if (imgs[i].image)
				free (imgs[i].image);
			imgs[i].image = malloc (RGBSIZE);
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
		r->ofcall.rread.count = RGBSIZE - r->ifcall.tread.offset;
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
	case 2:
	case 5:
		respond(r, "unimplemented");
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
			atimes[f->path] = time(NULL);
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
			atimes[f->path] = time(NULL);
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

	debug ("fs_stat name:%s path:%d\n", f->name, f->path);
}

static void fs_write(Ixp9Req *r)
{
	debug ("fs_write\n");

	FidAux *f = r->fid->aux;
	char *buf;

	if (r->ifcall.twrite.count == 0) {
		mtimes[f->path] = time(NULL);
		respond (r, NULL);
		return;
	}

	buf = malloc (r->ifcall.twrite.count + 1);
	memcpy(buf, r->ifcall.twrite.data, r->ifcall.twrite.count);
	buf[r->ifcall.twrite.count] = '\0';

	switch (f->path) {
		// yes this is REALLY what i mean
	case 0:
	case 1:
	case 2:
	case 5:
	default:
		respond(r, "not even implemented");
		break;
	case 3:
		r->ofcall.rwrite.count = r->ifcall.twrite.count;
		tilt = atoi (buf);

		if (freenect_sync_set_tilt_degs (tilt, 0))
			respond(r, "kinect disconnected");
		else {
			mtimes[f->path] = time(NULL);
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
			mtimes[f->path] = time(NULL);
			respond(r, NULL);
		}
		break;
	}

	free (buf);
}

static void fs_clunk(Ixp9Req *r)
{
	debug ("fs_clunk\n");

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
	r->fid->aux = newfidaux("", &r->fid->qid);
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
	if (f->aux != NULL) {
		free (f->aux);
		f->aux = NULL;
	}

	debug ("fs_freefid\n");
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

int
main(int argc, char *argv[]) {
	int fd;
	IxpConn *acceptor;

	if (argc != 2) {
		fprintf (stderr, "usage:\n\t%s proto!addr[!port]\n", argv[0]);
		return -1;
	}

	fd = ixp_announce (argv[1]);
	if (fd < 0) {
		perror ("ixp_announce");
		return -1;
	}

	acceptor = ixp_listen(&server, fd, &p9srv, serve_9pcon, NULL);

	ixp_serverloop(&server);
	fprintf (stderr, "%s\n", ixp_errbuf());

	return -1;
}

