#include <gtk/gtk.h>
#include <stdlib.h>
#include <errno.h>

#define WIDTH 640
#define HEIGHT 480
#define STRIDE WIDTH * 4
#define PIXSIZE 640 * 480
#define BUFSIZE PIXSIZE * 4

gboolean
read_rgb (gpointer data) {
	GtkWidget *window = data;
	GdkPixbuf *pixbuf;
	GtkWidget *image;
	static guchar *buf = NULL;
	static gint i = 0;
	gint r, r2;

       	buf = malloc (640 * 480 * 4);

	while (i < PIXSIZE) {
		r = fread(&buf[i << 2], 3, 1, stdin);
		if (r < 0) {
			if (errno == EAGAIN)
				continue;
			else {
				free (buf);
				perror ("rgb read");
				return FALSE;
			}

			i += r;
		}

		if (feof(stdin))
			break;
	}
	i = 0;

	image = gtk_bin_get_child (GTK_BIN (window));
	if (image != NULL)
		gtk_container_remove (GTK_CONTAINER (window), image);

	pixbuf = gdk_pixbuf_new_from_data (buf, GDK_COLORSPACE_RGB,
			FALSE, 8, 640, 480, 640 * 4, NULL, NULL);
	image = gtk_image_new_from_pixbuf (pixbuf);

	gtk_container_add (GTK_CONTAINER (window), image);
	gtk_widget_show_all (window);

	return TRUE;
}

int
main (int argc, char **argv) {
	GtkWidget *window;

	gtk_init (&argc, &argv);

	window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
	g_signal_connect (window, "destroy", gtk_main_quit, NULL);
	gtk_widget_set_size_request (window, 640, 480);

	gtk_timeout_add (333, read_rgb, window);

	gtk_widget_show_all (window);
	gtk_main ();
}
