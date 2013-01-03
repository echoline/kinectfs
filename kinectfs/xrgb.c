#include <X11/Xlib.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
 
int main(void) {
   Display *d;
   Window w;
   XEvent e;
   int s, x, y, o;
   GC gc;
   XImage *i;
   char buf[640 * 480 * 4];
   Pixmap p;

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

   for (y = 0; y < 480; y++) for (x = 0; x < 640; x++) {
      o = 640 * 4 * y + 4 * x;

      buf[o] = 0;
      for (s = 3; s > 0; s--)
         buf[o + s] = getc(stdin);
   }

   i = XCreateImage(d, DefaultVisual(d, s), 24, ZPixmap, 0, buf, 640, 480,
                    32, 640 * 4);

   while (1) {
      XNextEvent(d, &e);

      if (e.type == Expose)
         XPutImage (d, w, gc, i, 0, 0, 0, 0, 640, 480);

      if (e.type == KeyPress)
         break;
   }
 
   XCloseDisplay(d);
   return 0;
}
