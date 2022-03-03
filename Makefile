CFLAGS=-g

all: kinectfs xjpg mjpeg

clean:
	rm -f kinectfs depthtorgb rgbtohsv xrgb xjpg mjpeg gtkrgbanim *.o *~

kinectfs: kinectfs.c
	gcc -g -o kinectfs kinectfs.c -I/usr/local/include/libfreenect -L/usr/local/lib -lixp -lixp_pthread -lfreenect -lm -DUSE_JPEG -ljpeg #-DUSE_AUDIO -DUSE_OGG -logg -lvorbis -lvorbisenc -DUSE_PLAN9
#	@echo SUDO: for port 564
#	sudo setcap 'cap_net_bind_service=+ep' kinectfs

depthtorgb: depthtorgb.c
	gcc -o depthtorgb depthtorgb.c -lm -g

xrgb: xrgb.c
	gcc -o xrgb xrgb.c -lX11 -g

xjpg: xjpg.c
	gcc -o xjpg xjpg.c `pkg-config --cflags --libs x11` `pkg-config --cflags --libs libjpeg` -g

rgbtohsv: rgbtohsv.c

gtkrgbanim: gtkrgbanim.c
	gcc -o gtkrgbanim gtkrgbanim.c `pkg-config --cflags --libs gtk+-2.0`
