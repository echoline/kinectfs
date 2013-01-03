CFLAGS=-g -I/usr/local/include/libfreenect

all: kinectfs depthtorgb xrgb

kinectfs: main.o fids.o
	gcc -o kinectfs main.o fids.o -lixp -lfreenect -lfreenect_sync

clean:
	rm -f kinectfs depthtorgb xrgb *.o *~

depthtorgb: depthtorgb.c
	gcc -o depthtorgb depthtorgb.c -lm -g

xrgb: xrgb.c
	gcc -o xrgb xrgb.c -lX11 -g
