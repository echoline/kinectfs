CFLAGS=-g

all: kinectfs depthtorgb xrgb rgbtohsv

kinectfs: kinectfs.c
	gcc -o kinectfs kinectfs.c -I/usr/local/include/libfreenect/ -I/usr/local/include/libixp/ -lixp -lfreenect -lfreenect_sync

clean:
	rm -f kinectfs depthtorgb rgbtohsv xrgb *.o *~

depthtorgb: depthtorgb.c
	gcc -o depthtorgb depthtorgb.c -lm -g

xrgb: xrgb.c
	gcc -o xrgb xrgb.c -lX11 -g

rgbtohsv: rgbtohsv.c
