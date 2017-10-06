CFLAGS=-g

all: kinectfs

clean:
	rm -f kinectfs depthtorgb rgbtohsv xrgb gtkrgbanim *.o *~

kinectfs: kinectfs.c
	gcc -g -o kinectfs kinectfs.c -lixp -lfreenect -lfreenect_sync -lm -DUSE_JPEG -ljpeg --std=c99
#	@echo SUDO: for port 564
#	sudo setcap 'cap_net_bind_service=+ep' kinectfs

depthtorgb: depthtorgb.c
	gcc -o depthtorgb depthtorgb.c -lm -g

xrgb: xrgb.c
	gcc -o xrgb xrgb.c -lX11 -g

rgbtohsv: rgbtohsv.c

gtkrgbanim: gtkrgbanim.c
	gcc -o gtkrgbanim gtkrgbanim.c `pkg-config --cflags --libs gtk+-2.0`
