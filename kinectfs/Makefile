CFLAGS=-g -I/usr/local/include/libfreenect

kinectfs: main.o
	gcc -o kinectfs main.o -lixp -lfreenect -lfreenect_sync

clean:
	rm -f kinectfs depthtoppm *.o *~

depthtoppm: depthtoppm.c
	gcc -o depthtoppm depthtoppm.c -lm -g

