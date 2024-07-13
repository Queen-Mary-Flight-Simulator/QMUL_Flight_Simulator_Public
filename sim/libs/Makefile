# Makefile to build simulator library files

CC=gcc
CFLAGS = -Wall -O2 -c -g

all: progs

progs: dted.o pnglib.o clocks.o glib.o maths.o target.o texture.o udplib.o weather.o \
soundlib.o navlib.o glibx.o fileio.o

	$(CC) $(CFLAGS) dted.c
	$(CC) $(CFLAGS) pnglib.c
	$(CC) $(CFLAGS) clocks.c
	$(CC) $(CFLAGS) glib.c
	$(CC) $(CFLAGS) glibx.c
	$(CC) $(CFLAGS) maths.c
	$(CC) $(CFLAGS) target.c
	$(CC) $(CFLAGS) texture.c
	$(CC) $(CFLAGS) udplib.c
	$(CC) $(CFLAGS) weather.c
	$(CC) $(CFLAGS) soundlib.c
	$(CC) $(CFLAGS) navlib.c
	$(CC) $(CFLAGS) fileio.c

clean:
	rm *.o
