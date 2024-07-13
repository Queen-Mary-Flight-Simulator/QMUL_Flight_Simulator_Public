# Makefile to build simulator library files

CC=gcc
CFLAGS = -Wall -O3 -c

all: progs

progs: iolib.o udplib.o
	$(CC) $(CFLAGS) iolib.c
	$(CC) $(CFLAGS) udplib.c

clean:
	rm *.o
