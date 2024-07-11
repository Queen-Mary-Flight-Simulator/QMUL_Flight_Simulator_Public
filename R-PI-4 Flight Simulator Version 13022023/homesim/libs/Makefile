# Makefile to build simulator library files

CC=gcc
CFLAGS = -Wall -O2 -c

all: progs

progs: dted.o pnglib.o clocks.o glib.o maths.o target.o udplib.o weather.o \
soundlib.o navlib.o fileio.o shader.o inslib.o splinelib.o jslib.o

clean:
	rm *.o
