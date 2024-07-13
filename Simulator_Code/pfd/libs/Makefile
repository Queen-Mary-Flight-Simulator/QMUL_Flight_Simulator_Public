# Makefile to build simulator library files

CC=gcc
CFLAGS = -Wall -O2 -c

all: progs

progs: dted.o pnglib.o clocks.o glib.o maths.o target.o udplib.o weather.o \
navlib.o fileio.o shader.o inslib.o splinelib.o

clean:
	rm *.o
