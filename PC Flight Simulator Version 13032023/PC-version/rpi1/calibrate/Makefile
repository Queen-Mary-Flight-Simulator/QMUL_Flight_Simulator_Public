#
# Flight Simulator Makefile for I/O system
#

# Clear the list of suffixes
.SUFFIXES:
# Default compilation Suffix rules for .c .o
.SUFFIXES: .c .o

#
# Macros
#

CC = gcc
GCC_OPTIONS = -O2 -Wall

OBJECTS = iotest.o iolink.o

COMMON_OBJECTS = iolib.o udplib.o
COMMON_OBJECTS_DIR = ../libs

#
# Targets
#

### iotest program

iotest: $(OBJECTS) $(addprefix $(COMMON_OBJECTS_DIR)/, $(COMMON_OBJECTS))
	$(CC) -o $@ $(OBJECTS) $(addprefix $(COMMON_OBJECTS_DIR)/, $(COMMON_OBJECTS))


### SUFFIX rule statement

%.o : %.c
	$(CC) $(GCC_OPTIONS) -c $<


### Clean up

clean:
	rm *.o iotest

