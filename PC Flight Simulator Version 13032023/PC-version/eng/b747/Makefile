#
# Flight Simulator Makefile for ENG
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

ifeq ($(OS),Windows_NT)
    LIBS = -lwsock32  -lalut -lopenal
else
    LIBS = -lm  -lalut -lopenal
endif

OBJECTS = englink.o b747.o iolib.o engines.o sounds.o systems.o

COMMON_OBJECTS = clocks.o maths.o udplib.o soundlib.o splinelib.o
COMMON_OBJECTS_DIR = ../libs

#
# Targets
#

### b747 program

b747: $(OBJECTS) $(addprefix $(COMMON_OBJECTS_DIR)/, $(COMMON_OBJECTS))
	$(CC) -o $@ $(OBJECTS) $(addprefix $(COMMON_OBJECTS_DIR)/, $(COMMON_OBJECTS)) $(LIBS)

### SUFFIX rule statement

%.o : %.c
	$(CC) $(GCC_OPTIONS) -c $<


### Clean up

clean:
ifeq ($(OS),Windows_NT)
	rm *.o b747.exe
else
	rm *.o b747
endif

