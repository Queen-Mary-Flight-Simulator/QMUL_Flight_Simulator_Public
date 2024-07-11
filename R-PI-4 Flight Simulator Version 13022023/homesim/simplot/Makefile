#
# Makefile for simplot
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

LIBS = -lm

OBJECTS = aero.o aerolink.o englink.o engines.o fcs.o stab.o iolib.o\
model.o gear.o simplot.o simulate.o systems.o

COMMON_OBJECTS = clocks.o dted.o maths.o \
weather.o fileio.o inslib.o splinelib.o

COMMON_OBJECTS_DIR = ../libs/

#
# Targets
#

### simplot program

simplot: $(OBJECTS) $(addprefix $(COMMON_OBJECTS_DIR)/, $(COMMON_OBJECTS))
	$(CC) -o $@ $(OBJECTS) $(addprefix $(COMMON_OBJECTS_DIR)/, $(COMMON_OBJECTS)) $(LIBS)

### SUFFIX rule statement

%.o : %.c
	$(CC) $(GCC_OPTIONS) -c $<

### Clean up

clean:
	rm *.o simplot

