#
# Flight Simulator Makefile for B747 NFD

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
    LIBS = -lwsock32 -lglfw3 -lopengl32 -lglew32 -lgdi32 -lpng -lfreetype -lfreeimage
else
    LIBS = -lm -lglfw -lGL -lGLEW -lpng -lfreetype -lfreeimage
endif

OBJECTS = nfd-compass.o b747.o fcu.o nav.o navinfo.o iolib.o \
navlink.o nfd.o panel.o panellib.o radio.o systems.o diagnostics.o

COMMON_OBJECTS = clocks.o glib.o pnglib.o maths.o \
udplib.o navlib.o fileio.o shader.o

COMMON_OBJECTS_DIR = ../libs

#
# Targets
#

### B747 program

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
