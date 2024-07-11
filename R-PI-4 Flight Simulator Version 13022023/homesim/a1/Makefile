#
# Flight Simulator Makefile for A1
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
    LIBS = -lwsock32 -lglfw3 -lopengl32 -lglew32 -lgdi32 -lpng -lfreetype -lfreeimage -lalut -lopenal
else
    LIBS = -lm -lglfw -lGL -lGLEW -lpng -lfreetype -lfreeimage -lalut -lopenal
endif

OBJECTS = aero.o aerolink.o ai.o asi.o compass.o a1.o \
fcs.o iolib.o model.o pfd.o systems.o \
engines.o englink.o sounds.o \
fcu.o nav.o navlink.o nfd.o panel.o \
approach.o dataview.o diagnostics.o gui.o ios.o ioslink.o map.o plot.o scan.o script.o\
turnslip.o egt.o clock.o enginegauges_left.o enginegauges_right.o blank.o \
alt.o knobs.o vsi.o rpm.o mp.o magcompass.o aoa.o gmeter.o radio.o

COMMON_OBJECTS = clocks.o dted.o glib.o maths.o navlib.o pnglib.o \
soundlib.o target.o udplib.o weather.o fileio.o shader.o jslib.o

COMMON_OBJECTS_DIR = ../libs

#
# Targets
#

### A1 program

a1: $(OBJECTS) $(addprefix $(COMMON_OBJECTS_DIR)/, $(COMMON_OBJECTS))
	$(CC) -o $@ $(OBJECTS) $(addprefix $(COMMON_OBJECTS_DIR)/, $(COMMON_OBJECTS)) $(LIBS)


### SUFFIX rule statement

%.o : %.c
	$(CC) $(GCC_OPTIONS) -c $<


### Clean up

clean:
ifeq ($(OS),Windows_NT)
	rm *.o a1.exe
else
	rm *.o a1
endif

