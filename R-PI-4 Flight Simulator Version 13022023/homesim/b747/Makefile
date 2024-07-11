#
# Flight Simulator Makefile for PFD
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

OBJECTS = aero.o aerolink.o ai.o alt.o asi.o compass.o b747.o eicas.o \
fcs.o gear.o iolib.o model.o pfd.o systems.o vsi.o \
engines.o englink.o sounds.o \
fcu.o nav.o navinfo.o navlink.o nfd.o nfd-compass.o panel.o panellib.o radio.o fma.o\
approach.o dataview.o diagnostics.o gui.o ios.o ioslink.o map.o plot.o scan.o script.o

COMMON_OBJECTS = clocks.o dted.o glib.o maths.o navlib.o pnglib.o jslib.o\
soundlib.o target.o udplib.o weather.o fileio.o shader.o inslib.o splinelib.o

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

