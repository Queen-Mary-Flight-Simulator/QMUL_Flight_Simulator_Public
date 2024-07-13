#
# Makefile for OpenGL Flight Simulator Instructor Station
#
# Use 'make' to build
# Use 'make clean' to delete .o and executable files
#

.SUFFIXES:
.SUFFIXES: .c .o

CC = gcc
GCC_OPTIONS= -O2 -Wall

ifeq ($(OS),Windows_NT)
    LIBS = -lwsock32 -lglfw3 -lopengl32 -lglew32 -lgdi32 -lpng -lfreetype -lfreeimage
else
    LIBS = -lm -lglfw -lGL -lGLEW -lpng -lfreetype -lfreeimage
endif

OBJECTS = ios.o map.o ioslink.o approach.o plot.o iolib.o\
script.o gui.o scan.o dataview.o diagnostics.o

COMMON_OBJECTS = maths.o navlib.o pnglib.o glib.o udplib.o fileio.o shader.o
COMMON_OBJECTS_DIR = ../libs

ios: $(OBJECTS) $(addprefix $(COMMON_OBJECTS_DIR)/, $(COMMON_OBJECTS))
	$(CC) -o $@ $(OBJECTS) $(addprefix $(COMMON_OBJECTS_DIR)/, $(COMMON_OBJECTS)) $(LIBS)

%.o : %.c
	$(CC) $(GCC_OPTIONS) -c $<

clean:
ifeq ($(OS),Windows_NT)
	rm *.o ios.exe
else
	rm *.o ios
endif
