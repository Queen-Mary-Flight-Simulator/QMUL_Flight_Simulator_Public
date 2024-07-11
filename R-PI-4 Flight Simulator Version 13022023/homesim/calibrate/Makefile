#
# Makefile for I/O calibration
# Uses OpenGL4 version of glib
# DJA 13/12/21
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
    LIBS = -lwsock32 -lglfw3 -lopengl32 -lglew32 -lgdi32 -lpng -lfreetype -lfreeimage 
else
    LIBS = -lglfw -lGL -lGLEW -lpng -lfreetype -lfreeimage -lm
endif
OBJECTS = display.o iotest.o

COMMON_OBJECTS = glib.o maths.o shader.o pnglib.o jslib.o
COMMON_OBJECTS_DIR = ../libs

#
# Targets
#

### iotest program

iotest: $(OBJECTS) $(addprefix $(COMMON_OBJECTS_DIR)/, $(COMMON_OBJECTS))
	$(CC) -o $@ $(OBJECTS) $(addprefix $(COMMON_OBJECTS_DIR)/, $(COMMON_OBJECTS)) $(LIBS)


### SUFFIX rule statement

%.o : %.c
	$(CC) $(GCC_OPTIONS) -c $<


### Clean up

clean:
ifeq ($(OS),Windows_NT)
	rm *.o iotest.exe
else
	rm *.o iotest
endif
