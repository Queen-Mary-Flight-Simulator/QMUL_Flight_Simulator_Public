#
# Makefile for off-line data viewer
#
# Type 'make dataview' to build
# Type 'make clean' to delete executable and object files
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

OBJECTS = dataview.o link.o plot.o gui.o scan.o


COMMON_OBJECTS = maths.o pnglib.o shader.o glib.o fileio.o
COMMON_OBJECTS_DIR = ../libs

dataview: $(OBJECTS) $(addprefix $(COMMON_OBJECTS_DIR)/, $(COMMON_OBJECTS))
	$(CC) -o $@ $(OBJECTS) $(addprefix $(COMMON_OBJECTS_DIR)/, $(COMMON_OBJECTS)) $(LIBS)

%.o : %.c
	$(CC) $(GCC_OPTIONS) -c $<

clean:
ifeq ($(OS),Windows_NT)
	rm *.o dataview.exe
else
	rm *.o dataview
endif
