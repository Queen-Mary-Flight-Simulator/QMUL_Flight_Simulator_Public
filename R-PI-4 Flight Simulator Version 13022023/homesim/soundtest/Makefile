#
# Makefile for Soundtest
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
#GL_LIBS = -lGLU -lGL -lglut

LIBS = -lalut -lopenal -lm -lpthread

OBJECTS = soundtest.o

COMMON_OBJECTS = soundlib.o maths.o

COMMON_OBJECTS_DIR = ../libs

#
# Targets
#

### soundtest program

soundtest: $(OBJECTS) $(addprefix $(COMMON_OBJECTS_DIR)/, $(COMMON_OBJECTS))
	$(CC) -o $@ $(OBJECTS) $(addprefix $(COMMON_OBJECTS_DIR)/, $(COMMON_OBJECTS)) $(LIBS)


### SUFFIX rule statement

%.o : %.c
	$(CC) $(GCC_OPTIONS) -c $<


### Clean up

clean:
	rm *.o soundtest

