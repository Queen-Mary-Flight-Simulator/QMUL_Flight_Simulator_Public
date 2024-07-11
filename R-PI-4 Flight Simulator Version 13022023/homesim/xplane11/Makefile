#
# XPlane Plugin
#

# Clear the list of suffixes
.SUFFIXES:
# Default compilation Suffix rules for .c .o
.SUFFIXES: .c .o

#
# Macros
#

COMPILER = c:/msys64/mingw64/bin/gcc
INCDIR = XPSDK302/SDK/CHeaders/XPLM
LIBDIR = XPSDK302/SDK/Libraries/Win
#GCC_OPTIONS = -m64 -O2 -Wall -Wshadow -Wfloat-equal -pedantic -fvisibility=hidden -fmessage-length=0 -DXPLM200 -DXPLM210 -DAPL=0 -DLIN=0 -DIBM=1 -I$(INCDIR)
#GCC_OPTIONS = -m64 -O2 -Wall -fvisibility=hidden -DXPLM200 -DXPLM210 -DAPL=0 -DLIN=0 -DIBM=1 -I$(INCDIR)
GCC_OPTIONS = -m64 -shared -O2 -Wall -DXPLM200 -DXPLM210 -DAPL=0 -DLIN=0 -DIBM=1 -I$(INCDIR)
#LINK_OPTIONS = -m64 -static-libgcc -shared -fvisibility=hidden -Wl,--version-script=exports.txt
#LINK_OPTIONS = -m64 -O1 -shared -L$(LIBDIR) -lXPLM_64
LINK_OPTIONS = -m64 -static-libgcc -shared
#LIBS = -lwsock32
#LIBS = XPLM_64.dll


### TARGETS
    
hello: hello.o
	$(COMPILER) $(LINK_OPTIONS) hello.o -o win.xpl XPLM_64.lib -lopengl32 
    
terrain: terrain.o
	$(COMPILER) $(LINK_OPTIONS) terrain.o -o win.xpl XPLM_64.lib 
    
Camera: Camera.o
	$(COMPILER) $(LINK_OPTIONS) Camera.o -o win.xpl XPLM_64.lib -lopengl32 
    
cgi: cgi.o
	$(COMPILER) $(LINK_OPTIONS) cgi.o -o win.xpl udplib.o XPLM_64.lib -lwsock32


### SUFFIX rule statement

%.o : %.c
	$(COMPILER) $(GCC_OPTIONS) -c $<


### Clean up

clean:
	rm *.o *.xpl

