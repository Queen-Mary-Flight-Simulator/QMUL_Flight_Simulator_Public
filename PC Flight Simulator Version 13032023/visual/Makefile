# Basic makefile for building CGI visualisation software for flight simulator
# 10th May 2011, GTS

.SUFFIXES:
.SUFFIXES: .cpp .c .o

EXEC = cgi

CXX = g++

CC = gcc

CXXFLAGS = -O2 -Wall

OSG_LIBS = -losgViewer -losgText -losgGA -losgDB -losgUtil -losg
OSG_INCDIR += /c/Program\ Files\ \(x86\)/OpenSceneGraph/include

LIBS += -lopengl32 -lglu32 $(OSG_LIBS) -lwsock32
INC += $(OSG_INCDIR)

OBJ = $(EXEC).o hud.o ai.o asi.o alt.o compass.o glib.o maths.o

-include $(OBJ:.o=.d)

vars:
	@echo $(OS)
	@echo $(shell uname)
	@echo $(CPPFLAGS)
	@echo $(LIBS)
	@echo $(INC)


$(EXEC): $(OBJ)
	@rm -f $(EXEC).exe
	$(CXX) -o $(EXEC) $(OBJ) $(CXXFLAGS) -I$(INC) -L/usr/mingw/lib -L/c/Program\ Files\ \(x86\)/OpenSceneGraph/lib $(LIBS)

%.o: %.c
	$(CXX) -c $< -I$(INC) $(CXXFLAGS)
	$(CXX) -MM $*.c > $*.d

%.o: %.cpp
	$(CXX) -c $< -I$(INC) $(CXXFLAGS)
	$(CXX) -MM $*.cpp > $*.d

clean:
	rm *.o *.d $(EXEC).exe
