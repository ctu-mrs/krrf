
CXX=g++
TARGET=testGui
LIBTARGET=libgui.a


LDFLAGS=-L/storage/praha1/home/vonasek/opt/lib #-lz 
LDFLAGS+=`pkg-config --libs cairo`
CXXFLAGS=`pkg-config --cflags cairo`

CXXINCLUDE+=`sdl-config --cflags`
LDFLAGS+=`sdl-config --libs` -lSDL_gfx -lSDL_image

#CXXDEFINE+=-ggdb3
CXXDEFINE+=-O2
#CXXDEFINE+=-Wall


LDFLAGS+=-lpthread -lm #-lz

SRC=$(wildcard *.cc)
OBJ=$(patsubst %.cc,%.o,$(SRC))
COBJ=$(patsubst %.cc,%.o,$(wildcard CPainter*.cc))
COBJ+=gnuplot.o

all: $(OBJ)
	$(CXX) $(OBJ) -o $(TARGET) $(LDFLAGS) 
	ar cru $(LIBTARGET) $(COBJ)

.cc.o:
	$(CXX)  $(CXXFLAGS) $(CXXDEFINE) -c  $(CXXINCLUDE) $< 


clean:
	$(RM) $(OBJ) $(TARGET) $(TARGET).core $(LIBTARGET)
