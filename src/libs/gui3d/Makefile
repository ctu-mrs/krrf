
CXX=ccache g++
CXXINCLUDE+=-I/usr/local/include
LDFLAGS+=-L/usr/local/lib -lpthread -lm -lglut -lGL -lGLU -lm -lX11 -lXmu 
TARGET=testgui3d
LIBTARGET=libgui3d.a

LDFLAGS+=-L../gui -lgui

SRC=$(wildcard *.cc)
OBJS=$(patsubst %.cc,%.o,$(wildcard *.cc))
COBJS=$(patsubst %.cc,%.o,$(wildcard CPainter3D*.cc))

CXXDEFINE += -O2
#CXXDEFINE += -ggdb3

all: $(OBJS)
	$(CXX) $(OBJS) $(LDFLAGS) -o $(TARGET)
	ar cru $(LIBTARGET) $(COBJS)


.cc.o:
	$(CXX) $(CXXDEFINE) $(CXXINCLUDE) -c $<


clean:
	rm -f $(OBJS) $(TARGET) $(LIBTARGET);


