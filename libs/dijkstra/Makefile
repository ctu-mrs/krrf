
CXX=g++ 
CXXINCLUDE= -O2 -I/usr/local/include
LDFLAGS=-L/usr/local/lib -lpthread -lm
TARGET=testDijkstra
LIBTARGET=libdijkstra.a

SRC=$(wildcard *.cc)
OBJS=$(patsubst %.cc,%.o,$(wildcard *.cc))

all: $(OBJS)
	$(CXX) $(OBJS) $(LDFLAGS) -o $(TARGET)
	ar cru $(LIBTARGET) dijkstra.o


.cc.o:
	$(CXX) $(CXXINCLUDE) -c $<


clean:
	rm -f $(OBJS) $(TARGET) $(LIBTARGET)


