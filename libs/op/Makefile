
CXX=g++ 
CXXINCLUDE= -O2 -I/usr/local/include -std=c++11
LDFLAGS=-L/usr/local/lib -lpthread -lm
TARGET=testOPloader
LIBTARGET=libOPLoader.a

SRC=$(wildcard *.cpp)
OBJS=$(patsubst %.cpp,%.o,$(wildcard *.cpp))

all: $(OBJS)
#$(CXX) $(OBJS) $(LDFLAGS) -o $(TARGET)
	ar cru $(LIBTARGET) dataset_loader_obst.o


.cc.o:
	$(CXX) $(CXXINCLUDE) -c $<

.cpp.o:
	$(CXX) $(CXXINCLUDE) -c $<


clean:
	rm -f $(OBJS) $(TARGET) $(LIBTARGET)


