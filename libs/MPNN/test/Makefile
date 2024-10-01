
CXX=g++
CXXINCLUDE+=-I/usr/local/include -I../include -I../include/DNN
LDFLAGS+=-L/usr/local/lib -lpthread -lm -L../lib -lDNN
TARGET=testDNN

SRC=$(wildcard *.cc)
OBJS=$(patsubst %.cc,%.o,$(wildcard *.cc))

all: $(OBJS)
	$(CXX) $(OBJS) $(LDFLAGS) -o $(TARGET)


.cc.o:
	$(CXX) $(CXXINCLUDE) -c $<


clean:
	rm -f $(OBJS) $(TARGET);


