
CXX=g++  
CXXINCLUDE+= -O2 -I../include -I./include/DNN -I/usr/local/include
#CXXINCLUDE+= -ggdb3 -I../include -I./include/DNN -I/usr/local/include
LDFLAGS+=-L/usr/local/lib -lpthread -lm
LIBDIR=../lib
LIBNAME=libDNN.a

SRC=$(wildcard *.cc)
OBJS=$(patsubst %.cc,%.o,$(wildcard *.cc))

all: $(OBJS)
	ar cru libDNN.a $(OBJS)
	mv $(LIBNAME) $(LIBDIR)

.cc.o:
	$(CXX) $(CXXINCLUDE) -c $<


clean:
	rm -f $(OBJS)
	rm -f $(LIBDIR)/$(LIBNAME)


