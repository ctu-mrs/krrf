
CXX=g++  


CXXINCLUDE=  -I../include -I./include/DNN -I/usr/local/include
LDFLAGS= -L/usr/local/lib -lpthread -lm

CXXINCLUDE+=-O3
#CXXINCLUDE+=-ggdb3

#CXXINCLUDE+=-fprofile-generate
#LDFLAGS+=-fprofile-generate

#CXXINCLUDE+=-fprofile-use
#LDFLAGS+=-fprofile-use


#LIBDIR=../lib
LIBDIR=.
LIBNAME=libDNN.a

SRCALL=$(wildcard *.cc)
OBJSALL=$(patsubst %.cc,%.o,$(SRCALL))


SRC=$(filter-out testmpnn.cc, $(SRCALL))
OBJS=$(patsubst %.cc,%.o,$(SRC))

all: $(OBJS) test
	ar cru libDNN.a $(OBJS)

.cc.o:
	$(CXX) $(CXXINCLUDE) -c $<

test: $(OBJSALL)
	$(CXX) $(CXXINCLUDE) $(LDFLAGS) $(OBJSALL) -o testmpnn


clean:
	rm -f $(OBJS) testmpnn testmpnn.o
#rm -f $(LIBDIR)/$(LIBNAME)


