
#CC = g++  -O2
CC = g++  -O3 -funroll-loops
CFLAGS	= -I. -I..
LDFLAGS	= -L. -L..

SRCS=$(wildcard *.cc)
OBJS=$(patsubst %.cc,%.o,$(SRCS))

all: library 

library: libRAPID.a

libRAPID.a: $(OBJS)
	ar ruv libRAPID.a $(OBJS)
#	$(CC) -o libRAPID $(OBJS)

.cc.o:
	$(CC) ${CFLAGS} -c $<

.c.o:
	$(CC) ${CFLAGS} -c $<

clean:
	rm -f $(OBJS) libRAPID.a
