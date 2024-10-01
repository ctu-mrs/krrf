CXX? = g++
CFLAGS = -Wall -Wconversion -O2 -fPIC
SHVER = 1

all: svm.o
#	$(CXX) svm.o -o libsvm.so.$(SHVER)
	ar cru liblocalsvm.a svm.o

clean:
	rm -f *~ svm.o libsvm.so.1 liblocalsvm.a
