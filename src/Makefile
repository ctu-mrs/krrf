 
$(shell echo "#define GIT_VERSION \""`git rev-parse HEAD`\" > gitv.h)  

CXX=g++
CXXINCLUDE+= -std=c++11 #-I/usr/local/include

CXXINCLUDE+=  -I./libs -I./libs/mpnn2 -Ilibs/libsvm -I./libs/flann
LDFLAGS+= -llz4 -Llibs/gui -lgui  -L./libs/rapid -lRAPID -L./libs/mpnn2 -lDNN -Llibs/libsvm 
LDFLAGS+= -lflann -lflann_cpp 

OPSYS=$(shell uname)
ifeq ($(OPSYS),Linux) 
LDFLAGS+=-lrt
endif

CXXINCLUDE+=`sdl-config --cflags`
LDFLAGS+=`sdl-config --libs` -lSDL_gfx -lSDL_image

CXXINCLUDE+=`pkg-config cairo --cflags`
LDFLAGS+=`pkg-config cairo --libs`

#for profiller -pg for BOTH LDFLAGS and CXXINCLUDE 

CXXINCLUDE+=`gsl-config --cflags`  
LDFLAGS+=`gsl-config --libs`  

#LDFLAGS+=-pg
#CXXINCLUDE+=-pg

CXXINCLUDE+=-fdiagnostics-color=always

CXXDEFINE+=-O3
#CXXDEFINE+=-ggdb3

ROBOT_OBJS=CRobot2D.o CRobotCarLike.o CRobotDiff.o CRobotBike.o CRobot3D.o
 

BASIC_OBJS= CParseArgs.o 
BASIC_OBJS+=dijkstra.o mapr.o map3r.o map.o painting.o ut.o Voronoi.o WLog.o blender.o types.o polygonUtils.o mesh.o timerN.o
BASIC_OBJS+=libs/triangle/triangle.o
BASIC_OBJS+=options2D.o options3D.o rrt2D.o 
BASIC_OBJS+=staticObstacles2D.o random.o


ODEROBOTS_OBJS=$(patsubst %.cc, %.o, $(wildcard CRobotODE*.cc))
#ODE_OBJS+=paintingODE.o typesODE.o $(ODEROBOTS_OBJS)


DIS_OBJS=$(ROBOT_OBJS) $(BASIC_OBJS) 
CONTROL_OBJS=$(ROBOT_OBJS) $(BASIC_OBJS) tcontrol.o

DIS_OBJS_P=$(ROBOT_OBJS) $(BASIC_OBJS) test_P.o
DIS_OBJS_TSP=$(ROBOT_OBJS) $(BASIC_OBJS) test_krrf.o
DIS_OBJS_KRRF_DRAW=$(ROBOT_OBJS) $(BASIC_OBJS) test_krrf_draw.o
DIS_OBJS_LAZY=$(ROBOT_OBJS) $(BASIC_OBJS) test_lazy.o
DIS_OBJS_PQUEUE=$(ROBOT_OBJS) $(BASIC_OBJS) test_pqueue.o
DIS_OBJS_FLANN=$(ROBOT_OBJS) $(BASIC_OBJS) test_flann.o


#OMPL2_OBJS=$(ROBOT_OBJS) $(BASIC_OBJS) ompl2D.o
#OMPL3_OBJS=$(ROBOT_OBJS) $(BASIC_OBJS) ompl3D.o


#CXXDEFINE+=-Wall #-pedantic -Wno-long-long
LDFLAGS+=-L/usr/local/lib -lpthread -lm 
LDFLAGS+=-Llibs/op -lOPLoader 
#LDFLAGS+=-lrt

SRC=$(wildcard *.cc)
OBJS=$(patsubst %.cc,%.o,$(wildcard *.cc))



all: support  test_krrf

#test: support $(DIS_OBJS) 
#	$(CXX) $(DIS_OBJS) $(LDFLAGS) -o test

#testcontrol: support $(CONTROL_OBJS) 
#	$(CXX) $(CONTROL_OBJS) $(LDFLAGS) -o testcontrol

#test_P: support $(DIS_OBJS_P) 
#	$(CXX) $(DIS_OBJS_P) $(LDFLAGS) -o test_P

test_krrf: support $(DIS_OBJS_TSP) 
	$(CXX) $(DIS_OBJS_TSP) $(LDFLAGS) -o test_krrf

#test_lazy: support $(DIS_OBJS_LAZY) 
#	$(CXX) $(DIS_OBJS_LAZY) $(LDFLAGS) -o test_lazy

#test_pqueue: support $(DIS_OBJS_PQUEUE) 
#	$(CXX) $(DIS_OBJS_PQUEUE) $(LDFLAGS) -o test_pqueue

test_krrf_draw: support $(DIS_OBJS_KRRF_DRAW) 
	$(CXX) $(DIS_OBJS_KRRF_DRAW) $(LDFLAGS) -o test_krrf_draw

#test_flann: support $(DIS_OBJS_FLANN) 
#	$(CXX) $(DIS_OBJS_FLANN) $(LDFLAGS) -o test_flann

support: mytriangle guii rapidd mpnnn $(BASIC_OBJS)


guii:
	$(MAKE) -C libs/gui 

.cc.o:
	$(CXX) $(CXXDEFINE) $(CXXINCLUDE) -c $<

mpnnn:
	$(MAKE) -C libs/mpnn2

lkh:
	$(MAKE) -C libs/LKH/LKH-2.0.10
	cp libs/LKH/LKH-2.0.10/LKH .


rapidd:
	$(MAKE) -C libs/rapid

flann:
	$(MAKE) -C libs/flann

opproblem:
	$(MAKE) -C libs/op

mytriangle:
	$(MAKE) -C libs/triangle trilibrary


ddijkstra:
	$(MAKE) -C libs/dijkstra

clean:
	rm -f $(OBJS)  *.core r2 r3 tpart tprm rmod texploration texploration2 libmpexploration.a rdis r3dis ompl3 ompl2 test test_P test_krrf test_krrf_draw LKH

cleanpng:
	rm -f *.png

cleanall:
	rm -f $(OBJS) 
	$(MAKE) -C libs/gui clean
	$(MAKE) -C libs/rapid clean
	$(MAKE) -C libs/dijkstra clean
	$(MAKE) -C libs/mpnn2 clean
	$(MAKE) -C libs/triangle clean

cleanlog:
	rm -f _rbtr.*.log

clean_statlogpic:
	rm *.pdf *.png *.log *.stat

