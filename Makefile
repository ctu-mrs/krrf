 
$(shell echo "#define GIT_VERSION \""`git rev-parse HEAD`\" > gitv.h)  

#CXX=colorgcc
#CXX=g++
CXX=ccache g++
CXXINCLUDE+= -std=c++11 #-I/usr/local/include
TARGET=rbtr

#CXXINCLUDE+=-I/home/petr2/opt/include
#LDFLAGS+=-L/home/petr2/opt/lib

#CXXINCLUDE+=-I/home/petr2/planning-template/flann/src/cpp/flann
#LDFLAGS+=-L/home/petr2/planning-template/flann/build/lib


CXXINCLUDE+= -Ilibs/gui3d -I./libs -I./libs/mpnn2 -Ilibs/libsvm -I./libs/flann
LDFLAGS+= -llz4 -Llibs/gui -lgui  -L./libs/rapid -lRAPID -L./libs/mpnn2 -lDNN -Llibs/libsvm #-llocalsvm #-lode

#for ompl, assuming it is in opt/include
#CXXINCLUDE+=-I/home/petr2/opt/include/
#LDFLAGS+=-L/home/petr2/opt/lib #-lompl
#CXXINCLUDE+=-I/usr/include/eigen3   

#CXXINCLUDE+=-I/home/vojta/opt/include/
#LDFLAGS+=-L/home/vojta/opt/lib -lompl

#CXXINCLUDE+=-I/storage/praha1/home/vonasek/opt/include -fPIC 
#LDFLAGS+=-lboost_system -lboost_serialization
#LDFLAGS+=-L/software/boost/1.68.0/intel-17.0.1/lib -lboost_system

LDFLAGS+= -lflann -lflann_cpp 



#for OMPL at Metacenter

#CXXINCLUDE+=-I/storage/praha1/home/vonasek/opt/include/ompl-1.5 -fPIC
#CXXINCLUDE+=-I/storage/praha1/home/vonasek/opt/include/eigen3 -fPIC
#LDFLAGS+=-L/storage/praha1/home/vonasek/opt/lib

#GITVERSION:=$(shell git rev-parse HEAD)
#CXXINCLUDE+=-DGIT_VERSION=\"$(GITVERSION)\"


#CXXINCLUDE+=-I./libs/
#LDFLAGS+=./libs/ozcollide/libozcollide.a

#LDFLAGS+=-L./libs/libhungarian/ -lhungarian

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
 

BASIC_OBJS= CParseArgs.o #CPSO.o perm.o quasiRand.o pca.o art3D.o art.o 
BASIC_OBJS+=dijkstra.o mapr.o map3r.o map.o painting.o ut.o Voronoi.o WLog.o blender.o types.o polygonUtils.o mesh.o timerN.o
BASIC_OBJS+=libs/triangle/triangle.o
#BASIC_OBJS+= rrt3.o rrt3Bidirect.o rrt3DD.o rrtnD.o gsl.o 
BASIC_OBJS+=options2D.o options3D.o rrt2D.o #prm2D.o
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

support: opproblem mytriangle guii rapidd mpnnn $(BASIC_OBJS)


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
	rm -f $(OBJS) $(TARGET) *.core r2 r3 tpart tprm rmod texploration texploration2 libmpexploration.a rdis r3dis ompl3 ompl2 test test_P test_krrf test_krrf_draw

cleanpng:
	rm -f *.png

cleanall:
	rm -f $(OBJS) $(TARGET) $(TARGET).core 
	$(MAKE) -C libs/gui clean
	$(MAKE) -C libs/gui3d clean
	$(MAKE) -C libs/libsvm clean
	$(MAKE) -C libs/rapid clean
	$(MAKE) -C libs/dijkstra clean
	$(MAKE) -C libs/mpnn2 clean
	$(MAKE) -C libs/op clean
	$(MAKE) -C libs/triangle clean

cleanlog:
	rm -f _rbtr.*.log

clean_statlogpic:
	rm *.pdf *.png *.log *.stat

