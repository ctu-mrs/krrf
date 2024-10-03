#ifndef OPTIONS3D_H
#define OPTIONS3D_H

#include "CParseArgs.h"

namespace rrtPlanning {

struct CommonOptions3D {

    CommonOptions3D() {
        distanceToGoal = -1; 
        goalBias = -1; 
        rrtSize = 100;
        sgfile = NULL;
        generateSGs = -1;
        sgindex = -1;
        prefix = NULL;
        painterType = 0;
        drawSolution = 0;
        numIter = 20;
        robotFile = NULL;
        mapFile = NULL;
        problemType = 0;
        timeOut = -1;
        appendStat = -1;
        centerRobotX = defaultCenter();
        centerRobotY = defaultCenter();
        centerRobotZ = defaultCenter();
        useBBox = 0;
        rotationWeight = 1.0;
        incFile = NULL;
        expansionStep = 0.5;

    }

    // return true of all centerRobotX are not 10000;
    bool isRobotCenterOK() {
        return (centerRobotX != defaultCenter()) && (centerRobotY != defaultCenter()) && (centerRobotZ != defaultCenter());
    }

    double defaultCenter() { return 10000; }

    double distanceToGoal;
    double goalBias; 
    double centerRobotX;
    double centerRobotY;
    double centerRobotZ;
    int rrtSize;
    int appendStat;
    int useBBox;

    char *sgfile;
    int generateSGs;
    int sgindex;
    double sgRadius;
    double rotationWeight;
    char *prefix;
    int painterType;
    int  drawSolution;
    int numIter;
    int randomSeed;
    char *robotFile;
    char *mapFile;
    char *incFile;
    int problemType;
    int timeOut;
    double expansionStep;
};

void addCommon3DOptions(CmdOptions &o, CommonOptions3D &data);



}






#endif
