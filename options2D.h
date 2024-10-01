#ifndef OPTIONS2D_H
#define OPTIONS2D_H


#include "CParseArgs.h"

namespace rrtPlanning {

struct CommonOptions2D {

    CommonOptions2D() {
        distanceToGoal = -1; 
        goalBias = -1; 
        rrtSize = 100;
        sgfile = NULL;
        generateSGs = -1;
        sgindex = -1;
        prefix = NULL;
        painterType = 0;
        painterScale = 1.0;
        drawSolution = 0;
        generateVideo = 0;
        numIter = 20;
        robotFile = NULL;
        mapFile = NULL;
        trajs = NULL;
        enableBacward = 1;
        rseed = -1;
        userLog = 1;
        problem = "-1";
        initRobotScale = 1.0;
        growSpeed = 1.03;
        cdResolution = 2;
        amax = 1;

        //for KRRF
        N_mcp = 100;
        exploringProba = 0.5;
        maxEdgeTime = 1.0;
        regExpl = 500;


        edgeTime = 1;
        edgeSamples = 3;
        inputBranching = 2;
    }



    /* variables for geometric 2D planning */

    double distanceToGoal;
    double goalBias;
    double growSpeed;
    double painterScale;
    double cdResolution;
    int rrtSize;
    int userLog;
    int amax;

    char *sgfile;
    int generateVideo;
    int generateSGs;
    int sgindex;
    double sgRadius;
    double initRobotScale;
    char *prefix;
    int painterType;
    int  drawSolution;
    int  numIter; 
    char *robotFile;
    char *robotGridFile;
    char *mapFile;
    char *targets;
    char *trajs;

    double expansionStep;

    int enableBacward, rseed; //problem;
    char *problem;

    /* variables for 2D control planners */
    int inputBranching;
    double edgeTime;
    int edgeSamples;

    //vars for KRRF
    int N_mcp;
    double exploringProba;
    double maxEdgeTime;
    double regExpl;

};

void addCommon2DOptions(CmdOptions &o, CommonOptions2D &data);
void addCommon2DControlOptions(CmdOptions &o, CommonOptions2D &data);



}

#endif


