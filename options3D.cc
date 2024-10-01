
#include "options3D.h"

namespace rrtPlanning {


void addCommon3DOptions(CmdOptions &o, CommonOptions3D &data) {

    o.addOption(Option<double>("dtg",&data.distanceToGoal,-1,"distance to goal"));
    o.addOption(Option<double>("gb",&data.goalBias,-1,"goal bias in %"));
    o.addOption(Option<double>("estep",&data.expansionStep,0.5,"max distance of expansion (in MU)"));

    o.addOption(Option<double>("rw",&data.rotationWeight, 0.2, "rotation scale for 6D metric"));

    o.addOption(Option<int>("size",&data.rrtSize,"edge samples"));

    o.addOption(Option<int>("genconfig",&data.generateSGs,0,"generate N(=arg) random start/goal points, then terminates. see sgRadius"));
    o.addOption(Option<double>("sgradius",&data.sgRadius,15,"radius defining safety radius around robot - only if genconfig is used"));
    o.addOption(Option<char *>("sgfile",&data.sgfile,NULL,"start/goal file - generated using -genconfig option, see -sgindex"));
    o.addOption(Option<int>("sgindex",&data.sgindex,-1,"index of start/goal in the -sgfile"));

    o.addOption(Option<char *>("o",&data.prefix,"prefix for output files"));
    o.addOption(Option<int>("pt",&data.painterType,1,"0=no gui, 1=povray"));
    o.addOption(Option<int>("draw",&data.drawSolution,0,"1=animate results"));
    o.addOption(Option<int>("iters",&data.numIter,20, "number of trials"));

    o.addOption(Option<char *>("robot",&data.robotFile,"RAW file with definition of the robot"));
    o.addOption(Option<char *>("map",&data.mapFile,"RAW file with definition of the robot"));
    o.addOption(Option<char *>("inc",&data.incFile,NULL, "if given, PREFIX.map.inc is replaced by this file"));

    o.addOption(Option<int>("problem",&data.problemType,"problem type"));
    

    o.addOption(Option<int>("srand",&data.randomSeed,-1,"<=0: randomSeed is current time, otherwise used given one"));
    o.addOption(Option<int>("timeout",&data.timeOut,-1,"time out in second. After this, RRT will be automatically terminated"));
    
    o.addOption(Option<int>("astat",&data.appendStat,-1,"1 = append new statistics to .stat file (if exists)"));
    
    o.addOption(Option<int>("mapbox",&data.useBBox,0,"1 = added extra AABB bounding box to map "));
    
    o.addOption(Option<double>("crobotx",&data.centerRobotX,data.defaultCenter(),"if set, defines robot origin, so loaded robot will be centered around this point"));
    o.addOption(Option<double>("croboty",&data.centerRobotY,data.defaultCenter(),"if set, defines robot origin, so loaded robot will be centered around this point"));
    o.addOption(Option<double>("crobotz",&data.centerRobotZ,data.defaultCenter(),"if set, defines robot origin, so loaded robot will be centered around this point"));
      
}

}






