
#include "options2D.h"

namespace rrtPlanning {


void addCommon2DOptions(CmdOptions &o, CommonOptions2D &data) {

    o.addOption(Option<double>("dtg",&data.distanceToGoal,-1,"distance to goal"));
    o.addOption(Option<double>("gb",&data.goalBias,-1,"goal bias in %"));
    o.addOption(Option<double>("estep",&data.expansionStep,10,"max distance of expansion (in MU)"));
    o.addOption(Option<double>("cdr",&data.cdResolution,1, "resolution of CD"));
        
    o.addOption(Option<char *>("robot",&data.robotFile,"robot in TRI format or :IAxY or :LAxBxC "));
    o.addOption(Option<char *>("map",&data.mapFile,"map in TRI format"));
    o.addOption(Option<char *>("target",&data.targets,"targets format"));

    o.addOption(Option<char *>("traj",&data.trajs,NULL,"trajectory format"));

    o.addOption(Option<char *>("problem",&data.problem,"-1","problem number"));

    o.addOption(Option<int>("size",&data.rrtSize,"edge samples"));
    
    o.addOption(Option<int>("srand",&data.rseed,-1, "random seed, if -1, then time is used"));
    o.addOption(Option<int>("log",&data.userLog,1, "1= prefix.pid.log, otherwise NO log"));

    o.addOption(Option<int>("genvideo",&data.generateVideo,0,"generate video of whole planning process"));

    o.addOption(Option<int>("genconfig",&data.generateSGs,0,"generate N(=arg) random start/goal points, then terminates. see sgRadius"));
    o.addOption(Option<double>("sgradius",&data.sgRadius,15,"radius defining safety radius around robot - only if genconfig is used"));
    o.addOption(Option<char *>("sgfile",&data.sgfile,NULL,"start/goal file - generated using -genconfig option, see -sgindex"));
    o.addOption(Option<int>("sgindex",&data.sgindex,-1,"index of start/goal in the -sgfile"));

    o.addOption(Option<char *>("o",&data.prefix,"prefix for output files"));
    o.addOption(Option<int>("pt",&data.painterType,0,"0=gui, 1=png, 2=pdf, 3=xfig, 4=gnuplot"));
    o.addOption(Option<double>("pts",&data.painterScale,1.0, "Scale of painter, "));
        
    o.addOption(Option<int>("draw",&data.drawSolution,0,"1=animate results"));
    o.addOption(Option<double>("growspeed",&data.growSpeed,1.03,"percentage of grow tree when -draw 2 is used, 1.03 means 3% "));
        
    o.addOption(Option<int>("iters",&data.numIter,20, "number of trials"));
    
    o.addOption(Option<int>("backward",&data.enableBacward,1,"1=enable backward motion for the robot, 0=disable"));
    

    o.addOption(Option<char *>("rgrid",&data.robotGridFile,NULL, "robot in GRID format"));

    o.addOption(Option<char *>("robot",&data.robotFile,".tri file or :IAxY or :LAxBxC "));
    o.addOption(Option<double>("iscale",&data.initRobotScale,1.0,"initial scale"));

    o.addOption(Option<int>("amax",&data.amax,1,"number of trials for connecting multi-goal trajectory"));
}



void addCommon2DControlOptions(CmdOptions &o, CommonOptions2D &data) {

    o.addOption(Option<double>("etime",&data.edgeTime,1,"time in [s] for one RRT/PRM edge"));
    o.addOption(Option<int>("esamples",&data.edgeSamples,10," discretization step for RRT edge"));
    o.addOption(Option<int>("ibranch",&data.inputBranching,2,"branching factor for input"));
 
    o.addOption(Option<double>("mxtime",&data.maxEdgeTime,1.0, "Max time of edge"));
    o.addOption(Option<double>("exprob",&data.exploringProba,0.5,"q probability"));
    o.addOption(Option<int>("mcp",&data.N_mcp,100,"number of points for Monte Carlo propagation"));
    o.addOption(Option<double>("regexpl",&data.regExpl,500,"Exploring region diameter"));



}


}
