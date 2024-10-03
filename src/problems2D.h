
#ifndef PROBLEMS_2D_H
#define PROBLEMS_2D_H

namespace rrtPlanning {

template< typename State >
static void selectProblem(State &initState, State &goalState, const int number) {



    WDEBUG("Selecting start/goal for problem " << number);
    switch(number) {
        case 0:
            {
                WDEBUG("map.txt");
                initState[0] = 100; initState[1] = 100; initState[2] = M_PI/2; 
                goalState[0] = 730; goalState[1] = 500; goalState[2] = -M_PI/2;
                break;
            }

        case 1: { 
                    WDEBUG("potholes map");
                    initState[0] = 100; initState[1] = -800; initState[2] = 0; 
                    goalState[0] = 800; goalState[1] = -100; goalState[2] = 0;
                    break;
                }


        case 2: {
                    WDEBUG("bt1/bugtraps map");
                    initState[0] = 650; initState[1] = 500; initState[2] = M_PI/2;
                    goalState[0] = 1000; goalState[1] = 850; goalState[2] = M_PI;
                    break;
                }
        case 3:{

                   WDEBUG("gearCorridor3b.txt");
                   initState[0] = -100; initState[1] = -250; initState[2] = M_PI/2; 
                   goalState[0] = 500; goalState[1] = -100; goalState[2] = M_PI/2;
                   break;
               }
        case 4:{

                   WDEBUG("2e map ");
                   initState[0] = 650; initState[1] = -220; initState[2] = 0;
                   goalState[0] = -50; goalState[1] = -220; goalState[2] = M_PI;
                   break;
               }
        case 5:{

                   WDEBUG("np3 ");
                   initState[0] = 0; initState[1] = -320; initState[2] = 0;
                   goalState[0] = 500; goalState[1] = -320; goalState[2] = 0;
                   break;
               } 
        case 6:{

                   WDEBUG("empty1600 ");
                   initState[0] = 400; initState[1] = 500; initState[2] = 0;
                   goalState[0] = 1200; goalState[1] = 500; goalState[2] = 0;
                   break;
               } 
        case 7:{
                   WDEBUG("maps/etfa19/etest2 ");
                   /*
                   initState[0] = 710; initState[1] = -490; initState[2] = 0;
                   goalState[0] = 200; goalState[1] = -503; goalState[2] = M_PI;
                    */
                   goalState[0] = 710; goalState[1] = -490; goalState[2] = 0;
                   initState[0] = 200; initState[1] = -503; initState[2] = M_PI;
                   break;
               }
         case 8: {
                    WDEBUG("maps/np2d/np-*");
                    initState[0] = 150; initState[1] = -300; initState[2] = 0;
                    goalState[0] = 1400; goalState[1] = -700; goalState[2] = 0;
                    break;
                }

         case 9: {
                    WDEBUG("maps/np2d/x-y2");
                    initState[0] = 200; initState[1] = -380; initState[2] = 0;
                    goalState[0] = 1100; goalState[1] = -500; goalState[2] = 0;
                    break;
                }



         case 10:{
                   WDEBUG("maps/etfa19/ceckoMap, pool.cecko, robot: maps/etfa19/ceckoRobot.tri, peeling at: maps/etfa19/2d-c/*.tri ");
                   goalState[0] = 400; goalState[1] = -550; goalState[2] = M_PI;
                   initState[0] = 800; initState[1] = -550; initState[2] = 0;
                   break;
               }
        
        case 11:
            {
                WDEBUG("bigBT.txt");
                goalState[0] = 3000; goalState[1] = 1200; goalState[2] = M_PI/2; 
                initState[0] = 400; initState[1] = 800; initState[2] = M_PI/2;
                break;
            }

        case 12:
            {
                WDEBUG("chodbaL.txt");
                goalState[0] = -50; goalState[1] = 800; goalState[2] = -M_PI/2; 
                initState[0] = -600; initState[1] = -100; initState[2] = M_PI/2;
                break;
            }

        case 13:
            {
                WDEBUG("cmaze.txt");
                goalState[0] = 9400; goalState[1] = 9400; goalState[2] = -M_PI/2; 
                initState[0] = 100; initState[1] = 9500; initState[2] = M_PI/2;
                break;
            }
        case 14:
            {
                WDEBUG("crazy.txt");
                goalState[0] = 200; goalState[1] = 200; initState[2] = 0; 
                initState[0] = 580; initState[1] = 70; goalState[2] = 0 ;
                break;
            }
        case 15:
            {
                WDEBUG("crazy.txt");
                initState[0] = 200; initState[1] = 200; initState[2] = 0; 
                goalState[0] = 580; goalState[1] = 70; goalState[2] = 0 ;
                break;
            }

        case 16:
            {
                WDEBUG("elektron.txt");
                initState[0] = 3500; initState[1] = 700; initState[2] = -M_PI; 
                goalState[0] = 580; goalState[1] = 70; goalState[2] = 0 ;
                break;
            }

        case 17:
            {
                WDEBUG("gear5.txt"); //would be great for videjko
                initState[0] = 10; initState[1] = -90; initState[2] = M_PI/2; 
                goalState[0] = 190; goalState[1] = -90; goalState[2] = M_PI/2 ;
                break;
            }
        case 18:
            {
                WDEBUG("jari-huge.txt"); //would be great for TSP - all to all computation
                initState[0] = 40; initState[1] = 40; initState[2] = M_PI/2; 
                goalState[0] = 2000; goalState[1] = 2300; goalState[2] = -M_PI/2 ;
                break;
            }
        case 19:
            {
                WDEBUG("m3.simple.txt");
                initState[0] = 40; initState[1] = 40; initState[2] = M_PI/2; 
                goalState[0] = 470; goalState[1] = 535; goalState[2] = -3*M_PI/4;
                break;
            }
        case 20:
            {
                WDEBUG("m3.simple.txt");
                goalState[0] = 40; goalState[1] = 40; goalState[2] = M_PI/2; 
                initState[0] = 470; initState[1] = 535; initState[2] = -3*M_PI/4;
                break;
            }
        case 21:
            {
                WDEBUG("tunnel_twisted.txt"); //would be great for TSP - all to all computation
                initState[0] = 40; initState[1] = -440; initState[2] = 0; 
                goalState[0] = 860; goalState[1] = -40; goalState[2] = M_PI ;
                break;
            }
         case 100:{
                   WDEBUG("maps/aro/pf1");
                   initState[0] = 1700; initState[1] = -1950; initState[2] = 0;
                   goalState[0] = 2300; goalState[1] = -1950; goalState[2] = 0;
                   break;
                  }
         case 101:{
                   WDEBUG("maps/aro/aro2");
                   initState[0] = 3000; initState[1] = -3950; initState[2] = 0;
                   goalState[0] = 4700; goalState[1] = -3950; goalState[2] = 0;
                   break;
                  }

        case 200:
            {
                WDEBUG("volumes.txt");
                initState[0] = 390; initState[1] = 100; initState[2] = 0; 
                goalState[0] = 730; goalState[1] = 500; goalState[2] = M_PI/2;
                break;
            }

        case 300: {  // for ARO - to show how start/goal position influences runtime
                    WDEBUG("potholes map"); 
                    initState[0] = 200; initState[1] = -700; initState[2] = 0;
                    goalState[0] = 20; goalState[1] = -800; goalState[2] = 0; 
                    break;
                }

        case 301: {  // for ARO - to show how start/goal position influences runtime
                    WDEBUG("potholes map"); 
                    initState[0] = 100; initState[1] = -800; initState[2] = 0; 
                    goalState[0] = 200; goalState[1] = -500; goalState[2] = 0;
                    break;
                }

        case 400:
            {
                WDEBUG("map.txt");
                goalState[0] = 100; goalState[1] = 100; goalState[2] = M_PI/2; 
                initState[0] = 730; initState[1] = 500; initState[2] = M_PI/2;
                break;
            }
        case 402: {
                    WDEBUG("bt1/bugtraps map");
                    goalState[0] = 650; goalState[1] = 500; goalState[2] = M_PI/2;
                    initState[0] = 1000; initState[1] = 850; initState[2] = M_PI;
                    break;
                }
        case 403:{

                   WDEBUG("gearCorridor3b.txt");
                   goalState[0] = -100; goalState[1] = -250; goalState[2] = M_PI/2; 
                   initState[0] = 500; initState[1] = -100; initState[2] = M_PI/2;
                   break;
               }
        case 405:{

                   WDEBUG("np3 ");
                   goalState[0] = 0; goalState[1] = -320; goalState[2] = 0;
                   initState[0] = 500; initState[1] = -320; initState[2] = 0;
                   break;
               } 
        case 406:{

                   WDEBUG("empty1600 ");
                   goalState[0] = 400; goalState[1] = 500; goalState[2] = 0;
                   initState[0] = 1200; initState[1] = 500; initState[2] = 0;
                   break;
               }
        case 411:
            {
                WDEBUG("bigBT.txt");
                initState[0] = 3000; initState[1] = 1200; initState[2] = M_PI/2; 
                goalState[0] = 400; goalState[1] = 800; goalState[2] = M_PI/2;
                break;
            }

        case 412:
            {
                WDEBUG("chodbaL.txt");
                initState[0] = -50; initState[1] = 800; initState[2] = -M_PI/2; 
                goalState[0] = -600; goalState[1] = -100; goalState[2] = M_PI/2;
                break;
            }

        case 413:
            {
                WDEBUG("cmaze.txt");
                initState[0] = 9400; initState[1] = 9400; initState[2] = -M_PI/2; 
                goalState[0] = 100; goalState[1] = 9500; goalState[2] = M_PI/2;
                break;
            }
        case 414:
            {
                WDEBUG("crazy.txt");
                goalState[0] = 200; goalState[1] = 200; goalState[2] = 0; 
                initState[0] = 580; initState[1] = 70; initState[2] = 0 ;
                break;
            }
        case 415:
            {
                WDEBUG("crazy.txt");
                goalState[0] = 200; goalState[1] = 200; goalState[2] = 0; 
                initState[0] = 580; initState[1] = 70; initState[2] = 0 ;
                break;
            }

        case 416:
            {
                WDEBUG("elektron.txt");
                goalState[0] = 3500; goalState[1] = 700; goalState[2] = -M_PI; 
                initState[0] = 580; initState[1] = 70; initState[2] = 0 ;
                break;
            }

        case 417:
            {
                WDEBUG("gear5.txt"); //would be great for videjko
                goalState[0] = 10; goalState[1] = -90; goalState[2] = M_PI/2; 
                initState[0] = 190; initState[1] = -90; initState[2] = M_PI/2 ;
                break;
            }
        case 418:
            {
                WDEBUG("jari-huge.txt"); //would be great for TSP - all to all computation
                goalState[0] = 40; goalState[1] = 40; goalState[2] = M_PI/2; 
                initState[0] = 2000; initState[1] = 2300; initState[2] = -M_PI/2 ;
                break;
            }
        case 419:
            {
                WDEBUG("m3.simple.txt");
                goalState[0] = 40; goalState[1] = 40; goalState[2] = M_PI/2; 
                initState[0] = 470; initState[1] = 535; initState[2] = -3*M_PI/4;
                break;
            }
        case 420:
            {
                WDEBUG("m3.simple.txt");
                initState[0] = 40; initState[1] = 40; initState[2] = M_PI/2; 
                goalState[0] = 470; goalState[1] = 535; goalState[2] = -3*M_PI/4;
                break;
            }


         default: {
                 WDEBUG("No problem was selected, use -problem!");
                 exit(0);
    }
    }
}



template<typename State>
void fillStartGoal(const char *problemTypeStr, State &initState, State &goalState, const SDimension &d) {

    /* problemType: either integer (points to problems2D.cc), or
     * X,Y:X:Y, where X,Y a percentage (relative) positions in the x- and y- dimensions, respectively.
     e.g. '0.1,0.2:0.9,0.4' defines start configuration (0.1, 0.2) and goal as (0.9 0.4), where these values are relative (%) positions in the w-space
     */
    std::string problemType( problemTypeStr );

    if (problemType.find(':') == std::string::npos) {
        const int ptype = atoi(problemType.c_str());
            if (ptype != -1) {
                selectProblem(initState, goalState, ptype);
                return;
            }
    } else {
        for(int i=0;i<(int)problemType.size();i++) {
            if (problemType[i] == ',' || problemType[i] == ':') {
                problemType[i] = ' ';
            }
        }
        vector<double> data(lineToNum<double>(problemType));
        if (data.size() == 4) {
            TPoint sp( d.somePoint(data[0], data[1]) );
            initState[0] = sp.x;
            initState[1] = sp.y;
            TPoint ep( d.somePoint(data[2], data[3]) );
            goalState[0] = ep.x;
            goalState[1] = ep.y;
            WDEBUG("Relative start goal from " << printString(data));
            WDEBUG("start=" << printString(initState));
            WDEBUG("goal=" << printString(goalState));
            return;
        }
    }

    WDEBUG("no start/goal given, provide -sgindex/-sgfile or -problem");
    exit(0);
}





}

#endif



