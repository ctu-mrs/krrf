#ifndef PROBLEMS_3D_H
#define PROBLEMS_3D_H


namespace rrtPlanning {

/** fill init and goal state for th eselected problem */
template<typename ST>
void selectProblemSettings(const int problemType, ST &initState, ST &goalState, vector<TLight> &lights, 
        TPoint3 &camera, TPoint3 &lookAt) 
{
    WDEBUG("3D problem type is " << problemType);
    const double TR = M_PI/180.0;

    camera =TPoint3(0,0,0);
    lookAt = TPoint3(0,0,0);

	lights.push_back(TLight(TPoint3(12,12,7)));
	lights.push_back(TLight(TPoint3(1,12,7)));
	lights.push_back(TLight(TPoint3(-5,-12,7)));
	lights.push_back(TLight(TPoint3(-50,-50,70)));
	lights.push_back(TLight(TPoint3(50,50,70)));
	lights.push_back(TLight(TPoint3(-50,0,70)));
	lights.push_back(TLight(TPoint3(0,-50,70)));
	lights.push_back(TLight(TPoint3(50,0,70)));
	lights.push_back(TLight(TPoint3(0,50,70)));
	lights.push_back(TLight(TPoint3(0,0,40)));


    switch (problemType) {
        case 10: {
                     // alpha puzzle (lavalle front page ) 
                     WDEBUG("puzzle original");
                     initState[0] = 0; initState[1] = 0; initState[2] = 0;
                     goalState[0] = -10; goalState[1] = 63; goalState[2] = 13;
                     camera.x = 5; camera.y = 100; camera.z = 45;
                     lookAt.x = 20; lookAt.y = 0; lookAt.z = 0;
//                     cx = 5; cy = 100; cz = 45; clx = 20; cly = 0; clz = 0;
                     lights.clear();
                     lights.push_back(TLight(TPoint3(10,32,47)));
                     lights.push_back(TLight(TPoint3(-5,-12,37)));
                     lights.push_back(TLight(TPoint3(50,50,0)));
                     break;
                 }
        case 11: {
                     //alpha puzzle 2 (posunut stred robotu)
                     WDEBUG("puzzle centered (original size)");
                     camera.x = 5; camera.y = 100; camera.z = 45;
                     lookAt.x = 20; lookAt.y = 0; lookAt.z = 0;
                     initState[0] = 17.439; initState[1] = 4.572; initState[2] = 0;
                     goalState[0] = 13.835; goalState[1] = 78; goalState[2] = 0;
                     break;
                 }
        case 12: {
                     //alpha puzzle 1.2 (scale 1.2)
                     WDEBUG("puzzle 1.2");
                     camera.x = 5; camera.y = 100; camera.z = 45;
                     lookAt.x = 20; lookAt.y = 0; lookAt.z = 0;
                     initState[0] = -0.1; initState[1] = 5; initState[2] = 0.8;
                     goalState[0] = -12; goalState[1] = 74; goalState[2] = 0;
                     break;
                 }
        case 13: {	
                     //alpha puzzle 1.2 (scale 1.2), (tycky proti sobe)
                     WDEBUG("puzle 1.2, tycky proti sobe");
                     initState[0] = 30.628; initState[1] = 5.757; initState[2] = 7.593;
                     initState[3] = -160.337*TR; initState[4] = -12.954*TR; initState[5] = 187.356*TR;
                     goalState[0] = -12; goalState[1] = 74; goalState[2] = 0;
                     break;
                 }
        case 14: {
                     //alpha puzzle 1.1 (scale 1.1) 
                     WDEBUG("puzzle 1.1");
                     initState[0] = -2.6; initState[1] = 2.34; initState[2] = 0.71;
                     goalState[0] = -8; goalState[1] = 57; goalState[2] = -1.4;
                     camera.x = 150; camera.y = 100; camera.z = 45;
                     lookAt.x = 20; lookAt.y = 0; lookAt.z = 0;
//                     cx = 150; cy = 100; cz = 45; clx = 20; cly = 0; clz = 0;
                     lights.clear();
                     lights.push_back(TLight(TPoint3(10,32,47)));
                     lights.push_back(TLight(TPoint3(-5,-12,37)));
                     lights.push_back(TLight(TPoint3(50,50,0)));
                     break;
                 }
        case 15: {
                     // jezek v kleci real
                     WDEBUG("jezek real");
                     initState[0] = 0; initState[1] = 0; initState[2] = 0;
                     goalState[0] = 5.5; goalState[1] = 1.4; goalState[2] = 0;
                     camera.x = 12; camera.y = -2; camera.z = 6;
//                     cx = 1; cy = -10; cz = 3;	
//                     cx = 12; cy = -2; cz = 6; // same view as case 60

                     lights.clear();
                     lights.push_back(TLight(TPoint3(12,12,7)));
                     lights.push_back(TLight(TPoint3(1,12,7)));
                     lights.push_back(TLight(TPoint3(-5,-12,7)));
                     lights.push_back(TLight(TPoint3(50,0,70)));
                     lights.push_back(TLight(TPoint3(0,0,40)));
                     break;
                 }
        case 16: {	
                     WDEBUG("flange - potrubi od parasol library");
                     initState[0] = 0; initState[1] = 0; initState[2] = 0;
                     goalState[0] = -0.808; goalState[1] = 2.075; goalState[2] = 1.656;
                     camera.x = -5; camera.y = -2; camera.z = 3;
//                     cx = -5; cy = -2; cz = 3;
                     lights.clear();
                     lights.push_back(TLight(TPoint3(4,4,7)));	
                     lights.push_back(TLight(TPoint3(1,4,7)));	
                     lights.push_back(TLight(TPoint3(-5,-4,7)));	
                     lights.push_back(TLight(TPoint3(-5,-3,7)));	
                     // robot3d: metric = dx,dy,dz,0.1*dalpha,0.1*dbeta, 0.1*dgaba
                     break;
                 }
        case 17: {	
                     WDEBUG("bugtrap  od parasol library");
                     // -old - goal is too far from the bugtrap, 
                     //initState[0] = 11; initState[1] = -1.414; initState[2] = -0.666; // vevnitr bugutrapu
                     //goalState[0] = -17; goalState[1] = 0; goalState[2] = 1.656; // vne:$

                     // better for rrt-path
                     goalState[0] = 11; goalState[1] = -1.414; goalState[2] = -0.666; // vevnitr bugutrapu
                     initState[0] = 1; initState[1] = 15; initState[2] = 1.656; // vne:$
                     camera.x = 25; camera.y = 35; camera.z= 52;
                     lookAt.x = 5; lookAt.y = 5; lookAt.z = 10;
//                     cx = 25; cy = 35; cz = 52;
  //                   clx = 5; cly = 5; clz = 10;
                     lights.clear();
                     lights.push_back(TLight(TPoint3(0,0,-40)));
                     lights.push_back(TLight(TPoint3(8,-8,10)));
                     lights.push_back(TLight(TPoint3(-8,8,20)));
                     lights.push_back(TLight(TPoint3(5,5,0)));
                     lights.push_back(TLight(TPoint3(-20,20,-40)));
                     lights.push_back(TLight(TPoint3(-50,5,-40)));
                     lights.push_back(TLight(TPoint3(0,50,-10)));
                     break;
                 }
        case 21: {
                     // jezek v kleci real - ale pohybliva je klecNoPlane, robot je klecNoPlane, mapa je jerekRealScaleCavity2WithPlane
                     WDEBUG("jezek real inverse");
                     initState[0] = 0; initState[1] = 0; initState[2] = 0;
                     goalState[0] = -4; goalState[1] = -3; goalState[2] = 0;
                     camera.x = 1; camera.y = -10; camera.z = 3;
//                     cx = 1; cy = -10; cz = 3;	
                     break;
                 }
        case 22: {		 
                     // alpha puzzle (lavalle front page) - prizniva pozice pro vysvobozeni 
                     // lepsi je pozit 23 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                     WDEBUG("puzzle original,  robot centered");
                     initState[0] = 27.775; initState[1] = 17.913; initState[2] = 8.401;
                     goalState[0] = 23.931; goalState[1] = 65.306; goalState[2] = 8.591;
                     camera.x = 5; camera.y = 100; camera.z = 45;
                     lookAt.x = 20; lookAt.y = 0; lookAt.z = 0;
//                     cx = 5; cy = 100; cz = 45; clx = 20; cly = 0; clz = 0;
                     lights.clear();
                     lights.push_back(TLight(TPoint3(10,32,47)));
                     lights.push_back(TLight(TPoint3(-5,-12,37)));
                     lights.push_back(TLight(TPoint3(50,50,0)));
                     break;
                 }
        case 23: {		 
                     // alpha puzzle (lavalle front page) - velmi NEprizniva pozice pro vysvobozeni 
                     // maps: maps/puzzle.raw and maps/puzzleRobotCenter.raw
                     // FOR THINNING, see 63!!
                     WDEBUG("puzzle original,  robot centered");
                     initState[0] = -6.609; initState[1] = -1.099; initState[2] = -4.801;
                     initState[3] = -17.569*TR; initState[4] = 5.079*TR; initState[5] = -178.696*TR;
                     goalState[0] = 23.931; goalState[1] = 65.306; goalState[2] = 8.591;
//                     cx = 5; cy = 100; cz = 45; clx = 20; cly = 0; clz = 0;
                     camera.x = 5; camera.y = 100; camera.z = 45;
                     lookAt.x = 20; lookAt.y = 0; lookAt.z = 0;

                     lights.clear();
                     lights.push_back(TLight(TPoint3(10,32,47)));
                     lights.push_back(TLight(TPoint3(-5,-12,37)));
                     lights.push_back(TLight(TPoint3(50,50,0)));
                     break;
                 }
        case 24: {		 
                     // heart
                     WDEBUG("heart from inside to outside");
                     initState[0] = 0.023; initState[1] = -1.553; initState[2] = 0;
                     initState[3] = 0; initState[4] = 90*TR; initState[5] = 0;
                     goalState[0] = -0.012; goalState[1] = 3.0; goalState[2] = 0;
                     goalState[3] = 0; goalState[4] = 90*TR; goalState[5] = 90*TR;
                     //cx = -6; cy = -3; cz = 3; clx = 0; cly = 0; clz = 0;
                     camera.x = -6; camera.y = -3; camera.z = 3;
                     lookAt.x = 0; lookAt.y = 0; lookAt.z = 0;
                     break;
                 }
        case 25: {		 
                     // heart but start and goal state are changed
                     WDEBUG("heart from outside to inside");
                     goalState[0] = 0.023; goalState[1] = -1.553; goalState[2] = 0;
                     goalState[3] = 0; goalState[4] = 90*TR; goalState[5] = 0;
                     initState[0] = -0.012; initState[1] = 3.0; initState[2] = 0;
                     initState[3] = 0; initState[4] = 90*TR; initState[5] = 90*TR;
                     //cx = -6; cy = -3; cz = 3; clx = 0; cly = 0; clz = 0;
                     camera.x = -6; camera.y = -3; camera.z = 3;
                     lookAt.x = 0; lookAt.y = 0; lookAt.z = 0;
                     break;
                 }
        case 26: {		 
                     // heart but start and goal state are changed
                     WDEBUG("heart from outside to inside");
                     goalState[0] = 0.00; goalState[1] = -1.5; goalState[2] = 0;
                     goalState[3] = 0; goalState[4] = 90*TR; goalState[5] = 0;
                     initState[0] = 0.0; initState[1] = 3.0; initState[2] = 0;
                     initState[3] = 0; initState[4] = 90*TR; initState[5] = 90*TR;
//                     cx = -6; cy = -3; cz = 3; clx = 0; cly = 0; clz = 0;
                     camera.x = -6; camera.y = -3; camera.z = -3;
                     lookAt.x = 0; lookAt.y = 0; lookAt.z = 0;
                     break;
                 }
        case 27: {
                     WDEBUG("configuration for: room2"); // maps/room2_new.raw and room2r_tycka.raw
                     initState[0] = 10; initState[1] = 5.8; initState[2] = 2.0;
                     goalState[0] = 10; goalState[1] = -8; goalState[2] = 2;
                     lights.clear();
                     lights.push_back(TLight(TPoint3(8,4,20)));
                     lights.push_back(TLight(TPoint3(9,8,6)));
                     lights.push_back(TLight(TPoint3(-7,8,6),false,0.5));
                     lights.push_back(TLight(TPoint3(25,-13,8)));
                     lights.push_back(TLight(TPoint3(-2,-11,7)));
                     //cx = -20; cy = 5; cz = 15; 
                     camera.x = -20; camera.y = 5; camera.z = 15;
                     break;
                 }
        case 28: {	
                     WDEBUG("bugtrap  od parasol library _ small version (bugtrapMapSmall) ");
//                     			initState[0] = 11; initState[1] = -1.414; initState[2] = -0.666; // uvnitr
  //                   			goalState[0] = -3.04; goalState[1] = 0; goalState[2] = 1.656; // vne

                     goalState[0] = 11; goalState[1] = -1.414; goalState[2] = -0.666; // uvnitr
                     initState[0] = -3.04; initState[1] = 0; initState[2] = 1.656; initState[3] = M_PI/2;//  vne
//                     cx = -25; cy = -45; cz = -52;
//                     clx = 5; cly = 5; clz = 10;
                     camera.x = -25; camera.y = -45; camera.z = -52;
                     lookAt.x = 5; lookAt.y = 5; lookAt.z = 10;

                     lights.clear();
                     lights.push_back(TLight(TPoint3(0,0,-25)));
                     lights.push_back(TLight(TPoint3(11,-1,-1)));
                     lights.push_back(TLight(TPoint3(-8,8,20)));
                     lights.push_back(TLight(TPoint3(0,-55,0)));
                     lights.push_back(TLight(TPoint3(30,30,-35)));
                     lights.push_back(TLight(TPoint3(-35,5,-35)));
                     lights.push_back(TLight(TPoint3(0,50,40)));
                     lights.push_back(TLight(TPoint3(30,-30,10)));
                     break;
                 }
        case 29: {
                     WDEBUG("configuration for: room2"); // maps/room2_new.raw and room2r_tycka.raw
                     initState[0] = -7; initState[1] = 5.8; initState[2] = 2.0;
                     goalState[0] = -7; goalState[1] = -8; goalState[2] = 2;

                     // for testing art3d
             //        initState[0] = -7.809; initState[1] = 6.879; initState[2] = 3.25;
               //      goalState[0] = -7; goalState[1] = -8; goalState[2] = 2;
                     lights.clear();
                     lights.push_back(TLight(TPoint3(8,4,20)));
                     lights.push_back(TLight(TPoint3(9,8,6)));
                     lights.push_back(TLight(TPoint3(-7,8,6),false,0.5));
                     lights.push_back(TLight(TPoint3(25,-13,8)));
                     lights.push_back(TLight(TPoint3(-2,-11,7)));
//                     cx = -20; cy = 5; cz = 15; 
                     camera.x = -20; camera.y = 5; camera.z = 15;
                     break;
                 }
        case 32: {
                     WDEBUG("configuration for: room2_tunnel, room_tunnel_long and room_tunnel_long_narrow");
                     initState[0] = 10; initState[1] = 8; initState[2] = 2.0;
                     goalState[0] = -6; goalState[1] = -13; goalState[2] = 2; // -13
//                     cx = 20; cy = 20; cz = 20;
                     camera = TPoint3(20,20,20);
                     lights.clear();
                     lights.push_back(TLight(TPoint3(12,8,7)));
                     lights.push_back(TLight(TPoint3(-5,14,8)));
                     lights.push_back(TLight(TPoint3(1,4,25)));
                     lights.push_back(TLight(TPoint3(-3,-17,8)));
                     lights.push_back(TLight(TPoint3(12,-14,7)));
                     lights.push_back(TLight(TPoint3(20,0,17)));
                     break;
                 }
          case 37: {
                     WDEBUG("configuration for chevrolet: CRobot3d: chevrolet/objects.all.raw, chevrolet/robot.raw");
                     initState[0] = 0.236; initState[1] = 0.096; initState[2] = 0.293; initState[3] = 0; initState[4] = 0; initState[5] = 0*89.526*TR;
                     goalState[0] = 0.994; goalState[1] = 0.843; goalState[2] = 0.505; goalState[3] = 0; goalState[4] = 0; goalState[5] = 0*89.526*TR;
//                     cx = 2; cy = -1.6; cz = 1;
                     camera = TPoint3(2,-1.6, 1);
                     lights.clear();
                     lights.push_back(TLight(TPoint3(4,3,5)));
                     lights.push_back(TLight(TPoint3(-4,3,5)));
                     lights.push_back(TLight(TPoint3(-4,-3,5)));
                     lights.push_back(TLight(TPoint3(1,0,3)));
                     lights.push_back(TLight(TPoint3(14,13,15)));
                     lights.push_back(TLight(TPoint3(-24,23,25)));
                     lights.push_back(TLight(TPoint3(-24,-23,35)));
                     break;
                 }
         case 38: {
                     WDEBUG("configuration for: room3 with two (non)symmetric windows"); // maps/room2_new.raw and room2r_tycka.raw
                     initState[0] = 2.5; initState[1] = 5.8; initState[2] = 2.0;
                     goalState[0] = 2.5; goalState[1] = -8; goalState[2] = 2.0;
                     lights.clear();
                     lights.push_back(TLight(TPoint3(8,4,20)));
                     lights.push_back(TLight(TPoint3(9,8,6)));
                     lights.push_back(TLight(TPoint3(-7,8,6)));
                     lights.push_back(TLight(TPoint3(25,-13,8)));
                     lights.push_back(TLight(TPoint3(-2,-11,7)));
                     camera = TPoint3(20,9,15);
//                     cx = -20; cy = 9; cz = 15; 
                     break;
                 }

        case 39: {
                     WDEBUG("configuration for: roomPoles"); // maps/room2_new.raw and room2r_tycka.raw
                     initState[0] = 0; initState[1] = 6; initState[2] = 2.0;
                     goalState[0] = 0; goalState[1] = -6; goalState[2] = 2;
                     lights.clear();
                     lights.push_back(TLight(TPoint3(8,4,20)));
                     lights.push_back(TLight(TPoint3(9,8,6)));
                     lights.push_back(TLight(TPoint3(-7,8,6),false,0.5));
                     lights.push_back(TLight(TPoint3(25,-13,8)));
                     lights.push_back(TLight(TPoint3(-2,-11,7)));
                     //cx = -20; cy = 5; cz = 15; 
                     camera.x = -20; camera.y = 5; camera.z = 15;
                     break;
                 }
 


         case 50: {
                     WDEBUG("configuration for gear in assembly scenario: gearMap and gearDecimate ");

                     initState[0] = 2; initState[1] = -2; initState[2] = 0.5;
                     goalState[0] = -2; goalState[1] = 2; goalState[2] = 0.5;
                     lights.clear();
                     lights.push_back(TLight(TPoint3(0,-5,20)));
                     lights.push_back(TLight(TPoint3(-5,0,6)));
                     lights.push_back(TLight(TPoint3(5,0,6)));
                     camera = TPoint3(-2,-6,2);
                     //cx = -2; cy = -6; cz = 2; 
                     break;
                 }
        case 60: {
                     // jezek v kleci real - klec se zaplatama
                     WDEBUG("jezek real klec One Window");
                     initState[0] = 0; initState[1] = 0; initState[2] = 0;
                     goalState[0] = 5.5; goalState[1] = 1.4; goalState[2] = 0;
                     camera = TPoint3(12,-2,6);
                     //cx = 12; cy = -2; cz = 6;

                     lights.clear();
                     lights.push_back(TLight(TPoint3(12,12,7)));
                     lights.push_back(TLight(TPoint3(1,12,7)));
                     lights.push_back(TLight(TPoint3(-5,-12,7)));
                     lights.push_back(TLight(TPoint3(50,0,70)));
                     lights.push_back(TLight(TPoint3(0,0,40)));
                     break;
                 } 
        case 61: {
                     WDEBUG("configuration for: room3, room6 + robots: tycka, room2r_b, room2_robot2s_a");
                     initState[0] = -6.7; initState[1] = -13; initState[2] = 2.5;
                     goalState[0] = -6.7; goalState[1] = 7; goalState[2] = 2.5;
                     //cx = -20; cy = -20; cz = 20;
                     camera = TPoint3(-20,-20,20);
                     lights.clear();
                     lights.push_back(TLight(TPoint3(12,8,7)));
                     lights.push_back(TLight(TPoint3(-5,14,8)));
                     lights.push_back(TLight(TPoint3(1,4,25)));
                     lights.push_back(TLight(TPoint3(-3,-17,8)));
                     lights.push_back(TLight(TPoint3(12,-14,7)));
                     lights.push_back(TLight(TPoint3(20,0,17)));
                     break;
                }
        case 63: {		 
                     // alpha puzzle (lavalle front page) - velmi NEprizniva pozice pro vysvobozeni 
                     // maps: maps/puzzle.raw and maps/puzzleRobotCenter.raw
                     // requires scaling 11.6!!
                     WDEBUG("puzzle original,  robot centered");
                     initState[0] = 7.449; initState[1] = 6.514; initState[2] = -13.025;
                     initState[3] = -55.195*TR; initState[4] = 58.298*TR; initState[5] = 67.972*TR;
                     goalState[0] = 22; goalState[1] = 47; goalState[2] = 11;
                     
//                     cx = 105; cy = 100; cz = 45; clx = 20; cly = 0; clz = 0;
                     camera = TPoint3(105,100,45);
                     lookAt = TPoint3(20,0,0);

                     lights.clear();
                     lights.push_back(TLight(TPoint3(10,32,47)));
                     lights.push_back(TLight(TPoint3(-5,-12,37)));
                     lights.push_back(TLight(TPoint3(50,50,0)));
                     break;
                 }
        case 64: {		 
                     WDEBUG("maps/etfa/krouzekRobot.raw map:   maps: any of maps/etfa/krouzek-map*.raw ");
                     WDEBUG("scaling: maps/etfa/robot-krouzek , also pool.krouzek* ");
                     goalState[0] = 2; goalState[1] = -3; goalState[2] = 1.52;
                     initState[0] = -3; initState[1] = 2; initState[2] = 1.52;
                     //cx = -7; cy = -7; cz = 6; clx = 0; cly = 0; clz = 0;
                     camera = TPoint3(-7,-7,6);
                     lights.clear();
                     lights.push_back(TLight(TPoint3(10,32,47)));
                     lights.push_back(TLight(TPoint3(-5,-12,37)));
                     lights.push_back(TLight(TPoint3(50,50,0)));
                     break;
                 }
        case 65: {		 // box of 9 places for nine bottles 
                     WDEBUG("maps/etfa/bednaCylinderRobot.raw,   map: maps/etfa/bedna-mapLoose.raw");
                     WDEBUG("scaling files: maps/etfa/robot-cylinder/*.raw , also pool.cylinder ");
                     goalState[0] = 2.5; goalState[1] = -2.5; goalState[2] = 1.2;
                     initState[0] = -2.5; initState[1] = 2.5; initState[2] = 1.2;
                     //cx = -7; cy = -7; cz = 6; clx = 0; cly = 0; clz = 0;
                     camera = TPoint3(-7,-7,6);
                     lights.clear();
                     lights.push_back(TLight(TPoint3(10,32,47)));
                     lights.push_back(TLight(TPoint3(-5,-12,37)));
                     lights.push_back(TLight(TPoint3(50,50,0)));
                     break;
                 }
        case 66: {		 
                     WDEBUG("maps/etfa/krouzekLongRobot.raw map:   maps: any of maps/etfa/krouzekLong-map*.raw ");
                     WDEBUG("scaling: maps/etfa/robot-krouzek-long , also pool.krouzekLong ");
                     goalState[0] = 2; goalState[1] = -3; goalState[2] = 1.6;
                     initState[0] = -3; initState[1] = 2; initState[2] = 1.6;
                     //cx = -7; cy = -7; cz = 6; clx = 0; cly = 0; clz = 0;
                     camera = TPoint3(-7,-7,6);
                     lights.clear();
                     lights.push_back(TLight(TPoint3(10,32,47)));
                     lights.push_back(TLight(TPoint3(-5,-12,37)));
                     lights.push_back(TLight(TPoint3(50,50,0)));
                     break;
                 }
        case 67: {		 
                     WDEBUG("maps/etfa/c-robot/*      map:maps/etfa/c-robotMap");
                     goalState[0] = -4; goalState[1] = -3; goalState[2] = 3.2;
                     initState[0] = 4; initState[1] = 4; initState[2] = 3.2;
                     //cx = -7; cy = -7; cz = 6; clx = 0; cly = 0; clz = 0;
                     camera = TPoint3(-7,-7,6);
                     lights.clear();
                     lights.push_back(TLight(TPoint3(10,32,47)));
                     lights.push_back(TLight(TPoint3(0,-8,2)));
                     lights.push_back(TLight(TPoint3(50,50,0)));
                     break;
                 }

        case 68: {		 
                     WDEBUG("maps/etfa/c(1/2/3)-peel-top/*      map:maps/etfa/c-robotMap.raw");
                     goalState[0] = -4; goalState[1] = -2; goalState[2] = 4.5;
                     initState[0] = 4; initState[1] = 4; initState[2] = 4.5;
                     //cx = -7; cy = -7; cz = 6; clx = 0; cly = 0; clz = 0;
                     camera = TPoint3(-7,-7,6);
                     lights.clear();
                     lights.push_back(TLight(TPoint3(10,32,47)));
                     lights.push_back(TLight(TPoint3(0,-8,2)));
                     lights.push_back(TLight(TPoint3(50,50,0)));
                     break;
                 }
        case 69: {		 
                     WDEBUG("maps/etfa/cwide-peel-top/*      map:maps/etfa/c-robotTightMapWide.raw");
                     goalState[0] = -3; goalState[1] = -2; goalState[2] = 4.5;
                     initState[0] = 3; initState[1] = 4; initState[2] = 4.5;
                     //cx = -7; cy = -7; cz = 6; clx = 0; cly = 0; clz = 0;
                     camera = TPoint3(-7,-7,6);
                     lights.clear();
                     lights.push_back(TLight(TPoint3(10,32,47)));
                     lights.push_back(TLight(TPoint3(0,-8,2)));
                     lights.push_back(TLight(TPoint3(50,50,0)));
                     break;
                 }

        case 70: {		 
                     WDEBUG("maps/etfa/krouzekLongRobot.raw map:   maps: any of maps/etfa/krouzekLong-map*.raw ");
                     WDEBUG("scaling: maps/etfa/robot-krouzek-long , also pool.krouzekLong ");
                     initState[0] = 2; initState[1] = -3; initState[2] = 1.6;
                     goalState[0] = -3; goalState[1] = 2; goalState[2] = 1.6;
                     //cx = -7; cy = -7; cz = 6; clx = 0; cly = 0; clz = 0;
                     camera = TPoint3(-7,-7,6);
                     lights.clear();
                     lights.push_back(TLight(TPoint3(10,32,47)));
                     lights.push_back(TLight(TPoint3(-5,-12,37)));
                     lights.push_back(TLight(TPoint3(50,50,0)));
                     break;
                 }
        case 80: {
                     WDEBUG("configuration for maps/icar19: 1-2-1.raw .. 1-3-1.raw... and maps/icar19/npRobot*.raw");
                     initState[0] = -7.83; initState[1] = 8.221; initState[2] = 3.915;
                     goalState[0] = 10.831; goalState[1] = -12.39; goalState[2] = 3.95;
//                     cx = -20; cy = -20; cz = 20;
                     camera = TPoint3(-20,-20,20);
                     lights.clear();
                     lights.push_back(TLight(TPoint3(12,8,7)));
                     lights.push_back(TLight(TPoint3(-5,14,8)));
                     lights.push_back(TLight(TPoint3(1,4,25)));
                     lights.push_back(TLight(TPoint3(-3,-17,8)));
                     lights.push_back(TLight(TPoint3(12,-14,7)));
                     lights.push_back(TLight(TPoint3(20,0,17)));
                     break;
                }
         case 81: {
                     WDEBUG("configuration for maps/icar19: 1-2-1.raw .. 1-3-1.raw... and maps/icar19/npRobot*.raw");
                     initState[0] = 10.831; initState[1] = -12.96; initState[2] = 3.915;
                     goalState[0] = 10.831; goalState[1] = 10.5; goalState[2] = 3.95; 
                     camera = TPoint3(-20,-20,20);
                     lights.clear();
                     lights.push_back(TLight(TPoint3(12,8,7)));
                     lights.push_back(TLight(TPoint3(-5,14,8)));
                     lights.push_back(TLight(TPoint3(1,4,25)));
                     lights.push_back(TLight(TPoint3(-3,-17,8)));
                     lights.push_back(TLight(TPoint3(12,-14,7)));
                     lights.push_back(TLight(TPoint3(20,0,17)));
                     break;
                }
          case 82: {
                     WDEBUG("configuration for maps/icar19: toys*.raw and robot: maps/icar19/robotCylinder-Cross-Gear.raw ");
                     initState[0] = 10; initState[1] = 12; initState[2] = 4.229;
                     goalState[0] = -4; goalState[1] =-15; goalState[2] = 4.229;
                     //cx = -20; cy = -20; cz = 20;
                     camera = TPoint3(-20,-20,20);
                     lights.clear();
                     lights.push_back(TLight(TPoint3(12,8,7)));
                     lights.push_back(TLight(TPoint3(-5,14,8)));
                     lights.push_back(TLight(TPoint3(1,4,25)));
                     lights.push_back(TLight(TPoint3(-3,-17,8)));
                     lights.push_back(TLight(TPoint3(12,-14,7)));
                     lights.push_back(TLight(TPoint3(20,0,17)));
                     break;
                }
           case 83: {
                     WDEBUG("configuration for escher.raw and escherRobot.raw: R/V robot");
                     WDEBUG("From outside to inside of the box!!");
                     initState[0] = -8; initState[1] = -1; initState[2] = 7; //right
//                     goalState[0] = 13; goalState[1] = -1; goalState[2] = 7; // left
                     goalState[0] = 2.15; goalState[1] = -1; goalState[2] = 7; // middle
//                     cx = -20; cy = -10; cz = 20;   clx = 0; cly = 0; clz = 3;
                     camera = TPoint3(-20,-10,20);
                     lookAt = TPoint3(0,0,3);
                     lights.clear();
                     lights.push_back(TLight(TPoint3(12,8,7)));
                     lights.push_back(TLight(TPoint3(-5,14,8)));
                     lights.push_back(TLight(TPoint3(1,4,25)));
                     lights.push_back(TLight(TPoint3(-3,-17,8)));
                     lights.push_back(TLight(TPoint3(12,-14,7)));
                     lights.push_back(TLight(TPoint3(20,0,17)));
                     break;
                }
            case 84: {
                     WDEBUG("configuration for escher.raw and escherRobot.raw: R/V robot");
                     initState[0] = 2.15; initState[1] = -1; initState[2] = 7; // middle
                     goalState[0] = 13; goalState[1] = -1; goalState[2] = 7;
                     //cx = -20; cy = -10; cz = 20;   clx = 0; cly = 0; clz = 3;
                     camera = TPoint3(-20,-10,20);
                     lookAt = TPoint3(0,0,3);
                     lights.clear();
                     lights.push_back(TLight(TPoint3(12,8,7)));
                     lights.push_back(TLight(TPoint3(-5,14,8)));
                     lights.push_back(TLight(TPoint3(1,4,25)));
                     lights.push_back(TLight(TPoint3(-3,-17,8)));
                     lights.push_back(TLight(TPoint3(12,-14,7)));
                     lights.push_back(TLight(TPoint3(20,0,17)));
                     break;
                }
          case 85: {
                     WDEBUG("configuration for maps/jint20/emental maps, robots: maps/jint20/robot*");
                     initState[0] = 3; initState[1] = -11; initState[2] = 4;
                     goalState[0] = 3; goalState[1] = 8; goalState[2] = 4;
                     //cx = -20; cy = -20; cz = 20;
                     camera = TPoint3(-20,-20,20);
                     lights.clear();
                     lights.push_back(TLight(TPoint3(12,8,7)));
                     lights.push_back(TLight(TPoint3(-5,14,8)));
                     lights.push_back(TLight(TPoint3(1,4,25)));
                     lights.push_back(TLight(TPoint3(-3,-17,8)));
                     lights.push_back(TLight(TPoint3(12,-14,7)));
                     lights.push_back(TLight(TPoint3(20,0,17)));
                     break;
                }
          case 86: {
                     WDEBUG("configuration for maps/jint20/emental maps, robots: maps/jint20/robot*");
                     initState[0] = 3; initState[1] = -14; initState[2] = 4;
                     goalState[0] = 3; goalState[1] = 10; goalState[2] = 4;
                     //cx = -20; cy = -20; cz = 20;
                     camera = TPoint3(-20,-20,20);
                     lights.clear();
                     lights.push_back(TLight(TPoint3(12,8,7)));
                     lights.push_back(TLight(TPoint3(-5,14,8)));
                     lights.push_back(TLight(TPoint3(1,4,25)));
                     lights.push_back(TLight(TPoint3(-3,-17,8)));
                     lights.push_back(TLight(TPoint3(12,-14,7)));
                     lights.push_back(TLight(TPoint3(20,0,17)));
                     break;
                }
         case 87: {
                     WDEBUG("configuration for maps/icar19/ BIG* - big1-2-1.raw .. big1-3-1.raw... and maps/icar19/npRobot*.raw");
                     WDEBUG("configuration for maps/jint20-tunnels/ and maps/jint20-tunnels/npRobot*");
                     initState[0] = 6; initState[1] = -14; initState[2] = 6;
                     goalState[0] = 6; goalState[1] = 16; goalState[2] = 6; 
                     camera = TPoint3(-20,-20,20);
                     lights.clear();
                     lights.push_back(TLight(TPoint3(14,8,7)));
                     lights.push_back(TLight(TPoint3(-5,16,8)));
                     lights.push_back(TLight(TPoint3(1,5,25)));
                     lights.push_back(TLight(TPoint3(-3,-20,8)));
                     lights.push_back(TLight(TPoint3(12,-17,7)));
                     lights.push_back(TLight(TPoint3(20,0,17)));
                     break;
                }



          case 90: {
                     WDEBUG("configuration for maps/pores/*  + maps/pores/poresRobot.raw");
                     initState[0] = 0; initState[1] = 16; initState[2] = 0;
                     goalState[0] = 0; goalState[1] = -16; goalState[2] = 0;
                     //cx = -20; cy = -20; cz = 20;
                     camera = TPoint3(-20,-20,20);
                     lights.clear();
                     lights.push_back(TLight(TPoint3(12,8,7)));
                     lights.push_back(TLight(TPoint3(-5,14,8)));
                     lights.push_back(TLight(TPoint3(1,4,25)));
                     lights.push_back(TLight(TPoint3(-3,-17,8)));
                     lights.push_back(TLight(TPoint3(12,-14,7)));
                     lights.push_back(TLight(TPoint3(20,0,17)));
                     break;
                }






        default: {
                     WDEBUG("no problem type was chosen (value=" << problemType << ").  Please see the list of problems in test3d switch statement in t3d.cc");
                     exit(0);
                 }
    }

}


}
#endif

