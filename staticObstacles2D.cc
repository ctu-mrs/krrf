#include <vector>
#include "types.h"
#include "ut.h"


namespace rrtPlanning {


/** map consisting of one narrow passage, which is extended until start lies inside 

    example of run in: etfa18.runGap.pl





  |     
  |        \     /  
  |         \---/
  |           |     <---- narrow passage size (%)
  |    depth/----\   <- the angle here is the 'angle' and the 'x' dimension is the depth
  |        /      \
  |       /        \
  |      /          \
  |     |           |
  |=====|           |======
  |    leftx (%)      rightX (%)
  

  note: (%) values are in percent of the respect dimension size

  */

vector< Triangle > addGap(const double leftX, const double rightX, const double npSize, const double funnelAngle, const double funnelDepth, const SDimension &mapDimension) {

    const double sx = mapDimension.getSize(0);
    const double sy = mapDimension.getSize(1);

    WDEBUG("Generating gap obstacle leftX=" << leftX << ", rightX = " << rightX << ", funnelAngle=" << funnelAngle << ", funnelDepth=" << funnelDepth);
    const double ox1 = leftX*sx;
    const double ox2 = rightX*sx;

    vector< vector< TPoint > > mapTriangles;
    vector < vector< TPoint> > mapBoxes;

    const double eps = 5;
    const double gap = npSize * sy;

    mapBoxes.push_back(vector<TPoint>()); // bottom horiz
    mapBoxes.back().push_back(TPoint(0, eps));
    mapBoxes.back().push_back(TPoint(sx, -eps));

    mapBoxes.push_back(vector<TPoint>());  // top horiz
    mapBoxes.back().push_back(TPoint(0, sy+eps));
    mapBoxes.back().push_back(TPoint(sx, sy-eps));

    mapBoxes.push_back(vector<TPoint>());  // left vertical
    mapBoxes.back().push_back(TPoint(-eps, sy+eps));
    mapBoxes.back().push_back(TPoint(+eps, 0-eps));

    mapBoxes.push_back(vector<TPoint>());  // right vertical
    mapBoxes.back().push_back(TPoint(sx-eps, sy+eps));
    mapBoxes.back().push_back(TPoint(sx+eps, 0-eps));


    mapBoxes.push_back(vector<TPoint>());  // bottom obstacles
    mapBoxes.back().push_back(TPoint(ox1, sy/2-gap));
    mapBoxes.back().push_back(TPoint(ox2, 0));


    mapBoxes.push_back(vector<TPoint>());  // top obstacle
    mapBoxes.back().push_back(TPoint(ox1, sy));
    mapBoxes.back().push_back(TPoint(ox2, sy/2+gap));

    /*
    const double angleD = funnelAngle*M_PI/180;
    const double a = tan(angleD)*funnelDepth;

    mapBoxes.push_back(vector<TPoint>());  // bottom left box
    mapBoxes.back().push_back(TPoint(ox1-funnelDepth, sy/2-gap-a));
    mapBoxes.back().push_back(TPoint(ox1, 0));

    mapTriangles.push_back( vector<TPoint> ());
    mapTriangles.back().push_back(TPoint(ox1-funnelDepth, sy/2-gap-a));
    mapTriangles.back().push_back(TPoint(ox1, sy/2-gap-a));
    mapTriangles.back().push_back(TPoint(ox1, sy/2-gap));

    mapBoxes.push_back(vector<TPoint>());  // top left box
    mapBoxes.back().push_back(TPoint(ox1-funnelDepth, sy));
    mapBoxes.back().push_back(TPoint(ox1, sy/2+gap+a));
    mapTriangles.push_back( vector<TPoint> ());
    mapTriangles.back().push_back(TPoint(ox1-funnelDepth, sy/2+gap+a));
    mapTriangles.back().push_back(TPoint(ox1, sy/2+gap+a));
    mapTriangles.back().push_back(TPoint(ox1, sy/2+gap));
    */

    for(int i=0;i<(int)mapBoxes.size();i++) {
        vector< vector< TPoint > > tmp( box2triangles(mapBoxes[i][0], mapBoxes[i][1]));
        std::copy( tmp.begin(), tmp.end(), std::back_inserter(mapTriangles));

    }

    // shift All boxes to dimension of the map
    for(int i=0;i<(int)mapTriangles.size();i++) {
        for(int j=0;j<(int)mapTriangles[i].size();j++) {
            mapTriangles[i][j].x += mapDimension.min.x;
            mapTriangles[i][j].y += mapDimension.min.y;
        }
    }

    vector< Triangle > newMapTriangles( oldTriangles2newTriangles( mapTriangles ) );



    return newMapTriangles;

}

} // namespace

