#include "ut.h"
#include "mesh.h"
#include "dijkstra.h"



namespace rrtPlanning {

using namespace std;


	
static MPNN::ANNpoint scaling;

Mesh::Mesh() {
//    graph = NULL;
    kdTree = NULL;
}

Mesh::~Mesh() {
    clear();
}


void Mesh::clear() {

  //  clearGraph();

    edges.clear();
    pts.clear();
    triangles.clear();
	if (kdTree) {
        delete kdTree;
    }
    kdTree = NULL;
}

/*
void Mesh::clearGraph() {
    if (graph) {
        for(int i=0;i<(int)pts.size();i++) {
            delete graph[i];
        }
        delete [] graph;
        graph = NULL;
    }
}
*/

static int getIndexKDTree(vector<TPoint> &pts, const TPoint &p, MPNN::MultiANN<int> *kdTree) {

	double dann = INFINITY;
	const double distanceThreshold = 0.01;

	MPNN::ANNpoint query = MPNN::annAllocPt(2);
	query[0] = p.x;
	query[1] = p.y;


	int idx = 0;
	if (kdTree->size > 0) {
		int nidx;
		idx = kdTree->NearestNeighbor(query,nidx,dann);
	}
	MPNN::annDeallocPt(query);

	if (dann > distanceThreshold || kdTree->size == 0) {
		MPNN::ANNpointArray annpt = MPNN::annAllocPts(1,2);
		annpt[0][0] = p.x;
		annpt[0][1] = p.y;
		idx = kdTree->size;
		kdTree->AddPoint(annpt[0],idx);
		MPNN::annDeallocPts(annpt);
		pts.push_back(p);
	}

	return idx;
}




/** voronoi diagram can be generated using  https://lynx1.felk.cvut.cz/svn/vojta/huhuhu/segVoronoi, see rrtPlanning/scripts/mapToTriangles/createVD for details
  the VD is loaded from file containing edges - each edge per line (fromx fromy tox toy )
  */
void Mesh::loadVoronoiDiagram(const char *filename) {
    type = TYPE_VD;

    pts.clear();
    edges.clear();
    triangles.clear();

    //clearGraph();

	ifstream ifs(filename);
	string line;

	int topology[2] = {1,1};
	scaling = MPNN::annAllocPt(2);
	scaling[0] = 1;
	scaling[1] = 1;

	kdTree = new MPNN::MultiANN<int>(2,1,topology,scaling);
    
    WDEBUG("Loading MESH.voronoi from file " << filename);

    const double mscale = 1;
	while(ifs) {
		std::getline(ifs,line);
		vector<double> vd(lineToNum<double>(line));
		if (vd.size() == 4) {
			TPoint p1(mscale*vd[0],mscale*vd[1]);
			TPoint p2(mscale*vd[2],mscale*vd[3]);
				
            int idx1 = getIndexKDTree(pts, p1,kdTree);
            int idx2 = getIndexKDTree(pts, p2,kdTree);
                
            edges.push_back(SEdge(idx1,idx2));
            edges.push_back(SEdge(idx2,idx1));
		}
		vd.clear();
	}

    WDEBUG("Loaded MESH.voronoi: " << pts.size() << " pts, " << edges.size() << " edges from " << filename);

   // MPNN::annDeallocPt(scaling);


    for(int i=0;i<(int)edges.size();i++) {
        const int from = edges[i].from;
        const int to = edges[i].to;
        if ((from >= 0) && (from < (int)pts.size()) && (to>=0) && (to<(int)pts.size())) {
            const double dist = pointDistanceEucleid(pts[from],pts[to]);
            edges[i].cost = dist;
        }
    }
    //makeGraph();

}



void Mesh::loadMeshTriangles(const char *filename) {

    WDEBUG("Loading MESH.mesh from " << filename);
    type = TYPE_MESH;

    pts.clear();
    edges.clear();
    triangles.clear();

	ifstream ifs(filename);
	string line;

	int topology[2] = {1,1};
	scaling = MPNN::annAllocPt(2);
	scaling[0] = 1;
	scaling[1] = 1;

	kdTree = new MPNN::MultiANN<int>(2,1,topology,scaling);


    const double mscale = 1;
	while(ifs) {
		std::getline(ifs,line);
		vector<double> vd(lineToNum<double>(line));
		if (vd.size() == 6) {
			TPoint p1(mscale*vd[0],mscale*vd[1]);
			TPoint p2(mscale*vd[2],mscale*vd[3]);
			TPoint p3(mscale*vd[4],mscale*vd[5]);
			if (1 /*|| triangleArea(p1,p2,p3) > triangleAreaThreshold*/) {
				int idx1 = getIndexKDTree(pts, p1,kdTree);
				int idx2 = getIndexKDTree(pts, p2,kdTree);
				int idx3 = getIndexKDTree(pts, p3,kdTree);
                edges.push_back(SEdge(idx1,idx2));
                edges.push_back(SEdge(idx2,idx1));
                edges.push_back(SEdge(idx1,idx3));
                edges.push_back(SEdge(idx3,idx1));
                edges.push_back(SEdge(idx2,idx3));
                edges.push_back(SEdge(idx3,idx2));
				
				vector<TPoint> tr;
				tr.push_back(p1);
				tr.push_back(p2);
				tr.push_back(p3);
				triangles.push_back(tr);
				tr.clear();
			}
		}
		vd.clear();
	}

    WDEBUG("Loaded MESH.mesh: " << pts.size() << " pts, " << triangles.size() <<" triangles, " << edges.size() << " edges from " << filename);

    for(int i=0;i<(int)edges.size();i++) {
        const int from = edges[i].from;
        const int to = edges[i].to;
        if ((from >= 0) && (from < (int)pts.size()) && (to>=0) && (to<(int)pts.size())) {
            const double dist = pointDistanceEucleid(pts[from],pts[to]);
            edges[i].cost = dist;
        }
    }
//    makeGraph();

}

/* create 'm' */
/*
void Mesh::makeGraph() {
    graph = new double*[pts.size()];

    for(int i=0;i<(int)pts.size();i++) {
        graph[i] = new double[pts.size()];
        for(int j=0;j<(int)pts.size();j++) {
            graph[i][j] = -1;
        }
    }

    WDEBUG("Filling graph edges");
    for(int i=0;i<(int)edges.size();i++) {
        const int from = edges[i].from;
        const int to = edges[i].to;
        const double cost = edges[i].cost;
        if ((from >= 0) && (from < (int)pts.size()) && (to>=0) && (to<(int)pts.size())) {
            graph[from][to] = cost;
        } else {
            WDEBUG("Bad edge[" << i << "].from,to=" << from <<","<<to << ", but pts.size=" << pts.size());
            exit(0);
        }
    }

}
*/
    
vector<int> Mesh::findPath(const TPoint &from, const TPoint &to) {

    if (kdTree) {
        int nidx = 0;
        double dann = INFINITY;
        MPNN::ANNpoint query = MPNN::annAllocPt(2);


        query[0] = from.x; query[1] = from.y;
        int fromIdx = kdTree->NearestNeighbor(query,nidx,dann);

        nidx = 0;
        query[0] = to.x; query[1] = to.y;
        int toIdx = kdTree->NearestNeighbor(query,nidx,dann);

        MPNN::annDeallocPt(query);

        return findPath(fromIdx,toIdx);

    } else {

        WDEBUG("Cannot find path, because kdTree is NULL!");

    }

    return vector<int>();
}


vector<int> Mesh::findPath(const int fromIdx, const int toIdx) {
        //return dijkstraLiteSearch(graph, pts.size(),fromIdx,toIdx);
        return dijkstraLiteSearch2(edges,fromIdx,toIdx);
}

    
std::vector<TPoint> Mesh::intPathToPts(const std::vector<int> &path) const {
    vector<TPoint> res;
    for(int i=0;i<(int)path.size();i++) {
        res.push_back(pts[path[i]]);
    }
    return res;
}


// remove points of given index from the mesh/ basically, we just remove edges indicent with these points
void Mesh::removePts(const std::vector<int> &idx) {
    
    // rebuild the graph
    vector<bool> shouldBeRemoved(pts.size(),false);
    for(int i=0;i<(int)idx.size();i++) {
        if (idx[i]>=0 && idx[i] < (int)pts.size()) {
            shouldBeRemoved[idx[i]] = true;
        } else {
            WDEBUG("Bad index of point to be removed!. index: "<< idx[i] << ", pts.size=" << pts.size());
        }
    }
    WDEBUG("Rebuilding mesh. num of points to be removed: " << idx.size());
    int cnt = 0;
    for(int i=0;i<(int)edges.size();i++) {
        if (shouldBeRemoved[ edges[i].from ] || shouldBeRemoved[ edges[i].to ]) {
            edges[i].cost = 100000;
            cnt++;
        }
    }
    WDEBUG("Affected edges: " << cnt);
    //clearGraph();
    //makeGraph();
}


}

