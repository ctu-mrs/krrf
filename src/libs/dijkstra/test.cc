
#include "dijkstra_lite.h"

#include <vector>
#include <iostream>
#include <stdlib.h>

using namespace std;
using namespace imr::dijkstra;
      
typedef double Weight;


/** loads graph from file; on each line one edgee is defined in form:
  * from to cost
  */
void testDijkstraFromFile(int argc, char **argv) {

    cerr << "usage: " << argv[0] << " graph file \n";

	const char *filename = argv[1];

	ifstream ifs(filename);

	CDijkstraLite<Weight> dij;
	int from,to;
	double cost;
	int minidx = -1;
	int maxidx = -1;

	cerr << "Loading " << filename << " ..";
	while(ifs) {
		if (ifs >> from >> to >> cost) {
			cerr << "from to cost: "<< from << " "<< to << " "<<cost;
			dij.addEdge(from,to,cost);
			if (from < to) {
				if (from < minidx || minidx == -1) {
					minidx = from;
				}
				if (to > maxidx || maxidx == -1) {
					maxidx = to;
				}
			} else {
				if (to < minidx || minidx == -1) {
					minidx = to;
				}
				if (from > maxidx || maxidx == -1) {
					maxidx = from;
				}
			}
		} else {
//			ifs.clear();
//			ifs.ignore(10000,'\n');
		}
	}
	ifs.close();

	cerr << "Loaded graph: minVertex: " << minidx << ", maxVertex: " << maxidx << "\n";

	vector<int> p;
	vector<Weight> dist;
	dij.solve(0,p,dist);


	cerr << "Preds: ";
	for(int i=0;i<p.size();i++) 
		cerr << "["<<i<<"]="<< p[i] << " ";
	cerr << "\n";

	cerr << "Dists: ";
	for(int i=0;i<dist.size();i++)
		cerr << "["<<i<<"]="<<dist[i]<<" ";
	cerr << "\n";

	int l = 5;
	vector<int> res;
	while(l != 0) {
		res.push_back(l);
		l = p[l];
	}
	res.push_back(l);

	cerr << "Result: ";
	for(int i=0;i<res.size();i++) 
		cerr << res[i] << " ";
	cerr << "\n";
}


/** loads graph from file; on each line one edgee is defined in form:
  * from to cost
  */
void testSaveDistanceMatrixForAllPts(int argc, char **argv) {

	if (argc < 4) {
		cerr << "usage: " << argv[0] << " <graphFile> <outputFile> <minVertexIndex>\n";
		cerr << "<graphFile>        file with one edge per line: from to cost\n";
		cerr << "<outputFile>       file with output costs between pairs of vertices: fromVertex toVertex cost\n";
		cerr << "<minVertexIndex>   start index of vertex to be processed\n";
		cerr << "loads file containing graph ane stores cost between various vertices - eg. result [1 5 100] means\n";
		cerr << "that cost of shortest route between vertex 1 and 5 is 1000. The vertives to be provessed start from\n";
		cerr << "given minVertexIndex\n";
	}

	const char *ifilename = argv[1];
	const char *ofilename = argv[2];
	const int startIdx = atoi(argv[3]);

	ifstream ifs(ifilename);

	CDijkstraLite<Weight> dij;
	int from,to;
	double cost;
	int minidx = -1;
	int maxidx = -1;

	cerr << "Loading " << ifilename << " ..";
	while(ifs) {
		if (ifs >> from >> to >> cost) {
//			cerr << "from to cost: "<< from << " "<< to << " "<<cost;
			dij.addEdge(from,to,cost);
			if (from < to) {
				if (from < minidx || minidx == -1) {
					minidx = from;
				}
				if (to > maxidx || maxidx == -1) {
					maxidx = to;
				}
			} else {
				if (to < minidx || minidx == -1) {
					minidx = to;
				}
				if (from > maxidx || maxidx == -1) {
					maxidx = from;
				}
			}
		} else {
//			ifs.clear();
//			ifs.ignore(10000,'\n');
		}
	}
	ifs.close();
	cerr << "ok\n";

	maxidx++;

	cerr << "Loaded graph: minVertex: " << minidx << ", maxVertex: " << maxidx << "\n";
	
	ofstream ofs(ofilename);

	for(int f = startIdx; f < maxidx; f++) {
		cerr << "from idx = " << f << " .. ";
		vector<int> p;
		vector<Weight> dist;
		dij.solve(f,p,dist);

		for(int j=startIdx;j<(int)dist.size();j++) {
			ofs << f << " " << j << " " << dist[j] << "\n";
		}
		p.clear();
		dist.clear();
		cerr << "ok\n";
	}
	ofs.close();

/*
	cerr << "Preds: ";
	for(int i=0;i<p.size();i++) 
		cerr << "["<<i<<"]="<< p[i] << " ";
	cerr << "\n";

	cerr << "Dists: ";
	for(int i=0;i<dist.size();i++)
		cerr << "["<<i<<"]="<<dist[i]<<" ";
	cerr << "\n";

	int l = 5;
	vector<int> res;
	while(l != 0) {
		res.push_back(l);
		l = p[l];
	}
	res.push_back(l);

	cerr << "Result: ";
	for(int i=0;i<res.size();i++) 
		cerr << res[i] << " ";
	cerr << "\n";
*/

}



int test() {

	vector<int> p;
	vector<Weight> dist;

	CDijkstraLite<Weight> dij;

	dij.addEdge(0,1,2);
	dij.addEdge(0,2,1);
	dij.addEdge(1,3,3);
	dij.addEdge(1,4,1);
	dij.addEdge(2,3,1);
	dij.addEdge(3,4,4);
	dij.addEdge(4,5,1);

	dij.solve(0,p,dist);

	cerr << "Preds: ";
	for(int i=0;i<p.size();i++) 
		cerr << "["<<i<<"]="<< p[i] << " ";
	cerr << "\n";

	cerr << "Dists: ";
	for(int i=0;i<dist.size();i++)
		cerr << "["<<i<<"]="<<dist[i]<<" ";
	cerr << "\n";

	int l = 5;
	vector<int> res;
	while(l != 0) {
		res.push_back(l);
		l = p[l];
	}
	res.push_back(l);

	cerr << "Result: ";
	for(int i=0;i<res.size();i++) 
		cerr << res[i] << " ";
	cerr << "\n";
}

int main(int argc, char **argv) {

//	testSaveDistanceMatrixForAllPts(argc,argv);

	testDijkstraFromFile(argc,argv);

}


