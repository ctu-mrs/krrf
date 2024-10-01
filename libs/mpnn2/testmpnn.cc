#include <iostream>
#include <stdlib.h>
#include <fstream>
#include <vector>

#include "multiann.h"
#include "ANN.h"

using namespace std;

int main(int argc, char **argv) {

    const bool writeToFiles = false;

    time_t t;
    time(&t);
    srand(t);


    // number of dimensions of each point.
    const int dimension = 2;

    // topology[] - defines how distance between two points is computed. 1 is for Euclidean metric
    // scale[] - defines scale of each dimension. 
    // see MPNN/test/test.cc function Metric, where topology is explained
    int *topology = new int[dimension];
    MPNN::ANNcoord *scale = new MPNN::ANNcoord[dimension];
    for(int i=0;i<dimension;i++) {
        scale[i] = 1.0;
        topology[i] = 1;
    }


    // kd-tree that hold int user data type. second parameter ('1') defines that only one nearest neighbor will be later searched
    MPNN::MultiANN<int> *kdTree = new MPNN::MultiANN<int>(dimension,1,topology,(MPNN::ANNpoint)scale);



    // feed the tree with some random points using 'AddPoint' method
    const int numPoints = 250000;
    vector< vector<double> > data;

    MPNN::ANNpointArray annpt = MPNN::annAllocPts(1,dimension);
    for(int i=0;i<numPoints;i++) {
        vector<double> tmp(dimension,0);
        for(int j=0;j<dimension;j++) {
            annpt[0][j] = rand() % 1000 - rand() % 1000;
            tmp[j] = annpt[0][j];
        }

        // add to kd-tree and also to 'data' array.
        kdTree->AddPoint(annpt[0],i);
        data.push_back(tmp);
        if (! (i % 10000)) {
            cerr << i << " ";
        }
    }
    MPNN::annDeallocPts(annpt);
    
    cerr << "KDtree build finished\n";



    // fing nearest points to randomly generated points 
    // and saves then into 'nearest.dat' file. The file can be viewed in gnuplot by "plot 'nearest.dat' u 1:2 w lp" command
    std::cerr << "Searching for nearest neighbor..\n";

    ofstream ofs;
    
    if (writeToFiles) {
        ofs.open("nearest.dat");
    }

    int idx;
    double dann = INFINITY;

    MPNN::ANNpoint query = MPNN::annAllocPt(dimension);
    for(int i=0;i<numPoints;i++) {
        for(int j=0;j<dimension;j++) {
            query[j] = rand()%1000 - rand()%1000;
        }

        // find nearest neighbor to the point 'query'. The result is 'nearest' which is index to 'data'
        int nearest = (int)kdTree->NearestNeighbor(query,idx,dann);

        if (writeToFiles) {
            for(int j=0;j<dimension;j++) {
                ofs << query[j] << " ";
            }
            ofs <<"\n";
            for(int j=0;j<dimension;j++) {
                ofs << data[nearest][j] << " ";
            }
            ofs << "\n\n";
        }
        
        if (! (i % 10000)) {
            cerr << i << " ";
        }
    }
    ofs.close();

    delete kdTree;

}


