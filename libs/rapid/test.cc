
#include "RAPID.H"
#include <iostream>
#include <fstream>
#include <sstream>
#include <math.h>
#include <string>
#include <stdlib.h>
#include <vector>


using namespace std;

template<typename T>
std::vector<T> lineToNum(const std::string &line) {

	std::vector<T> res;
	std::stringstream iss(line);
	T a;
	while(iss >> a) {
		res.push_back(a);
	}
	return res;
}

void loadRawTriangls(const char *filename, vector< vector<double> > &pts) {
    ifstream ifs(filename);
    pts.clear();
    while (ifs) {
        string line;
        getline(ifs, line);
        vector<double> tmp( lineToNum<double>(line) );
        if (tmp.size() == 9) {
            pts.push_back(tmp);
        }
    }
    ifs.close();
    cerr << "Loaded " << pts.size() << " triangles from " << filename << "\n";
}

void addTriangles(RAPID_model *m, const vector< vector<double> > &pts) {

    double p1[3];
    double p2[3];
    double p3[3];
    for(int i=0;i<(int)pts.size();i++) {
        p1[0] = pts[i][0]; p1[1] = pts[i][1]; p1[2] = pts[i][2];
        p2[0] = pts[i][3]; p2[1] = pts[i][4]; p2[2] = pts[i][5];
        p3[0] = pts[i][6]; p3[1] = pts[i][7]; p3[2] = pts[i][8];
        m->AddTri(p1,p2,p3, i);
    }

}


int main(int argc, char **argv) {

    if (argc != 4) {
        cerr << "usage: " << argv[0] << " <numtests>  <infile.raw> <infile.raw>\n";
        cerr << "Supply two .raw files with 3D triangles to test random collision detection \n\n";
        exit(0);
    }
    const int numTests = atoi(argv[1]);
    const char *raw1 = argv[2];
    const char *raw2 = argv[3];


    RAPID_model *m1 = new RAPID_model();
    m1->BeginModel();
    vector< vector<double> > pts;
    loadRawTriangls(raw1, pts);
    addTriangles(m1, pts);
    m1->EndModel();
    pts.clear();
 
 
    RAPID_model *m2 = new RAPID_model();
    m2->BeginModel();
    loadRawTriangls(raw2, pts);
    addTriangles(m2, pts);
    m2->EndModel();

    double R1[3][3], R2[3][3], T1[3], T2[3];

    R1[0][0] = R1[1][1] = R1[2][2] = 1.0;
    R1[0][1] = R1[1][0] = R1[2][0] = 0.0;
    R1[0][2] = R1[1][2] = R1[2][1] = 0.0;

    R2[0][0] = R2[1][1] = R2[2][2] = 1.0;
    R2[0][1] = R2[1][0] = R2[2][0] = 0.0;
    R2[0][2] = R2[1][2] = R2[2][1] = 0.0;

    T1[0] = 0.0;  T1[1] = 0.0; T1[2] = 0.0;
    T2[0] = 0.0;  T2[1] = 0.0; T2[2] = 0.0;


    srand(123);

    const double minval = -5;
    const double maxval = 5;

    for(int i = 0; i < numTests; i++) {
        const double t1 = 1.0*rand() / RAND_MAX;
        const double t2 = 1.0*rand() / RAND_MAX;
        const double t3 = 1.0*rand() / RAND_MAX;
        const double x = (maxval - minval)*t1 +  minval;
        const double y = (maxval - minval)*t2 +  minval;
        const double z = (maxval - minval)*t3 +  minval;
        T2[0] = x;
        T2[1] = y;
        T2[2] = z;

//        Collision_info info;
        RAPID_Collide(R1, T1, m1, R2, T2, m2, RAPID_ALL_CONTACTS);
        std::cout << i << " " << x << " " << y << " " << z << " ";
        std::cout << RAPID_num_contacts << "\n";
 
    }
    


    // Now we can perform a collision query:

   

}
