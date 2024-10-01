#ifndef MESH_2D_H__
#define MESH_2D_H__

#include "types.h"
#include "ut.h"
#include "ANN.h"
#include "multiann.h"
#include <vector>

namespace rrtPlanning {


class Mesh {

    public:

        enum TMesh {
            TYPE_MESH = 0,
            TYPE_VD,
            TYPE_NONE
        };

    Mesh();

    ~Mesh();

    void loadMeshTriangles(const char *filename);
    void loadVoronoiDiagram(const char *filename);

    TMesh getMeshType() const { return type; } 

    std::vector< std::vector< TPoint > > triangles;
    std::vector< TPoint > pts;
    std::vector< SEdge > edges;
    std::vector<int> findPath(const TPoint &from, const TPoint &to);
    std::vector<int> findPath(const int fromIdx, const int toIdx);

    std::vector<TPoint> intPathToPts(const std::vector<int> &path) const;

    const std::vector< std::vector< TPoint > > &getTriangles() const { return triangles; }
    const std::vector< SEdge > &getEdges() const { return edges; }
    const TPoint getPoint(const int idx) const { return pts[idx]; }

    void removePts(const std::vector<int> &idx);

    private:
    void clear();
    void makeGraph();

//    double **graph;
	MPNN::MultiANN<int> *kdTree;
    TMesh type;

//    void clearGraph();

};


} // namespace


#endif
