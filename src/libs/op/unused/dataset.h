typedef struct HeapPoint2D {
     double x;
     double y;
     HeapPoint2D() {
         this->x = NAN;
         this->y = NAN;
     }

     HeapPoint2D(double x_, double y_) {
         this->x = x_;
         this->y = y_;
     }

     std::vector<double> toVector() {
         std::vector<double> vector;
         vector.push_back(x);
         vector.push_back(y);
         return vector;
     }
     double distance(HeapPoint2D other) {
         double diffx = this->x - other.x;
         double diffy = this->y - other.y;
         return sqrt(diffx * diffx + diffy * diffy);
     }

     HeapPoint2D getStateInDistance(HeapPoint2D to, double in_distance) {
         double mutual_distance = this->distance(to);
         double x_ = this->x + (to.x - this->x) * (in_distance / mutual_distance);
         double y_ = this->y + (to.y - this->y) * (in_distance / mutual_distance);
         return HeapPoint2D(x_, y_);
     }
} HeapPoint2D;


template<typename T>
struct DatasetOBST_OP {
     std::vector<GraphNode<T>> graph;
     double Tmax; //= available time budget per path
     unsigned int P; //= number of paths (=1)
     unsigned int startID;
     unsigned int goalID;
     std::vector<MapPoint> map_points;
     std::vector<Obstacle> obstacles;
     std::vector<Obstacle> convex_regions;
     std::vector<Visibility> visibility_lists;
     std::vector<Visibility> city2map_visibility_lists;
     std::vector<Visibility> map_visibility_lists;
     std::vector<Visibility> city2city_visibility_lists;
     std::vector<int> border;
     std::string name;
};


template<typename T>
struct DatasetOBST_OP_MeshObjects {
     double Tmax; //= available time budget per path
     unsigned int P; //= number of paths (=1)
     unsigned int startID;
     unsigned int goalID;
     std::string name;
     std::vector<MeshObject*> mesh_objects;
     std::map<std::string,double> target_reward_map;
};


template<typename T>
struct GraphNode {

     GraphNode() {
         T data;
         reward = 0;
         id = 0;
         cluster_id = -1;
     }

     GraphNode(T data_, double price_, unsigned int id_, int cluster_id_) {
         data = data_;
         reward = price_;
         id = id_;
         cluster_id = cluster_id_;
     }

     std::vector<double> toVector() {
         std::vector<double> vector= data.toVector();
         vector.push_back(reward);
         return vector;
     }

     double distanceTo(GraphNode gn1) {
         return this->data.distance(gn1.data);
     }

     T getStateInDistance(GraphNode node_to,double actualDistance){
         return this->data.getStateInDistance(node_to.data,actualDistance);
     }

     T data;
     double reward;
     unsigned int id;
     int cluster_id;
};

typedef struct MapPoint {
     double x;
     double y;
     unsigned int id;

     Point toPoint(){
         return Point(this->x,this->y);
     }

     Coords toCoords(){
         return Coords(this->x,this->y);
     }

} MapPoint;

typedef struct Obstacle {
     std::vector<int> point_indexes;
} Obstacle;

typedef struct Visibility {
     std::vector<int> point_indexes;
     VisibilityVertexType visibility_type;
} Visibility;

enum VisibilityVertexType {
     city2map_visibility_vertex, map_visibility_vertex, 
city2city_visibility_vertex
};


