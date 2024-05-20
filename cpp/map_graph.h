#ifndef MAP_GRAPH_H
#define MAP_GRAPH_H
#include <memory>
#include <osmium/geom/coordinates.hpp>

using namespace std;

typedef size_t VertexId;
typedef size_t EdgeId;
typedef vector<pair<VertexId, EdgeId> > traversal_path;


struct Edge {
    // EdgeId id;
    vector<VertexId> nodes;
    float cost{};
};

struct MapGraph {
    unordered_map<VertexId, osmium::geom::Coordinates> coords;
    unordered_map<EdgeId, Edge> edges;
    unordered_map<VertexId, vector<EdgeId> > go_from_vertex;
    unordered_map<VertexId, vector<EdgeId> > vertex_index;
};


VertexId middle_to_end(const MapGraph &graph, VertexId given, bool need_last);

void bfs(const MapGraph &graph, VertexId from, VertexId to);

unique_ptr<traversal_path> dijkstra(const MapGraph &graph, VertexId from, VertexId to);

unique_ptr<traversal_path> a_star(const MapGraph &graph, VertexId from, VertexId to);

#endif //MAP_GRAPH_H
