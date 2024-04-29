#ifndef MAP_GRAPH_H
#define MAP_GRAPH_H
#include <vector>
#include <map>
#include <queue>

using namespace std;

typedef size_t VertexId;
typedef size_t EdgeId;
struct Edge {
    // EdgeId id;
    vector<VertexId> nodes;
    float cost;
};

struct MapGraph {
    map<EdgeId, Edge> edges;
    map<VertexId, vector<EdgeId>> go_from_vertex;
    map<VertexId, vector<EdgeId>> vertex_index;

};

void bfs(const MapGraph &graph, VertexId from, VertexId to);

void dijkstra(const MapGraph &graph, VertexId from, VertexId to);

#endif //MAP_GRAPH_H
