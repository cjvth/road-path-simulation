#include "map_graph.h"

#include <iostream>
#include <set>

VertexId middle_to_end(const MapGraph &graph, const VertexId given, const bool need_last) {
    const vector<EdgeId> &edges = graph.vertex_index.at(given);
    if (edges.size() > 1)
        return given;
    const auto &[nodes, cost] = graph.edges.at(edges[0]);
    if (need_last)
        return nodes[nodes.size() - 1];
    return nodes[0];
}

void bfs(const MapGraph &graph, const VertexId from, const VertexId to) {
    queue<VertexId> to_visit({from});
    set<VertexId> checked;
    map<VertexId, pair<VertexId, EdgeId> > came_from;
    while (!to_visit.empty()) {
        VertexId cur = to_visit.front();
        if (cur == to) {
            cout << "Found de way\n";
            VertexId x = to;
            do {
                const VertexId prev = came_from[x].first;
                cout << prev << "\n";
                x = prev;
            } while (x != from);
            return;
        }
        for (EdgeId edge: graph.go_from_vertex.at(cur)) {
            const vector<VertexId> &nodes = graph.edges.at(edge).nodes;
            if (VertexId next = nodes[0] + nodes[nodes.size() - 1] - cur; !checked.contains(next)) {
                to_visit.push(next);
                checked.insert(next);
                came_from[next] = {cur, edge};
            }
        }
        to_visit.pop();
    }
}

bool dijkstra(const MapGraph &graph, const VertexId from, const VertexId to) {
    priority_queue<pair<float, VertexId>, vector<pair<float, VertexId> >, greater<> > to_visit;
    map<VertexId, float> path_cost;
    set<VertexId> checked;
    map<VertexId, pair<VertexId, EdgeId> > came_from;
    to_visit.emplace(0, from);
    while (!to_visit.empty()) {
        auto [cost, cur] = to_visit.top();
        to_visit.pop();
        if (checked.contains(cur))
            continue;
        checked.insert(cur);
        if (cur == to) {
            // cout << "Found de way with cost " << cost << endl;
            // VertexId x = to;
            // while (x != from) {
            //     cout << x << ", ";
            //     x = came_from[x].first;
            // }
            // cout << from << endl;
            return true;
        }
        if (!graph.go_from_vertex.contains(cur))
            continue;
        for (EdgeId edge_id: graph.go_from_vertex.at(cur)) {
            auto &[nodes, edge_cost] = graph.edges.at(edge_id);
            if (VertexId next = nodes[0] + nodes[nodes.size() - 1] - cur; !checked.contains(next)) {
                if (float new_cost = cost + edge_cost; !path_cost.contains(next) || new_cost < path_cost[next]) {
                    path_cost[next] = new_cost;
                    to_visit.emplace(new_cost, next);
                    came_from[next] = {cur, edge_id};
                }
            }
        }
    }
    return false;
}
