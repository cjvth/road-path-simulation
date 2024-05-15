#include "map_graph.h"

#include <iostream>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <osmium/geom/haversine.hpp>

VertexId middle_to_end(const MapGraph &graph, const VertexId given, const bool need_last) {
    const vector<EdgeId> &edges = graph.vertex_index.at(given);
    if (edges.size() > 1)
        return given;
    const auto &[nodes, cost] = graph.edges.at(edges[0]);
    if (need_last)
        return nodes[nodes.size() - 1];
    return nodes[0];
}

inline double calc_heuristic(const osmium::geom::Coordinates &p1, const osmium::geom::Coordinates &p2) {
    // return osmium::geom::haversine::distance(p1, p2);
    // constexpr double x_degree = 63995;
    constexpr double x_degree = 40075016.7 / 360;
    constexpr double y_degree = 40075016.7 / 360;
    double dx = abs(p1.x - p2.x);
    if (dx > 180)
        dx -= 180;
    double dy = abs(p1.y - p2.y);
    // return sqrt((dx * x_degree) * (dx * x_degree) + (dy * y_degree) * (dy * y_degree));
    return dx * x_degree + dy * y_degree;
}

void bfs(const MapGraph &graph, const VertexId from, const VertexId to) {
    queue<VertexId> to_visit({from});
    unordered_set<VertexId> checked;
    unordered_map<VertexId, pair<VertexId, EdgeId> > came_from;
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
    unordered_map<VertexId, float> path_cost;
    unordered_set<VertexId> checked;
    unordered_map<VertexId, pair<VertexId, EdgeId> > came_from;
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
            // cout << total_ops << " " << to_visit.size() << endl;
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

bool a_star(const MapGraph &graph, const VertexId from, const VertexId to) {
    // cost with estimation, known cost, vertex
    priority_queue<tuple<double, double, VertexId>, vector<tuple<double, double, VertexId> >, greater<> > to_visit;
    unordered_map<VertexId, double> path_cost;
    unordered_set<VertexId> checked;
    unordered_map<VertexId, pair<VertexId, EdgeId> > came_from;
    unordered_map<VertexId, double> heuristics;
    to_visit.emplace(0, 0, from);
    while (!to_visit.empty()) {
        auto [est_cost, cost, cur] = to_visit.top();
        to_visit.pop();
        if (const bool inserted = checked.insert(cur).second; !inserted)
            continue;
        if (cur == to) {
            // cout << "Found de way with cost " << cost << endl;
            // VertexId x = to;
            // while (x != from) {
            //     cout << x << ", ";
            //     x = came_from[x].first;
            // }
            // cout << from << endl;
            // cout << total_ops << " " << to_visit.size() << endl;
            return true;
        }
        {
            const auto &go_from_cur = graph.go_from_vertex.find(cur);
            if (go_from_cur == graph.go_from_vertex.end())
                continue;
            for (const auto &value = go_from_cur->second; EdgeId edge_id: value) {
                auto &[nodes, edge_cost] = graph.edges.at(edge_id);
                if (VertexId next = nodes[0] + nodes[nodes.size() - 1] - cur; !checked.contains(next)) {
                    if (double new_cost = cost + edge_cost; !path_cost.contains(next) || new_cost < path_cost[next]) {
                        if (!heuristics.contains(next))
                            heuristics[next] = calc_heuristic(graph.coords.at(next), graph.coords.at(to));
                        path_cost[next] = new_cost;
                        to_visit.emplace(new_cost + heuristics[next], new_cost, next);
                        came_from[next] = {cur, edge_id};
                    }
                }
            }
        }
    }
    return false;
}
