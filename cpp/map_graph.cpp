#include "map_graph.h"

#include <iostream>
#include <memory>
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


unique_ptr<traversal_path> get_path(const VertexId from, const VertexId to, int n_steps,
                                    const unordered_map<VertexId, pair<VertexId, EdgeId> > &came_from) {
    // returns {{from, edge1}, {v1, edge2}, ..., {vN-1, edgeN}} but not `to` vertex
    auto path = make_unique<vector<pair<VertexId, EdgeId> > >(n_steps);
    VertexId x = to;
    do {
        n_steps--;
        auto &mapped = came_from.at(x);
        (*path)[n_steps] = mapped;
        x = mapped.first;
    } while (x != from);
    return path;
}


unique_ptr<traversal_path> dijkstra(const MapGraph &graph, const VertexId from, const VertexId to) {
    priority_queue<pair<float, VertexId>, vector<pair<float, VertexId> >, greater<> > to_visit;
    unordered_map<VertexId, float> path_cost;
    unordered_set<VertexId> checked;
    unordered_map<VertexId, pair<VertexId, EdgeId> > came_from;
    unordered_map<VertexId, int> n_steps;
    to_visit.emplace(0, from);
    n_steps[from] = 0;
    while (!to_visit.empty()) {
        auto [cost, cur] = to_visit.top();
        to_visit.pop();
        if (checked.contains(cur))
            continue;
        checked.insert(cur);
        if (cur == to) {
            return get_path(from, to, n_steps.at(to), came_from);
        }
        if (!graph.go_from_vertex.contains(cur))
            continue;
        const int new_n_steps = n_steps.at(cur) + 1;
        for (EdgeId edge_id: graph.go_from_vertex.at(cur)) {
            auto &[nodes, edge_cost] = graph.edges.at(edge_id);
            if (VertexId next = nodes[0] + nodes[nodes.size() - 1] - cur; !checked.contains(next)) {
                if (float new_cost = cost + edge_cost; !path_cost.contains(next) || new_cost < path_cost[next]) {
                    path_cost[next] = new_cost;
                    to_visit.emplace(new_cost, next);
                    came_from[next] = {cur, edge_id};
                    n_steps[next] = new_n_steps;
                }
            }
        }
    }
    return nullptr;
}

unique_ptr<traversal_path> a_star(const MapGraph &graph, const VertexId from, const VertexId to) {
    // cost with estimation, known cost, vertex
    priority_queue<tuple<double, double, VertexId>, vector<tuple<double, double, VertexId> >, greater<> > to_visit;
    unordered_map<VertexId, double> path_cost;
    unordered_set<VertexId> checked;
    unordered_map<VertexId, pair<VertexId, EdgeId> > came_from;
    unordered_map<VertexId, double> heuristics;
    unordered_map<VertexId, int> n_steps;
    to_visit.emplace(0, 0, from);
    n_steps[from] = 0;
    while (!to_visit.empty()) {
        auto [est_cost, cost, cur] = to_visit.top();
        to_visit.pop();
        if (const bool inserted = checked.insert(cur).second; !inserted)
            continue;
        if (cur == to) {
            return get_path(from, to, n_steps.at(to), came_from);
        } {
            const int new_n_steps = n_steps.at(cur) + 1;
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
                        n_steps[next] = new_n_steps;
                    }
                }
            }
        }
    }
    return nullptr;
}
