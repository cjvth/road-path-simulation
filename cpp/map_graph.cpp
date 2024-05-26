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


const unordered_set<EdgeId> *get_go_from_vertex(const MapGraph &graph, const VertexId at, const EdgeId from) {
    if (const auto special_v = graph.allowed_go_from_vertex.find(at);
        special_v != graph.allowed_go_from_vertex.end()) {
        if (const auto go_to = special_v->second.find(from); go_to != special_v->second.end()) {
            return &go_to->second;
        }
    }
    if (const auto go_to = graph.go_from_vertex.find(at); go_to != graph.go_from_vertex.end()) {
        return &go_to->second;
    }
    return nullptr;
}


constexpr double default_x_degree = 40075016.7 / 360;
constexpr double default_y_degree = 40075016.7 / 360;
constexpr double y_diff_error = 0.0001;

struct HeuristicState {
    double x_degree{};
    double y_degree = default_y_degree;
    double max_latitude{};
    double x_coeff{};

    explicit HeuristicState(const osmium::geom::Coordinates &to) {
        update_x_degree(to);
    }

    void update_x_degree(const osmium::geom::Coordinates &c) {
        max_latitude = min(90., c.y + y_diff_error);
        x_coeff = cos(osmium::geom::deg_to_rad(max_latitude));
        x_degree = default_x_degree * x_coeff;
    }
};

inline double calc_heuristic(const osmium::geom::Coordinates &prob, const osmium::geom::Coordinates &dest,
                             HeuristicState &h_state) {
    // return osmium::geom::haversine::distance(prob, dest);
    // constexpr double x_degree = 63995;

    if (abs(prob.y) - h_state.max_latitude > y_diff_error) {
        h_state.update_x_degree(prob);
    }
    double dx = abs(prob.x - dest.x);
    if (dx > 180)
        dx -= 180;
    double dy = abs(prob.y - dest.y);
    return sqrt(dx * h_state.x_degree * (dx * h_state.x_degree) +
                dy * h_state.y_degree * (dy * h_state.y_degree));
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


pair<double, unique_ptr<traversal_path> > dijkstra(const MapGraph &graph, const VertexId from, const VertexId to) {
    if (from == to) {
        return {0, make_unique<traversal_path>(0)};
    }
    priority_queue<pair<float, VertexId>, vector<pair<float, VertexId> >, greater<> > to_visit;
    unordered_map<VertexId, float> path_cost;
    unordered_set<VertexId> checked;
    unordered_map<VertexId, pair<VertexId, EdgeId> > came_from;
    came_from[from] = {-1, -1};
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
            return {cost, get_path(from, to, n_steps.at(to), came_from)};
        }
        if (!graph.go_from_vertex.contains(cur))
            continue;
        if (const auto go_from_vertex = get_go_from_vertex(graph, cur, came_from.at(cur).second);
            go_from_vertex != nullptr) {
            const int new_n_steps = n_steps.at(cur) + 1;
            for (EdgeId edge_id: *go_from_vertex) {
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
    }
    return {0, nullptr};
}

pair<double, unique_ptr<traversal_path> > a_star(const MapGraph &graph, const VertexId from, const VertexId to) {
    if (from == to) {
        return {0, make_unique<traversal_path>(0)};
    }
    // cost with estimation, known cost, vertex
    priority_queue<tuple<double, double, VertexId>, vector<tuple<double, double, VertexId> >, greater<> > to_visit;
    unordered_map<VertexId, double> path_cost;
    unordered_set<VertexId> checked;
    unordered_map<VertexId, pair<VertexId, EdgeId> > came_from;
    came_from[from] = {-1, -1};
    unordered_map<VertexId, double> heuristics;
    unordered_map<VertexId, int> n_steps;
    auto h_state = HeuristicState(graph.coords.at(to));
    to_visit.emplace(0, 0, from);
    n_steps[from] = 0;
    while (!to_visit.empty()) {
        auto [est_cost, cost, cur] = to_visit.top();
        to_visit.pop();
        if (const bool inserted = checked.insert(cur).second; !inserted)
            continue;
        if (cur == to) {
            return {cost, get_path(from, to, n_steps.at(to), came_from)};
        }
        if (const auto go_from_vertex = get_go_from_vertex(graph, cur, came_from.at(cur).second);
            go_from_vertex != nullptr) {
            const int new_n_steps = n_steps.at(cur) + 1;
            for (EdgeId edge_id: *go_from_vertex) {
                auto &[nodes, edge_cost] = graph.edges.at(edge_id);
                if (VertexId next = nodes[0] + nodes[nodes.size() - 1] - cur; !checked.contains(next)) {
                    if (double new_cost = cost + edge_cost;
                        !path_cost.contains(next) || new_cost < path_cost[next]) {
                        if (!heuristics.contains(next))
                            heuristics[next] = calc_heuristic(graph.coords.at(next), graph.coords.at(to), h_state);
                        path_cost[next] = new_cost;
                        to_visit.emplace(new_cost + heuristics[next], new_cost, next);
                        came_from[next] = {cur, edge_id};
                        n_steps[next] = new_n_steps;
                    }
                }
            }
        }
    }
    return {0, nullptr};
}
