#include <filesystem>
#include <iostream>
#include <fstream>
#include <osmium/visitor.hpp>
#include <osmium/io/xml_input.hpp>
#include "read_handler.h"
#include "map_graph.h"

using namespace std;


void print_path(const VertexId from, const VertexId to, const unique_ptr<traversal_path> &path) {
    if (path == nullptr) {
        cout << from << " -> " << to << ": RECIEVED NULLPTR PATH\n";
        return;
    }
    for (const auto [v, e]: *path) {
        cout << v << ", ";
    }
    cout << to << endl;
}

void compare_algorithms(const MapGraph &graph, ifstream points) {
    int same = 0;
    int different = 0;
    int total = 0;
    for (int i = 0;; i++) {
        VertexId from, to;
        points >> from >> to;
        if (points.eof()) {
            cout << "Same: " << same << " out of " << total << endl;
            cout << "Different: " << different << " out of " << total << endl;
            return;
        }
        from = middle_to_end(graph, from, i % 2);
        to = middle_to_end(graph, to, i % 4 > 1);
        // cout << from << " " << to << endl;
        // usleep(100);
        const auto &[d_cost, d_path] = dijkstra(graph, from, to);
        // ReSharper disable once CppTooWideScopeInitStatement
        const auto &[a_cost, a_path] = a_star(graph, from, to);
        if (d_path == nullptr ^ a_path == nullptr) {
            if (d_path == nullptr) {
                cout << "Only Dijkstra failed. A*:\n";
                print_path(from, to, a_path);
            } else {
                cout << "Only A* failed. Dijkstra:\n";
                print_path(from, to, d_path);
            }
        } else if (d_path != nullptr) {
            if (*d_path != *a_path) {
                cout << "Algorithms gave different result. Dijkstra with cost " << d_cost << " and A* " << a_cost <<
                        endl;
                print_path(from, to, d_path);
                print_path(from, to, a_path);
                cout << endl;
                different++;
            } else {
                same++;
            }
            total++;
        }
    }
}

int main(const int argc, char *argv[]) {
    if (argc < 3) {
        cerr << "Not enough arguments";
        return 1;
    }
    const auto osm_file = filesystem::path(argv[1]);
    const auto points_file = filesystem::path(argv[2]);
    osmium::io::Reader reader{osm_file};
    ReadHandler handler{};
    osmium::apply(reader, handler);
    const MapGraph &graph = handler.graph;

    ifstream points(points_file);
    compare_algorithms(graph, points_file);

    // constexpr VertexId from = 820268080;
    // constexpr VertexId to = 178735877;

    // print_path(from, to, dijkstra(graph, from, to).second);
    // print_path(from, to, a_star(graph, from, to).second);

    return 0;
}
