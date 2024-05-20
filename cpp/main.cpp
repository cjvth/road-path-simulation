#include <filesystem>
#include <iostream>
#include <fstream>
#include <osmium/visitor.hpp>
#include <osmium/io/xml_input.hpp>
#include "read_handler.h"
#include "map_graph.h"

using namespace std;


void print_path(const VertexId from, const VertexId to, const traversal_path &path) {
    for (const auto [v, e]: path) {
        cout << v << ", ";
    }
    cout << to << endl;
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

    int success_dijkstra = 0;
    int success_a_star = 0;
    constexpr int total = 10000;
    ifstream points(points_file);
    for (int i = 0; i < total; i++) {
        VertexId from, to;
        points >> from >> to;
        from = middle_to_end(graph, from, i % 2);
        to = middle_to_end(graph, to, i % 4 > 1);
        cout << from << " -> " << to << ":\n";
        // if (const auto path = dijkstra(graph, from, to)) {
        //     print_path(from, to, *path);
        //     success_dijkstra++;
        // }
        if (const auto path = a_star(graph, from, to)) {
            print_path(from, to, *path);
            success_a_star++;
        }
        cout << "\n";
    }

    // dijkstra(graph, 681808138, 320138418);
    // a_star(graph, 681808138, 320138418);

    // cout << "Success: " << success << " out of " << total << endl;
    // cout << "Success A*: " << success_a_star << " out of " << total << endl;

    return 0;
}
