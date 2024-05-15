#include <filesystem>
#include <iostream>
#include <fstream>
#include <osmium/visitor.hpp>
#include <osmium/io/xml_input.hpp>
#include "read_handler.h"
#include "map_graph.h"

using namespace std;


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

    int success = 0;
    int success_a_star = 0;
    constexpr int total = 10000;
    ifstream points(points_file);
    for (int i = 0; i < total; i++) {
        VertexId from, to;
        points >> from >> to;
        from = middle_to_end(graph, from, i % 2);
        to = middle_to_end(graph, to, i % 4 > 1);
        // cout << from << " " << to << ":\n";
        // if (dijkstra(graph, from, to)) {
        //     success++;
        // }
        if (a_star(graph, from, to)) {
            success_a_star++;
        }
        // cout << "\n\n\n";
    }

    // dijkstra(graph, 681808138, 320138418);
    // a_star(graph, 681808138, 320138418);

    // cout << "Success: " << success << " out of " << total << endl;
    // cout << "Success A*: " << success_a_star << " out of " << total << endl;

    return 0;
}
