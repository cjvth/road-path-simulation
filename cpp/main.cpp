#include <filesystem>
#include <iostream>
#include <osmium/visitor.hpp>
#include <osmium/io/xml_input.hpp>
#include "read_handler.h"

using namespace std;


int main(const int argc, char *argv[]) {
    if (argc < 2) {
        cerr << "Not enough arguments";
        return 1;
    }
    const auto osm_file = filesystem::path(argv[1]);
    osmium::io::Reader reader{osm_file};
    ReadHandler handler{};
    osmium::apply(reader, handler);
    const MapGraph &graph = handler.graph;
    dijkstra(graph, 659897210, 964942448);
    // dijkstra(graph, 659897210, 279873701);

    return 0;
}
