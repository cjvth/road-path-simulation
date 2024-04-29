#include "read_handler.h"

#include <osmium/osm/way.hpp>

// void ReadHandler::node(const osmium::Node& x) {
//
// }

void ReadHandler::way(const osmium::Way &x) {
    const EdgeId id = x.id();
    // graph.edges[id].id = id;
    for (auto a: x.nodes()) {
        graph.edges[id].nodes.push_back(a.ref());
        graph.vertex_index[a.ref()].push_back(id);
    }
    graph.go_from_vertex[x.nodes()[0].ref()].push_back(id);
    if (!x.tags()["oneway"] && !x.tags()["roundabout"])
    // if constexpr (true)
        graph.go_from_vertex[x.nodes()[x.nodes().size() - 1].ref()].push_back(id);
    graph.edges[id].cost = stof(x.tags().get_value_by_key("cost"));
}
