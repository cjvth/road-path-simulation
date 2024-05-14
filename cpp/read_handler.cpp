#include "read_handler.h"

#include <osmium/osm/node.hpp>
#include <osmium/osm/way.hpp>

// void ReadHandler::node(const osmium::Node &n) {
//
// }

void ReadHandler::way(const osmium::Way &w) {
    const EdgeId id = w.id();
    // graph.edges[id].id = id;
    for (auto n: w.nodes()) {
        graph.edges[id].nodes.push_back(n.ref());
        graph.vertex_index[n.ref()].push_back(id);
    }
    graph.go_from_vertex[w.nodes()[0].ref()].push_back(id);
    if (!(w.tags().has_tag("oneway", "yes") ||
          w.tags().has_tag("junction", "roundabout") ||
          w.tags().has_tag("junction", "circular") && !w.tags().has_tag("oneway", "no")))
        graph.go_from_vertex[w.nodes()[w.nodes().size() - 1].ref()].push_back(id);
    graph.edges[id].cost = stof(w.tags().get_value_by_key("cost"));
}
