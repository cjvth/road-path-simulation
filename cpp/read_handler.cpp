#include "read_handler.h"

#include <iostream>
#include <cstring>
#include <unordered_set>
#include <osmium/osm/node.hpp>
#include <osmium/osm/relation.hpp>
#include <osmium/osm/way.hpp>

void ReadHandler::node(const osmium::Node &n) {
    graph.coords[n.id()] = n.location();
}

void ReadHandler::way(const osmium::Way &w) {
    const EdgeId id = w.id();
    // graph.edges[id].id = id;
    for (auto n: w.nodes()) {
        graph.edges[id].nodes.push_back(n.ref());
        graph.vertex_index[n.ref()].push_back(id);
    }
    graph.go_from_vertex[w.nodes()[0].ref()].insert(id);
    if (!(w.tags().has_tag("oneway", "yes") ||
          w.tags().has_tag("junction", "roundabout") ||
          w.tags().has_tag("junction", "circular") && !w.tags().has_tag("oneway", "no")))
        graph.go_from_vertex[w.nodes()[w.nodes().size() - 1].ref()].insert(id);
    graph.edges[id].cost = stof(w.tags().get_value_by_key("cost"));
}

void ReadHandler::relation(const osmium::Relation &r) {
    if (!r.tags().has_tag("type", "restriction"))
        return;
    const char *restriction = r.tags().get_value_by_key("restriction");
    if (restriction == nullptr) {
        cerr << "Skipping bad restriction " << r.id() << endl;
        return;
    }
    EdgeId from = 0;
    EdgeId to = 0;
    VertexId via = 0;
    for (auto &i: r.cmembers()) {
        if (const auto s = i.role(); strcmp(s, "from") == 0)
            from = i.ref();
        else if (strcmp(s, "to") == 0)
            to = i.ref();
        else if (strcmp(s, "via") == 0)
            via = i.ref();
        else {
            cerr << "Skipping restriction " << r.id() << " with bad member " << i.type() << endl;
            return;
        }
    }
    if (strncmp(restriction, "no_", 3) == 0) {
        auto &via_place = graph.allowed_go_from_vertex[via];
        unordered_set<EdgeId> *via_from_set;
        if (const auto via_from_place = via_place.find(from);
            via_from_place == via_place.end()) {
            via_from_set = &via_place.insert({from, graph.go_from_vertex.at(via)}).first->second;
        } else {
            via_from_set = &via_from_place->second;
        }
        via_from_set->erase(to);
    } else if (strncmp(restriction, "only_", 5) == 0) {
        auto &via_place = graph.allowed_go_from_vertex[via];
        if (const auto via_from_place = via_place.find(from);
            via_from_place == via_place.end()) {
            via_place[from] = {to};
        } else {
            if (!via_from_place->second.contains(to)) {
                cerr << "Has conflicting restriction " << r.id() << endl;
            } else {
#ifndef NDEBUG
                if (r.id() != 2603626)
                    cerr << "Restriction may be redundant: " << r.id() << endl;
#endif
                via_from_place->second.clear();
                via_from_place->second.insert(to);
            }
        }
    } else {
        cerr << "Unknown restriction type " << restriction << " in " << r.id() << endl;
    }
}
