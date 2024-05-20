#include "add_visits_handler.h"
#include <osmium/builder/attr.hpp>

AddVisitsHandler::AddVisitsHandler(osmium::io::Writer &writer, const unordered_map<VertexId, int> &vertex_visits,
                                   const unordered_map<EdgeId, int> &edge_visits): writer(writer),
    vertex_visits(vertex_visits),
    edge_visits(edge_visits) {
}

void AddVisitsHandler::node(const osmium::Node &n) const {
    using namespace osmium::builder::attr;
    osmium::memory::Buffer buffer{1000, osmium::memory::Buffer::auto_grow::yes};
    int visits = 0;
    if (const auto pos = vertex_visits.find(n.id()); pos != vertex_visits.end()) {
        visits = pos->second;
    }
    osmium::builder::add_node(buffer,
                              _id(n.id()),
                              _location(n.location()),
                              _tags(n.tags()),
                              _tag("visits", to_string(visits))
    );
    writer(move(buffer));
}

void AddVisitsHandler::way(const osmium::Way &w) const {
    using namespace osmium::builder::attr;
    osmium::memory::Buffer buffer{1000, osmium::memory::Buffer::auto_grow::yes};
    int visits = 0;
    if (const auto pos = edge_visits.find(w.id()); pos != edge_visits.end()) {
        visits = pos->second;
    }
    osmium::builder::add_way(buffer,
                             _id(w.id()),
                             _nodes(w.nodes()),
                             _tags(w.tags()),
                             _tag("visits", to_string(visits))
    );
    writer(move(buffer));
}
