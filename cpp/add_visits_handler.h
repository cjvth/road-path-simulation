#ifndef ADD_USAGE_HANDLER_H
#define ADD_USAGE_HANDLER_H

#include <osmium/handler.hpp>
#include <osmium/memory/buffer.hpp>
#include <osmium/io/writer.hpp>

#include "map_graph.h"

struct AddVisitsHandler : osmium::handler::Handler {
    osmium::io::Writer &writer;
    const unordered_map<VertexId, int> &vertex_visits;
    const unordered_map<EdgeId, int> &edge_visits;

    void node(const osmium::Node &n) const;

    void way(const osmium::Way &w) const;

    AddVisitsHandler(osmium::io::Writer &writer, const unordered_map<VertexId, int> &vertex_visits,
                     const unordered_map<EdgeId, int> &edge_visits);
};

#endif //ADD_USAGE_HANDLER_H
