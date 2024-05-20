#ifndef READHANDLER_H
#define READHANDLER_H
#include <osmium/handler.hpp>

#include "map_graph.h"


struct ReadHandler : osmium::handler::Handler {
    MapGraph graph{};

    void node(const osmium::Node &n);

    void way(const osmium::Way &w);
};

#endif //READHANDLER_H
