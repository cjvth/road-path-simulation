#ifndef READHANDLER_H
#define READHANDLER_H
#include <osmium/handler.hpp>

#include "map_graph.h"


struct ReadHandler : osmium::handler::Handler {
    MapGraph graph{};
    // void node(const osmium::Node& x);
    void way(const osmium::Way& x);
};

#endif //READHANDLER_H
