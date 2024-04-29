add_executable(paths
        main.cpp
        read_handler.cpp
        read_handler.h
        map_graph.cpp
        map_graph.h)
target_link_libraries(paths expat)
