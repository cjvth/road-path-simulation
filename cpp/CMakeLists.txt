add_executable(paths
        main.cpp
        read_handler.cpp
        read_handler.h
        map_graph.cpp
        map_graph.h
        add_visits_handler.cpp
        add_visits_handler.h
)
target_link_libraries(paths expat)
