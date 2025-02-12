cmake_minimum_required(VERSION 3.28)
project(OSProject)

set(CMAKE_CXX_STANDARD 17)

add_executable(OSProject main.cpp
        Graph.cpp
        Graph.h
        Tree.cpp
        Tree.h
        mst_algorithm.cpp
        mst_algorithm.h
        server.cpp
        client.cpp
        prim_algorithm.cpp
        prim_algorithm.h
        kruskal_algorithm.cpp
        kruskal_algorithm.h
        mst_algorithm_factory.cpp
        mst_algorithm_factory.h
        main.cpp
        union_find.cpp
        union_find.h
        leader_follower_server.cpp
)
