cmake_minimum_required(VERSION 3.28)
project(OSProject)

set(CMAKE_CXX_STANDARD 17)

add_executable(OSProject main.cpp
        Graph.cpp
        include/graph.h
        Tree.cpp
        include/tree.h
        mst_algorithm.cpp
        include/mst_algorithm.h
        server.cpp
        client.cpp
        prim_algorithm.cpp
        include/prim_algorithm.h
        kruskal_algorithm.cpp
        include/kruskal_algorithm.h
        mst_algorithm_factory.cpp
        include/mst_algorithm_factory.h
        main.cpp
        union_find.cpp
        include/union_find.h
        leader_follower_server.cpp
        graph_visualizer.h
        include/ThreadSafeQueue.hpp
        include/MetricsCalculator.hpp
        include/WorkerPool.hpp
        include/LeaderFollowerServer.hpp
        src/main.cpp
        src/MetricsCalculator.cpp
        src/WorkerPool.cpp
        src/LeaderFollowerServer.cpp
)
