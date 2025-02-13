cmake_minimum_required(VERSION 3.28)
project(OSProject)

set(CMAKE_CXX_STANDARD 17)

add_executable(OSProject
        src/graph.cpp
        include/graph.h
        src/tree.cpp
        include/tree.h
        include/mst_algorithm.h
        src/client.cpp
        src/prim_algorithm.cpp
        include/prim_algorithm.h
        src/kruskal_algorithm.cpp
        include/kruskal_algorithm.h
        src/mst_algorithm_factory.cpp
        include/mst_algorithm_factory.h
        src/union_find.cpp
        include/union_find.h
        include/graph_visualizer.h
        include/ThreadSafeQueue.hpp
        include/MetricsCalculator.hpp
        include/WorkerPool.hpp
        include/LeaderFollowerServer.hpp
        src/main.cpp
        src/MetricsCalculator.cpp
        src/WorkerPool.cpp
        src/LeaderFollowerServer.cpp
)
