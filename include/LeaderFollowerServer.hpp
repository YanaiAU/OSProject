#ifndef LEADER_FOLLOWER_SERVER_HPP
#define LEADER_FOLLOWER_SERVER_HPP

#include "ThreadSafeQueue.hpp"
#include "WorkerPool.hpp"
#include "graph.h"
#include <vector>
#include <thread>
#include <memory>

struct ClientResponse {
    int totalWeight;
    int longestDistance;
    double averageDistance;
    int shortestMSTEdge;
    int numMSTEdges;
    Graph::Edge mstEdges[100];
};

class LeaderFollowerServer {
private:
    ThreadSafeQueue<int> clientQueue;
    std::vector<std::thread> clientHandlers;
    std::vector<std::unique_ptr<WorkerPool>> metricPools;
    std::atomic<bool> running{true};
    int serverSocket;
    const int numHandlers;

    void handleClient(int clientSocket, WorkerPool& metricPool);
    void clientHandlerThread(int handlerId);

public:
    LeaderFollowerServer(int handlers = 3);
    ~LeaderFollowerServer();

    bool start();
    void run();
    void stop();
};

#endif // LEADER_FOLLOWER_SERVER_HPP