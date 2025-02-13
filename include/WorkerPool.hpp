#ifndef WORKER_POOL_HPP
#define WORKER_POOL_HPP

#include "ThreadSafeQueue.hpp"
#include "MetricsCalculator.hpp"
#include <vector>
#include <thread>
#include <memory>

struct Task {
    MetricResult::Type type;
    std::vector<std::tuple<int, int, int, int>> mstEdges;
    int numVertices;
};

class WorkerPool {
private:
    ThreadSafeQueue<Task> taskQueue;
    ThreadSafeQueue<MetricResult> resultQueue;
    std::vector<std::thread> workers;
    std::atomic<bool> running{true};
    const int numWorkers;

    void workerThread();

public:
    WorkerPool(int workers = 4);
    ~WorkerPool();

    void submitTask(const Task& task);
    bool getResult(MetricResult& result);
    void stop();
};

#endif // WORKER_POOL_HPP