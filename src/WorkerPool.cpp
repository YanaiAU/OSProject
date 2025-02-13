#include "WorkerPool.hpp"

WorkerPool::WorkerPool(int workers) : numWorkers(workers) {
    for (int i = 0; i < numWorkers; i++) {
        this->workers.emplace_back(&WorkerPool::workerThread, this);
    }
}

WorkerPool::~WorkerPool() {
    stop();
}

void WorkerPool::workerThread() {
    while (running) {
        Task task;
        if (!taskQueue.pop(task)) continue;

        MetricResult result;
        switch (task.type) {
            case MetricResult::TOTAL_WEIGHT:
                result = MetricsCalculator::calculateTotalWeight(task.mstEdges);
            break;
            case MetricResult::LONGEST_DISTANCE:
                result = MetricsCalculator::calculateLongestDistance(task.mstEdges, task.numVertices);
            break;
            case MetricResult::SHORTEST_EDGE:
                result = MetricsCalculator::calculateShortestEdge(task.mstEdges);
            break;
            case MetricResult::AVERAGE_DISTANCE:
                result = MetricsCalculator::calculateAverageDistance(task.mstEdges, task.numVertices);
            break;
        }

        resultQueue.push(result);
    }
}

void WorkerPool::submitTask(const Task& task) {
    taskQueue.push(task);
}

bool WorkerPool::getResult(MetricResult& result) {
    return resultQueue.pop(result);
}

void WorkerPool::stop() {
    running = false;
    taskQueue.stopQueue();
    resultQueue.stopQueue();

    for (auto& worker : workers) {
        if (worker.joinable()) {
            worker.join();
        }
    }
}