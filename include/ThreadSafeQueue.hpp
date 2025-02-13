#ifndef THREAD_SAFE_QUEUE_HPP
#define THREAD_SAFE_QUEUE_HPP

#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>

template<typename T>
class ThreadSafeQueue {
private:
    std::queue<T> queue;
    std::mutex mutex;
    std::condition_variable cond;
    std::atomic<bool> stop{false};

public:
    void push(const T& value) {
        std::lock_guard<std::mutex> lock(mutex);
        queue.push(value);
        cond.notify_one();
    }

    bool pop(T& value) {
        std::unique_lock<std::mutex> lock(mutex);
        cond.wait(lock, [this] { return !queue.empty() || stop; });
        if (stop && queue.empty()) return false;
        value = queue.front();
        queue.pop();
        return true;
    }

    void stopQueue() {
        std::lock_guard<std::mutex> lock(mutex);
        stop = true;
        cond.notify_all();
    }
};

#endif // THREAD_SAFE_QUEUE_HPP