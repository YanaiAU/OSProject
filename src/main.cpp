#include "LeaderFollowerServer.hpp"
#include <thread>
#include <iostream>

int main() {
    LeaderFollowerServer server;
    
    if (!server.start()) {
        return 1;
    }

    // Handle graceful shutdown
    std::thread([&server]() {
        char input;
        std::cin >> input;
        server.stop();
    }).detach();

    server.run();
    return 0;
}