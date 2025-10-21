#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <ctime>
#include <iomanip>
#include <sstream>

int main() {
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) return 1;

    sockaddr_in server{};
    server.sin_family = AF_INET;
    server.sin_port = htons(7070);
    inet_pton(AF_INET, "127.0.0.1", &server.sin_addr);

    if (connect(sock, (struct sockaddr*)&server, sizeof(server)) < 0) {
        std::cerr << "Connection failed\n";
        return 1;
    }

    for (int i = 0; i < 100; ++i) {
        std::stringstream ss;
        std::time_t t = std::time(nullptr);
        ss << "{ \"time\": \"" << std::put_time(std::localtime(&t), "%H:%M:%S") 
           << "\", \"value\": " << (rand() % 100) << " }\n";

        std::string msg = ss.str();
        send(sock, msg.c_str(), msg.size(), 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    close(sock);
    return 0;
}