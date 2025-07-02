// ConfigSynchronizer.cpp
#include "config_synchronizer.h"
#include "config.h" // for g_config and loadConfig
#include <iostream>
#include <string>
#include <map>
#include <thread>
#include <mutex>
#include <vector>
#include <sstream>
#include <fstream>
#include <atomic>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <cstring>
#include <signal.h>

// Helper function to trim from both ends of a string
static std::string trim(const std::string& str) {
    size_t first = str.find_first_not_of(" \t\n\r\f\v");
    if (std::string::npos == first) {
        return str;
    }
    size_t last = str.find_last_not_of(" \t\n\r\f\v");
    return str.substr(first, (last - first + 1));
}

// Global map to hold config data for the synchronizer
static std::map<std::string, std::map<std::string, std::string>> g_sync_config_data;
static std::mutex g_config_mutex;

ConfigSynchronizer::ConfigSynchronizer(const std::string& config_path)
    : m_config_path(config_path), m_shutdown_flag(false) {}

ConfigSynchronizer::~ConfigSynchronizer() {
    stop();
}

void ConfigSynchronizer::start() {
    m_thread = std::thread(&ConfigSynchronizer::run, this);
}

void ConfigSynchronizer::stop() {
    m_shutdown_flag.store(true);
    if (m_thread.joinable()) {
        m_thread.join();
    }
}

void ConfigSynchronizer::run() {
    std::cout << "ConfigSynchronizer thread started." << std::endl;
    if (!load_config()) {
        std::cerr << "Failed to load config for synchronizer." << std::endl;
        return;
    }

    send_config_to_wpf();
    receive_config_updates();

    std::cout << "ConfigSynchronizer thread finished." << std::endl;
}

bool ConfigSynchronizer::load_config() {
    std::ifstream file(m_config_path);
    if (!file.is_open()) {
        std::cerr << "Error: Cannot open config file '" << m_config_path << "'" << std::endl;
        return false;
    }

    std::lock_guard<std::mutex> lock(g_config_mutex);
    g_sync_config_data.clear();
    std::string current_section;
    std::string line;

    while (std::getline(file, line)) {
        line = trim(line);
        if (line.empty() || line[0] == '#' || line[0] == ';') {
            continue;
        }
        if (line[0] == '[' && line.back() == ']') {
            current_section = line.substr(1, line.length() - 2);
        } else {
            size_t eq_pos = line.find('=');
            if (eq_pos != std::string::npos) {
                std::string key = trim(line.substr(0, eq_pos));
                std::string value = trim(line.substr(eq_pos + 1));
                g_sync_config_data[current_section][key] = value;
            }
        }
    }
    return true;
}

void ConfigSynchronizer::save_config() {
    std::lock_guard<std::mutex> lock(g_config_mutex);
    std::ofstream file(m_config_path);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open config file for writing." << std::endl;
        return;
    }

    for (const auto& section_pair : g_sync_config_data) {
        file << "[" << section_pair.first << "]" << std::endl;
        for (const auto& key_value_pair : section_pair.second) {
            file << key_value_pair.first << " = " << key_value_pair.second << std::endl;
        }
        file << std::endl;
    }
    std::cout << "Configuration saved to " << m_config_path << std::endl;
}

std::string ConfigSynchronizer::serialize_config() {
    std::lock_guard<std::mutex> lock(g_config_mutex);
    std::stringstream ss;
    for (const auto& section_pair : g_sync_config_data) {
        for (const auto& key_value_pair : section_pair.second) {
            ss << "[" << section_pair.first << "]"
               << key_value_pair.first << "=" << key_value_pair.second << "\n";
        }
    }
    std::string content = ss.str();
    return std::to_string(content.length()) + "\n" + content;
}

void ConfigSynchronizer::update_config_from_string(const std::string& data) {
    std::stringstream ss(data);
    std::string line;
    int updates_count = 0;

    std::lock_guard<std::mutex> lock(g_config_mutex);
    while (std::getline(ss, line, '\n')) {
        if (line.empty() || line[0] != '[') continue;

        size_t section_end = line.find(']');
        size_t equals_pos = line.find('=', section_end);

        if (section_end != std::string::npos && equals_pos != std::string::npos) {
            std::string section = line.substr(1, section_end - 1);
            std::string key = line.substr(section_end + 1, equals_pos - (section_end + 1));
            std::string value = line.substr(equals_pos + 1);
            g_sync_config_data[section][key] = value;
            updates_count++;
        }
    }

    if (updates_count > 0) {
        std::cout << "Updated " << updates_count << " config items from WPF." << std::endl;
        save_config();
        // Signal the main thread to reload the config
        g_config_updated_flag.store(true);
    }
}

bool ConfigSynchronizer::send_config_to_wpf() {
    std::string host;
    int port = 0;
    {
        std::lock_guard<std::mutex> lock(g_config_mutex);
        if (g_sync_config_data.count("CONFIG_SYNC") && g_sync_config_data["CONFIG_SYNC"].count("WPF_HOST")) {
            host = g_sync_config_data["CONFIG_SYNC"]["WPF_HOST"];
        } else {
            std::cerr << "WPF_HOST not found in config." << std::endl;
            return false;
        }
        if (g_sync_config_data.count("CONFIG_SYNC") && g_sync_config_data["CONFIG_SYNC"].count("WPF_RECV_PORT")) {
            port = std::stoi(g_sync_config_data["CONFIG_SYNC"]["WPF_RECV_PORT"]);
        } else {
            std::cerr << "WPF_RECV_PORT not found in config." << std::endl;
            return false;
        }
    }


    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        std::cerr << "Error creating socket" << std::endl;
        return false;
    }

    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    inet_pton(AF_INET, host.c_str(), &server_addr.sin_addr);

    if (connect(sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        std::cerr << "Connection to WPF failed" << std::endl;
        close(sock);
        return false;
    }

    std::string config_str = serialize_config();
    send(sock, config_str.c_str(), config_str.length(), 0);
    std::cout << "Sent config to WPF." << std::endl;

    close(sock);
    return true;
}

void ConfigSynchronizer::handle_client_connection(int client_sock) {
    // Set a receive timeout on the client socket. This is crucial to prevent
    // the thread from blocking indefinitely on a recv() call if a client
    // connects but doesn't send data. An indefinite block would prevent a clean
    // shutdown of the application.
    struct timeval tv;
    tv.tv_sec = 1;  // 1-second timeout
    tv.tv_usec = 0;
    setsockopt(client_sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);

    char buffer[4096] = {0};
    std::string header;
    char c;

    // Read header
    while (recv(client_sock, &c, 1, 0) > 0) {
        if (c == '\n') break;
        header += c;
    }

    if (header.empty()) {
        close(client_sock);
        return;
    }

    size_t expected_length = std::stoull(header);
    std::string received_data;
    received_data.reserve(expected_length);
    size_t total_received = 0;

    while (total_received < expected_length) {
        ssize_t bytes_received = recv(client_sock, buffer, sizeof(buffer), 0);
        if (bytes_received <= 0) {
            std::cerr << "Error receiving config data body: connection closed, timed out, or error." << std::endl;
            close(client_sock);
            return;
        }
        received_data.append(buffer, bytes_received);
        total_received += bytes_received;
    }

    if (!m_shutdown_flag.load()) {
        std::cout << "Received config data from WPF." << std::endl;
        update_config_from_string(received_data);
    }

    close(client_sock);
}

void ConfigSynchronizer::receive_config_updates() {
    int port = 0;
    {
        std::lock_guard<std::mutex> lock(g_config_mutex);
        if (g_sync_config_data.count("CONFIG_SYNC") && g_sync_config_data["CONFIG_SYNC"].count("CPP_RECV_PORT")) {
            port = std::stoi(g_sync_config_data["CONFIG_SYNC"]["CPP_RECV_PORT"]);
        } else {
            std::cerr << "CPP_RECV_PORT not found in config." << std::endl;
            return;
        }
    }

    int listen_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_sock < 0) {
        std::cerr << "Error creating listening socket." << std::endl;
        return;
    }

    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(port);

    if (bind(listen_sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        std::cerr << "Bind failed on port " << port << std::endl;
        close(listen_sock);
        return;
    }

    if (listen(listen_sock, 5) < 0) {
        std::cerr << "Listen failed." << std::endl;
        close(listen_sock);
        return;
    }

    std::cout << "Listening for config updates on port " << port << std::endl;

    while (!m_shutdown_flag.load()) {
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(listen_sock, &readfds);

        struct timeval timeout;
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;

        int activity = select(listen_sock + 1, &readfds, nullptr, nullptr, &timeout);

        if (activity < 0 && errno != EINTR) {
            break;
        }

        if (activity > 0 && FD_ISSET(listen_sock, &readfds)) {
            int client_sock = accept(listen_sock, nullptr, nullptr);
            if (client_sock >= 0) {
                handle_client_connection(client_sock);
            }
        }
    }

    close(listen_sock);
}
