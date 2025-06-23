#include "web_api.h"
#include "crow.h"
#include "nlohmann/json.hpp"
#include "config.h" // g_config と loadConfig() のために

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <thread>
#include <atomic>

// グローバルなCrowアプリケーションインスタンスとスレッド
crow::SimpleApp app;
std::thread api_thread;
std::atomic<bool> server_running(false);

// config.iniのパス
const std::string CONFIG_FILE_PATH = "config.ini";

// config.iniをコメントを保持したまま更新するヘルパー関数
bool update_config_file(const nlohmann::json& new_config) {
    std::ifstream infile(CONFIG_FILE_PATH);
    if (!infile.is_open()) {
        CROW_LOG_ERROR << "Could not open config file for reading: " << CONFIG_FILE_PATH;
        return false;
    }

    std::vector<std::string> lines;
    std::string line;
    while (std::getline(infile, line)) {
        lines.push_back(line);
    }
    infile.close();

    // JSONから更新するキーと値のマップを作成
    std::map<std::string, std::map<std::string, std::string>> updates;
    for (auto const& [section, keys] : new_config.items()) {
        for (auto const& [key, val] : keys.items()) {
            updates[section][key] = val.dump();
            // JSONの文字列値はクォートで囲まれているので削除
            if (val.is_string()) {
                updates[section][key] = val.get<std::string>();
            }
        }
    }

    // ファイルの各行をチェックし、必要に応じて更新
    std::string current_section;
    for (size_t i = 0; i < lines.size(); ++i) {
        std::string trimmed_line = lines[i];
        trimmed_line.erase(0, trimmed_line.find_first_not_of(" \t"));
        trimmed_line.erase(trimmed_line.find_last_not_of(" \t") + 1);

        if (trimmed_line.empty() || trimmed_line[0] == '#' || trimmed_line[0] == ';') {
            continue; // コメントや空行はスキップ
        }

        if (trimmed_line[0] == '[' && trimmed_line.back() == ']') {
            current_section = trimmed_line.substr(1, trimmed_line.length() - 2);
            continue;
        }

        size_t equals_pos = trimmed_line.find('=');
        if (equals_pos != std::string::npos) {
            std::string key = trimmed_line.substr(0, equals_pos);
            key.erase(key.find_last_not_of(" \t") + 1);

            if (!current_section.empty() && updates.count(current_section) && updates[current_section].count(key)) {
                lines[i] = key + "=" + updates[current_section][key];
                updates[current_section].erase(key);
                if (updates[current_section].empty()) {
                    updates.erase(current_section);
                }
            }
        }
    }

    // ファイルに存在しなかった新しいセクション/キーを追加
    for (const auto& [section, keys] : updates) {
        lines.push_back(""); // 前のセクションとの間に空行を挿入
        lines.push_back("[" + section + "]");
        for (const auto& [key, value] : keys) {
            lines.push_back(key + "=" + value);
        }
    }

    // 更新された内容でファイルを書き換える
    std::ofstream outfile(CONFIG_FILE_PATH, std::ios::trunc);
    if (!outfile.is_open()) {
        CROW_LOG_ERROR << "Could not open config file for writing: " << CONFIG_FILE_PATH;
        return false;
    }
    for (size_t i = 0; i < lines.size(); ++i) {
        outfile << lines[i] << (i == lines.size() - 1 ? "" : "\n");
    }
    outfile.close();
    return true;
}

// APIサーバーのメインロジック
void run_server() {
    // GET /api/config: 現在の設定をJSONで返す
    // 簡単のため、ファイルから直接読み込んで返します
    CROW_ROUTE(app, "/api/config").methods(crow::HTTPMethod::Get)
    ([]() {
        // ここでは簡単のため、ファイルの内容をそのままテキストとして返します。
        // 本格的にはiniファイルをパースしてJSONで返すのが望ましいです。
        std::ifstream f(CONFIG_FILE_PATH);
        if (!f.is_open()) {
            return crow::response(500, "Could not read config file");
        }
        std::stringstream buffer;
        buffer << f.rdbuf();
        return crow::response(buffer.str());
    });

    // POST /api/config: 設定を更新する
    CROW_ROUTE(app, "/api/config").methods(crow::HTTPMethod::Post)
    ([](const crow::request& req) {
        auto body = crow::json::load(req.body);
        if (!body) {
            return crow::response(400, "Invalid JSON");
        }

        if (update_config_file(nlohmann::json::parse(body.dump()))) {
            loadConfig(CONFIG_FILE_PATH.c_str());
            CROW_LOG_INFO << "Config file updated and reloaded successfully.";
            return crow::response(200, "Config updated. Some changes may require an application restart.");
        } else {
            CROW_LOG_ERROR << "Failed to update config file.";
            return crow::response(500, "Failed to update config file.");
        }
    });

    app.port(8080).multithreaded().run();
    server_running = false;
}

void start_web_api_server() {
    if (!server_running) {
        server_running = true;
        api_thread = std::thread(run_server);
        CROW_LOG_INFO << "Web API server starting on port 8080...";
    }
}

void stop_web_api_server() {
    if (server_running) {
        CROW_LOG_INFO << "Stopping Web API server...";
        app.stop();
        if (api_thread.joinable()) {
            api_thread.join();
        }
        CROW_LOG_INFO << "Web API server stopped.";
    }
}

