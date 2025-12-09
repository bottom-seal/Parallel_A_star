// include/helper.h
#pragma once

#include <cstdint>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
#include <chrono>

struct GridMap {
    int width  = 0;
    int height = 0;

    // 0 = path, 1 = wall
    std::vector<uint8_t> cells; // flat, row-major: idx = y * width + x

    // Conventional start/goal
    int start_x = 0, start_y = 0;
    int goal_x  = 0, goal_y  = 0;

    int start_idx = 0;
    int goal_idx  = 0;

    inline uint8_t at(int x, int y) const {
        return cells[static_cast<size_t>(y) * width + x];
    }

    inline bool is_wall(int x, int y) const {
        return at(x, y) != 0;
    }

    inline bool in_bounds(int x, int y) const {
        return x >= 0 && x < width && y >= 0 && y < height;
    }
};

inline bool is_power_of_two(int n) {
    return n > 0 && (n & (n - 1)) == 0;
}

inline GridMap load_grid_map_from_file(const std::string &path) {
    std::ifstream fin(path);
    if (!fin) {
        throw std::runtime_error("Failed to open map file: " + path);
    }

    std::string line;
    std::vector<uint8_t> data;
    data.reserve(1024);

    int width  = -1;
    int height = 0;

    while (std::getline(fin, line)) {
        int row_count = 0;

        for (char c : line) {
            if (c == '0' || c == '1') {
                uint8_t v = static_cast<uint8_t>(c - '0');
                data.push_back(v);
                ++row_count;
            }
            // ignore other characters (spaces, tabs, etc.)
        }

        if (row_count == 0) {
            continue; // skip empty lines
        }

        if (width == -1) {
            width = row_count;
        } else if (row_count != width) {
            throw std::runtime_error(
                "Inconsistent row width in map file (got " +
                std::to_string(row_count) + ", expected " +
                std::to_string(width) + ")");
        }

        ++height;
    }

    if (width <= 0 || height <= 0) {
        throw std::runtime_error("Map file appears to be empty or invalid.");
    }

    if (width != height) {
        throw std::runtime_error("Map is not square (width != height).");
    }

    if (!is_power_of_two(width)) {
        throw std::runtime_error(
            "Map size is not a power of two (size = " + std::to_string(width) + ")");
    }

    if (static_cast<int>(data.size()) != width * height) {
        throw std::runtime_error(
            "Data size mismatch: collected " + std::to_string(data.size()) +
            " cells but expected " + std::to_string(width * height));
    }

    GridMap map;
    map.width  = width;
    map.height = height;
    map.cells  = std::move(data);

    // Your convention:
    map.start_x = map.width  - 1;
    map.start_y = 1;
    map.goal_x  = 0;
    map.goal_y  = map.height - 2;

    map.start_idx = map.start_y * map.width + map.start_x;
    map.goal_idx  = map.goal_y  * map.width + map.goal_x;

    return map;
}

// simple timing helper
struct Timer {
    using clock = std::chrono::high_resolution_clock;
    clock::time_point start;

    Timer() : start(clock::now()) {}

    double elapsed_ms() const {
        auto now = clock::now();
        return std::chrono::duration<double, std::milli>(now - start).count();
    }
};
