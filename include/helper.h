// include/helper.h
#pragma once

#include <cstdint>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
#include <chrono>
#include <algorithm>

struct GridMap {
    int width  = 0;
    int height = 0;

    // 0 = path, 1 = wall
    std::vector<uint8_t> cells; //flat, row-major: idx = y * width + x, col or row shouldn't matter as access is random?

    //set later
    int start_x = 0, start_y = 0;
    int goal_x  = 0, goal_y  = 0;

    int start_idx = 0;
    int goal_idx  = 0;

    inline uint8_t at(int x, int y) const {//return data in (x, y)
        return cells[static_cast<size_t>(y) * width + x];
    }

    inline bool is_wall(int x, int y) const {//return (x, y) is a wall
        return at(x, y) != 0;
    }

    inline bool in_bounds(int x, int y) const {// return if (x, y) is in the map
        return x >= 0 && x < width && y >= 0 && y < height;
    }
};

inline bool is_power_of_two(int n) {
    return n > 0 && ( ((n & (n - 1)) == 0) || (n % 16 == 0) );
}

inline GridMap load_grid_map_from_file(const std::string &path) {
    std::ifstream fin(path);
    if (!fin) {
        throw std::runtime_error("Failed to open map file: " + path);
    }

    std::string line;//for a single row
    std::vector<uint8_t> data;//for whole map
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
            continue; //skip empty lines
        }

        if (width == -1) {
            width = row_count;
        } else if (row_count != width) {//all row should be same legnth
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
    map.cells  = std::move(data);//data -> map.cells

    //all map have same starting amd goal
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

template<typename State>
inline std::vector<int> reconstruct_path(const GridMap &map,
                                         const State &state)
{
    std::vector<int> path;
    int start = map.start_idx;
    int goal  = map.goal_idx;

    int cur = goal;

    // If goal was never reached (paranoia check)
    if (get_parent(state, cur) == -1 && cur != start) {
        return path; // empty = no path
    }

    while (true) {
        path.push_back(cur);
        if (cur == start) break;

        int p = get_parent(state, cur);
        if (p == -1) {
            // Broken parent chain: return empty to signal failure
            path.clear();
            return path;
        }
        cur = p;
    }

    // Currently goal -> ... -> start, so reverse to start -> goal
    std::reverse(path.begin(), path.end());
    return path;
}

// Save a copy of the map with the path marked as '2' into a text file.
// '0' = free, '1' = wall, '2' = path. Easy to visualize in Python.
inline void save_path_as_grid_txt(const GridMap &map,
                                  const std::vector<int> &path,
                                  const std::string &filename)
{
    int W = map.width;
    int H = map.height;

    // Start from original map values ('0' or '1')
    std::vector<char> out(static_cast<size_t>(W) * H);
    for (int i = 0; i < W * H; ++i) {
        out[i] = char('0' + map.cells[static_cast<size_t>(i)]);  // '0' or '1'
    }

    // Mark path cells as '2'
    for (int idx : path) {
        if (idx >= 0 && idx < W * H) {
            out[idx] = '2';
        }
    }

    std::ofstream fout(filename);
    if (!fout) {
        std::cerr << "Failed to open output file: " << filename << "\n";
        return;
    }

    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            fout << out[y * W + x];
        }
        fout << "\n";
    }
}