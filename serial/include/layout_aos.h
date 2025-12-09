// include/layout_aos.h
#pragma once
#include <vector>
#include <limits>
#include <cstdint>

struct NodeAoS {
    float   g;
    float   f;
    int     parent;
    uint8_t state; // 0=unvisited,1=open,2=closed

    NodeAoS()
        : g(std::numeric_limits<float>::infinity()),
          f(std::numeric_limits<float>::infinity()),
          parent(-1),
          state(0) {}
};

struct AStarStateAoS {
    std::vector<NodeAoS> nodes;
    explicit AStarStateAoS(int N) : nodes(N) {}
};

using AStarState = AStarStateAoS;

// accessors
inline float get_g(const AStarState &s, int idx)       { return s.nodes[idx].g; }
inline void  set_g(AStarState &s, int idx, float v)    { s.nodes[idx].g = v; }

inline float get_f(const AStarState &s, int idx)       { return s.nodes[idx].f; }
inline void  set_f(AStarState &s, int idx, float v)    { s.nodes[idx].f = v; }

inline int   get_parent(const AStarState &s, int idx)  { return s.nodes[idx].parent; }
inline void  set_parent(AStarState &s, int idx, int p) { s.nodes[idx].parent = p; }

inline uint8_t get_state(const AStarState &s, int idx) { return s.nodes[idx].state; }
inline void    set_state(AStarState &s, int idx, uint8_t v) { s.nodes[idx].state = v; }
