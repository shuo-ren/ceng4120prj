#pragma once
#include "common/a_star.h"
#include "dataloader.h"
#include <vector>

// Routing context for all strategies
struct RouteContext {
    const NodeMap& nodes;
    const AdjacencyList& adj;
    const NetList& nets;
    std::vector<astar::RouteResult> results;
    std::vector<unsigned char> occ;
    bool debug;
    int threads;
};

// Abstract base class
class Strategy {
public:
    virtual void run(RouteContext& ctx, double budget) = 0;
    virtual ~Strategy() = default;
};
