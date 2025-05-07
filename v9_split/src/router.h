// ------------------------------------------------------------
// File: src/router.h
// ------------------------------------------------------------
#pragma once
#include "dataloader.h"
#include "common/a_star.h"
#include "common/loop_removal.h"
#include "common/congestion.h"
#include "strategy/quality.h"
#include "strategy/hybrid.h"
#include "strategy/throughput.h"
#include <memory>
#include <vector>
#include <string>

struct RouteResult : public astar::RouteResult {};

class Router {
public:
    Router(const NodeMap &, const AdjacencyList &, const NetList &,
           const std::string &, bool, int, double, double);
    void route();
private:
    void firstRound();
    std::unique_ptr<IStrategy> pickStrategy(size_t);
    void writeResult();

    const NodeMap &nodes; const AdjacencyList &adj; const NetList &nets;
    std::string outPath; bool debug; int threads;
    double t_read, limit;

    std::vector<uint8_t> occ;
    std::vector<astar::RouteResult> results;
    looprem::LoopRemover loopRem;
    congestion::CongestionManager congMgr;
};