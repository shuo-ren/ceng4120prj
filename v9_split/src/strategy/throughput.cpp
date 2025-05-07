// ------------------------------------------------------------
// File: src/strategy/throughput.cpp
// ------------------------------------------------------------
#include "strategy/throughput.h"
void ThroughputStrategy::run(RouteContext &ctx, double) {
    // single adaptive reroute pass on failed nets (simplified)
    std::vector<size_t> retry;
    for (size_t i = 0; i < ctx.results.size(); ++i)
        if (!ctx.results[i].allSinks) retry.push_back(i);

    #pragma omp parallel for schedule(dynamic) num_threads(ctx.threads)
    for (int k = 0; k < (int)retry.size(); ++k) {
        size_t idx = retry[k];
        auto res = astar::routeNet(ctx.nodes, ctx.adj, ctx.occ, ctx.nets[idx], true,
                                   ctx.threads, ctx.debug);
        #pragma omp critical
        {
            ctx.results[idx] = std::move(res);
        }
    }
}