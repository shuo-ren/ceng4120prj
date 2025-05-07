#include "strategy/quality.h"
#include <algorithm>
#include <chrono>
#include <iostream>
#include <numeric>
#include <omp.h>

using Clock = std::chrono::high_resolution_clock;

void QualityStrategy::run(RouteContext &ctx, double budget) {
    auto start = Clock::now();

    std::vector<astar::RouteResult> current(ctx.nets.size());
    int baselineWL = 0;

    #pragma omp parallel for schedule(dynamic) reduction(+:baselineWL)
    for (int i = 0; i < (int)ctx.nets.size(); ++i) {
        current[i] = astar::routeNet(ctx.nodes, ctx.adj, ctx.occ, ctx.nets[i], ctx.debug, ctx.threads, false);
        baselineWL += current[i].wirelength();
    }

    if (ctx.debug)
        std::cout << "[Q] baseline WL = " << baselineWL << std::endl;

    std::vector<astar::RouteResult> best = current;
    int bestWL = baselineWL;
    int rounds = 0;

    while (true) {
        bool improved = false;
        std::vector<astar::RouteResult> trial(ctx.nets.size());
        int totalWL = 0;

        #pragma omp parallel for schedule(dynamic) reduction(+:totalWL)
        for (int i = 0; i < (int)ctx.nets.size(); ++i) {
            auto reroute = astar::routeNet(ctx.nodes, ctx.adj, ctx.occ, ctx.nets[i], false, ctx.threads, false);
            trial[i] = reroute;
            totalWL += reroute.wirelength();
        }

        if (totalWL < bestWL) {
            bestWL = totalWL;
            best = trial;
            improved = true;
        }

        rounds++;
        auto elapsed = std::chrono::duration<double>(Clock::now() - start).count();

        if (ctx.debug)
            std::cout << "[Q] round " << rounds << " WL = " << totalWL << " (elapsed " << elapsed << " s)\n";

        if (!improved || elapsed > budget * 0.95) break;
    }

    ctx.results.swap(best);

    if (ctx.debug) {
        double used = std::chrono::duration<double>(Clock::now() - start).count();
        std::cout << "[Q] hill‑climb converged after " << rounds << " rounds\n";
        std::cout << "[Q] final WL = " << bestWL << "   time used " << used << " s\n";
    }
}
