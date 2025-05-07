// ------------------------------------------------------------
//  File: src/strategy/hybrid.cpp
// ------------------------------------------------------------
#include "strategy/hybrid.h"
#include "common/a_star.h"
#include <algorithm>

void HybridStrategy::run(RouteContext &ctx, double budget)
{
    /* ---------- 1. 选出 “长网” (前 5 % 最大 bounding‑box 周长) ---------- */
    std::vector<std::pair<size_t, int>> metric;   // (idx, perimeter)
    metric.reserve(ctx.nets.size());

    for (size_t i = 0; i < ctx.nets.size(); ++i) {
        const Net &net = ctx.nets[i];
        int xmin = ctx.nodes[net.source].begin_x;
        int xmax = xmin;
        int ymin = ctx.nodes[net.source].begin_y;
        int ymax = ymin;
        for (int s : net.sinks) {
            xmin = std::min(xmin, ctx.nodes[s].begin_x);
            xmax = std::max(xmax, ctx.nodes[s].begin_x);
            ymin = std::min(ymin, ctx.nodes[s].begin_y);
            ymax = std::max(ymax, ctx.nodes[s].begin_y);
        }
        metric.emplace_back(i, (xmax - xmin) + (ymax - ymin));
    }

    std::sort(metric.begin(), metric.end(),
              [](auto &a, auto &b) { return a.second > b.second; });

    size_t selCnt = std::max<size_t>(1, metric.size() * 5 / 100);  // 前 5 %
    std::vector<size_t> longNets;
    longNets.reserve(selCnt);
    for (size_t j = 0; j < selCnt; ++j) longNets.push_back(metric[j].first);

    /* ---------- 2. rip‑up 这些长网 ---------- */
    for (size_t idx : longNets) {
        auto &r = ctx.results[idx];
        for (auto [u, v] : r.edges) {
            if (ctx.occ[u]) --ctx.occ[u];
            if (ctx.occ[v]) --ctx.occ[v];
        }
        r.edges.clear();
        r.allSinks = false;
    }

    /* ---------- 3. 带历史成本 (alpha>0) 重新布线 ---------- */
    #pragma omp parallel for schedule(dynamic) num_threads(ctx.threads)
    for (int k = 0; k < (int)longNets.size(); ++k) {
        size_t idx = longNets[k];
        auto res = astar::routeNet(ctx.nodes, ctx.adj, ctx.occ,
                                   ctx.nets[idx], /*useHistory=*/true,
                                   ctx.threads, ctx.debug);
        #pragma omp critical
        {
            ctx.results[idx] = std::move(res);
        }
    }
}
// ------------------------------------------------------------
