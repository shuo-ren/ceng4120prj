#include "router.h"
#include <chrono>
#include <numeric>
using Clock = std::chrono::high_resolution_clock;

Router::Router(const NodeMap &n, const AdjacencyList &a, const NetList &nl,
               const std::string &out, bool dbg, int th,
               double rd, double tl)
    : nodes(n), adj(a), nets(nl), outPath(out), debug(dbg), threads(th),
      t_read(rd), limit(tl), occ(nodes.size(), 0),
      loopRem(nodes), congMgr(nodes, adj, occ) {}

void Router::firstRound() {
    results.resize(nets.size());
    std::vector<size_t> order(nets.size());
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](size_t a, size_t b) {
        return nets[a].sinks.size() < nets[b].sinks.size();
    });

    #pragma omp parallel for schedule(dynamic) num_threads(threads)
    for (int i = 0; i < (int)order.size(); ++i) {
        size_t idx = order[i];
        auto res = astar::routeNet(nodes, adj, occ, nets[idx], false, threads, debug);
        #pragma omp critical
        {
            results[idx] = std::move(res);
        }
    }
    loopRem.removeAll(results);
    congMgr.initialResolve(results);
}

void Router::writeResult() {
    std::vector<size_t> ord(nets.size());
    std::iota(ord.begin(), ord.end(), 0);
    std::sort(ord.begin(), ord.end(), [&](size_t a, size_t b) {
        return nets[a].id < nets[b].id;
    });
    std::ofstream ofs(outPath);
    size_t ok = 0;
    for (size_t idx : ord) {
        auto &r = results[idx];
        if (!r.allSinks || r.removed) continue;
        ++ok;
        ofs << r.id << ' ' << r.name << '\n';
        for (auto [u, v] : r.edges) ofs << u << ' ' << v << '\n';
        ofs << '\n';
    }
    ofs.close();
    std::cout << "Remaining successfully routed nets: " << ok << '/' << nets.size() << '\n';
}

std::unique_ptr<IStrategy> Router::pickStrategy(size_t n) {
    if (n <= 100) return std::make_unique<QualityStrategy>();
    if (n <= 8000) return std::make_unique<HybridStrategy>();
    return std::make_unique<ThroughputStrategy>();
}

void Router::route() {
    auto t0 = Clock::now();
    firstRound();
    auto t1 = Clock::now();
    double t_first = std::chrono::duration<double>(t1 - t0).count();
    double t_write_est = t_read * 0.5;
    double budget = (limit - t_read - t_write_est - t_first) * 0.95;
    if (budget < 0) budget = 0;
    if (debug) std::cout << "Strategy budget " << budget << " s\n";

    RouteContext ctx{nodes, adj, nets, occ, results, debug, threads};
    auto strat = pickStrategy(nets.size());
    strat->run(ctx, budget);

    loopRem.removeAll(results);
    congMgr.finalResolve(results);

    writeResult();
}