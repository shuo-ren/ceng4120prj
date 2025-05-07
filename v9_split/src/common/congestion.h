// ------------------------------------------------------------
// File: src/common/congestion.h
// ------------------------------------------------------------
#pragma once
#include "dataloader.h"
#include "common/a_star.h"
#include <unordered_set>
#include <unordered_map>
#include <algorithm>

namespace congestion {

class CongestionManager {
public:
    CongestionManager(const NodeMap &n, const AdjacencyList &a, std::vector<uint8_t> &o)
        : nodes(n), adj(a), occ(o) {}

    void initialResolve(std::vector<astar::RouteResult> &results) { /* no-op */ }

    void finalResolve(std::vector<astar::RouteResult> &results) {
        std::unordered_map<int, std::unordered_set<size_t>> node2nets;
        for (size_t i = 0; i < results.size(); ++i) {
            auto &r = results[i];
            if (r.removed) continue;
            for (auto [u, v] : r.edges) {
                node2nets[u].insert(i);
                node2nets[v].insert(i);
            }
        }
        std::unordered_set<int> congNodes;
        for (auto &[node, nets] : node2nets) if (nets.size() > 1) congNodes.insert(node);
        if (congNodes.empty()) return;

        // contribution list
        std::vector<std::pair<size_t, int>> contrib;
        for (auto node : congNodes)
            for (auto idx : node2nets[node]) {
                if (results[idx].removed) continue;
                contrib.emplace_back(idx, 1);
            }
        // count occurrences
        std::unordered_map<size_t, int> cnt;
        for (auto &p : contrib) cnt[p.first]++;
        std::vector<std::pair<size_t, int>> vec(cnt.begin(), cnt.end());
        std::sort(vec.begin(), vec.end(), [](auto &a, auto &b) { return a.second > b.second; });

        for (auto &[idx, _] : vec) {
            bool still = false;
            for (auto n : congNodes) {
                size_t used = 0;
                for (auto net : node2nets[n]) if (!results[net].removed) ++used;
                if (used > 1) { still = true; break; }
            }
            if (!still) break;
            results[idx].removed = true;
            for (auto [u, v] : results[idx].edges) {
                if (occ[u]) --occ[u];
                if (occ[v]) --occ[v];
            }
        }
    }

private:
    const NodeMap &nodes; const AdjacencyList &adj; std::vector<uint8_t> &occ;
};

} // namespace congestion