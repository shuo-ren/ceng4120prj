#include "common/loop_removal.h"
#include <unordered_set>
#include <unordered_map>
#include <queue>
#include <algorithm>

using looprem::LoopRemover;
using astar::RouteResult;

/* 构造函数：引用节点信息 */
LoopRemover::LoopRemover(const NodeMap& _nodes) : nodes(_nodes) {}

/* 公共接口：对所有 nets 去环 */
void LoopRemover::removeAll(std::vector<RouteResult>& results) {
    for (auto& net : results) {
        if (!net.edges.empty() && net.allSinks && !net.removed) {
            net.edges = removeLoopsInternal(net.edges, net.id, {});
        }
    }
}

/* 内部函数：移除环并保留主路径 */
std::vector<std::pair<int, int>>
LoopRemover::removeLoopsInternal(const std::vector<std::pair<int, int>>& edges,
                                 int netId, const std::vector<int>& sinks)
{
    // 建邻接表
    std::unordered_map<int, std::vector<int>> g;
    for (auto [u, v] : edges) {
        g[u].push_back(v);
        g[v].push_back(u);
    }

    std::unordered_set<std::string> kept;
    std::unordered_set<int> visited;
    std::queue<int> q;

    // 从任意点出发，BFS 构造树（可选从 netId 或 sinks[0]）
    if (g.count(netId)) q.push(netId);
    else if (!sinks.empty()) q.push(sinks[0]);
    else if (!g.empty()) q.push(g.begin()->first);
    else return {}; // 空图

    visited.insert(q.front());
    std::unordered_map<int, int> parent;

    while (!q.empty()) {
        int u = q.front(); q.pop();
        for (int v : g[u]) {
            if (visited.count(v)) continue;
            visited.insert(v);
            parent[v] = u;
            q.push(v);
            int min_id = std::min(u, v);
            int max_id = std::max(u, v);
            kept.insert(std::to_string(min_id) + "_" + std::to_string(max_id));
        }
    }

    // 过滤原始 edges，只保留 BFS 树边
    std::vector<std::pair<int, int>> cleaned;
    for (auto [u, v] : edges) {
        int min_id = std::min(u, v);
        int max_id = std::max(u, v);
        std::string key = std::to_string(min_id) + "_" + std::to_string(max_id);
        if (kept.count(key)) {
            cleaned.emplace_back(u, v);
        }
    }

    return cleaned;
}
