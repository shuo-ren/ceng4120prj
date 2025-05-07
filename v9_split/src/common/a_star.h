// ------------------------------------------------------------
//  File: src/common/a_star.h
// ------------------------------------------------------------
#pragma once
#include "dataloader.h"
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <omp.h>

namespace astar {

struct RouteResult {
    int id;
    std::string name;
    std::vector<std::pair<int, int>> edges;
    bool allSinks = false;   // 所有 sink 是否成功连通
    bool removed  = false;   // 拥塞阶段被移除
};

/* ---------- 小顶堆节点 ---------- */
struct PQ {
    int     n;      // 节点 id
    double  g;      // 已走代价
    double  f;      // g + h
    int     p;      // 父节点
    bool operator<(const PQ &o) const { return f > o.f; }
};

/* 曼哈顿启发函数 */
inline double manhattan(const Node &A, const Node &B) {
    return std::abs(A.begin_x - B.begin_x) +
           std::abs(A.begin_y - B.begin_y);
}

/* ------------------------------------------------------------------
   routeNet : 单网多源 A* 路由
   useHistory = true  ⇒  代价加 alpha·occ，避开已有占用
-------------------------------------------------------------------*/
inline RouteResult routeNet(const NodeMap &nodes,
                            const AdjacencyList &adj,
                            std::vector<uint8_t> &occ,
                            const Net &net,
                            bool useHistory,
                            int threads,
                            bool debug)
{
    const double alpha = useHistory ? 0.6 : 0.0;   // 历史成本权重

    std::vector<std::pair<int,int>> edges;         // 输出边
    std::unordered_set<int> tree{net.source};      // 已连通节点
    std::vector<uint8_t> localOcc;

    /* 复制全局占用到线程私有副本 */
    #pragma omp critical
    { localOcc = occ; }

    for (int sink : net.sinks) {

        /* ---------- A* 初始化 (多源) ---------- */
        std::priority_queue<PQ> pq;
        std::unordered_map<int,double> g;
        std::unordered_map<int,int>    par;

        for (int s : tree) {
            pq.push({s, 0.0, manhattan(nodes[s], nodes[sink]), -1});
            g[s] = 0.0;  par[s] = -1;
        }

        bool found = false;
        int  expand = 0, maxExpand = 50000;

        /* ---------- A* 主循环 ---------- */
        while (!pq.empty() && expand < maxExpand) {
            auto cur = pq.top(); pq.pop();
            if (cur.n == sink) { found = true; break; }
            if (cur.g != g[cur.n]) continue;   // 路径已更新
            ++expand;

            for (int nb : adj[cur.n]) {
                if (localOcc[nb] && !tree.count(nb)) continue;  // 冲突

                double ng = cur.g + 1 + alpha * occ[nb];
                if (!g.count(nb) || ng < g[nb]) {
                    g[nb] = ng;  par[nb] = cur.n;
                    pq.push({nb, ng, ng + manhattan(nodes[nb], nodes[sink]), cur.n});
                }
            }
        }

        if (!found) {
            if (debug) std::cerr << "Net " << net.id
                                 << " failed sink " << sink << '\n';
            continue;               // sink 未连，稍后标失败
        }

        /* ---------- 重建并记录路径 ---------- */
        std::vector<int> path;
        for (int v = sink; v != -1; v = par[v]) path.push_back(v);
        std::reverse(path.begin(), path.end());

        for (size_t i = 1; i < path.size(); ++i) {
            int u = path[i-1], v = path[i];
            edges.emplace_back(u, v);
            localOcc[u]++; localOcc[v]++;
            tree.insert(v);
        }
    }

    /* ---------- 把本网占用写回全局 ---------- */
    #pragma omp critical
    {
        for (auto [u, v] : edges) { occ[u]++; occ[v]++; }
    }

    bool ok = true;
    for (int s : net.sinks)
        if (!tree.count(s)) { ok = false; break; }

    return { net.id, net.name, std::move(edges), ok, false };
}

} // namespace astar
// ------------------------------------------------------------
