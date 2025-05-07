#pragma once
#include <vector>
#include <string>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <fstream>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <numeric>
#include <omp.h>

#include "dataloader.h"
#include "routecache.h"

class Router {
public:
    Router(const NodeMap &n, const AdjacencyList &a, const NetList &nl,
           const std::string &out, bool dbg = false, int num_threads = 8)
        : nodes(n), adj(a), nets(nl), outPath(out), debug(dbg), threads(num_threads)
    { occ.resize(nodes.size(), 0); }

    void setCache(RouteCache *c) { cache = c; }

    /* -------- top‑level -------- */
    void route() {
        std::ofstream ofs(outPath);
        if(!ofs) throw std::runtime_error("open out");

        // 创建两个顺序数组，一个用于路由顺序，一个用于输出顺序
        std::vector<size_t> routeOrder(nets.size());
        std::vector<size_t> outputOrder(nets.size());
        std::iota(routeOrder.begin(), routeOrder.end(), 0);
        std::iota(outputOrder.begin(), outputOrder.end(), 0);

        // 按汇点数量排序路由顺序
        std::sort(routeOrder.begin(), routeOrder.end(),
                 [&](size_t a, size_t b){
                     return nets[a].sinks.size() < nets[b].sinks.size();
                 });

        // 按 id 升序排序输出顺序
        std::sort(outputOrder.begin(), outputOrder.end(),
                 [&](size_t a, size_t b){
                     return nets[a].id < nets[b].id;
                 });

        // 创建路由结果存储结构
        struct RouteResult {
            int id;
            std::string name;
            std::vector<std::pair<int, int>> edges;
        };
        std::vector<RouteResult> results(nets.size());

        // 按优化后的顺序进行路由
        #pragma omp parallel for schedule(dynamic) num_threads(threads)
        for(int i = 0; i < routeOrder.size(); i++) {
            size_t idx = routeOrder[i];
            const Net &net = nets[idx];
            
            std::vector<std::pair<int, int>> edges;
            
            if(cache && cache->hasNet(net.id)) {
                // 从缓存获取路由
                const auto &el = cache->getEdges(net.id);
                edges = el.edges;
                
                // 更新本地占用
                #pragma omp critical
                {
                    for(auto &[u, v] : edges) {
                        occ[u] = occ[v] = 1;
                    }
                }
            } else {
                // 执行新路由
                edges = routeNetParallel(net);
            }
            
            // 保存结果
            #pragma omp critical
            {
                results[idx] = {net.id, net.name, edges};
            }
        }

        // 按 id 顺序输出
        for(size_t idx : outputOrder) {
            const auto &result = results[idx];
            ofs << result.id << ' ' << result.name << '\n';
            for(auto &[u, v] : result.edges) {
                ofs << u << ' ' << v << '\n';
            }
            ofs << '\n';
        }
        
        if(cache) cache->save("cache.dat");
    }

private:
    /* ===== data ===== */
    const NodeMap& nodes;
    const AdjacencyList& adj;
    const NetList& nets;
    std::string outPath;
    bool debug;
    int threads;
    std::vector<char> occ;          // global occupancy bitmap
    RouteCache *cache = nullptr;

    /* ===== A* ===== */
    struct PQ{int n; double g, f; int p; bool operator<(const PQ&o)const{return f>o.f;}};
    
    double h(int a, int b) const {
        const auto &A = nodes[a], &B = nodes[b];
        return std::abs(A.begin_x - B.begin_x) + std::abs(A.begin_y - B.begin_y);
    }

    std::vector<int> astar(const std::unordered_set<int>& tree, int goal, 
                           const std::vector<char>& localOcc) {
        std::priority_queue<PQ> pq;
        std::unordered_map<int, double> g;
        std::unordered_map<int, int> par;

        for(int s : tree) {
            pq.push({s, 0, h(s, goal), -1});
            g[s] = 0.0;  par[s] = -1;
        }

        // 添加搜索限制
        int maxExpand = 50000;  // 可调参数
        int expanded = 0;

        while(!pq.empty() && expanded < maxExpand) {
            expanded++;
            auto cur = pq.top(); pq.pop();
            if(cur.n == goal) { break; }
            if(cur.g != g[cur.n]) continue;

            for(int nb : adj[cur.n]) {
                if(localOcc[nb] && !tree.count(nb)) continue;
                
                // 基本路径代价
                double ng = cur.g + 1;
                
                if(!g.count(nb) || ng < g[nb]) {
                    g[nb] = ng; par[nb] = cur.n;
                    pq.push({nb, ng, ng + h(nb, goal), cur.n});
                }
            }
        }

        if(!par.count(goal)) return {};
        std::vector<int> path;
        for(int v = goal; v != -1; v = par[v]) path.push_back(v);
        std::reverse(path.begin(), path.end());
        return path;
    }

    // 并行路由单个网络
    std::vector<std::pair<int, int>> routeNetParallel(const Net& net) {
        std::vector<std::pair<int, int>> allEdges;
        std::unordered_set<int> tree{net.source};
        std::vector<char> localOcc;
        
        // 获取全局占用状态的一个本地副本
        #pragma omp critical
        {
            localOcc = occ;
        }

        // 为每个汇点计算路径
        for(int sink : net.sinks) {
            auto path = astar(tree, sink, localOcc);
            if(path.empty()) {
                if(debug) std::cerr << "Net " << net.id << " failed sink " << sink << '\n';
                continue;
            }
            
            // 处理路径
            if(path.size() >= 2) {
                // 检查路径上的节点是否已被占用
                bool pathValid = true;
                for(int v : path) {
                    if(localOcc[v] && !tree.count(v)) {
                        pathValid = false;
                        break;
                    }
                }
                
                if(pathValid) {
                    int prev = path.front();
                    for(size_t i = 1; i < path.size(); ++i) {
                        int v = path[i];
                        allEdges.emplace_back(prev, v);
                        localOcc[prev] = localOcc[v] = 1;
                        prev = v; 
                        tree.insert(v);
                    }
                }
            }
        }
        
        // 更新全局占用状态
        #pragma omp critical
        {
            for(auto &[u, v] : allEdges) {
                occ[u] = occ[v] = 1;
            }
            
            if(cache) {
                cache->addNet(net.id, net.name, allEdges);
            }
        }
        
        return allEdges;
    }
};