// router.h

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
#include <functional>

#include "dataloader.h"

class Router {
public:
    Router(const NodeMap &n, const AdjacencyList &a, const NetList &nl,
           const std::string &out, bool dbg = false, int num_threads = 8)
        : nodes(n), adj(a), nets(nl), outPath(out), debug(dbg), threads(num_threads)
    { occ.resize(nodes.size(), 0); }

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
            
            // 执行新路由
            std::vector<std::pair<int, int>> edges = routeNetParallel(net);
            
            // 保存结果
            #pragma omp critical
            {
                results[idx] = {net.id, net.name, edges};
            }
        }

        // 检测和移除环路
        if(debug) std::cout << "Checking and removing loops in routes..." << std::endl;
        for(size_t idx : outputOrder) {
            const Net &net = nets[idx];
            results[idx].edges = removeRouteLoops(results[idx].edges, net.source, net.sinks);
        }
        if(debug) std::cout << "Loop removal completed." << std::endl;

        // 按 id 顺序输出
        for(size_t idx : outputOrder) {
            const auto &result = results[idx];
            ofs << result.id << ' ' << result.name << '\n';
            for(auto &[u, v] : result.edges) {
                ofs << u << ' ' << v << '\n';
            }
            ofs << '\n';
        }
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
        }
        
        return allEdges;
    }

    // 检测并移除路由环路，保持源点到汇点的连通性
    std::vector<std::pair<int, int>> removeRouteLoops(const std::vector<std::pair<int, int>>& edges, 
                                                     int source, const std::vector<int>& sinks) {
        if(edges.empty()) return edges;
        
        // 1. 建立邻接表表示图
        std::unordered_map<int, std::vector<int>> graph;
        for(const auto& [u, v] : edges) {
            graph[u].push_back(v);
            graph[v].push_back(u); // 因为边是无向的
        }
        
        // 2. 找出从源点到每个汇点的最短路径，标记必要边
        std::unordered_set<std::string> essentialEdges;
        
        for(int sink : sinks) {
            // 使用BFS找最短路径
            std::queue<int> q;
            std::unordered_map<int, int> parent;
            std::unordered_set<int> visited;
            
            q.push(source);
            visited.insert(source);
            parent[source] = -1;
            bool found = false;
            
            while(!q.empty() && !found) {
                int current = q.front();
                q.pop();
                
                if(current == sink) {
                    found = true;
                    break;
                }
                
                for(int neighbor : graph[current]) {
                    if(!visited.count(neighbor)) {
                        visited.insert(neighbor);
                        parent[neighbor] = current;
                        q.push(neighbor);
                    }
                }
            }
            
            // 如果找到路径，标记路径上的所有边为必要边
            if(found) {
                int current = sink;
                while(parent[current] != -1) {
                    int p = parent[current];
                    int min_node = std::min(current, p);
                    int max_node = std::max(current, p);
                    essentialEdges.insert(std::to_string(min_node) + "_" + std::to_string(max_node));
                    current = p;
                }
            }
        }
        
        // 3. 建立一个边集合，用于环路检测
        std::unordered_set<std::string> edgeSet;
        for(const auto& [u, v] : edges) {
            // 确保无向边的标准表示(小节点在前)
            int min_node = std::min(u, v);
            int max_node = std::max(u, v);
            edgeSet.insert(std::to_string(min_node) + "_" + std::to_string(max_node));
        }
        
        // 4. 使用DFS检测环路
        std::unordered_set<int> visited;
        std::unordered_map<int, int> parent;
        std::vector<std::pair<int, int>> loopEdges; // 存储环中的边
        
        // 深度优先搜索函数
        std::function<bool(int, int)> dfs = [&](int node, int par) -> bool {
            visited.insert(node);
            parent[node] = par;
            
            for(int neighbor : graph[node]) {
                // 跳过父节点
                if(neighbor == par) continue;
                
                if(visited.count(neighbor)) {
                    // 找到环路，回溯并收集环中的边
                    int current = node;
                    int next = neighbor;
                    loopEdges.clear();
                    
                    while(current != neighbor) {
                        // 确保边的标准表示
                        int min_node = std::min(current, next);
                        int max_node = std::max(current, next);
                        loopEdges.emplace_back(min_node, max_node);
                        
                        next = current;
                        current = parent[current];
                    }
                    
                    // 添加最后一条边，完成环
                    int min_node = std::min(current, next);
                    int max_node = std::max(current, next);
                    loopEdges.emplace_back(min_node, max_node);
                    
                    return true; // 找到环路
                }
                
                if(!visited.count(neighbor)) {
                    if(dfs(neighbor, node)) return true;
                }
            }
            
            return false;
        };
        
        bool hasRemovedEdge = false;
        
        // 对每个连通分量运行DFS，直到没有环路
        do {
            hasRemovedEdge = false;
            visited.clear();
            
            for(const auto& [node, _] : graph) {
                if(!visited.count(node)) {
                    loopEdges.clear();
                    if(dfs(node, -1) && !loopEdges.empty()) {
                        // 找到环路，尝试移除非必要边
                        bool removed = false;
                        
                        for(const auto& [u, v] : loopEdges) {
                            int min_node = std::min(u, v);
                            int max_node = std::max(u, v);
                            std::string edge_key = std::to_string(min_node) + "_" + std::to_string(max_node);
                            
                            // 如果这条边不是必要边，可以移除
                            if(!essentialEdges.count(edge_key)) {
                                edgeSet.erase(edge_key);
                                removed = true;
                                hasRemovedEdge = true;
                                
                                // 更新图结构
                                auto& neighbors_u = graph[u];
                                auto& neighbors_v = graph[v];
                                neighbors_u.erase(std::remove(neighbors_u.begin(), neighbors_u.end(), v), neighbors_u.end());
                                neighbors_v.erase(std::remove(neighbors_v.begin(), neighbors_v.end(), u), neighbors_v.end());
                                
                                if(debug) std::cout << "Removed non-essential edge " << u << "-" << v << " from a loop" << std::endl;
                                break;
                            }
                        }
                        
                        // 如果环中全是必要边，这种情况理论上不应该出现
                        // 但如果确实出现，我们可以选择移除最后一条边，并重新计算源点到汇点的路径
                        if(!removed && !loopEdges.empty()) {
                            const auto& [u, v] = loopEdges.back();
                            int min_node = std::min(u, v);
                            int max_node = std::max(u, v);
                            std::string edge_key = std::to_string(min_node) + "_" + std::to_string(max_node);
                            
                            edgeSet.erase(edge_key);
                            hasRemovedEdge = true;
                            
                            // 更新图结构
                            auto& neighbors_u = graph[u];
                            auto& neighbors_v = graph[v];
                            neighbors_u.erase(std::remove(neighbors_u.begin(), neighbors_u.end(), v), neighbors_u.end());
                            neighbors_v.erase(std::remove(neighbors_v.begin(), neighbors_v.end(), u), neighbors_v.end());
                            
                            if(debug) std::cout << "Warning: Removed essential edge " << u << "-" << v << " from a loop" << std::endl;
                        }
                        
                        // 如果移除了边，需要重新开始DFS
                        if(hasRemovedEdge) break;
                    }
                }
            }
            
            // 如果移除了边，重置visited并重新检测环路
            if(hasRemovedEdge) {
                visited.clear();
            }
        } while(hasRemovedEdge);
        
        // 5. 重建没有环路的边集合
        std::vector<std::pair<int, int>> newEdges;
        for(const auto& [u, v] : edges) {
            int min_node = std::min(u, v);
            int max_node = std::max(u, v);
            std::string edge_key = std::to_string(min_node) + "_" + std::to_string(max_node);
            
            if(edgeSet.count(edge_key)) {
                newEdges.emplace_back(u, v);
            }
        }
        
        return newEdges;
    }
};