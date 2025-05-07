// MeduimRouter.h - Optimized for medium designs
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
#include <chrono>

#include "dataloader.h"

class MediumRouter {
    using Clock = std::chrono::high_resolution_clock;
public:
    MediumRouter(const NodeMap &n, const AdjacencyList &a, const NetList &nl,
               const std::string &out, bool dbg, int num_threads,
               double readTimeSec, double timeLimitSec)
        : nodes(n), adj(a), nets(nl), outPath(out), debug(dbg),
          threads(num_threads), t_read(readTimeSec), limit(timeLimitSec)
    {
        occ.resize(nodes.size(), 0);
        
        if(debug) {
            std::cout << "Using LargeRouter optimized for very large designs\n";
        }
    }

    void route() {
        auto routeStart = Clock::now();
        
        /* ① 首轮布线 - 保守策略 */
        firstRoundRouting();
        
        /* ② 预算计算 - 大型设计时间预算更严格 */
        auto roundEnd = Clock::now();
        double t_first = std::chrono::duration<double>(roundEnd - routeStart).count();
        double t_write_est = t_read * 0.5;
        double budget = (limit - t_read - t_write_est - t_first) * 0.80; // More conservative
        
        if(debug) {
            std::cout << "Time budget for adaptive reroute: " << budget << " s\n";
        }
        
        /* ③ 有限次数的 rip‑up & reroute */
        if(budget > 10.0) adaptiveRipupReroute(budget);
        
        /* ③-½ 计算最终阶段的预算 */
        auto rrrEnd = Clock::now();
        double t_reroute = std::chrono::duration<double>(rrrEnd - roundEnd).count();
        double finalBudget = (limit - t_read - t_write_est - t_first - t_reroute) * 0.90;
        
        /* ③-④ 增加 aggressive routing 环节 */
        if(finalBudget > 20.0) {
            finalAggressiveRouting(finalBudget * 0.6); // Use 60% of remaining time
            
            /* ③-⑤ 计算最终尖锐路由的预算 */
            auto aggressiveEnd = Clock::now();
            double t_aggressive = std::chrono::duration<double>(aggressiveEnd - rrrEnd).count();
            double sharpBudget = (limit - t_read - t_write_est - t_first - t_reroute - t_aggressive) * 0.95;
            
            /* ③-⑥ 最后尝试锐利的路由方法解决剩余少量nets */
            if(sharpBudget > 30.0) sharp_routing(sharpBudget);
        }
        
        /* ④ 输出文件 */
        writeResult();
    }

private:
    const NodeMap&        nodes;
    const AdjacencyList&  adj;
    const NetList&        nets;
    std::string           outPath;
    bool                  debug;
    int                   threads;
    
    std::vector<uint8_t>  occ;          // Occupancy bitmap
    
    // Parameters optimized for large designs
    static constexpr int MAX_EXPAND = 50000;       // Increased search depth
    static constexpr int MAX_ITERATIONS = 10;       // Fewer iterations
    static constexpr double HISTORY_WEIGHT = 0.8;  // Heavier congestion penalty
    static constexpr double MIN_TIME_PER_ITERATION = 20.0; // seconds
    
    struct RouteResult {
        int id{};
        std::string name;
        std::vector<std::pair<int,int>> edges;
        bool allSinks{false};
        bool removed{false};
    };
    
    std::vector<RouteResult> results;
    std::vector<bool>        routedOK;
    
    double t_read;
    double limit;
    
    // A* priority queue structure
    struct PQ {
        int n; double g, f; int p;
        bool operator<(const PQ& o) const { return f > o.f; }
    };

    double h(int a, int b) const {
        const auto &A = nodes[a], &B = nodes[b];
        return std::abs(A.begin_x - B.begin_x) + 
               std::abs(A.begin_y - B.begin_y);
    }
    
    void firstRoundRouting() {
        results.resize(nets.size());
        routedOK.assign(nets.size(), false);
        
        // For large designs, order nets by sink count AND estimated complexity
        std::vector<size_t> order(nets.size());
        std::iota(order.begin(), order.end(), 0);
        
        // More sophisticated ordering for large designs
        std::sort(order.begin(), order.end(), 
                 [&](size_t a, size_t b) {
                     // First sort by number of sinks
                     if(nets[a].sinks.size() != nets[b].sinks.size()) {
                         return nets[a].sinks.size() < nets[b].sinks.size();
                     }
                     
                     // Then estimate complexity by total Manhattan distance
                     double distA = 0, distB = 0;
                     for(int sink : nets[a].sinks) {
                         distA += h(nets[a].source, sink);
                     }
                     for(int sink : nets[b].sinks) {
                         distB += h(nets[b].source, sink);
                     }
                     return distA < distB;
                 });
        
        #pragma omp parallel for schedule(dynamic) num_threads(threads)
        for(int i = 0; i < order.size(); ++i) {
            size_t idx = order[i];
            const Net& net = nets[idx];
            
            // Large designs: apply small initial congestion weight
            auto res = routeNet(net, 0.1);
            
            #pragma omp critical
            {
                results[idx] = { net.id, net.name, 
                                std::move(res.edges), 
                                res.ok, false };
                routedOK[idx] = res.ok;
            }
        }
        
        loopRemovalAll();
        resolveCongestion();
    }
    
    void adaptiveRipupReroute(double budget) {
        auto iterStart = Clock::now();
        int iteration = 0;
        
        // For large designs, be very careful with time
        double estimatedTimePerIteration = budget / MAX_ITERATIONS;
        
        while(iteration < MAX_ITERATIONS) {
            // Find failed and removed nets
            std::vector<size_t> retry;
            for(size_t i = 0; i < results.size(); ++i) {
                if(!results[i].allSinks || results[i].removed) {
                    retry.push_back(i);
                }
            }
            
            if(retry.empty()) break;
            
            // Check if we have enough time for this iteration
            double elapsed = std::chrono::duration<double>(
                Clock::now() - iterStart).count();
            double remainingTime = budget - elapsed;
            
            if(remainingTime < MIN_TIME_PER_ITERATION) {
                if(debug) {
                    std::cout << "Stopping reroute: remaining time " 
                              << remainingTime << "s is insufficient\n";
                }
                break;
            }
            
            if(debug) {
                std::cout << "-- Iter " << ++iteration
                          << " retry " << retry.size() << " nets"
                          << " (remaining: " << remainingTime << "s)\n";
            }
            
            // Rip-up nets to retry
            for(size_t idx : retry) {
                for(auto [u,v] : results[idx].edges) {
                    if(occ[u]) --occ[u];
                    if(occ[v]) --occ[v];
                }
                results[idx].edges.clear();
            }
            
            // Calculate adaptive history weight
            double adaptiveWeight = HISTORY_WEIGHT * (1.0 + 0.2 * iteration);
            
            // Reroute with increasing history penalty
            #pragma omp parallel for schedule(dynamic) num_threads(threads)
            for(int k = 0; k < retry.size(); ++k) {
                size_t idx = retry[k];
                const Net& net = nets[idx];
                auto res = routeNet(net, adaptiveWeight);
                
                #pragma omp critical
                {
                    results[idx].edges = std::move(res.edges);
                    results[idx].allSinks = res.ok;
                    results[idx].removed = !res.ok;
                }
            }
            
            loopRemovalAll();
            resolveCongestion();
            
            // Update time estimate
            double iterTime = std::chrono::duration<double>(
                Clock::now() - iterStart).count() - elapsed;
            estimatedTimePerIteration = (estimatedTimePerIteration + iterTime) / 2;
            
            // Check if we're out of time
            elapsed = std::chrono::duration<double>(
                Clock::now() - iterStart).count();
            remainingTime = budget - elapsed;
            
            if(remainingTime < estimatedTimePerIteration) {
                if(debug) {
                    std::cout << "Stopping after iteration " << iteration
                              << ": remaining time " << remainingTime 
                              << "s is insufficient\n";
                }
                break;
            }
        }
    }
    
    struct RouteRet {
        std::vector<std::pair<int,int>> edges;
        bool ok;
    };
    
    RouteRet routeNet(const Net& net, double historyWeight) {
        auto penalty = [&](int nb) {
            return historyWeight * occ[nb];
        };
        
        std::vector<std::pair<int,int>> allEdges;
        std::unordered_set<int> tree{net.source};
        std::vector<uint8_t> localOcc;
        
        #pragma omp critical
        { localOcc = occ; }
        
        // For large designs: order sinks by distance from source
        std::vector<int> orderedSinks = net.sinks;
        std::sort(orderedSinks.begin(), orderedSinks.end(),
                 [&](int a, int b) {
                     return h(net.source, a) < h(net.source, b);
                 });
        
        for(int sink : orderedSinks) {
            auto path = astar(tree, sink, localOcc, penalty);
            if(path.empty()) {
                if(debug) {
                    std::cerr << "Net " << net.id 
                              << " failed sink " << sink << '\n';
                }
                continue;
            }
            
            // Validate path
            bool valid = true;
            for(int v : path) {
                if(localOcc[v] && !tree.count(v)) { 
                    valid = false; 
                    break;
                }
            }
            
            if(valid && path.size() >= 2) {
                int prev = path.front();
                for(size_t i = 1; i < path.size(); ++i) {
                    int v = path[i];
                    allEdges.emplace_back(prev, v);
                    localOcc[prev]++; localOcc[v]++;
                    prev = v; tree.insert(v);
                }
            }
        }
        
        #pragma omp critical
        { for(auto [u,v]: allEdges){ occ[u]++; occ[v]++; } }
        
        bool ok = true;
        for(int s : net.sinks) {
            if(!tree.count(s)) { 
                ok = false; 
                break; 
            }
        }
        
        return { std::move(allEdges), ok };
    }
    
    std::vector<int> astar(const std::unordered_set<int>& tree, int goal,
                          const std::vector<uint8_t>& localOcc,
                          std::function<double(int)> extraCost) {
        std::priority_queue<PQ> pq;
        std::unordered_map<int,double> g;
        std::unordered_map<int,int> par;
        
        for(int s : tree) {
            pq.push({s, 0, h(s,goal), -1});
            g[s] = 0; par[s] = -1;
        }
        
        int expand = 0;
        while(!pq.empty() && expand < MAX_EXPAND) {
            auto cur = pq.top(); pq.pop();
            if(cur.n == goal) break;
            if(cur.g != g[cur.n]) continue;
            ++expand;
            
            for(int nb : adj[cur.n]) {
                if(localOcc[nb] && !tree.count(nb)) continue;
                double ng = cur.g + 1 + extraCost(nb);
                if(!g.count(nb) || ng < g[nb]) {
                    g[nb] = ng; par[nb] = cur.n;
                    pq.push({nb, ng, ng + h(nb,goal), cur.n});
                }
            }
        }
        
        if(!par.count(goal)) return {};
        
        std::vector<int> path;
        for(int v = goal; v != -1; v = par[v]) path.push_back(v);
        std::reverse(path.begin(), path.end());
        return path;
    }
    
    void loopRemovalAll() {
        for(size_t i = 0; i < results.size(); ++i) {
            if(results[i].removed) continue;
            const Net &net = nets[i];
            results[i].edges = 
                removeRouteLoops(results[i].edges, net.source, net.sinks);
        }
    }
    
    std::vector<std::pair<int,int>> removeRouteLoops(
        const std::vector<std::pair<int,int>>& edges,
        int source, const std::vector<int>& sinks) {
        
        if(edges.empty()) return edges;
        
        // Build graph
        std::unordered_map<int, std::vector<int>> graph;
        for(auto [u,v] : edges) {
            graph[u].push_back(v);
            graph[v].push_back(u);
        }
        
        // Mark essential edges
        std::unordered_set<std::string> essential;
        auto edgeKey = [](int x, int y) {
            int a = std::min(x,y), b = std::max(x,y);
            return std::to_string(a) + "_" + std::to_string(b);
        };
        
        for(int sink : sinks) {
            std::queue<int> q;
            std::unordered_set<int> vis;
            std::unordered_map<int,int> par;
            q.push(source); vis.insert(source); par[source] = -1;
            bool found = false;
            
            while(!q.empty() && !found) {
                int cur = q.front(); q.pop();
                if(cur == sink) { found = true; break; }
                for(int nb : graph[cur]) {
                    if(vis.insert(nb).second) {
                        par[nb] = cur; q.push(nb);
                    }
                }
            }
            
            if(found) {
                int cur = sink;
                while(par[cur] != -1) {
                    essential.insert(edgeKey(cur, par[cur]));
                    cur = par[cur];
                }
            }
        }
        
        // Build edge set
        std::unordered_set<std::string> edgeSet;
        for(auto [u,v] : edges) edgeSet.insert(edgeKey(u,v));
        
        // Find and remove cycles
        std::unordered_set<int> visited;
        std::unordered_map<int,int> parent;
        
        std::function<bool(int,int,std::vector<std::pair<int,int>>&)> dfs =
        [&](int node, int par, std::vector<std::pair<int,int>>& loop) -> bool {
            visited.insert(node); parent[node] = par;
            for(int nb : graph[node]) {
                if(nb == par) continue;
                if(visited.count(nb)) {
                    loop.clear();
                    int a = node, b = nb;
                    while(a != nb) {
                        loop.emplace_back(a, parent[a]);
                        a = parent[a];
                    }
                    loop.emplace_back(node, nb);
                    return true;
                }
                if(dfs(nb, node, loop)) return true;
            }
            return false;
        };
        
        bool removedEdge = true;
        while(removedEdge) {
            removedEdge = false;
            visited.clear();
            
            for(auto& [node, _] : graph) {
                if(!visited.count(node)) {
                    std::vector<std::pair<int,int>> loop;
                    if(dfs(node, -1, loop) && !loop.empty()) {
                        // Try removing non-essential edge first
                        bool deleted = false;
                        for(auto [u,v] : loop) {
                            auto key = edgeKey(u,v);
                            if(!essential.count(key)) {
                                edgeSet.erase(key);
                                auto& gu = graph[u];
                                auto& gv = graph[v];
                                gu.erase(std::remove(gu.begin(), gu.end(), v), gu.end());
                                gv.erase(std::remove(gv.begin(), gv.end(), u), gv.end());
                                removedEdge = deleted = true;
                                break;
                            }
                        }
                        
                        // If all edges were essential, still need to break the cycle
                        if(!deleted) {
                            auto [u,v] = loop.back();
                            auto key = edgeKey(u,v);
                            edgeSet.erase(key);
                            auto& gu = graph[u];
                            auto& gv = graph[v];
                            gu.erase(std::remove(gu.begin(), gu.end(), v), gu.end());
                            gv.erase(std::remove(gv.begin(), gv.end(), u), gv.end());
                            removedEdge = true;
                        }
                        
                        if(removedEdge) break; // Restart DFS after breaking cycle
                    }
                }
            }
        }
        
        // Rebuild edge list
        std::vector<std::pair<int,int>> newEdges;
        for(auto [u,v] : edges) {
            if(edgeSet.count(edgeKey(u,v))) {
                newEdges.emplace_back(u,v);
            }
        }
        
        return newEdges;
    }
    
    void resolveCongestion() {
        std::unordered_map<int, std::unordered_set<size_t>> nodeToNets;
        for(size_t i = 0; i < results.size(); ++i) {
            if(results[i].removed) continue;
            for(auto [u,v] : results[i].edges) {
                nodeToNets[u].insert(i);
                nodeToNets[v].insert(i);
            }
        }
        
        std::unordered_set<int> cong;
        for(auto& [n, ns] : nodeToNets) {
            if(ns.size() > 1) cong.insert(n);
        }
        
        if(cong.empty()) return;
        
        if(debug) {
            std::cout << "Detected " << cong.size() 
                      << " congested nodes\n";
        }
        
        // For large designs, prioritize removing nets with more sinks
        // in addition to congestion contribution
        std::vector<std::tuple<size_t, int, size_t>> order;
        for(size_t i = 0; i < results.size(); ++i) {
            if(results[i].removed) continue;
            int contrib = 0;
            for(auto [u,v] : results[i].edges) {
                if(cong.count(u)) ++contrib;
                if(cong.count(v)) ++contrib;
            }
            if(contrib) {
                // Include sink count as a factor
                order.emplace_back(i, contrib, nets[i].sinks.size());
            }
        }
        
        // Sort by congestion contribution first, then by sink count
        std::sort(order.begin(), order.end(),
                 [](const auto& a, const auto& b) {
                     if(std::get<1>(a) != std::get<1>(b)) {
                         return std::get<1>(a) > std::get<1>(b);
                     }
                     return std::get<2>(a) > std::get<2>(b);
                 });
        
        for(const auto& [idx, _, __] : order) {
            bool still = false;
            for(auto& [node, ns] : nodeToNets) {
                if(ns.size() > 1) { still = true; break; }
            }
            if(!still) break;
            
            results[idx].removed = true;
            for(auto [u,v] : results[idx].edges) {
                nodeToNets[u].erase(idx);
                nodeToNets[v].erase(idx);
                if(occ[u]) --occ[u];
                if(occ[v]) --occ[v];
            }
        }
    }
    
    void writeResult() {
        std::vector<size_t> outOrder(results.size());
        std::iota(outOrder.begin(), outOrder.end(), 0);
        std::sort(outOrder.begin(), outOrder.end(),
                 [&](size_t a, size_t b) {
                     return nets[a].id < nets[b].id;
                 });
        
        std::ofstream ofs(outPath);
        if(!ofs) throw std::runtime_error("open out file");
        
        size_t okCnt = 0;
        for(size_t idx : outOrder) {
            if(!results[idx].allSinks) continue;
            if(results[idx].removed) continue;
            
            ++okCnt;
            auto& r = results[idx];
            ofs << r.id << ' ' << r.name << '\n';
            for(auto [u,v] : r.edges) ofs << u << ' ' << v << '\n';
            ofs << '\n';
        }
        ofs.close();
        
        std::cout << "Remaining successfully routed nets: "
                  << okCnt << '/' << nets.size() << '\n';
    }


    void finalAggressiveRouting(double remainingTime) {
        auto startTime = Clock::now();
        
        if (debug) {
            std::cout << "Starting final aggressive routing with " 
                      << remainingTime << " seconds remaining\n";
        }
        
        // Identify failed nets
        std::vector<size_t> failedNets;
        for (size_t i = 0; i < results.size(); ++i) {
            if (!results[i].allSinks || results[i].removed) {
                failedNets.push_back(i);
            }
        }
        
        if (failedNets.empty()) {
            if (debug) std::cout << "No failed nets to process\n";
            return;
        }
        
        if (debug) {
            std::cout << "Found " << failedNets.size() 
                      << " failed nets to attempt aggressive routing\n";
        }
        
        // Sort failed nets by complexity (fewer sinks first)
        std::sort(failedNets.begin(), failedNets.end(), 
                  [&](size_t a, size_t b) {
                      return nets[a].sinks.size() < nets[b].sinks.size();
                  });
        
        // Maximum time for a single net
        const double maxTimePerNet = std::min(remainingTime / 10.0, 30.0);
        // Maximum A* search expansion (more aggressive than normal)
        const int aggressiveMaxExpand = MAX_EXPAND * 3;
        
        // Track nets that were modified during this process
        std::unordered_set<size_t> modifiedNets;
        
        for (size_t idx : failedNets) {
            // Check remaining time
            double elapsed = std::chrono::duration<double>(
                Clock::now() - startTime).count();
            if (elapsed > remainingTime - maxTimePerNet) {
                if (debug) {
                    std::cout << "Time limit approaching, stopping aggressive routing\n";
                }
                break;
            }
            
            const Net& net = nets[idx];
            
            if (debug) {
                std::cout << "Attempting aggressive routing for net " 
                          << net.id << " (" << net.name << ") with " 
                          << net.sinks.size() << " sinks\n";
            }
            
            // Save current state for potential rollback
            auto netStartTime = Clock::now();
            std::vector<uint8_t> originalOcc = occ;
            std::vector<RouteResult> originalResults = results;
            
            // Track which nets we need to route
            std::unordered_set<size_t> netsToRoute{idx};
            bool success = true;
            
            // First pass: try to route this net with higher priority, potentially
            // removing other nets that are in the way
            std::vector<std::pair<int, int>> potentialConflicts;
            
            // Step 1: Remove this net's current routes (if any)
            for (auto [u, v] : results[idx].edges) {
                if (occ[u]) --occ[u];
                if (occ[v]) --occ[v];
            }
            results[idx].edges.clear();
            
            // Step 2: Find a path regardless of congestion
            // Create a more aggressive A* search function
            auto aggressiveRoute = [&](const Net& currentNet) -> RouteRet {
                std::vector<std::pair<int, int>> allEdges;
                std::unordered_set<int> tree{currentNet.source};
                std::vector<uint8_t> localOcc = occ;
                
                // Find congested nodes along possible paths
                for (int sink : currentNet.sinks) {
                    // First try to find path without considering congestion
                    auto searchPath = [&](int t, const std::vector<uint8_t>& searchOcc, 
                                          std::function<double(int)> penaltyFn, 
                                          int maxExpand) -> std::vector<int> {
                        std::priority_queue<PQ> pq;
                        std::unordered_map<int, double> g;
                        std::unordered_map<int, int> par;
                        
                        for (int s : tree) {
                            pq.push({s, 0, h(s, t), -1});
                            g[s] = 0; par[s] = -1;
                        }
                        
                        int expand = 0;
                        while (!pq.empty() && expand < maxExpand) {
                            auto cur = pq.top(); pq.pop();
                            if (cur.n == t) break;
                            if (cur.g != g[cur.n]) continue;
                            ++expand;
                            
                            for (int nb : adj[cur.n]) {
                                double ng = cur.g + 1 + penaltyFn(nb);
                                if (!g.count(nb) || ng < g[nb]) {
                                    g[nb] = ng; par[nb] = cur.n;
                                    pq.push({nb, ng, ng + h(nb, t), cur.n});
                                }
                            }
                        }
                        
                        if (!par.count(t)) return {};
                        
                        std::vector<int> path;
                        for (int v = t; v != -1; v = par[v]) path.push_back(v);
                        std::reverse(path.begin(), path.end());
                        return path;
                    };
                    
                    // First try an ignored-congestion search to find potential conflicts
                    auto path = searchPath(sink, localOcc, 
                                          [&](int nb) { return 0.0; }, 
                                          aggressiveMaxExpand);
                    
                    if (path.empty()) {
                        return { {}, false }; // Even with no congestion, no path exists
                    }
                    
                    // Identify net conflicts
                    for (int v : path) {
                        if (localOcc[v] > 0) {
                            potentialConflicts.emplace_back(v, localOcc[v]);
                        }
                    }
                    
                    // Now try with some congestion awareness but prioritize this net
                    auto penaltyPath = searchPath(sink, localOcc, 
                                                [&](int nb) { return 0.1 * localOcc[nb]; }, 
                                                aggressiveMaxExpand);
                    
                    if (penaltyPath.empty()) {
                        return { {}, false };
                    }
                    
                    if (penaltyPath.size() >= 2) {
                        int prev = penaltyPath.front();
                        for (size_t i = 1; i < penaltyPath.size(); ++i) {
                            int v = penaltyPath[i];
                            allEdges.emplace_back(prev, v);
                            localOcc[prev]++; localOcc[v]++;
                            prev = v;
                            tree.insert(v);
                        }
                    }
                }
                
                bool ok = true;
                for (int s : currentNet.sinks) {
                    if (!tree.count(s)) {
                        ok = false;
                        break;
                    }
                }
                
                return { std::move(allEdges), ok };
            };
            
            // Try to route this net
            auto netResult = aggressiveRoute(net);
            
            if (!netResult.ok) {
                // If we couldn't route even with aggressive settings, skip this net
                if (debug) {
                    std::cout << "Aggressive routing failed for net " 
                              << net.id << " even without congestion\n";
                }
                
                // Restore original state
                occ = originalOcc;
                results = originalResults;
                continue;
            }
            
            // Step 3: Identify which nets are affected by our new routing
            // Collect node-to-net mapping
            std::unordered_map<int, std::unordered_set<size_t>> nodeToNets;
            for (size_t i = 0; i < results.size(); ++i) {
                if (i == idx || results[i].removed) continue;
                for (auto [u, v] : results[i].edges) {
                    nodeToNets[u].insert(i);
                    nodeToNets[v].insert(i);
                }
            }
            
            // Find nets that might conflict with our new path
            std::unordered_set<size_t> affectedNets;
            for (auto [u, v] : netResult.edges) {
                occ[u]++; occ[v]++;
                
                // Add nets using these nodes to our affected set
                if (nodeToNets.count(u)) {
                    for (size_t netIdx : nodeToNets[u]) {
                        affectedNets.insert(netIdx);
                    }
                }
                if (nodeToNets.count(v)) {
                    for (size_t netIdx : nodeToNets[v]) {
                        affectedNets.insert(netIdx);
                    }
                }
            }
            
            // Step 4: See if our priority routing broke other nets
            // Remove affected nets and try to re-route them
            for (size_t affectedIdx : affectedNets) {
                // Remove this net's routes
                for (auto [u, v] : results[affectedIdx].edges) {
                    if (occ[u]) --occ[u];
                    if (occ[v]) --occ[v];
                }
                results[affectedIdx].edges.clear();
                results[affectedIdx].allSinks = false;
                netsToRoute.insert(affectedIdx);
            }
            
            // Step 5: Add our new route for this net
            results[idx].edges = netResult.edges;
            results[idx].allSinks = true;
            results[idx].removed = false;
            
            // Step 6: Try to route all affected nets
            for (size_t netToRoute : netsToRoute) {
                if (netToRoute == idx) continue; // Already routed
                
                // Use normal routing for other nets
                auto res = routeNet(nets[netToRoute], 0.5); // Medium congestion weight
                
                results[netToRoute].edges = std::move(res.edges);
                results[netToRoute].allSinks = res.ok;
                results[netToRoute].removed = !res.ok;
                
                if (!res.ok) {
                    success = false; // One of our affected nets failed
                }
            }
            
            // Step 7: Accept or revert changes
            if (success) {
                // All affected nets were routed successfully
                if (debug) {
                    std::cout << "Successfully routed net " << net.id 
                              << " and all " << affectedNets.size() 
                              << " affected nets\n";
                }
                
                // Add all nets we modified to our tracking set
                modifiedNets.insert(idx);
                for (size_t netIdx : affectedNets) {
                    modifiedNets.insert(netIdx);
                }
            } else {
                // Restore original state
                if (debug) {
                    std::cout << "Failed to route all affected nets, "
                              << "reverting changes\n";
                }
                occ = originalOcc;
                results = originalResults;
            }
            
            // Check if we've spent too much time on this net
            double netTime = std::chrono::duration<double>(
                Clock::now() - netStartTime).count();
                
            if (netTime > maxTimePerNet) {
                if (debug) {
                    std::cout << "Net " << net.id << " took " << netTime 
                              << "s (exceeding limit), adjusting future estimates\n";
                }
            }
        }
        
        // Fix any loops or congestion in modified nets
        if (!modifiedNets.empty()) {
            if (debug) {
                std::cout << "Performing cleanup on " << modifiedNets.size() 
                          << " modified nets\n";
            }
            
            // Loop removal
            for (size_t idx : modifiedNets) {
                if (results[idx].removed) continue;
                const Net &net = nets[idx];
                results[idx].edges = 
                    removeRouteLoops(results[idx].edges, net.source, net.sinks);
            }
            
            // Final congestion resolution
            resolveCongestion();
        }
        
        double totalTime = std::chrono::duration<double>(
            Clock::now() - startTime).count();
            
        if (debug) {
            std::cout << "Final aggressive routing completed in " 
                      << totalTime << " seconds\n";
        }
    }



    void sharp_routing(double remainingTime) {
        auto startTime = Clock::now();
        
        if (debug) {
            std::cout << "Starting sharp routing with " 
                      << remainingTime << " seconds remaining\n";
        }
        
        // Identify failed nets
        std::vector<size_t> failedNets;
        for (size_t i = 0; i < results.size(); ++i) {
            if (!results[i].allSinks || results[i].removed) {
                failedNets.push_back(i);
            }
        }
        
        if (failedNets.empty()) {
            if (debug) std::cout << "No failed nets to process in sharp routing\n";
            return;
        }
        
        if (debug) {
            std::cout << "Found " << failedNets.size() 
                      << " failed nets for sharp routing\n";
        }
        
        // If we have too many failed nets, this approach isn't practical
        if (failedNets.size() > 10) {
            if (debug) {
                std::cout << "Too many failed nets (" << failedNets.size() 
                          << ") for sharp routing, skipping\n";
            }
            return;
        }
        
        // Since we have very few nets, we can allocate a generous time budget per net
        const double maxTimePerNet = std::min(remainingTime / (failedNets.size() + 1), 120.0);
        
        // Use very aggressive parameters for this final attempt
        const int sharpMaxExpand = MAX_EXPAND * 5; // 5x normal expansion limit
        const int maxAttempts = 10; // Try multiple path variations
        
        // Track summary statistics
        int totalSuccessfulNets = 0;
        
        for (size_t idx : failedNets) {
            // Check if we still have enough time
            double elapsed = std::chrono::duration<double>(
                Clock::now() - startTime).count();
            if (elapsed > remainingTime - maxTimePerNet) {
                if (debug) {
                    std::cout << "Time limit approaching, stopping sharp routing\n";
                }
                break;
            }
            
            const Net& net = nets[idx];
            
            if (debug) {
                std::cout << "Attempting sharp routing for net " 
                          << net.id << " (" << net.name << ") with " 
                          << net.sinks.size() << " sinks\n";
            }
            
            auto netStartTime = Clock::now();
            
            // Save current state for potential rollback
            std::vector<uint8_t> originalOcc = occ;
            std::vector<RouteResult> originalResults = results;
            bool success = false;
            
            // Create a node to net mapping for quickly identifying affected nets
            std::unordered_map<int, std::unordered_set<size_t>> nodeToNets;
            for (size_t i = 0; i < results.size(); ++i) {
                if (i == idx || results[i].removed) continue;
                for (auto [u, v] : results[i].edges) {
                    nodeToNets[u].insert(i);
                    nodeToNets[v].insert(i);
                }
            }
            
            // Most aggressive A* search with minimal congestion consideration
            auto sharpAStar = [&](const std::unordered_set<int>& sources, int goal) -> std::vector<int> {
                std::priority_queue<PQ> pq;
                std::unordered_map<int, double> g;
                std::unordered_map<int, int> par;
                
                for (int s : sources) {
                    pq.push({s, 0, h(s, goal), -1});
                    g[s] = 0; par[s] = -1;
                }
                
                int expand = 0;
                while (!pq.empty() && expand < sharpMaxExpand) {
                    auto cur = pq.top(); pq.pop();
                    if (cur.n == goal) break;
                    if (cur.g != g[cur.n]) continue;
                    ++expand;
                    
                    for (int nb : adj[cur.n]) {
                        // Minimal congestion consideration
                        double penalty = 0.1 * occ[nb];
                        double ng = cur.g + 1 + penalty;
                        if (!g.count(nb) || ng < g[nb]) {
                            g[nb] = ng; par[nb] = cur.n;
                            pq.push({nb, ng, ng + h(nb, goal), cur.n});
                        }
                    }
                }
                
                if (!par.count(goal)) return {};
                
                std::vector<int> path;
                for (int v = goal; v != -1; v = par[v]) path.push_back(v);
                std::reverse(path.begin(), path.end());
                return path;
            };
            
            // Try multiple attempts with different strategies
            for (int attempt = 0; attempt < maxAttempts && !success; ++attempt) {
                // Start fresh from original state for each attempt
                occ = originalOcc;
                results = originalResults;
                
                // Remove this net's current routes (if any)
                for (auto [u, v] : results[idx].edges) {
                    if (occ[u]) --occ[u];
                    if (occ[v]) --occ[v];
                }
                results[idx].edges.clear();
                
                std::vector<std::pair<int, int>> newEdges;
                std::unordered_set<int> tree{net.source};
                bool allSinksRouted = true;
                
                // Add randomization for different attempts
                double randomFactor = 0.2 + (0.5 * attempt / maxAttempts);
                
                // Route each sink
                for (int sink : net.sinks) {
                    auto path = sharpAStar(tree, sink);
                    
                    if (path.empty()) {
                        allSinksRouted = false;
                        break;
                    }
                    
                    if (path.size() >= 2) {
                        int prev = path.front();
                        for (size_t i = 1; i < path.size(); ++i) {
                            int v = path[i];
                            newEdges.emplace_back(prev, v);
                            // We don't update occ yet - will do that after checking if rerouting works
                            prev = v;
                            tree.insert(v);
                        }
                    }
                }
                
                if (!allSinksRouted) {
                    continue; // Try next attempt
                }
                
                // Find which nets are affected by our new routing
                std::unordered_set<size_t> affectedNets;
                
                for (auto [u, v] : newEdges) {
                    // Check which nets use these nodes
                    if (nodeToNets.count(u)) {
                        for (size_t netIdx : nodeToNets[u]) {
                            affectedNets.insert(netIdx);
                        }
                    }
                    if (nodeToNets.count(v)) {
                        for (size_t netIdx : nodeToNets[v]) {
                            affectedNets.insert(netIdx);
                        }
                    }
                }
                
                if (debug) {
                    std::cout << "Attempt " << attempt + 1 
                              << ": Found path affecting " 
                              << affectedNets.size() << " other nets\n";
                }
                
                // Update node occupancy with our new routing
                for (auto [u, v] : newEdges) {
                    occ[u]++;
                    occ[v]++;
                }
                
                // Update our net's result
                results[idx].edges = newEdges;
                results[idx].allSinks = true;
                results[idx].removed = false;
                
                // Remove affected nets
                for (size_t affectedIdx : affectedNets) {
                    for (auto [u, v] : results[affectedIdx].edges) {
                        if (occ[u]) --occ[u];
                        if (occ[v]) --occ[v];
                    }
                    results[affectedIdx].edges.clear();
                    results[affectedIdx].allSinks = false;
                }
                
                // Try to reroute all affected nets
                bool allAffectedRouted = true;
                std::vector<size_t> reroutingOrder(affectedNets.begin(), affectedNets.end());
                
                // Sort affected nets by complexity (simpler nets first)
                std::sort(reroutingOrder.begin(), reroutingOrder.end(),
                         [&](size_t a, size_t b) {
                             return nets[a].sinks.size() < nets[b].sinks.size();
                         });
                
                for (size_t affectedIdx : reroutingOrder) {
                    // Use standard routing for affected nets, but with increased congestion tolerance
                    auto res = routeNet(nets[affectedIdx], 0.3);
                    
                    if (!res.ok) {
                        allAffectedRouted = false;
                        if (debug) {
                            std::cout << "Failed to reroute affected net " 
                                      << nets[affectedIdx].id << "\n";
                        }
                        break;
                    }
                    
                    results[affectedIdx].edges = std::move(res.edges);
                    results[affectedIdx].allSinks = true;
                    results[affectedIdx].removed = false;
                }
                
                // Check if our solution is valid
                if (allAffectedRouted) {
                    success = true;
                    if (debug) {
                        std::cout << "Successfully routed net " << net.id 
                                  << " and rerouted all " << affectedNets.size() 
                                  << " affected nets\n";
                    }
                    
                    // Clean up any loops
                    results[idx].edges = 
                        removeRouteLoops(results[idx].edges, net.source, net.sinks);
                        
                    for (size_t affectedIdx : affectedNets) {
                        const Net &affectedNet = nets[affectedIdx];
                        results[affectedIdx].edges = 
                            removeRouteLoops(results[affectedIdx].edges, 
                                            affectedNet.source, 
                                            affectedNet.sinks);
                    }
                }
            }
            
            // If we couldn't find a valid solution, revert to original state
            if (!success) {
                if (debug) {
                    std::cout << "Failed to find valid route for net " 
                              << net.id << " after " << maxAttempts << " attempts\n";
                }
                occ = originalOcc;
                results = originalResults;
            } else {
                totalSuccessfulNets++;
            }
            
            double netTime = std::chrono::duration<double>(
                Clock::now() - netStartTime).count();
                
            if (debug) {
                std::cout << "Sharp routing for net " << net.id 
                          << " took " << netTime << " seconds ("
                          << (success ? "success" : "failure") << ")\n";
            }
        }
        
        // Final congestion resolution for all nets
        resolveCongestion();
        
        double totalTime = std::chrono::duration<double>(
            Clock::now() - startTime).count();
            
        if (debug) {
            std::cout << "Sharp routing completed in " << totalTime 
                      << " seconds, fixed " << totalSuccessfulNets 
                      << "/" << failedNets.size() << " nets\n";
        }
    }





};