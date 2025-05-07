// MediumRouter.h - Optimized for medium designs (design2, design3, design4)
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
            std::cout << "Using MediumRouter optimized for medium-sized designs\n";
        }
    }

    void route() {
        auto routeStart = Clock::now();
        
        /* ① 首轮布线 - 按复杂度排序 */
        firstRoundRouting();
        
        /* ② 循环迭代改进 */
        auto roundEnd = Clock::now();
        double t_first = std::chrono::duration<double>(roundEnd - routeStart).count();
        double t_write_est = t_read * 0.5;
        double budget = (limit - t_read - t_write_est - t_first) * 0.92; // Medium designs
        
        if(debug) {
            std::cout << "Time budget for adaptive reroute: " << budget << " s\n";
        }
        
        if(budget > 2.0) adaptiveReroute(budget);
        
        /* ③ 输出结果 */
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
    
    // Parameters optimized for medium designs
    static constexpr int MAX_EXPAND = 50000; 
    static constexpr int MAX_ITERATIONS = 6;
    static constexpr double HISTORY_WEIGHT = 0.6;
    static constexpr double MIN_TIME_PER_ITERATION = 10; // seconds
    
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
        
        // Sort nets by sinks count (simpler nets first)
        std::vector<size_t> order(nets.size());
        std::iota(order.begin(), order.end(), 0);
        std::sort(order.begin(), order.end(), 
                 [&](size_t a, size_t b) {
                     return nets[a].sinks.size() < nets[b].sinks.size();
                 });
        
        #pragma omp parallel for schedule(dynamic) num_threads(threads)
        for(int i = 0; i < order.size(); ++i) {
            size_t idx = order[i];
            const Net& net = nets[idx];
            auto res = routeNet(net, false); // No history cost initially
            
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
    
    void adaptiveReroute(double budget) {
        auto iterStart = Clock::now();
        int iteration = 0;
        
        while(iteration < MAX_ITERATIONS) {
            // Find nets to retry
            std::vector<size_t> retry;
            for(size_t i = 0; i < results.size(); ++i) {
                if(!results[i].allSinks || results[i].removed) {
                    retry.push_back(i);
                }
            }
            
            if(retry.empty()) break;
            
            // Check time
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
            
            // Rip-up
            for(size_t idx : retry) {
                for(auto [u,v] : results[idx].edges) {
                    if(occ[u]) --occ[u];
                    if(occ[v]) --occ[v];
                }
                results[idx].edges.clear();
            }
            
            // Reroute with history cost
            #pragma omp parallel for schedule(dynamic) num_threads(threads)
            for(int k = 0; k < retry.size(); ++k) {
                size_t idx = retry[k];
                const Net& net = nets[idx];
                auto res = routeNet(net, true); // Use history cost
                
                #pragma omp critical
                {
                    results[idx].edges = std::move(res.edges);
                    results[idx].allSinks = res.ok;
                    results[idx].removed = !res.ok;
                }
            }
            
            loopRemovalAll();
            resolveCongestion();
            
            // Check if we're out of time
            elapsed = std::chrono::duration<double>(
                Clock::now() - iterStart).count();
            remainingTime = budget - elapsed;
            
            if(remainingTime < MIN_TIME_PER_ITERATION) {
                if(debug) {
                    std::cout << "Stopping after iteration " << iteration
                              << ": remaining time " << remainingTime 
                              << "s is insufficient for another iteration\n";
                }
                break;
            }
        }
    }
    
    struct RouteRet {
        std::vector<std::pair<int,int>> edges;
        bool ok;
    };
    
    RouteRet routeNet(const Net& net, bool useHistory) {
        auto penalty = [&](int nb) {
            return useHistory ? HISTORY_WEIGHT * occ[nb] : 0.0;
        };
        
        std::vector<std::pair<int,int>> allEdges;
        std::unordered_set<int> tree{net.source};
        std::vector<uint8_t> localOcc;
        
        #pragma omp critical
        { localOcc = occ; }
        
        for(int sink : net.sinks) {
            auto path = astar(tree, sink, localOcc, penalty);
            if(path.empty()) {
                if(debug) {
                    std::cerr << "Net " << net.id 
                              << " failed sink " << sink << '\n';
                }
                continue;
            }
            
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
        
        std::vector<std::pair<size_t, int>> order;
        for(size_t i = 0; i < results.size(); ++i) {
            if(results[i].removed) continue;
            int contrib = 0;
            for(auto [u,v] : results[i].edges) {
                if(cong.count(u)) ++contrib;
                if(cong.count(v)) ++contrib;
            }
            if(contrib) order.emplace_back(i, contrib);
        }
        
        std::sort(order.begin(), order.end(),
                 [](auto& a, auto& b) {
                     return a.second > b.second;
                 });
        
        for(auto [idx, _] : order) {
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
};