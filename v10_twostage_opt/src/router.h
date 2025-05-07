// Modified router.h with adaptive parameters based on netlist size

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

class Router {
    using Clock = std::chrono::high_resolution_clock;
public:
    Router(const NodeMap &n, const AdjacencyList &a, const NetList &nl,
           const std::string &out, bool dbg, int num_threads,
           double readTimeSec, double timeLimitSec)
        : nodes(n), adj(a), nets(nl), outPath(out), debug(dbg),
          threads(num_threads), t_read(readTimeSec), limit(timeLimitSec)
    {
        occ.resize(nodes.size(), 0);                // 占用计数
        
        // Set parameters based on netlist size
        setupParametersByNetlistSize();
    }

    /* ───────────────────────── 顶层 ───────────────────────── */
    void route() {
        auto routeStart = Clock::now();

        /* ① 首轮并行布线 */
        firstRoundRouting();

        /* ② 预算计算 */
        auto roundEnd = Clock::now();
        double t_first     = std::chrono::duration<double>(roundEnd - routeStart).count();
        double t_write_est = t_read * 0.5;
        double budget      = (limit - t_read - t_write_est - t_first) * timeBudgetRatio;

        if(debug){
            std::cout << "Time budget for adaptive reroute: "
                      << budget << " s\n";
        }

        /* ③ 自适应迭代 rip‑up & reroute */
        if(budget > minimumRerouteTime) adaptiveRipupReroute(budget);

        /* ④ 输出文件 */
        writeResult();
    }

private:
    /* ───────── 数据成员 ───────── */
    const NodeMap&        nodes;
    const AdjacencyList&  adj;
    const NetList&        nets;
    std::string           outPath;
    bool                  debug;
    int                   threads;

    std::vector<uint8_t>  occ;          // 占用计数 bitmap

    // Adaptive parameters
    int maxExpand;                      // A* max expansion limit
    int maxRerouteIterations;           // Maximum reroute iterations
    double timeBudgetRatio;             // Time budget ratio
    double minimumRerouteTime;          // Minimum time required to attempt reroute
    double timePerIteration;            // Estimated time needed per iteration

    struct RouteResult {
        int id{};
        std::string name;
        std::vector<std::pair<int,int>> edges;
        bool allSinks{false};           // sink 是否全部连通
        bool removed{false};            // 拥塞/迭代失败被删
    };
    std::vector<RouteResult> results;
    std::vector<bool>        routedOK;

    double t_read;                      // 读取时间
    double limit;                       // 总时限

    /* ───────── 设置基于网表大小的参数 ───────── */
    void setupParametersByNetlistSize() {
        // Default parameters (medium size)
        maxExpand = 60000;
        maxRerouteIterations = 10;
        
        size_t netSize = nets.size();
        
        // Very small design (e.g., design1)
        if (netSize < 100) {
            maxExpand = 40000;
            maxRerouteIterations = 20;     // More iterations for small designs
            timeBudgetRatio = 0.98;        // Use more of the available time
            minimumRerouteTime = 1.0;      // Even 1 second is enough to try rerouting
            timePerIteration = 0.2;        // Each iteration is fast
        }
        // Small design (e.g., design2)
        else if (netSize < 500) {
            maxExpand = 50000;
            maxRerouteIterations = 15;
            timeBudgetRatio = 0.95;        // Use slightly less time
            minimumRerouteTime = 2.0;      // Need at least 2 seconds
            timePerIteration = 0.5;
        }
        // Medium design (e.g., design3)
        else if (netSize < 2000) {
            maxExpand = 60000;
            maxRerouteIterations = 10;
            timeBudgetRatio = 0.90;        // More conservative
            minimumRerouteTime = 3.0;      // Need at least 3 seconds
            timePerIteration = 1.0;
        }
        // Large design (e.g., design4)
        else if (netSize < 5000) {
            maxExpand = 70000;
            maxRerouteIterations = 8;      // Fewer iterations for large designs
            timeBudgetRatio = 0.85;        // Much more conservative
            minimumRerouteTime = 5.0;      // Need at least 5 seconds
            timePerIteration = 2.0;
        }
        // Very large design (e.g., design5)
        else {
            maxExpand = 80000;
            maxRerouteIterations = 2;      // Very few iterations for very large designs
            timeBudgetRatio = 0.80;        // Extremely conservative
            minimumRerouteTime = 30.0;     // Need at least 10 seconds
            timePerIteration = 20.0;
        }
        
        if (debug) {
            std::cout << "Setting parameters for netlist with " << netSize << " nets:\n"
                      << "- maxExpand: " << maxExpand << "\n"
                      << "- maxRerouteIterations: " << maxRerouteIterations << "\n"
                      << "- timeBudgetRatio: " << timeBudgetRatio << "\n"
                      << "- minimumRerouteTime: " << minimumRerouteTime << " s\n"
                      << "- timePerIteration: " << timePerIteration << " s" << std::endl;
        }
    }

    /* ───────── A* 数据结构 ───────── */
    struct PQ {
        int n; double g, f; int p;
        bool operator<(const PQ& o) const { return f > o.f; }
    };

    double h(int a, int b) const {
        const auto &A = nodes[a], &B = nodes[b];
        return std::abs(A.begin_x - B.begin_x) +
               std::abs(A.begin_y - B.begin_y);
    }

    /* ─────────────────── 首轮并行布线 ─────────────────── */
    void firstRoundRouting() {
        results.resize(nets.size());
        routedOK.assign(nets.size(), false);

        std::vector<size_t> order(nets.size());
        std::iota(order.begin(), order.end(), 0);
        std::sort(order.begin(), order.end(),
                  [&](size_t a, size_t b){
                      return nets[a].sinks.size() < nets[b].sinks.size();
                  });

        #pragma omp parallel for schedule(dynamic) num_threads(threads)
        for(int i = 0; i < order.size(); ++i) {
            size_t idx  = order[i];
            const Net& net = nets[idx];
            auto res = routeNetParallel(net, false);   // useHistory = false

            #pragma omp critical
            {
                results[idx] = { net.id, net.name,
                                 std::move(res.edges),
                                 res.ok, false };
                routedOK[idx] = res.ok;
            }
        }

        loopRemovalAll();
        resolveCongestion();            // 可能设置 removed=true
    }

    /* ─────────────────── 自适应迭代 rip‑up & reroute ─────────────────── */
    void adaptiveRipupReroute(double budget) {
        auto iterStart = Clock::now();
        int iteration  = 0;

        while(iteration < maxRerouteIterations) {
            /* 1) 收集需重布的 nets */
            std::vector<size_t> retry;
            for(size_t i = 0; i < results.size(); ++i)
                if(!results[i].allSinks || results[i].removed)
                    retry.push_back(i);
            if(retry.empty()) break;

            double elapsed = std::chrono::duration<double>(
                                 Clock::now() - iterStart).count();
            
            // Check if we have enough time for another iteration
            double remainingTime = budget - elapsed;
            if(remainingTime < timePerIteration) {
                if(debug)
                    std::cout << "Stopping reroute: remaining time " 
                              << remainingTime << "s is less than estimated time per iteration " 
                              << timePerIteration << "s\n";
                break;
            }

            if(debug)
                std::cout << "-- Iter " << ++iteration
                          << " retry " << retry.size() << " nets"
                          << " (remaining time: " << remainingTime << "s)\n";

            /* 2) rip‑up */
            for(size_t idx : retry) {
                for(auto [u,v] : results[idx].edges) {
                    if(occ[u]) --occ[u];
                    if(occ[v]) --occ[v];
                }
                results[idx].edges.clear();
            }

            /* 3) 重布 */
            #pragma omp parallel for schedule(dynamic) num_threads(threads)
            for(int k = 0; k < retry.size(); ++k) {
                size_t idx = retry[k];
                const Net& net = nets[idx];
                auto res = routeNetParallel(net, true);   // useHistory = true

                #pragma omp critical
                {
                    results[idx].edges    = std::move(res.edges);
                    results[idx].allSinks = res.ok;
                    results[idx].removed  = !res.ok;       // 初步标记
                }
            }

            loopRemovalAll();
            resolveCongestion();        // 更新 removed

            elapsed = std::chrono::duration<double>(
                          Clock::now() - iterStart).count();
            
            // Check if we have enough time for another iteration
            remainingTime = budget - elapsed;
            if(remainingTime < timePerIteration) {
                if(debug)
                    std::cout << "Stopping reroute after iteration: remaining time " 
                              << remainingTime << "s is less than estimated time per iteration " 
                              << timePerIteration << "s\n";
                break;
            }
        }
    }

    /* ────────────────────────── 写文件 ────────────────────────── */
    void writeResult() {
        std::vector<size_t> outOrder(results.size());
        std::iota(outOrder.begin(), outOrder.end(), 0);
        std::sort(outOrder.begin(), outOrder.end(),
                  [&](size_t a, size_t b){
                      return nets[a].id < nets[b].id;
                  });

        std::ofstream ofs(outPath);
        if(!ofs) throw std::runtime_error("open out file");

        size_t okCnt = 0;
        for(size_t idx : outOrder) {
            if(!results[idx].allSinks)  continue;
            if(results[idx].removed)    continue;

            ++okCnt;
            auto &r = results[idx];
            ofs << r.id << ' ' << r.name << '\n';
            for(auto [u, v] : r.edges) ofs << u << ' ' << v << '\n';
            ofs << '\n';
        }
        ofs.close();

        std::cout << "Remaining successfully routed nets: "
                  << okCnt << '/' << nets.size() << '\n';
    }

    /* ────────────────────── A* 路由单网 ────────────────────── */
    struct RouteRet {
        std::vector<std::pair<int,int>> edges;
        bool ok;
    };

    RouteRet routeNetParallel(const Net& net, bool useHistory) {
        auto penalty = [&](int nb){
            return useHistory ? 0.5 * occ[nb] : 0.0;      // history cost
        };

        std::vector<std::pair<int,int>> allEdges;
        std::unordered_set<int> tree{net.source};
        std::vector<uint8_t> localOcc;

        #pragma omp critical
        { localOcc = occ; }

        for(int sink : net.sinks) {
            auto path = astar(tree, sink, localOcc, penalty);
            if(path.empty()){
                if(debug)
                    std::cerr << "Net " << net.id
                              << " failed sink " << sink << '\n';
                continue;
            }
            bool valid = true;
            for(int v : path)
                if(localOcc[v] && !tree.count(v)) { valid=false; break;}

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
        for(int s : net.sinks)
            if(!tree.count(s)) { ok = false; break; }

        return { std::move(allEdges), ok };
    }

    std::vector<int> astar(const std::unordered_set<int>& tree, int goal,
                           const std::vector<uint8_t>& localOcc,
                           std::function<double(int)> extraCost) {
        std::priority_queue<PQ> pq;
        std::unordered_map<int,double> g;
        std::unordered_map<int,int>    par;

        for(int s : tree){
            pq.push({s,0,h(s,goal),-1});
            g[s] = 0; par[s] = -1;
        }

        int expand = 0;
        while(!pq.empty() && expand < maxExpand) {
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

    /* ──────────────────── Loop Removal（全部 nets） ──────────────────── */
    void loopRemovalAll() {
        for(size_t i = 0; i < results.size(); ++i) {
            if(results[i].removed) continue;
            const Net &net = nets[i];
            results[i].edges =
                removeRouteLoops(results[i].edges, net.source, net.sinks);
        }
    }

    /* ──────────────────── 移除单网环路 ──────────────────── */
    std::vector<std::pair<int,int>>
    removeRouteLoops(const std::vector<std::pair<int,int>>& edges,
                     int source, const std::vector<int>& sinks) {

        if(edges.empty()) return edges;

        /* 1. 邻接表 */
        std::unordered_map<int,std::vector<int>> graph;
        for(auto [u,v]: edges){
            graph[u].push_back(v);
            graph[v].push_back(u);
        }

        /* 2. 标记必要边（源→每个 sink 最短路径） */
        std::unordered_set<std::string> essential;
        auto edgeKey=[&](int x,int y){
            int a=std::min(x,y), b=std::max(x,y);
            return std::to_string(a)+'_'+std::to_string(b);
        };

        for(int sink : sinks){
            std::queue<int> q;
            std::unordered_set<int> vis;
            std::unordered_map<int,int> par;
            q.push(source); vis.insert(source); par[source]=-1;
            bool found=false;

            while(!q.empty() && !found){
                int cur=q.front(); q.pop();
                if(cur==sink){found=true;break;}
                for(int nb:graph[cur]){
                    if(vis.insert(nb).second){
                        par[nb]=cur; q.push(nb);
                    }
                }
            }
            if(found){
                int cur=sink;
                while(par[cur]!=-1){
                    essential.insert(edgeKey(cur,par[cur]));
                    cur=par[cur];
                }
            }
        }

        /* 3. Edge set */
        std::unordered_set<std::string> edgeSet;
        for(auto [u,v]: edges) edgeSet.insert(edgeKey(u,v));

        /* 4. DFS 找环并删边 */
        std::unordered_set<int> visited;
        std::unordered_map<int,int> parent;

        std::function<bool(int,int,std::vector<std::pair<int,int>>&)> dfs =
        [&](int node,int par,std::vector<std::pair<int,int>>& loop)->bool{
            visited.insert(node); parent[node]=par;
            for(int nb:graph[node]){
                if(nb==par) continue;
                if(visited.count(nb)){
                    loop.clear();
                    int a=node,b=nb;
                    while(a!=nb){
                        loop.emplace_back(a,parent[a]);
                        a=parent[a];
                    }
                    loop.emplace_back(node,nb);
                    return true;
                }
                if(dfs(nb,node,loop)) return true;
            }
            return false;
        };

        bool removedEdge=true;
        while(removedEdge){
            removedEdge=false;
            visited.clear();

            for(auto &[node,_]:graph){
                if(!visited.count(node)){
                    std::vector<std::pair<int,int>> loop;
                    if(dfs(node,-1,loop) && !loop.empty()){
                        /* 尝试删非必要边 */
                        bool deleted=false;
                        for(auto [u,v]: loop){
                            auto key=edgeKey(u,v);
                            if(!essential.count(key)){
                                edgeSet.erase(key);
                                auto& gu=graph[u];
                                auto& gv=graph[v];
                                gu.erase(std::remove(gu.begin(),gu.end(),v),gu.end());
                                gv.erase(std::remove(gv.begin(),gv.end(),u),gv.end());
                                removedEdge=deleted=true; break;
                            }
                        }
                        /* 如果全是必要边，删最后一条 */
                        if(!deleted){
                            auto [u,v]=loop.back();
                            auto key=edgeKey(u,v);
                            edgeSet.erase(key);
                            auto& gu=graph[u];
                            auto& gv=graph[v];
                            gu.erase(std::remove(gu.begin(),gu.end(),v),gu.end());
                            gv.erase(std::remove(gv.begin(),gv.end(),u),gv.end());
                            removedEdge=true;
                        }
                        if(removedEdge) break;          // 重新 DFS
                    }
                }
            }
        }

        /* 5. 重建边集合 */
        std::vector<std::pair<int,int>> newEdges;
        for(auto [u,v]: edges)
            if(edgeSet.count(edgeKey(u,v)))
                newEdges.emplace_back(u,v);

        return newEdges;
    }

    /* ──────────────────── 拥塞检测与解决 ──────────────────── */
    void resolveCongestion() {
        std::unordered_map<int,std::unordered_set<size_t>> nodeToNets;
        for(size_t i=0;i<results.size();++i){
            if(results[i].removed) continue;
            for(auto [u,v]: results[i].edges){
                nodeToNets[u].insert(i);
                nodeToNets[v].insert(i);
            }
        }
        std::unordered_set<int> cong;
        for(auto &[n,ns]:nodeToNets) if(ns.size()>1) cong.insert(n);
        if(cong.empty()) return;

        if(debug)
            std::cout << "Detected " << cong.size()
                      << " congested nodes\n";

        std::vector<std::pair<size_t,int>> order;
        for(size_t i=0;i<results.size();++i){
            if(results[i].removed) continue;
            int contrib=0;
            for(auto [u,v]:results[i].edges){
                if(cong.count(u)) ++contrib;
                if(cong.count(v)) ++contrib;
            }
            if(contrib) order.emplace_back(i,contrib);
        }
        std::sort(order.begin(),order.end(),
                  [](auto&a,auto&b){return a.second>b.second;});

        for(auto [idx,_] : order){
            bool still=false;
            for(auto &[node,ns]:nodeToNets)
                if(ns.size()>1){ still=true; break; }
            if(!still) break;

            results[idx].removed=true;
            for(auto [u,v]:results[idx].edges){
                nodeToNets[u].erase(idx);
                nodeToNets[v].erase(idx);
                if(occ[u]) --occ[u];
                if(occ[v]) --occ[v];
            }
        }
    }
};