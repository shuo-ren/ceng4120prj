// SmallRouter.h – fully self‑contained multithread‑aware version
// -----------------------------------------------------------------------------
#pragma once
#include <vector>
#include <string>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <limits>
#include <chrono>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <random>
#include <omp.h>
#include "dataloader.h" 

/*---------------------------------------------------------------------------
 * Forward declarations (same structs used in DataLoader.h)
 *---------------------------------------------------------------------------*/
// struct Node {
//     int id, length;
//     int begin_x, begin_y, end_x, end_y;
// };
// struct Net {
//     int id;
//     std::string name;
//     int source;
//     std::vector<int> sinks;
// };
using NodeMap       = std::vector<Node>;
using AdjacencyList = std::vector<std::vector<int>>;
using NetList       = std::vector<Net>;

/*===========================================================================*/
class SmallRouter {
private:
    // ---------------- Tunable Parameters ------------------
    // These parameters control optimization behavior and runtime
    struct Parameters {
        // Optimization control
        int MAX_PASSES = 400;           // Maximum optimization passes to attempt
        int STAGNATION_LIMIT = 100;     // Break after this many non-improving passes
        double SAFETY_MARGIN = 15.0;     // Safety margin in seconds before timeLimit
        double MIN_REMAIN_TIME = 30.0;  // Minimum remaining time to start optimization
        
        // A* search parameters
        int BASE_MAX_EXPL = 2000000;    // Base exploration nodes per A* search
        int MAX_EXPL_CAP = 5000000;     // Maximum exploration limit
        int EXPL_TIME_FACTOR = 30000;   // Additional nodes per second of remaining time
        
        // Penalty strategy
        int EARLY_PENALTY = 1500;       // Cross penalty for early passes (< EARLY_PASS_THRESHOLD)
        int MID_PENALTY = 800;          // Cross penalty for middle passes
        int LATE_PENALTY = 400;         // Cross penalty for later passes
        int EARLY_PASS_THRESHOLD = 5;   // Threshold for early passes
        int MID_PASS_THRESHOLD = 15;    // Threshold for middle passes
        
        // Output
        bool VERBOSE_DEBUG = false;     // Print extra debug info during optimization
        
        // Heartbeat and monitoring
        bool ENABLE_HEARTBEAT = true;   // Enable heartbeat status messages
        int HEARTBEAT_INTERVAL = 10;    // Print status every N passes
        double TIME_CHECK_INTERVAL = 1.0; // Check time remaining every N seconds
    };
    
    Parameters params;

public:
    SmallRouter(const NodeMap &nodes, const AdjacencyList &adj,
                const NetList &nets, const std::string &outFile,
                bool debug, int threads, double readTime, double timeLimit)
        : nodes(nodes), adj(adj), nets(nets), outFile(outFile),
          debug(debug), threads(std::max(1, threads)),
          readTime(readTime), timeLimit(timeLimit) {
              
        // Adjust parameters based on design size or other criteria
        if (nets.size() > 1000) {
            // For larger designs, reduce exploration to save time
            params.BASE_MAX_EXPL = 100000;
            params.MAX_EXPL_CAP = 1000000;
        } else if (nets.size() < 50) {
            // For tiny designs, explore more aggressively
            params.BASE_MAX_EXPL = 500000;
            params.MAX_EXPL_CAP = 3000000;
            params.EXPL_TIME_FACTOR = 50000;
        }
    }

    /*---------------------------------------------------------------------*/
    void route() {
        auto globalStart = std::chrono::high_resolution_clock::now();
        /*---------------- containers ----------------*/
        std::unordered_map<int, std::vector<std::pair<int,int>>> result;
        std::unordered_map<int,int> congestion;
        std::unordered_map<int,int> netWL;

        baselineRoute(result, congestion, netWL);
        int totalWL = accumulateWL(netWL);
        if (debug) std::cout << "[Baseline] WL=" << totalWL << " nets=" << result.size() << "/" << nets.size() << "\n";

        /*--------------- optimisation window ---------------*/
        double elapsed = secondsSince(globalStart);
        double remain  = timeLimit - readTime - elapsed - params.SAFETY_MARGIN;
        if (debug && params.VERBOSE_DEBUG) {
            std::cout << "[Optimize] Time budget: " << timeLimit << "s, Elapsed: " << elapsed
                      << "s, Remaining: " << remain << "s\n";
        }
        
        if (result.size() == nets.size() && remain > params.MIN_REMAIN_TIME) {
            optimise(result, congestion, netWL, totalWL, globalStart);
        } else if (debug) {
            if (result.size() < nets.size()) {
                std::cout << "[Skip Optimize] Not all nets routed\n";
            } else {
                std::cout << "[Skip Optimize] Insufficient time remaining\n";
            }
        }
        
        /*--------------- write result ----------------------*/
        writeResults(result);
        double wall = secondsSince(globalStart);
        if (debug) std::cout << "[Done] WL=" << totalWL << " time=" << wall << "s\n";
    }

private:
    /*====================  HIGH‑LEVEL HELPERS  ====================*/
    double secondsSince(const std::chrono::high_resolution_clock::time_point &t0) const {
        return std::chrono::duration<double>(std::chrono::high_resolution_clock::now()-t0).count();
    }
    int accumulateWL(const std::unordered_map<int,int> &m) const {
        int s=0; for (auto &kv:m) s+=kv.second; return s; }

    /*====================  BASELINE ROUTER  =======================*/
    void baselineRoute(std::unordered_map<int,std::vector<std::pair<int,int>>> &result,
                       std::unordered_map<int,int> &congestion,
                       std::unordered_map<int,int> &netWL) {
        std::vector<size_t> order(nets.size());
        std::iota(order.begin(), order.end(), 0);
        std::sort(order.begin(), order.end(), [&](size_t a,size_t b){return nets[a].sinks.size()>nets[b].sinks.size();});

        for (size_t idx: order) {
            const Net &net = nets[idx];
            std::unordered_set<int> netNodes{net.source};
            std::vector<std::pair<int,int>> edges; int wl=0; bool ok=true;
            /* sinks desc distance */
            std::vector<std::pair<int,int>> ss; for(int s:net.sinks) ss.emplace_back(distance(net.source,s),s);
            std::sort(ss.begin(), ss.end(), std::greater<>());
            for (auto &pr:ss) {
                int sink = pr.second;
                int start = nearest(netNodes,sink);
                std::vector<int> path = findPathSimple(start,sink,congestion,net.id,netNodes);
                if (path.empty() && start!=net.source)
                    path = findPathSimple(net.source,sink,congestion,net.id,netNodes);
                if (path.empty()) path = findPathBFS(net.source,sink,congestion,net.id,netNodes);
                if (path.empty()) { ok=false; break; }
                append(path,edges,netNodes,wl);
            }
            if (ok) {
                result[net.id]=std::move(edges);
                for(int n:netNodes) congestion[n]=net.id;
                netWL[net.id]=wl;
            }
        }
    }

    /*====================  OPTIMISER  =============================*/
    struct Proposal {
        int netId=-1; 
        int newWL=0; 
        std::vector<std::pair<int,int>> edges; 
        std::vector<int> nodes;
    };

    void optimise(std::unordered_map<int,std::vector<std::pair<int,int>>> &result,
                  std::unordered_map<int,int> &congestion,
                  std::unordered_map<int,int> &netWL,
                  int &totalWL,
                  const std::chrono::high_resolution_clock::time_point &globalStart) {
        // Best solution tracking
        int bestWL = totalWL; 
        auto bestResult = result; 
        auto bestNetWL = netWL;
        
        // Stagnation tracking
        int stagnate = 0;
        int passesPerformed = 0;
        int improvingPasses = 0;
        
        // Time tracking
        auto lastHeartbeat = std::chrono::high_resolution_clock::now();
        auto lastTimeCheck = std::chrono::high_resolution_clock::now();
        
        if (debug && params.ENABLE_HEARTBEAT) {
            std::cout << "[Optimise] Starting optimization with WL=" << totalWL 
                      << ", time limit=" << timeLimit << "s" << std::endl;
        }
        
        for (int pass = 0; pass < params.MAX_PASSES; ++pass) {
            // Frequent time check
            auto currentTime = std::chrono::high_resolution_clock::now();
            double timeSinceLastCheck = std::chrono::duration<double>(currentTime - lastTimeCheck).count();
            
            if (timeSinceLastCheck > params.TIME_CHECK_INTERVAL) {
                lastTimeCheck = currentTime;
                double timeCheck = secondsSince(globalStart);
                
                if (timeCheck + 3.0 > timeLimit) {
                    if (debug) {
                        std::cout << "[Optimise] Breaking early due to time limit at pass " << pass 
                                  << " (elapsed=" << timeCheck << "s of " << timeLimit << "s)" << std::endl;
                    }
                    break;
                }
            }
            
            // Heartbeat output
            if (params.ENABLE_HEARTBEAT && debug && (pass % params.HEARTBEAT_INTERVAL == 0 || 
                std::chrono::duration<double>(currentTime - lastHeartbeat).count() > 5.0)) {
                lastHeartbeat = currentTime;
                double elapsed = secondsSince(globalStart);
                double remaining = timeLimit - elapsed;
                std::cout << "[Heartbeat] Pass " << pass << "/" << params.MAX_PASSES 
                          << ", Current WL=" << totalWL << ", Best WL=" << bestWL
                          << ", Time=" << elapsed << "s, Remaining=" << remaining << "s" 
                          << std::endl;
            }
            
            // Calculate time for optimisePass
            double remainingTime = timeLimit - secondsSince(globalStart);
            bool improved = optimisePass(result, congestion, netWL, totalWL, pass, remainingTime);
            passesPerformed++;
            
            if (improved) {
                improvingPasses++;
                stagnate = 0;  // Reset stagnation counter on improvement
                if (totalWL < bestWL) {
                    bestWL = totalWL;
                    bestResult = result;
                    bestNetWL = netWL;
                    
                    if (debug) {
                        std::cout << "[Optimise] Pass " << pass << " improved WL to " << bestWL 
                                  << " (Δ=" << (totalWL - bestWL) << ")" << std::endl;
                    }
                }
            } else if (++stagnate >= params.STAGNATION_LIMIT) {
                if (debug) {
                    std::cout << "[Optimise] Breaking due to stagnation after " << pass << " passes ("
                              << stagnate << " non-improving passes in a row)" << std::endl;
                }
                break;
            }
        }
        
        // Always return the best solution found
        result.swap(bestResult);
        netWL.swap(bestNetWL);
        totalWL = bestWL;
        
        if (debug) {
            std::cout << "[Optimise] Complete: " << passesPerformed << " passes ("
                      << improvingPasses << " improving), WL reduced from " 
                      << bestWL << " to " << totalWL << std::endl;
        }
    }

    bool optimisePass(std::unordered_map<int,std::vector<std::pair<int,int>>> &result,
                      std::unordered_map<int,int> &congestion,
                      std::unordered_map<int,int> &netWL,
                      int &totalWL,
                      int passIdx,
                      double remainingTime) {
        int nN = static_cast<int>(nets.size());
        std::vector<Proposal> prop(nN);

        // Calculate adaptive parameters based on pass index and remaining time
        int maxExpl = std::min(params.MAX_EXPL_CAP, 
                              params.BASE_MAX_EXPL + static_cast<int>(remainingTime * params.EXPL_TIME_FACTOR));
        
        int crossPenalty = (passIdx < params.EARLY_PASS_THRESHOLD) ? params.EARLY_PENALTY : 
                           (passIdx < params.MID_PASS_THRESHOLD) ? params.MID_PENALTY : 
                           params.LATE_PENALTY;
                           
        if (debug && params.VERBOSE_DEBUG) {
            std::cout << "[Pass " << passIdx << "] maxExpl=" << maxExpl 
                      << ", crossPenalty=" << crossPenalty 
                      << ", remainingTime=" << remainingTime << "s" << std::endl;
        }

        /* PLAN – parallel */
        auto planStart = std::chrono::high_resolution_clock::now();
        
#pragma omp parallel for schedule(dynamic) num_threads(threads)
        for (int i = 0; i < nN; ++i) {
            const Net &net = nets[i]; 
            if (!result.count(net.id)) continue;
            
            std::unordered_map<int,int> localCong = congestion;
            for (auto &e: result[net.id]) { 
                localCong.erase(e.first); 
                localCong.erase(e.second);
            }    
            
            std::unordered_set<int> netNodes{net.source};
            std::vector<std::pair<int,int>> edges; 
            int newWL = 0; 
            bool fail = false;
            
            // Create sink distance pairs
            std::vector<std::pair<int,int>> sinks; 
            for(int s : net.sinks) {
                sinks.emplace_back(distance(net.source, s), s);
            }
            
            // Randomize sink order to escape local minima
            static thread_local std::mt19937 rng(12345 + omp_get_thread_num());
            std::shuffle(sinks.begin(), sinks.end(), rng);
            // Resort by distance (introducing some variability while maintaining structure)
            std::sort(sinks.begin(), sinks.end());
            
            for (auto &pr : sinks) { 
                int sink = pr.second; 
                int start = nearest(netNodes, sink);
                // Use optimized path finding with adaptive parameters
                auto path = findPathOptimized(start, sink, localCong, net.id, netNodes, 
                                             true, crossPenalty, maxExpl);
                
                if (path.empty() && start != net.source) {
                    path = findPathOptimized(net.source, sink, localCong, net.id, netNodes,
                                            true, crossPenalty, maxExpl);
                }
                
                if (path.empty()) {
                    fail = true;
                    break;
                } 
                
                append(path, edges, netNodes, newWL);
            }    
            
            if (!fail && newWL < netWL[net.id]) { 
                prop[i].netId = net.id; 
                prop[i].newWL = newWL; 
                prop[i].edges = std::move(edges);
                prop[i].nodes.assign(netNodes.begin(), netNodes.end());
            }
        }

        /* COMMIT – serial - with prioritized proposals */
        auto commitStart = std::chrono::high_resolution_clock::now();
        double planTime = std::chrono::duration<double>(commitStart - planStart).count();
        bool improved = false;
        
        // Collect valid proposals
        std::vector<Proposal> validProps;
        for (auto &p : prop) {
            if (p.netId != -1) {
                validProps.push_back(p);
            }
        }
        
        if (debug && params.VERBOSE_DEBUG && !validProps.empty()) {
            std::cout << "[Pass " << passIdx << "] Generated " << validProps.size() 
                      << " improvement proposals in " << planTime << "s" << std::endl;
        }
        
        // Sort proposals by gain (largest improvements first)
        std::sort(validProps.begin(), validProps.end(), [&netWL](const Proposal &a, const Proposal &b) {
            return (netWL.at(a.netId) - a.newWL) > (netWL.at(b.netId) - b.newWL);
        });
        
        // Process sorted proposals
        int acceptedProps = 0;
        int totalSavings = 0;
        
        for (auto &p : validProps) {
            bool conflict = false;
            
            // Check for conflicts with current congestion map
            for (int n : p.nodes) {
                auto it = congestion.find(n);
                if (it != congestion.end() && it->second != p.netId) {
                    conflict = true;
                    break;
                }
            }
            
            if (conflict) continue;
            
            // Apply the proposal
            int savings = netWL[p.netId] - p.newWL;
            
            for (auto &e : result[p.netId]) {
                congestion.erase(e.first);
                congestion.erase(e.second);
            }
            
            totalWL -= netWL[p.netId];
            result[p.netId] = std::move(p.edges);
            netWL[p.netId] = p.newWL;
            totalWL += p.newWL;
            
            for (int n : p.nodes) {
                congestion[n] = p.netId;
            }
            
            acceptedProps++;
            totalSavings += savings;
            improved = true;
        }
        
        auto endTime = std::chrono::high_resolution_clock::now();
        
        // Use commitTime to avoid the unused variable warning
        double commitTime = std::chrono::duration<double>(endTime - commitStart).count();
        double totalTime = std::chrono::duration<double>(endTime - planStart).count();
        
        if (debug && params.VERBOSE_DEBUG) {
            if (acceptedProps > 0) {
                std::cout << "[Pass " << passIdx << "] Accepted " << acceptedProps << "/"
                          << validProps.size() << " proposals, saving " << totalSavings
                          << " WL, new total=" << totalWL 
                          << " (plan: " << planTime << "s, commit: " << commitTime << "s)" << std::endl;
            } else if (!validProps.empty()) {
                std::cout << "[Pass " << passIdx << "] No proposals accepted due to conflicts"
                          << " (time: " << totalTime << "s)" << std::endl;
            }
        }
        
        return improved;
    }

    /*====================  PATHFINDING  ============================*/
    using Edge = std::pair<int,int>;
    using PQElement = std::pair<int,int>; // cost,node

    std::vector<int> findPathSimple(int src, int dst, const std::unordered_map<int,int> &cong, 
                                   int netId, const std::unordered_set<int> &used) {
        return astar(src, dst, cong, netId, used, false, false, 0, 500000);
    }
    
    std::vector<int> findPathBFS(int src, int dst, const std::unordered_map<int,int> &cong, 
                                int netId, const std::unordered_set<int> &used) {
        std::queue<int> q;
        std::unordered_map<int,int> prev;
        std::unordered_set<int> vis{src};
        q.push(src);
        
        while (!q.empty()) {
            int cur = q.front();
            q.pop();
            
            if (cur == dst) return rebuild(src, dst, prev);
            
            for (int nb : adj[cur]) {
                if (vis.count(nb)) continue;
                if (used.count(nb) && nb != dst) continue;
                
                auto it = cong.find(nb);
                if (it != cong.end() && it->second != netId) continue;
                
                vis.insert(nb);
                prev[nb] = cur;
                q.push(nb);
            }
        }
        
        return {};
    }
    
    std::vector<int> findPathExtreme(int src, int dst, const std::unordered_map<int,int> &cong,
                                    int netId, const std::unordered_set<int> &used, int maxExpl = 500000) {
        return astar(src, dst, cong, netId, used, true, true, 1000, maxExpl);
    }
    
    std::vector<int> findPathAStar(int src, int dst, const std::unordered_map<int,int> &cong,
                                  int netId, const std::unordered_set<int> &used, int maxExpl = 500000) {
        return astar(src, dst, cong, netId, used, false, false, 0, maxExpl);
    }
    
    std::vector<int> findPathOptimized(int src, int dst, const std::unordered_map<int,int> &cong,
                                      int netId, const std::unordered_set<int> &used,
                                      bool emphasiseWL = true, int crossPenalty = 0, int maxExpl = 500000) {
        return astar(src, dst, cong, netId, used, false, emphasiseWL, crossPenalty, maxExpl);
    }

    /* core A* helper */
    std::vector<int> astar(int src, int dst, const std::unordered_map<int,int> &cong, int netId, 
                           const std::unordered_set<int> &used, bool allowCross, bool emphasiseWL, 
                           int crossPenalty, int maxExpl) {
        std::priority_queue<PQElement, std::vector<PQElement>, std::greater<PQElement>> pq;
        std::unordered_map<int,int> g, prev;
        g[src] = 0;
        pq.emplace(distance(src, dst), src);
        
        int explore = 0;
        while (!pq.empty() && explore < maxExpl) {
            auto [cost, u] = pq.top();
            pq.pop();
            ++explore;
            
            if (u == dst) return rebuild(src, dst, prev);
            
            for (int v : adj[u]) {
                if (used.count(v) && v != dst) continue;
                
                int nodeCost = nodes[v].length;
                auto it = cong.find(v);
                
                if (it != cong.end() && it->second != netId) {
                    if (!allowCross) continue;
                    nodeCost += crossPenalty;
                }
                
                int ng = g[u] + nodeCost;
                if (!g.count(v) || ng < g[v]) {
                    g[v] = ng;
                    prev[v] = u;
                    int h = distance(v, dst);
                    int f = ng + h + (emphasiseWL ? nodeCost : 0);
                    pq.emplace(f, v);
                }
            }
        }
        
        return {};
    }

    std::vector<int> rebuild(int src, int dst, const std::unordered_map<int,int> &prev) {
        std::vector<int> p;
        int cur = dst;
        while (cur != src) {
            p.push_back(cur);
            cur = prev.at(cur);
        }
        p.push_back(src);
        std::reverse(p.begin(), p.end());
        return p;
    }

    /*====================  GEOMETRY & UTIL  ========================*/
    int distance(int n1, int n2) const {
        const Node &a = nodes[n1], &b = nodes[n2];
        int x1 = (a.begin_x + a.end_x) / 2, y1 = (a.begin_y + a.end_y) / 2;
        int x2 = (b.begin_x + b.end_x) / 2, y2 = (b.begin_y + b.end_y) / 2;
        return std::abs(x1 - x2) + std::abs(y1 - y2);
    }
    
    int nearest(const std::unordered_set<int> &S, int sink) const {
        int best = *S.begin(), d = distance(best, sink);
        for (int n : S) {
            int dn = distance(n, sink);
            if (dn < d) {
                d = dn;
                best = n;
            }
        }
        return best;
    }

    void append(const std::vector<int> &path, std::vector<Edge> &edges, 
                std::unordered_set<int> &nodesSet, int &wl) const {
        for (size_t i = 1; i < path.size(); ++i) {
            edges.emplace_back(path[i-1], path[i]);
            nodesSet.insert(path[i]);
            wl += nodes[path[i]].length;
        }
    }

    /*====================  OUTPUT  =================================*/
    void writeResults(const std::unordered_map<int, std::vector<Edge>> &res) const {
        std::ofstream fout(outFile);
        if (!fout) throw std::runtime_error("Cannot open output file: " + outFile);
        
        std::vector<int> ids;
        ids.reserve(res.size());
        for (auto &kv : res) ids.push_back(kv.first);
        std::sort(ids.begin(), ids.end());
        
        for (size_t k = 0; k < ids.size(); ++k) {
            int id = ids[k];
            const std::string &name = nets[id].name;
            fout << id << ' ' << name << "\n";
            
            for (auto &e : res.at(id)) {
                fout << e.first << ' ' << e.second << "\n";
            }
            
            if (k + 1 < ids.size()) fout << "\n";
        }
    }

    /*====================  DATA  ===================================*/
    const NodeMap       &nodes;
    const AdjacencyList &adj;
    const NetList       &nets;
    std::string          outFile;
    bool                 debug;
    int                  threads;
    double               readTime;
    double               timeLimit;
};