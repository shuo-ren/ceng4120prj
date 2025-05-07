// ========================= router.h  =========================
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

#include "dataloader.h"
#include "routecache.h"

class Router {
public:
    Router(const NodeMap &n, const AdjacencyList &a, const NetList &nl,
           const std::string &out, bool dbg = false)
        : nodes(n), adj(a), nets(nl), outPath(out), debug(dbg)
    { occ.resize(nodes.size(), 0); }

    void setCache(RouteCache *c) { cache = c; }

    /* -------- top‑level -------- */
    void route() {
        std::ofstream ofs(outPath);
        if(!ofs) throw std::runtime_error("open out");

        /* 保证 net 按 id 升序输出 */
        std::vector<size_t> order(nets.size());
        std::iota(order.begin(), order.end(), 0);
        std::sort(order.begin(), order.end(),
                 [&](size_t a,size_t b){return nets[a].id < nets[b].id;});

        for(size_t idx:order){
            const Net &net=nets[idx];
            if(cache && cache->hasNet(net.id)) writeCached(net.id,ofs);
            else                              routeNet(net,ofs);
        }
        if(cache) cache->save("cache.dat");
    }

private:
    /* ===== data ===== */
    const NodeMap& nodes;
    const AdjacencyList& adj;
    const NetList& nets;
    std::string outPath;
    bool  debug;
    std::vector<char> occ;          // global occupancy bitmap
    RouteCache *cache=nullptr;

    /* ===== cache output ===== */
    void writeCached(int id,std::ofstream&ofs){
        const auto &el=cache->getEdges(id);
        ofs<<id<<' '<<el.name<<'\n';
        for(auto &[u,v]:el.edges){ ofs<<u<<' '<<v<<'\n'; occ[u]=occ[v]=1; }
        ofs<<'\n';
    }

    /* ===== A* ===== */
    struct PQ{int n;double g,f;int p;bool operator<(const PQ&o)const{return f>o.f;}};
    double h(int a,int b)const{
        const auto&A=nodes[a],&B=nodes[b];
        return std::abs(A.begin_x-B.begin_x)+std::abs(A.begin_y-B.begin_y);
    }

    std::vector<int> astar(const std::unordered_set<int>&tree,int goal){
        std::priority_queue<PQ> pq;
        std::unordered_map<int,double> g;
        std::unordered_map<int,int>    par;

        for(int s:tree){
            pq.push({s,0,h(s,goal),-1});
            g[s]=0.0;  par[s]=-1;
        }

        while(!pq.empty()){
            auto cur=pq.top(); pq.pop();
            if(cur.n==goal){ break; }
            if(cur.g!=g[cur.n]) continue;

            for(int nb:adj[cur.n]){
                if(occ[nb] && !tree.count(nb)) continue;   // <- 只阻塞其他网
                double ng=cur.g+1;
                if(!g.count(nb)||ng<g[nb]){
                    g[nb]=ng; par[nb]=cur.n;
                    pq.push({nb,ng,ng+h(nb,goal),cur.n});
                }
            }
        }

        if(!par.count(goal)) return {};
        std::vector<int> path;
        for(int v=goal; v!=-1; v=par[v]) path.push_back(v);
        std::reverse(path.begin(),path.end());
        return path;
    }

    /* ===== write new path & update cache ===== */
    void writePath(const Net&net,const std::vector<int>&p,
                   std::ofstream&ofs,std::unordered_set<int>&tree){
        if(p.size()<2) return;
        for(int v:p) if(occ[v] && !tree.count(v)) return;   // allow reuse inside tree

        int prev=p.front();
        std::vector<std::pair<int,int>> edges;
        for(size_t i=1;i<p.size();++i){
            int v=p[i];
            ofs<<prev<<' '<<v<<'\n';
            occ[prev]=occ[v]=1;
            edges.emplace_back(prev,v);
            prev=v; tree.insert(v);
        }
        if(cache) cache->addNet(net.id, net.name, edges);
    }

    /* ===== per‑net routing ===== */
    void routeNet(const Net&net,std::ofstream&ofs){
        ofs<<net.id<<' '<<net.name<<'\n';
        std::unordered_set<int> tree{net.source};

        for(int sink:net.sinks){
            auto path=astar(tree,sink);
            if(path.empty()){
                if(debug) std::cerr<<"Net "<<net.id<<" failed sink "<<sink<<'\n';
                continue;
            }
            writePath(net,path,ofs,tree);
        }
        ofs<<'\n';
    }
};
