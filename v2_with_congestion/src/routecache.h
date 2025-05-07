// routecache.h
#pragma once
#include <unordered_map>
#include <vector>
#include <string>
#include <fstream>

struct EdgeList {
    std::string name;
    std::vector<std::pair<int,int>> edges;
};

class RouteCache {
public:
    /* ----- query ----- */
    bool hasNet(int id)            const { return table.count(id); }
    const EdgeList& getEdges(int id)const { return table.at(id);   }
    std::string getNetName(int id) const { return table.at(id).name; }

    /* ----- add new net ----- */
    void addNet(int id,const std::string& name,
                const std::vector<std::pair<int,int>>& edges){
        table[id] = {name, edges};
    }

    /* ----- save / load ----- */
    void save(const std::string& fn) const {
        std::ofstream f(fn,std::ios::binary);
        size_t sz=table.size(); f.write((char*)&sz,sizeof(sz));
        for(auto &[id,el]:table){
            f.write((char*)&id,sizeof(id));
            size_t nlen=el.name.size(); f.write((char*)&nlen,sizeof(nlen));
            f.write(el.name.data(),nlen);
            size_t ec=el.edges.size(); f.write((char*)&ec,sizeof(ec));
            for(auto &[u,v]:el.edges){ f.write((char*)&u,4); f.write((char*)&v,4); }
        }
    }
    void load(const std::string& fn){
        std::ifstream f(fn,std::ios::binary); if(!f) return;
        size_t sz; f.read((char*)&sz,sizeof(sz));
        table.clear();
        for(size_t i=0;i<sz;++i){
            int id; f.read((char*)&id,4);
            size_t nlen; f.read((char*)&nlen,sizeof(nlen));
            std::string name(nlen,' '); f.read(&name[0],nlen);
            size_t ec; f.read((char*)&ec,sizeof(ec));
            std::vector<std::pair<int,int>> edges(ec);
            for(auto &e:edges){ f.read((char*)&e.first,4); f.read((char*)&e.second,4);}
            table[id]={name,std::move(edges)};
        }
    }
private:
    std::unordered_map<int,EdgeList> table;
};
