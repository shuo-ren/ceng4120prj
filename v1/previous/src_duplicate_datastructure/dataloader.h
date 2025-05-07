#pragma once

#include <string>
#include <unordered_map>
#include <vector>

struct Node {
    std::string id;
    std::string type;
    float length;
    float begin_x, begin_y, end_x, end_y;
    std::string name;
};

struct Net {
    std::string id;
    std::string name;
    std::string source;
    std::vector<std::string> sinks;
};

using NodeMap = std::unordered_map<std::string, Node>;
using AdjacencyList = std::unordered_map<std::string, std::vector<std::string>>;
using NetList = std::vector<Net>;

class DataLoader {
public:
    DataLoader(int num_threads = 4); // 默认线程数为4
    
    void loadDevice(const std::string& filename);
    void loadNetlist(const std::string& filename);
    
    const NodeMap& getNodes() const { return nodes; }
    const AdjacencyList& getAdjacency() const { return adjacency; }
    const NetList& getNetlist() const { return nets; }
    
private:
    NodeMap nodes;
    AdjacencyList adjacency;
    NetList nets;
    int num_threads; // 存储线程数
};