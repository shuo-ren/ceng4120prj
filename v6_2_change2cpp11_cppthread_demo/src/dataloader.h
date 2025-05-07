// dataloader.h
#pragma once
#include <vector>
#include <string>

struct Node {
    int id, length;
    int begin_x, begin_y, end_x, end_y;
};

struct Net {
    int id;                  // 行序号
    std::string name;        // 真正的 net 名 (nXXXX)
    int source;
    std::vector<int> sinks;
};

using NodeMap       = std::vector<Node>;
using AdjacencyList = std::vector<std::vector<int>>;
using NetList       = std::vector<Net>;

class DataLoader {
public:
    explicit DataLoader(int num_threads = 4);
    void loadDevice (const std::string& file);
    void loadNetlist(const std::string& file);

    const NodeMap&       getNodes()     const { return nodes; }
    const AdjacencyList& getAdjacency() const { return adj;   }
    const NetList&       getNetlist()   const { return nets;  }

private:
    NodeMap       nodes;
    AdjacencyList adj;
    NetList       nets;
    int           threads;
};
