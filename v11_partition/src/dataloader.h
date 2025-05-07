// dataloader.h

#pragma once
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <chrono>
#include <omp.h>
#include <algorithm>

struct Node {
    int id, length;
    int begin_x, begin_y, end_x, end_y;
};

struct Net {
    int id;
    std::string name;
    int source;
    std::vector<int> sinks;
};

using NodeMap       = std::vector<Node>;
using AdjacencyList = std::vector<std::vector<int>>;
using NetList       = std::vector<Net>;

class DataLoader {
public:
    explicit DataLoader(int num_threads = 4) : threads(num_threads < 1 ? 1 : num_threads) {}

    void loadDevice(const std::string& file) {
        auto t0 = std::chrono::high_resolution_clock::now();
        std::ifstream fin(file);
        if (!fin) throw std::runtime_error("open device " + file);

        size_t nNode; fin >> nNode;
        nodes.resize(nNode);
        adj.resize(nNode);
        std::string line; std::getline(fin, line); // EOL

        std::vector<std::string> nodeLines(nNode);
        for (size_t i = 0; i < nNode; ++i) std::getline(fin, nodeLines[i]);
        std::getline(fin, line); // blank
        std::vector<std::string> adjLines(nNode);
        for (size_t i = 0; i < nNode; ++i) std::getline(fin, adjLines[i]);
        fin.close();

        #pragma omp parallel for schedule(static) num_threads(threads)
        for (size_t i = 0; i < nNode; ++i) {
            std::istringstream is(nodeLines[i]);
            Node n; std::string type, name;
            is >> n.id >> type >> n.length >> n.begin_x >> n.begin_y >> n.end_x >> n.end_y;
            nodes[n.id] = n;
        }

        #pragma omp parallel for schedule(static) num_threads(threads)
        for (size_t i = 0; i < nNode; ++i) {
            std::istringstream is(adjLines[i]); int u, v; is >> u;
            while (is >> v) adj[u].push_back(v);
        }

        auto t1 = std::chrono::high_resolution_clock::now();
        std::cout << "Device loaded (" << nNode << " nodes) in "
                  << std::chrono::duration_cast<std::chrono::seconds>(t1 - t0).count() << " s\n";
    }

    void loadNetlist(const std::string& file) {
        auto t0 = std::chrono::high_resolution_clock::now();
        std::ifstream fin(file);
        if (!fin) throw std::runtime_error("open netlist " + file);

        nets.clear();
        std::string line; std::getline(fin, line);
        bool hasHeader = true;
        try { std::stoul(line); } catch (...) { hasHeader = false; fin.seekg(0); }

        std::vector<std::string> lines;
        while (std::getline(fin, line)) if (!line.empty()) lines.push_back(line);
        fin.close();

        nets.reserve(lines.size());
        #pragma omp parallel num_threads(threads)
        {
            std::vector<Net> local;
            #pragma omp for schedule(static)
            for (size_t i = 0; i < lines.size(); ++i) {
                std::istringstream is(lines[i]);
                Net n; is >> n.id >> n.name >> n.source;
                int s; while (is >> s) n.sinks.push_back(s);
                local.push_back(std::move(n));
            }
            #pragma omp critical
            nets.insert(nets.end(),
                        std::make_move_iterator(local.begin()),
                        std::make_move_iterator(local.end()));
        }

        auto t1 = std::chrono::high_resolution_clock::now();
        std::cout << "Netlist loaded (" << nets.size() << " nets) in "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() << " ms\n";
    }

    const NodeMap&       getNodes()     const { return nodes; }
    const AdjacencyList& getAdjacency() const { return adj;   }
    const NetList&       getNetlist()   const { return nets;  }

private:
    NodeMap       nodes;
    AdjacencyList adj;
    NetList       nets;
    int           threads;
};
