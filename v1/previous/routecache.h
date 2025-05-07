#pragma once

#include <vector>
#include <unordered_map>
#include <fstream>
#include <string>

// 为了哈希整数对
struct PairIntHash {
    std::size_t operator()(const std::pair<int, int> &p) const {
        return std::hash<int>{}(p.first) ^ (std::hash<int>{}(p.second) << 1);
    }
};

class RouteCache {
public:
    RouteCache();
    
    // 路径缓存操作
    bool hasPath(int source, int target) const;
    std::vector<int> getPath(int source, int target) const;
    void addPath(int source, int target, const std::vector<int>& path);
    
    // 网络模式缓存操作
    std::string generateNetPatternKey(int sourceNodeId, const std::vector<int>& sinkNodeIds);
    bool hasNetPattern(const std::string& patternKey) const;
    std::vector<std::pair<int, int>> getNetConnections(const std::string& patternKey) const;
    void addNetPattern(const std::string& patternKey, const std::vector<std::pair<int, int>>& connections);
    
    // 拥塞图操作
    float getCongestionFactor(int nodeId) const;
    void updateCongestion(int nodeId, float factor = 1.0f);
    void decayCongestion(float decayFactor = 0.9f);
    
    // 节点使用统计操作
    void recordNodeUsage(int nodeId);
    float getNodePreference(int nodeId) const;
    
    // 持久化操作
    bool saveToFile(const std::string& filename);
    bool loadFromFile(const std::string& filename);
    
private:
    // 使用整数键的哈希表
    std::unordered_map<std::pair<int, int>, std::vector<int>, PairIntHash> pathCache;
    std::unordered_map<std::string, std::vector<std::pair<int, int>>> netPatternCache;
    std::unordered_map<int, float> congestionMap;
    std::unordered_map<int, int> nodeUsage;
};