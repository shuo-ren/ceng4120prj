#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <fstream>

// 为了哈希std::pair
struct PairHash {
    template <class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2> &p) const {
        auto h1 = std::hash<T1>{}(p.first);
        auto h2 = std::hash<T2>{}(p.second);
        return h1 ^ (h2 << 1);
    }
};

// 网络特征结构，用于识别相似网络
struct NetPattern {
    std::string sourceType;
    std::vector<std::string> sinkTypes;
    size_t sinkCount;
    
    bool operator==(const NetPattern& other) const {
        return sourceType == other.sourceType && 
               sinkTypes == other.sinkTypes &&
               sinkCount == other.sinkCount;
    }
};

// 为了哈希NetPattern
struct NetPatternHash {
    std::size_t operator()(const NetPattern &p) const {
        std::size_t h = std::hash<std::string>{}(p.sourceType) ^ 
                        std::hash<size_t>{}(p.sinkCount);
        for (const auto& type : p.sinkTypes) {
            h ^= std::hash<std::string>{}(type);
        }
        return h;
    }
};

class RouteCache {
public:
    RouteCache();
    
    // 路径缓存操作
    bool hasPath(const std::string& source, const std::string& target) const;
    std::vector<std::string> getPath(const std::string& source, const std::string& target) const;
    void addPath(const std::string& source, const std::string& target, const std::vector<std::string>& path);
    
    // 网络模式缓存操作
    std::string generateNetPatternKey(const std::string& sourceType, const std::vector<std::string>& sinkTypes);
    bool hasNetPattern(const std::string& patternKey) const;
    std::vector<std::pair<std::string, std::string>> getNetConnections(const std::string& patternKey) const;
    void addNetPattern(const std::string& patternKey, const std::vector<std::pair<std::string, std::string>>& connections);
    
    // 拥塞图操作
    float getCongestionFactor(const std::string& nodeId) const;
    void updateCongestion(const std::string& nodeId, float factor = 1.0f);
    void decayCongestion(float decayFactor = 0.9f);
    
    // 基于节点类型的统计
    void recordNodeTypeUsage(const std::string& nodeType);
    float getNodeTypePreference(const std::string& nodeType) const;
    
    // 持久化操作
    bool saveToFile(const std::string& filename);
    bool loadFromFile(const std::string& filename);
    
    // 统计信息
    size_t getPathCacheSize() const { return pathCache.size(); }
    size_t getPatternCacheSize() const { return netPatternCache.size(); }
    
private:
    // 源-目标对的成功路径缓存
    std::unordered_map<std::pair<std::string, std::string>, 
                       std::vector<std::string>, 
                       PairHash> pathCache;
    
    // 网络模式缓存
    std::unordered_map<std::string, 
                       std::vector<std::pair<std::string, std::string>>> netPatternCache;
    
    // 拥塞热点图
    std::unordered_map<std::string, float> congestionMap;
    
    // 节点类型使用统计
    std::unordered_map<std::string, int> nodeTypeUsage;
};