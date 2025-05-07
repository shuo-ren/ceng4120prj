#include "routecache.h"
#include <iostream>
#include <algorithm>

RouteCache::RouteCache() {}

bool RouteCache::hasPath(int source, int target) const {
    auto key = std::make_pair(source, target);
    return pathCache.find(key) != pathCache.end();
}

std::vector<int> RouteCache::getPath(int source, int target) const {
    auto key = std::make_pair(source, target);
    auto it = pathCache.find(key);
    if (it != pathCache.end()) {
        return it->second;
    }
    return {};
}

void RouteCache::addPath(int source, int target, const std::vector<int>& path) {
    if (path.empty()) return;
    
    auto key = std::make_pair(source, target);
    pathCache[key] = path;
}

std::string RouteCache::generateNetPatternKey(int sourceNodeId, const std::vector<int>& sinkNodeIds) {
    std::string key = "S" + std::to_string(sourceNodeId) + "_";
    
    // 排序以确保相同组合生成相同的键
    std::vector<int> sortedSinkIds = sinkNodeIds;
    std::sort(sortedSinkIds.begin(), sortedSinkIds.end());
    
    for (const auto& id : sortedSinkIds) {
        key += "T" + std::to_string(id) + "_";
    }
    
    key += "C" + std::to_string(sinkNodeIds.size());
    
    return key;
}

bool RouteCache::hasNetPattern(const std::string& patternKey) const {
    return netPatternCache.find(patternKey) != netPatternCache.end();
}

std::vector<std::pair<int, int>> RouteCache::getNetConnections(const std::string& patternKey) const {
    auto it = netPatternCache.find(patternKey);
    if (it != netPatternCache.end()) {
        return it->second;
    }
    return {};
}

void RouteCache::addNetPattern(const std::string& patternKey, const std::vector<std::pair<int, int>>& connections) {
    if (connections.empty()) return;
    
    netPatternCache[patternKey] = connections;
}

float RouteCache::getCongestionFactor(int nodeId) const {
    auto it = congestionMap.find(nodeId);
    if (it != congestionMap.end()) {
        return it->second;
    }
    return 0.0f;
}

void RouteCache::updateCongestion(int nodeId, float factor) {
    congestionMap[nodeId] += factor;
}

void RouteCache::decayCongestion(float decayFactor) {
    for (auto& item : congestionMap) {
        item.second *= decayFactor;
    }
}

void RouteCache::recordNodeUsage(int nodeId) {
    nodeUsage[nodeId]++;
}

float RouteCache::getNodePreference(int nodeId) const {
    auto it = nodeUsage.find(nodeId);
    if (it != nodeUsage.end() && !nodeUsage.empty()) {
        return static_cast<float>(it->second) / static_cast<float>(nodeUsage.size());
    }
    return 0.0f;
}

bool RouteCache::saveToFile(const std::string& filename) {
    std::ofstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Failed to open cache file for writing: " << filename << std::endl;
        return false;
    }
    
    // 保存路径缓存大小
    size_t pathCacheSize = pathCache.size();
    file.write(reinterpret_cast<const char*>(&pathCacheSize), sizeof(pathCacheSize));
    
    // 保存路径缓存
    for (const auto& entry : pathCache) {
        // 保存源节点和目标节点
        file.write(reinterpret_cast<const char*>(&entry.first.first), sizeof(entry.first.first));
        file.write(reinterpret_cast<const char*>(&entry.first.second), sizeof(entry.first.second));
        
        // 保存路径
        size_t pathSize = entry.second.size();
        file.write(reinterpret_cast<const char*>(&pathSize), sizeof(pathSize));
        
        for (const auto& node : entry.second) {
            file.write(reinterpret_cast<const char*>(&node), sizeof(node));
        }
    }
    
    // 保存网络模式缓存大小
    size_t patternCacheSize = netPatternCache.size();
    file.write(reinterpret_cast<const char*>(&patternCacheSize), sizeof(patternCacheSize));
    
    // 保存网络模式缓存
    for (const auto& entry : netPatternCache) {
        // 保存模式键
        size_t keySize = entry.first.size();
        file.write(reinterpret_cast<const char*>(&keySize), sizeof(keySize));
        file.write(entry.first.c_str(), keySize);
        
        // 保存连接
        size_t connectionsSize = entry.second.size();
        file.write(reinterpret_cast<const char*>(&connectionsSize), sizeof(connectionsSize));
        
        for (const auto& connection : entry.second) {
            file.write(reinterpret_cast<const char*>(&connection.first), sizeof(connection.first));
            file.write(reinterpret_cast<const char*>(&connection.second), sizeof(connection.second));
        }
    }
    
    // 保存拥塞图大小
    size_t congestionMapSize = congestionMap.size();
    file.write(reinterpret_cast<const char*>(&congestionMapSize), sizeof(congestionMapSize));
    
    // 保存拥塞图
    for (const auto& entry : congestionMap) {
        file.write(reinterpret_cast<const char*>(&entry.first), sizeof(entry.first));
        file.write(reinterpret_cast<const char*>(&entry.second), sizeof(entry.second));
    }
    
    // 保存节点使用统计大小
    size_t nodeUsageSize = nodeUsage.size();
    file.write(reinterpret_cast<const char*>(&nodeUsageSize), sizeof(nodeUsageSize));
    
    // 保存节点使用统计
    for (const auto& entry : nodeUsage) {
        file.write(reinterpret_cast<const char*>(&entry.first), sizeof(entry.first));
        file.write(reinterpret_cast<const char*>(&entry.second), sizeof(entry.second));
    }
    
    std::cout << "Cache saved to " << filename << ": " 
              << pathCacheSize << " paths, " 
              << patternCacheSize << " patterns, "
              << congestionMapSize << " congestion entries" << std::endl;
              
    return true;
}

bool RouteCache::loadFromFile(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Failed to open cache file for reading: " << filename << std::endl;
        return false;
    }
    
    try {
        // 清除现有缓存
        pathCache.clear();
        netPatternCache.clear();
        congestionMap.clear();
        nodeUsage.clear();
        
        // 加载路径缓存
        size_t pathCacheSize;
        file.read(reinterpret_cast<char*>(&pathCacheSize), sizeof(pathCacheSize));
        
        for (size_t i = 0; i < pathCacheSize; ++i) {
            // 读取源节点和目标节点
            int source, target;
            file.read(reinterpret_cast<char*>(&source), sizeof(source));
            file.read(reinterpret_cast<char*>(&target), sizeof(target));
            
            // 读取路径
            size_t pathSize;
            file.read(reinterpret_cast<char*>(&pathSize), sizeof(pathSize));
            
            std::vector<int> path(pathSize);
            for (size_t j = 0; j < pathSize; ++j) {
                int node;
                file.read(reinterpret_cast<char*>(&node), sizeof(node));
                path[j] = node;
            }
            
            // 添加到路径缓存
            pathCache[std::make_pair(source, target)] = path;
        }
        
        // 加载网络模式缓存
        size_t patternCacheSize;
        file.read(reinterpret_cast<char*>(&patternCacheSize), sizeof(patternCacheSize));
        
        for (size_t i = 0; i < patternCacheSize; ++i) {
            // 读取模式键
            size_t keySize;
            file.read(reinterpret_cast<char*>(&keySize), sizeof(keySize));
            std::string key(keySize, ' ');
            file.read(&key[0], keySize);
            
            // 读取连接
            size_t connectionsSize;
            file.read(reinterpret_cast<char*>(&connectionsSize), sizeof(connectionsSize));
            
            std::vector<std::pair<int, int>> connections(connectionsSize);
            for (size_t j = 0; j < connectionsSize; ++j) {
                int source, target;
                file.read(reinterpret_cast<char*>(&source), sizeof(source));
                file.read(reinterpret_cast<char*>(&target), sizeof(target));
                connections[j] = std::make_pair(source, target);
            }
            
            // 添加到网络模式缓存
            netPatternCache[key] = connections;
        }
        
        // 加载拥塞图
        size_t congestionMapSize;
        file.read(reinterpret_cast<char*>(&congestionMapSize), sizeof(congestionMapSize));
        
        for (size_t i = 0; i < congestionMapSize; ++i) {
            int nodeId;
            float factor;
            file.read(reinterpret_cast<char*>(&nodeId), sizeof(nodeId));
            file.read(reinterpret_cast<char*>(&factor), sizeof(factor));
            
            // 添加到拥塞图
            congestionMap[nodeId] = factor;
        }
        
        // 加载节点使用统计
        size_t nodeUsageSize;
        file.read(reinterpret_cast<char*>(&nodeUsageSize), sizeof(nodeUsageSize));
        
        for (size_t i = 0; i < nodeUsageSize; ++i) {
            int nodeId;
            int count;
            file.read(reinterpret_cast<char*>(&nodeId), sizeof(nodeId));
            file.read(reinterpret_cast<char*>(&count), sizeof(count));
            
            // 添加到节点使用统计
            nodeUsage[nodeId] = count;
        }
        
        std::cout << "Cache loaded from " << filename << ": " 
                  << pathCache.size() << " paths, " 
                  << netPatternCache.size() << " patterns, "
                  << congestionMap.size() << " congestion entries" << std::endl;
                  
        return true;
    }
    catch (const std::exception& e) {
        std::cerr << "Error loading cache: " << e.what() << std::endl;
        return false;
    }
}