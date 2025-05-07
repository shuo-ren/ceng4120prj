#include "routecache.h"
#include <iostream>
#include <algorithm>

RouteCache::RouteCache() {}

bool RouteCache::hasPath(const std::string& source, const std::string& target) const {
    auto key = std::make_pair(source, target);
    return pathCache.find(key) != pathCache.end();
}

std::vector<std::string> RouteCache::getPath(const std::string& source, const std::string& target) const {
    auto key = std::make_pair(source, target);
    auto it = pathCache.find(key);
    if (it != pathCache.end()) {
        return it->second;
    }
    return {};
}

void RouteCache::addPath(const std::string& source, const std::string& target, const std::vector<std::string>& path) {
    if (path.empty()) return;
    
    auto key = std::make_pair(source, target);
    pathCache[key] = path;
}

std::string RouteCache::generateNetPatternKey(const std::string& sourceType, const std::vector<std::string>& sinkTypes) {
    std::string key = "S" + sourceType + "_";
    
    // 排序以确保相同组合生成相同的键
    std::vector<std::string> sortedSinkTypes = sinkTypes;
    std::sort(sortedSinkTypes.begin(), sortedSinkTypes.end());
    
    for (const auto& type : sortedSinkTypes) {
        key += "T" + type + "_";
    }
    
    key += "C" + std::to_string(sinkTypes.size());
    
    return key;
}

bool RouteCache::hasNetPattern(const std::string& patternKey) const {
    return netPatternCache.find(patternKey) != netPatternCache.end();
}

std::vector<std::pair<std::string, std::string>> RouteCache::getNetConnections(const std::string& patternKey) const {
    auto it = netPatternCache.find(patternKey);
    if (it != netPatternCache.end()) {
        return it->second;
    }
    return {};
}

void RouteCache::addNetPattern(const std::string& patternKey, const std::vector<std::pair<std::string, std::string>>& connections) {
    if (connections.empty()) return;
    
    netPatternCache[patternKey] = connections;
}

float RouteCache::getCongestionFactor(const std::string& nodeId) const {
    auto it = congestionMap.find(nodeId);
    if (it != congestionMap.end()) {
        return it->second;
    }
    return 0.0f;
}

void RouteCache::updateCongestion(const std::string& nodeId, float factor) {
    congestionMap[nodeId] += factor;
}

void RouteCache::decayCongestion(float decayFactor) {
    for (auto& item : congestionMap) {
        item.second *= decayFactor;
    }
}

void RouteCache::recordNodeTypeUsage(const std::string& nodeType) {
    nodeTypeUsage[nodeType]++;
}

float RouteCache::getNodeTypePreference(const std::string& nodeType) const {
    auto it = nodeTypeUsage.find(nodeType);
    if (it != nodeTypeUsage.end()) {
        return static_cast<float>(it->second) / static_cast<float>(nodeTypeUsage.size());
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
        // 保存源节点
        size_t sourceSize = entry.first.first.size();
        file.write(reinterpret_cast<const char*>(&sourceSize), sizeof(sourceSize));
        file.write(entry.first.first.c_str(), sourceSize);
        
        // 保存目标节点
        size_t targetSize = entry.first.second.size();
        file.write(reinterpret_cast<const char*>(&targetSize), sizeof(targetSize));
        file.write(entry.first.second.c_str(), targetSize);
        
        // 保存路径
        size_t pathSize = entry.second.size();
        file.write(reinterpret_cast<const char*>(&pathSize), sizeof(pathSize));
        
        for (const auto& node : entry.second) {
            size_t nodeSize = node.size();
            file.write(reinterpret_cast<const char*>(&nodeSize), sizeof(nodeSize));
            file.write(node.c_str(), nodeSize);
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
            // 保存起点
            size_t sourceSize = connection.first.size();
            file.write(reinterpret_cast<const char*>(&sourceSize), sizeof(sourceSize));
            file.write(connection.first.c_str(), sourceSize);
            
            // 保存终点
            size_t targetSize = connection.second.size();
            file.write(reinterpret_cast<const char*>(&targetSize), sizeof(targetSize));
            file.write(connection.second.c_str(), targetSize);
        }
    }
    
    // 保存拥塞图大小
    size_t congestionMapSize = congestionMap.size();
    file.write(reinterpret_cast<const char*>(&congestionMapSize), sizeof(congestionMapSize));
    
    // 保存拥塞图
    for (const auto& entry : congestionMap) {
        // 保存节点ID
        size_t nodeIdSize = entry.first.size();
        file.write(reinterpret_cast<const char*>(&nodeIdSize), sizeof(nodeIdSize));
        file.write(entry.first.c_str(), nodeIdSize);
        
        // 保存拥塞因子
        file.write(reinterpret_cast<const char*>(&entry.second), sizeof(entry.second));
    }
    
    // 保存节点类型使用统计大小
    size_t nodeTypeUsageSize = nodeTypeUsage.size();
    file.write(reinterpret_cast<const char*>(&nodeTypeUsageSize), sizeof(nodeTypeUsageSize));
    
    // 保存节点类型使用统计
    for (const auto& entry : nodeTypeUsage) {
        // 保存节点类型
        size_t typeSize = entry.first.size();
        file.write(reinterpret_cast<const char*>(&typeSize), sizeof(typeSize));
        file.write(entry.first.c_str(), typeSize);
        
        // 保存使用计数
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
        nodeTypeUsage.clear();
        
        // 加载路径缓存
        size_t pathCacheSize;
        file.read(reinterpret_cast<char*>(&pathCacheSize), sizeof(pathCacheSize));
        
        for (size_t i = 0; i < pathCacheSize; ++i) {
            // 读取源节点
            size_t sourceSize;
            file.read(reinterpret_cast<char*>(&sourceSize), sizeof(sourceSize));
            std::string source(sourceSize, ' ');
            file.read(&source[0], sourceSize);
            
            // 读取目标节点
            size_t targetSize;
            file.read(reinterpret_cast<char*>(&targetSize), sizeof(targetSize));
            std::string target(targetSize, ' ');
            file.read(&target[0], targetSize);
            
            // 读取路径
            size_t pathSize;
            file.read(reinterpret_cast<char*>(&pathSize), sizeof(pathSize));
            
            std::vector<std::string> path;
            for (size_t j = 0; j < pathSize; ++j) {
                size_t nodeSize;
                file.read(reinterpret_cast<char*>(&nodeSize), sizeof(nodeSize));
                std::string node(nodeSize, ' ');
                file.read(&node[0], nodeSize);
                path.push_back(node);
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
            
            std::vector<std::pair<std::string, std::string>> connections;
            for (size_t j = 0; j < connectionsSize; ++j) {
                // 读取起点
                size_t sourceSize;
                file.read(reinterpret_cast<char*>(&sourceSize), sizeof(sourceSize));
                std::string source(sourceSize, ' ');
                file.read(&source[0], sourceSize);
                
                // 读取终点
                size_t targetSize;
                file.read(reinterpret_cast<char*>(&targetSize), sizeof(targetSize));
                std::string target(targetSize, ' ');
                file.read(&target[0], targetSize);
                
                connections.emplace_back(source, target);
            }
            
            // 添加到网络模式缓存
            netPatternCache[key] = connections;
        }
        
        // 加载拥塞图
        size_t congestionMapSize;
        file.read(reinterpret_cast<char*>(&congestionMapSize), sizeof(congestionMapSize));
        
        for (size_t i = 0; i < congestionMapSize; ++i) {
            // 读取节点ID
            size_t nodeIdSize;
            file.read(reinterpret_cast<char*>(&nodeIdSize), sizeof(nodeIdSize));
            std::string nodeId(nodeIdSize, ' ');
            file.read(&nodeId[0], nodeIdSize);
            
            // 读取拥塞因子
            float factor;
            file.read(reinterpret_cast<char*>(&factor), sizeof(factor));
            
            // 添加到拥塞图
            congestionMap[nodeId] = factor;
        }
        
        // 加载节点类型使用统计
        size_t nodeTypeUsageSize;
        file.read(reinterpret_cast<char*>(&nodeTypeUsageSize), sizeof(nodeTypeUsageSize));
        
        for (size_t i = 0; i < nodeTypeUsageSize; ++i) {
            // 读取节点类型
            size_t typeSize;
            file.read(reinterpret_cast<char*>(&typeSize), sizeof(typeSize));
            std::string type(typeSize, ' ');
            file.read(&type[0], typeSize);
            
            // 读取使用计数
            int count;
            file.read(reinterpret_cast<char*>(&count), sizeof(count));
            
            // 添加到节点类型使用统计
            nodeTypeUsage[type] = count;
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