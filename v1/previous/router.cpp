#include "router.h"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <cstdlib>

Router::Router(const NodeMap& nodes, 
               const AdjacencyList& adjacency, 
               const NetList& nets,
               const std::string& outputFile,
               bool debug)
    : nodes(nodes), adjacency(adjacency), nets(nets), outputFile(outputFile), debug(debug) {
    startTime = std::chrono::high_resolution_clock::now();
    configureRoutingParameters();
}

void Router::configureRoutingParameters() {
    // 根据网络数量自动配置参数和算法策略
    size_t netCount = nets.size();
    
    // 设置时间限制
    timeLimit = (netCount > 10000) ? 2500.0 : 1000.0; // 秒
    
    // 默认设置 - 基于设计规模调整
    if (netCount <= 5000) {
        // 超小型设计：纯贪心
        useNegotiatedRouting = false;
        maxIterations = 1;
    } else if (netCount <= 5001) {
        // 小型设计：单次遍历加简单A*
        useNegotiatedRouting = false;
        maxIterations = 1;
    } else if (netCount <= 6000) {
        // 中型设计：最多2次谈判迭代
        useNegotiatedRouting = true;
        maxIterations = 2;
    } else {
        // 大型设计：有限谈判迭代
        useNegotiatedRouting = true;
        maxIterations = 3;
    }
    
    if (debug) {
        std::cout << "Configured for " << netCount << " nets:" << std::endl;
        std::cout << "  Time limit: " << timeLimit << " seconds" << std::endl;
        std::cout << "  Negotiated routing: " << (useNegotiatedRouting ? "Enabled" : "Disabled") << std::endl;
        std::cout << "  Max iterations: " << maxIterations << std::endl;
    }
}

bool Router::loadCache(const std::string& cacheFile) {
    bool success = routeCache.loadFromFile(cacheFile);
    if (success) {
        std::cout << "Successfully loaded route cache from " << cacheFile << std::endl;
    } else {
        std::cout << "Failed to load cache or cache file not found, starting with empty cache" << std::endl;
    }
    return success;
}

bool Router::saveCache(const std::string& cacheFile) {
    bool success = routeCache.saveToFile(cacheFile);
    if (success) {
        std::cout << "Successfully saved route cache to " << cacheFile << std::endl;
    } else {
        std::cout << "Failed to save cache to " << cacheFile << std::endl;
    }
    return success;
}

std::string Router::getNetName(int netId) {
    return "n" + std::to_string(netId);
}

void Router::routeNets() {
    startTime = std::chrono::high_resolution_clock::now();
    
    std::cout << "Starting routing of " << nets.size() << " nets to " << outputFile << std::endl;
    
    // 创建输出目录（如果需要）
    size_t lastSlash = outputFile.find_last_of("/\\");
    if (lastSlash != std::string::npos) {
        std::string outputDir = outputFile.substr(0, lastSlash);
        std::system(("mkdir -p " + outputDir).c_str());
    }
    
    // 按复杂性排序网络
    std::vector<Net> sortedNets = nets;
    std::sort(sortedNets.begin(), sortedNets.end(), 
              [](const Net& a, const Net& b) { 
                  return a.sinks.size() < b.sinks.size();
              });
    
    // 根据配置的策略执行路由
    if (useNegotiatedRouting) {
        // 谈判式路由策略（多次迭代）
        for (size_t iteration = 0; iteration < maxIterations; ++iteration) {
            float congestionMultiplier = 1.0f + iteration * 0.5f;
            
            // 检查剩余时间
            auto currentTime = std::chrono::high_resolution_clock::now();
            double elapsedSeconds = std::chrono::duration_cast<std::chrono::milliseconds>(
                currentTime - startTime).count() / 1000.0;
            
            // 如果已经使用了85%的时间，保存结果并退出
            if (elapsedSeconds > timeLimit * 0.85) {
                std::cout << "Time limit approaching (" << elapsedSeconds << "s), stopping iterations." << std::endl;
                break;
            }
            
            std::cout << "Starting routing iteration " << (iteration + 1) 
                      << "/" << maxIterations
                      << " (congestion multiplier: " << congestionMultiplier 
                      << ", elapsed time: " << elapsedSeconds << "s)" << std::endl;
            
            routeIteration(iteration, congestionMultiplier);
            
            // 检查是否所有网络都成功路由
            bool allSuccess = checkAllSuccess();
            if (allSuccess) {
                std::cout << "All nets successfully routed, stopping iterations early." << std::endl;
                break;
            }
            
            // 每次迭代后衰减拥塞记录
            routeCache.decayCongestion(0.8f);
        }
    } else {
        // 简单的单次路由策略
        simpleSinglePassRouting(sortedNets);
    }
    
    // 检查时间
    auto currentTime = std::chrono::high_resolution_clock::now();
    double elapsedSeconds = std::chrono::duration_cast<std::chrono::milliseconds>(
        currentTime - startTime).count() / 1000.0;
    std::cout << "Processing complete in " << elapsedSeconds << "s, writing results..." << std::endl;
    
    // 输出缓存使用统计
    std::cout << "Cache statistics: " << cacheHits << " hits, " 
              << cacheMisses << " misses, " 
              << (cacheHits + cacheMisses > 0 ? 
                 (cacheHits * 100.0 / (cacheHits + cacheMisses)) : 0.0) 
              << "% hit rate" << std::endl;
    
    // 按网络ID排序结果以保持一致的输出
    std::sort(results.begin(), results.end(), 
              [](const RouteResult& a, const RouteResult& b) { 
                  return a.netId < b.netId;
              });
    
    // 将结果写入输出文件
    std::ofstream outFile(outputFile);
    if (!outFile) {
        std::cerr << "Error: Could not open output file " << outputFile << std::endl;
        return;
    }
    
    size_t successfulRoutes = 0;
    size_t failedRoutes = 0;
    
    for (const auto& result : results) {
        outFile << result.netId << " " << result.netName << "\n";
        
        for (const auto& connection : result.connections) {
            outFile << connection.first << " " << connection.second << "\n";
        }
        
        outFile << "\n";
        
        if (result.success) {
            successfulRoutes++;
        } else {
            failedRoutes++;
        }
    }
    
    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(endTime - startTime).count();
    
    std::cout << "\n" << std::string(50, '=') << std::endl;
    std::cout << "Routing completed in " << duration << " seconds" << std::endl;
    std::cout << "Successfully routed: " << successfulRoutes << "/" << results.size() 
              << " nets (" << (successfulRoutes * 100.0 / results.size()) << "%)" << std::endl;
    std::cout << "Failed to route: " << failedRoutes << " nets" << std::endl;
    std::cout << "Used " << usedResources.size() << " nodes" << std::endl;
    std::cout << std::string(50, '=') << std::endl;
}

bool Router::checkAllSuccess() const {
    for (const auto& result : results) {
        if (!result.success) {
            return false;
        }
    }
    return true;
}

void Router::simpleSinglePassRouting(const std::vector<Net>& sortedNets) {
    results.clear();
    usedResources.clear();
    
    size_t netsRouted = 0;
    size_t totalNets = sortedNets.size();
    
    for (const auto& net : sortedNets) {
        // 检查剩余时间
        auto currentTime = std::chrono::high_resolution_clock::now();
        double elapsedSeconds = std::chrono::duration_cast<std::chrono::milliseconds>(
            currentTime - startTime).count() / 1000.0;
        
        // 如果已经使用了95%的时间，保存结果并退出
        if (elapsedSeconds > timeLimit * 0.95) {
            std::cout << "Time limit approaching (" << elapsedSeconds << "s), stopping routing." << std::endl;
            break;
        }
        
        // 首先尝试使用缓存进行路由
        RouteResult result = tryRouteCached(net, usedResources);
        
        // 如果缓存未命中，使用常规路由
        if (!result.success) {
            result = routeNet(net, {});
        }
        
        results.push_back(result);
        
        // 如果路由成功，更新使用的资源
        if (result.success) {
            for (const auto& connection : result.connections) {
                usedResources.insert(connection.first);
                usedResources.insert(connection.second);
            }
        }
        
        netsRouted++;
        
        // 每100个网络或5%进度记录一次
        if (debug && (netsRouted % 100 == 0 || netsRouted * 20 / totalNets > (netsRouted - 1) * 20 / totalNets)) {
            logProgress(netsRouted, totalNets, net.id);
        }
    }
    
    size_t successCount = 0;
    for (const auto& result : results) {
        if (result.success) {
            successCount++;
        }
    }
    
    std::cout << "Single-pass routing completed: " 
              << successCount << "/" << netsRouted 
              << " nets successful (" << (successCount * 100.0 / netsRouted) << "%)" << std::endl;
}

void Router::routeIteration(int iteration, float congestionMultiplier) {
    // 第一次迭代初始化结果数组，后续迭代重用
    if (iteration == 0) {
        results.clear();
        results.resize(nets.size());
    }
    
    // 每次迭代重置使用的资源
    usedResources.clear();
    
    // 固定已成功路由的网络（如果不是第一次迭代）
    std::unordered_set<int> fixedNets;
    if (iteration > 0) {
        for (size_t i = 0; i < nets.size(); ++i) {
            if (results[i].success) {
                fixedNets.insert(nets[i].id);
                
                // 将固定网络的资源标记为已使用
                for (const auto& connection : results[i].connections) {
                    usedResources.insert(connection.first);
                    usedResources.insert(connection.second);
                }
            }
        }
    }
    
    // 对所有网络进行路由（或重新路由）
    size_t netsRouted = 0;
    size_t totalNets = nets.size();
    
    for (size_t i = 0; i < nets.size(); ++i) {
        // 检查剩余时间
        auto currentTime = std::chrono::high_resolution_clock::now();
        double elapsedSeconds = std::chrono::duration_cast<std::chrono::milliseconds>(
            currentTime - startTime).count() / 1000.0;
        
        // 如果已经使用了95%的时间，保存结果并退出
        if (elapsedSeconds > timeLimit * 0.95) {
            std::cout << "Time limit approaching (" << elapsedSeconds << "s), stopping iteration." << std::endl;
            break;
        }
        
        const Net& net = nets[i];
        
        // 如果网络已固定，跳过
        if (fixedNets.find(net.id) != fixedNets.end()) {
            netsRouted++;
            continue;
        }
        
        // 首先尝试使用缓存进行路由
        RouteResult result = tryRouteCached(net, usedResources);
        
        // 如果缓存未命中，使用常规路由
        if (!result.success) {
            result = routeNet(net, fixedNets, congestionMultiplier);
        }
        
        // 更新结果
        results[i] = result;
        
        // 如果路由成功，更新使用的资源并添加到缓存
        if (result.success) {
            for (const auto& connection : result.connections) {
                usedResources.insert(connection.first);
                usedResources.insert(connection.second);
            }
            
            // 更新缓存
            std::string patternKey = routeCache.generateNetPatternKey(net.source, net.sinks);
            routeCache.addNetPattern(patternKey, result.connections);
            
            // 记录节点使用情况
            routeCache.recordNodeUsage(net.source);
            for (const auto& sink : net.sinks) {
                routeCache.recordNodeUsage(sink);
            }
        }
        
        netsRouted++;
        
        // 每100个网络或5%进度记录一次
        if (debug && (netsRouted % 100 == 0 || netsRouted * 20 / totalNets > (netsRouted - 1) * 20 / totalNets)) {
            logProgress(netsRouted, totalNets, net.id);
        }
    }
    
    // 计算当前迭代的成功率
    size_t successCount = 0;
    for (const auto& result : results) {
        if (result.success) {
            successCount++;
        }
    }
    
    std::cout << "Iteration " << (iteration + 1) << " completed: " 
              << successCount << "/" << nets.size() 
              << " nets successful (" << (successCount * 100.0 / nets.size()) << "%)" << std::endl;
}

RouteResult Router::tryRouteCached(const Net& net, const std::unordered_set<int>& avoidNodes) {
    RouteResult result;
    result.netId = net.id;
    result.netName = getNetName(net.id);  // 使用我们的辅助函数生成名称
    result.success = false;
    
    // 对于没有目标的网络，直接返回
    if (net.sinks.empty()) {
        result.success = true;
        return result;
    }
    
    // 检查网络模式缓存
    std::string patternKey = routeCache.generateNetPatternKey(net.source, net.sinks);
    
    if (routeCache.hasNetPattern(patternKey)) {
        // 尝试使用缓存的连接模式
        std::vector<std::pair<int, int>> cachedConnections = 
            routeCache.getNetConnections(patternKey);
        
        if (!cachedConnections.empty()) {
            // 验证这些连接是否可用（不与避开的节点冲突）
            bool conflictFound = false;
            std::unordered_set<int> usedNodes;
            
            for (const auto& conn : cachedConnections) {
                if (avoidNodes.find(conn.first) != avoidNodes.end() || 
                    avoidNodes.find(conn.second) != avoidNodes.end()) {
                    conflictFound = true;
                    break;
                }
                usedNodes.insert(conn.first);
                usedNodes.insert(conn.second);
            }
            
            // 检查是否所有目标都在使用的节点中
            for (const auto& sink : net.sinks) {
                if (usedNodes.find(sink) == usedNodes.end()) {
                    conflictFound = true;
                    break;
                }
            }
            
            if (!conflictFound) {
                // 缓存命中!
                result.connections = cachedConnections;
                result.success = true;
                cacheHits++;
                return result;
            }
        }
    }
    
    // 对于每对源-目标，尝试使用路径缓存
    bool allPathsFound = true;
    std::unordered_set<int> treeNodes = {net.source};
    std::vector<std::pair<int, int>> treeConnections;
    
    for (const auto& sink : net.sinks) {
        bool pathFound = false;
        
        // 从树中的每个节点尝试找到到目标的路径
        for (const auto& treeNode : treeNodes) {
            // 检查缓存
            if (routeCache.hasPath(treeNode, sink)) {
                std::vector<int> path = routeCache.getPath(treeNode, sink);
                
                // 验证路径是否有效（不与避开的节点冲突）
                bool pathValid = true;
                for (const auto& node : path) {
                    if (avoidNodes.find(node) != avoidNodes.end()) {
                        pathValid = false;
                        break;
                    }
                }
                
                if (pathValid) {
                    // 缓存命中!
                    for (size_t i = 0; i < path.size() - 1; ++i) {
                        treeConnections.emplace_back(path[i], path[i + 1]);
                        treeNodes.insert(path[i]);
                        treeNodes.insert(path[i + 1]);
                    }
                    pathFound = true;
                    cacheHits++;
                    break;
                }
            }
        }
        
        if (!pathFound) {
            allPathsFound = false;
            cacheMisses++;
            break;
        }
    }
    
    if (allPathsFound && !treeConnections.empty()) {
        result.connections = treeConnections;
        result.success = true;
    }
    
    return result;
}

RouteResult Router::routeNet(const Net& net, const std::unordered_set<int>& fixedNets, float congestionMultiplier) {
    RouteResult result;
    result.netId = net.id;
    result.netName = getNetName(net.id);  // 使用我们的辅助函数生成名称
    
    // 如果这个网络是固定的，则跳过处理
    if (fixedNets.find(net.id) != fixedNets.end()) {
        result.success = true;
        return result;
    }
    
    // 构建要避开的节点集合
    std::unordered_set<int> avoidNodes = usedResources;
    
    // 构建Steiner树
    auto [treeNodes, treeConnections] = buildSteinerTree(
        net.source, net.sinks, avoidNodes, congestionMultiplier);
    
    // 检查路由是否成功
    bool success = true;
    if (!net.sinks.empty()) {
        for (const auto& sink : net.sinks) {
            if (treeNodes.find(sink) == treeNodes.end()) {
                success = false;
                break;
            }
        }
    }
    
    // 更新拥塞图
    updateCongestionMap(treeNodes);
    
    // 如果路由成功，更新缓存
    if (success) {
        // 为每个源-目标对缓存路径
        for (const auto& sink : net.sinks) {
            // 重建从源到这个目标的路径
            std::vector<int> path;
            std::unordered_map<int, int> parentMap;
            
            // 从连接构建父映射
            for (const auto& conn : treeConnections) {
                parentMap[conn.second] = conn.first;
            }
            
            // 从目标回溯到源
            int current = sink;
            while (current != net.source && parentMap.find(current) != parentMap.end()) {
                path.push_back(current);
                current = parentMap[current];
            }
            path.push_back(net.source);
            
            // 反转路径以获得从源到目标的顺序
            std::reverse(path.begin(), path.end());
            
            // 将路径添加到缓存
            if (path.size() > 1) {
                routeCache.addPath(net.source, sink, path);
            }
        }
    }
    
    result.connections = treeConnections;
    result.success = success;
    
    return result;
}

std::vector<int> Router::optimizedAStarSearch(
    int start, 
    int target,
    const std::unordered_set<int>& avoidNodes,
    float congestionMultiplier) {
    
    // 直接连接检查
    if (start == target) {
        return {start};
    }
    
    // 避免节点检查
    if (avoidNodes.find(start) != avoidNodes.end() || avoidNodes.find(target) != avoidNodes.end()) {
        return {};
    }
    
    // 预计算目标点坐标
    int targetX = 0, targetY = 0;
    {
        if (target >= 0 && target < nodes.size()) {
            const auto& targetNode = nodes[target];
            targetX = (targetNode.begin_x + targetNode.end_x) / 2;
            targetY = (targetNode.begin_y + targetNode.end_y) / 2;
        }
    }
    
    // 使用open和closed集合
    std::unordered_map<int, float> gScore;
    std::unordered_map<int, int> cameFrom;
    std::unordered_set<int> closedSet;
    
    // 使用简单vector作为优先队列
    struct Entry {
        float fScore;
        int nodeId;
    };
    std::vector<Entry> openList;
    openList.reserve(1000);  // 预分配以减少重新分配
    
    // 初始化
    gScore[start] = 0;
    
    // 初始启发式估计
    float hStart = 0.0f;
    {
        if (start >= 0 && start < nodes.size()) {
            const auto& startNode = nodes[start];
            int x = (startNode.begin_x + startNode.end_x) / 2;
            int y = (startNode.begin_y + startNode.end_y) / 2;
            hStart = std::abs(x - targetX) + std::abs(y - targetY);
        }
    }
    
    openList.push_back({hStart, start});
    
    // 设置搜索限制
    const size_t MAX_ITERATIONS = 50000;
    size_t iterations = 0;
    
    while (!openList.empty() && iterations < MAX_ITERATIONS) {
        iterations++;
        
        // 找到f值最小的节点
        size_t current_idx = 0;
        float min_f = openList[0].fScore;
        for (size_t i = 1; i < openList.size(); ++i) {
            if (openList[i].fScore < min_f) {
                min_f = openList[i].fScore;
                current_idx = i;
            }
        }
        
        // 获取当前节点
        int current = openList[current_idx].nodeId;
        
        // 如果是目标节点，重建路径
        if (current == target) {
            std::vector<int> path;
            while (current != start) {
                path.push_back(current);
                current = cameFrom[current];
            }
            path.push_back(start);
            std::reverse(path.begin(), path.end());
            return path;
        }
        
        // 从openList中移除
        openList[current_idx] = openList.back();
        openList.pop_back();
        
        // 添加到closedSet
        closedSet.insert(current);
        
        // 获取邻居
        if (current >= 0 && current < adjacency.size()) {
            const auto& neighbors = adjacency[current];
            
            for (const auto& neighbor : neighbors) {
                // 跳过已关闭或应避开的节点
                if (closedSet.count(neighbor) > 0 || avoidNodes.count(neighbor) > 0) {
                    continue;
                }
                
                // 计算新的g值
                float edgeCost = 1.0f;
                auto congIt = congestionMap.find(neighbor);
                if (congIt != congestionMap.end()) {
                    edgeCost += congIt->second * 0.1f * congestionMultiplier;
                }
                
                // 考虑缓存的拥塞历史
                float cachedCongestion = routeCache.getCongestionFactor(neighbor);
                if (cachedCongestion > 0) {
                    edgeCost += cachedCongestion * 0.05f * congestionMultiplier;
                }
                
                float tentative_g = gScore[current] + edgeCost;
                
                // 检查是否已在openList中
                bool inOpenList = false;
                for (const auto& entry : openList) {
                    if (entry.nodeId == neighbor) {
                        inOpenList = true;
                        break;
                    }
                }
                
                // 如果不在openList或找到了更好的路径
                if (!inOpenList || tentative_g < gScore[neighbor]) {
                    cameFrom[neighbor] = current;
                    gScore[neighbor] = tentative_g;
                    
                    // 计算启发式
                    float h = 0.0f;
                    if (neighbor >= 0 && neighbor < nodes.size()) {
                        const auto& neighborNode = nodes[neighbor];
                        int x = (neighborNode.begin_x + neighborNode.end_x) / 2;
                        int y = (neighborNode.begin_y + neighborNode.end_y) / 2;
                        h = std::abs(x - targetX) + std::abs(y - targetY);
                    }
                    
                    float f = tentative_g + h;
                    
                    if (!inOpenList) {
                        openList.push_back({f, neighbor});
                    } else {
                        // 更新openList中的f值
                        for (auto& entry : openList) {
                            if (entry.nodeId == neighbor) {
                                entry.fScore = f;
                                break;
                            }
                        }
                    }
                }
            }
        }
    }
    
    // 未找到路径
    return {};
}

std::pair<std::unordered_set<int>, std::vector<std::pair<int, int>>> 
Router::buildSteinerTree(
    int source, 
    const std::vector<int>& sinks,
    const std::unordered_set<int>& avoidNodes,
    float congestionMultiplier) {
    
    std::unordered_set<int> treeNodes = {source};
    std::vector<std::pair<int, int>> treeConnections;
    std::unordered_set<int> remainingSinks(sinks.begin(), sinks.end());
    
    // 如果没有目标节点，则只返回源节点
    if (remainingSinks.empty()) {
        return {treeNodes, treeConnections};
    }
    
    // 尝试多达3次构建Steiner树，每次增加拥塞惩罚
    for (int attempt = 0; attempt < 3; attempt++) {
        float attemptCongestionMultiplier = congestionMultiplier * (1.0f + attempt * 0.3f);
        
        // 复制当前树状态用于当前尝试
        auto currentTreeNodes = treeNodes;
        auto currentConnections = treeConnections;
        auto currentRemainingSinks = remainingSinks;
        
        bool success = true;
        
        // 迭代添加路径到最近的目标
        while (!currentRemainingSinks.empty() && success) {
            std::vector<int> bestPath;
            int bestSink = -1;
            
            // 找到从任何树节点到任何目标的最短路径
            for (const auto& treeNode : currentTreeNodes) {
                for (const auto& sink : currentRemainingSinks) {
                    // 首先查看缓存
                    std::vector<int> path;
                    if (routeCache.hasPath(treeNode, sink)) {
                        path = routeCache.getPath(treeNode, sink);
                        
                        // 验证路径是否有效（不与避开的节点冲突）
                        bool pathValid = true;
                        for (const auto& node : path) {
                            if (avoidNodes.find(node) != avoidNodes.end()) {
                                pathValid = false;
                                break;
                            }
                        }
                        
                        if (!pathValid) {
                            // 如果缓存路径无效，使用A*搜索
                            path = optimizedAStarSearch(
                                treeNode, sink, avoidNodes, attemptCongestionMultiplier);
                        }
                    } else {
                        // 缓存未命中，使用A*搜索
                        path = optimizedAStarSearch(
                            treeNode, sink, avoidNodes, attemptCongestionMultiplier);
                        
                        // 如果找到路径，添加到缓存
                        if (!path.empty()) {
                            routeCache.addPath(treeNode, sink, path);
                        }
                    }
                    
                    if (!path.empty() && (bestPath.empty() || path.size() < bestPath.size())) {
                        bestPath = path;
                        bestSink = sink;
                    }
                }
            }
            
            // 如果找不到路径，标记此次尝试失败
            if (bestPath.empty()) {
                success = false;
                break;
            }
            
            // 将路径添加到树中
            for (size_t i = 0; i < bestPath.size() - 1; i++) {
                currentConnections.emplace_back(bestPath[i], bestPath[i + 1]);
                currentTreeNodes.insert(bestPath[i]);
                currentTreeNodes.insert(bestPath[i + 1]);
            }
            
            currentRemainingSinks.erase(bestSink);
        }
        
        // 如果成功，返回解决方案
        if (success) {
            return {currentTreeNodes, currentConnections};
        }
    }
    
    // 如果所有尝试都失败，返回最佳努力
    return {treeNodes, treeConnections};
}

void Router::updateCongestionMap(const std::unordered_set<int>& nodes) {
    for (const auto& node : nodes) {
        congestionMap[node]++;
        routeCache.updateCongestion(node);
    }
}

Router::Point Router::getNodeCenter(int nodeId) {
    if (nodeId >= 0 && nodeId < nodes.size()) {
        const auto& node = nodes[nodeId];
        return {(node.begin_x + node.end_x) / 2, (node.begin_y + node.end_y) / 2};
    }
    return {0, 0};
}

float Router::manhattanDistance(int node1, int node2) {
    Point p1 = getNodeCenter(node1);
    Point p2 = getNodeCenter(node2);
    
    return std::abs(p1.x - p2.x) + std::abs(p1.y - p2.y);
}

void Router::logProgress(size_t netsRouted, size_t totalNets, int netId) {
    if (!debug) return;
    
    auto currentTime = std::chrono::high_resolution_clock::now();
    double elapsedSeconds = std::chrono::duration_cast<std::chrono::milliseconds>(
        currentTime - startTime).count() / 1000.0;
    
    float percentComplete = (netsRouted * 100.0f / totalNets);
    
    std::cout << "[DEBUG] [" << std::fixed << std::setprecision(2) << elapsedSeconds << "s] "
              << "Progress: " << netsRouted << "/" << totalNets 
              << " nets (" << std::fixed << std::setprecision(1) << percentComplete << "%)" << std::endl;
    
    if (netId >= 0) {
        std::cout << "[DEBUG] [" << std::fixed << std::setprecision(2) << elapsedSeconds << "s] "
                  << "Currently routing net: " << netId << std::endl;
    }
    
    std::cout << "[DEBUG] [" << std::fixed << std::setprecision(2) << elapsedSeconds << "s] "
              << "Used resources: " << usedResources.size() 
              << ", Cache: " << cacheHits << " hits, " << cacheMisses << " misses"
              << ", Time remaining: " << std::fixed << std::setprecision(2) 
              << (timeLimit - elapsedSeconds) << "s" << std::endl;
}