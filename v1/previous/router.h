#pragma once

#include "dataloader.h"
#include "routecache.h"
#include <unordered_set>
#include <vector>
#include <chrono>

// 路由结果结构
struct RouteResult {
    int netId;
    std::string netName;
    std::vector<std::pair<int, int>> connections;
    bool success;
};

class Router {
public:
    Router(const NodeMap& nodes, 
           const AdjacencyList& adjacency, 
           const NetList& nets,
           const std::string& outputFile,
           bool debug = false);
    
    // 主路由函数
    void routeNets();
    
    // 缓存管理
    bool loadCache(const std::string& cacheFile);
    bool saveCache(const std::string& cacheFile);
    
private:
    struct Point {
        int x, y;  // 改为整数
    };
    
    // 简单的单次路由 (适用于小型设计)
    void simpleSinglePassRouting(const std::vector<Net>& sortedNets);
    
    // 谈判式路由的单次路由循环
    void routeIteration(int iteration, float congestionMultiplier);
    
    // 检查所有网络是否都成功路由
    bool checkAllSuccess() const;
    
    // 为单个网络寻找路由
    RouteResult routeNet(const Net& net, 
                        const std::unordered_set<int>& fixedNets,  // 改为整数集合
                        float congestionMultiplier = 1.0f);
    
    // 尝试使用缓存为网络寻找路由
    RouteResult tryRouteCached(const Net& net, 
                              const std::unordered_set<int>& avoidNodes);  // 改为整数集合
    
    // 高性能简化版A*搜索
    std::vector<int> optimizedAStarSearch(  // 返回整数ID路径
        int start, 
        int target,
        const std::unordered_set<int>& avoidNodes,
        float congestionMultiplier = 1.0f);
    
    // 构建Steiner树连接所有终端
    std::pair<std::unordered_set<int>, std::vector<std::pair<int, int>>> 
    buildSteinerTree(
        int source, 
        const std::vector<int>& sinks,
        const std::unordered_set<int>& avoidNodes,
        float congestionMultiplier = 1.0f);
    
    // 辅助函数
    float manhattanDistance(int node1, int node2);
    Point getNodeCenter(int nodeId);
    void updateCongestionMap(const std::unordered_set<int>& nodes);
    void logProgress(size_t netsRouted, size_t totalNets, int netId = -1);
    
    // 获取网络名称的辅助函数
    std::string getNetName(int netId);
    
    // 数据源
    const NodeMap& nodes;
    const AdjacencyList& adjacency;
    const NetList& nets;
    const std::string outputFile;
    bool debug;
    
    // 路由状态
    std::unordered_map<int, int> congestionMap;  // 节点拥塞历史
    std::vector<RouteResult> results;  // 最终路由结果
    std::unordered_set<int> usedResources;  // 当前使用的资源
    
    // 路由缓存
    RouteCache routeCache;
    
    // 统计和参数
    std::chrono::time_point<std::chrono::high_resolution_clock> startTime;
    size_t maxIterations;  // 最大谈判迭代次数
    bool useNegotiatedRouting;    // 是否使用谈判式路由
    double timeLimit;             // 时间限制(秒)
    
    // 缓存使用统计
    size_t cacheHits = 0;
    size_t cacheMisses = 0;
    
    // 根据设计规模自动确定参数
    void configureRoutingParameters();
};